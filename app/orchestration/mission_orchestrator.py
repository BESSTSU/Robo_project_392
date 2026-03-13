"""Mission FSM for fully automatic operation."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from app import config
from app.controllers.drive_client import DriveClient
from app.controllers.planting_sequence import PlantingSequence
from app.controllers.stepper_servo_client import StepperServoClient
from app.infrastructure.dual_camera import DualCameraManager
from app.models import AprilTagInfo, Detection, MissionSnapshot, MissionState
from app.services.apriltag_52h13 import AprilTag52h13Service
from app.services.pid import PID
from app.services.target_detector import TargetDetector

logger = logging.getLogger(__name__)


class MissionOrchestrator:
    def __init__(self):
        self.cameras = DualCameraManager()
        self.drive = DriveClient()
        self.actuator = StepperServoClient()
        self._stop_event = threading.Event()
        self.planting = PlantingSequence(
            self.actuator,
            set_status=self._set_status_text,
            should_stop=self._stop_event.is_set,
        )

        self.detector = TargetDetector()
        self.apriltag = AprilTag52h13Service()
        self.pid = PID(config.VISION.kp, config.VISION.ki, config.VISION.kd, config.VISION.integral_max)

        self.state = MissionState.IDLE
        self.running = False
        self.status_text = "Idle"
        self.target_class = ""
        self.april_info: AprilTagInfo | None = None

        self._mission_thread: threading.Thread | None = None

        self._front_frame_lock = threading.Lock()
        self._rear_frame_lock = threading.Lock()
        self._front_annotated: np.ndarray | None = None
        self._rear_annotated: np.ndarray | None = None
        self._front_idle_preview: np.ndarray | None = None
        self._rear_idle_preview: np.ndarray | None = None
        self._front_idle_preview_ts = 0.0
        self._rear_idle_preview_ts = 0.0

        self._distance_goal_cm = 0.0
        self._last_servo_step_ts = 0.0
        self._servo_cam_target_deg = float(config.ACTUATOR.servo_cam_start_deg)
        self._control_lock = threading.Lock()
        self._planting_busy = False
        self._measuring_busy = False
        self._approach_source = "front"
        self._no_box_streak = 0
        self._use_apriltag_distance_once = False
        self._pid_history = deque(maxlen=180)
        self._pid_seq = 0
        self._rear_measurement: dict | None = None
        self._apriltag_yaw_measurement: dict | None = None
        self._plant_map: list[dict] = []
        self._measure_idx = 0
        self._cm_per_px = float(config.VISION.rear_cm_per_px_default)
        self._cm_per_px_source = "default"
        self._manual_cm_per_px: float | None = None
        self._manual_cm_per_px_source: str | None = None
        self._align_hold_count = 0
        self._stuck_ref_ts = 0.0
        self._stuck_ref_dist_cm = 0.0
        self._stuck_retries = 0
        self._last_track_error = 0.0
        self._plants_done_current_pair = 0
        self._measure_interval_cm = 0.0
        self._preplant_adjust_pending = False
        self._preplant_adjust_search_start_cm = 0.0
        self._preplant_adjust_fallback_cm = 0.0
        self._preplant_adjust_goal_cm = 0.0
        self._preapproach_tag_attempted = False
        self._floor_profile_name = "normal"
        self._approach_min_norm = float(config.VISION.approach_min_norm)
        self._approach_max_norm = float(config.VISION.approach_max_norm)
        self._move_to_measure_speed_norm = float(config.VISION.move_to_measure_speed_norm)
        self._preplant_adjust_slow_norm = float(config.VISION.preplant_adjust_slow_norm)
        self._manual_drive_distance_norm = float(config.VISION.manual_drive_distance_norm)
        self._manual_rotate_norm_min = float(config.IMU.manual_rotate_norm_min)
        self._manual_rotate_norm_max = float(config.IMU.manual_rotate_norm_max)

    @staticmethod
    def _point_in_expanded_box(px: float, py: float, det: Detection, scale: float) -> bool:
        cx = (det.x1 + det.x2) * 0.5
        cy = (det.y1 + det.y2) * 0.5
        half_w = max(1.0, (det.x2 - det.x1) * 0.5 * scale)
        half_h = max(1.0, (det.y2 - det.y1) * 0.5 * scale)
        return (cx - half_w) <= px <= (cx + half_w) and (cy - half_h) <= py <= (cy + half_h)

    def _get_manual_drive_distance_norm(self) -> float:
        return float(self._manual_drive_distance_norm)

    def _get_approach_min_norm(self) -> float:
        return float(self._approach_min_norm)

    def _get_approach_max_norm(self) -> float:
        return float(self._approach_max_norm)

    def _get_move_to_measure_speed_norm(self) -> float:
        return float(self._move_to_measure_speed_norm)

    def _get_preplant_adjust_slow_norm(self) -> float:
        return float(self._preplant_adjust_slow_norm)

    def get_rear_scale_info(self) -> dict:
        return {
            "cm_per_px": float(round(self._cm_per_px, 6)),
            "source": str(self._cm_per_px_source),
            "manual_override": self._manual_cm_per_px is not None,
        }

    def _get_manual_rotate_norm_max(self) -> float:
        return float(self._manual_rotate_norm_max)

    def _get_manual_rotate_norm_min(self) -> float:
        return float(self._manual_rotate_norm_min)

    @staticmethod
    def _build_floor_profile_values() -> dict[str, dict[str, float]]:
        normal = {
            "approach_min_norm": float(config.VISION.approach_min_norm),
            "approach_max_norm": float(config.VISION.approach_max_norm),
            "move_to_measure_speed_norm": float(config.VISION.move_to_measure_speed_norm),
            "preplant_adjust_slow_norm": float(config.VISION.preplant_adjust_slow_norm),
            "manual_drive_distance_norm": float(config.VISION.manual_drive_distance_norm),
            "manual_rotate_norm_min": float(config.IMU.manual_rotate_norm_min),
            "manual_rotate_norm_max": float(config.IMU.manual_rotate_norm_max),
        }
        return {
            "smooth": {
                "approach_min_norm": max(0.05, normal["approach_min_norm"] * 0.90),
                "approach_max_norm": max(0.08, normal["approach_max_norm"] * 0.90),
                "move_to_measure_speed_norm": max(0.05, normal["move_to_measure_speed_norm"] * 0.90),
                "preplant_adjust_slow_norm": max(0.05, normal["preplant_adjust_slow_norm"] * 0.90),
                "manual_drive_distance_norm": max(0.05, normal["manual_drive_distance_norm"] * 0.90),
                "manual_rotate_norm_min": max(0.08, normal["manual_rotate_norm_min"] * 0.90),
                "manual_rotate_norm_max": max(0.10, normal["manual_rotate_norm_max"] * 0.90),
            },
            "normal": normal,
            "rough": {
                "approach_min_norm": min(1.0, normal["approach_min_norm"] * 1.20),
                "approach_max_norm": min(1.0, normal["approach_max_norm"] * 1.18),
                "move_to_measure_speed_norm": min(1.0, normal["move_to_measure_speed_norm"] * 1.20),
                "preplant_adjust_slow_norm": min(1.0, normal["preplant_adjust_slow_norm"] * 1.20),
                "manual_drive_distance_norm": min(1.0, normal["manual_drive_distance_norm"] * 1.20),
                "manual_rotate_norm_min": min(1.0, normal["manual_rotate_norm_min"] * 1.30),
                "manual_rotate_norm_max": min(1.0, normal["manual_rotate_norm_max"] * 1.15),
            },
        }

    def _try_read_apriltag_once(self, frame: np.ndarray) -> tuple[AprilTagInfo | None, dict | None, np.ndarray]:
        dets, det_anno = self.detector.detect_all(
            frame, ("facewoodenbox", "woodenbox"), max_per_class=3, max_total=6
        )
        face_candidates = [d for d in dets if d.class_name.strip().lower() == "facewoodenbox"]
        woodenbox_candidates = [d for d in dets if d.class_name.strip().lower() == "woodenbox"]
        allow_woodenbox_fallback = bool(config.VISION.apriltag_allow_woodenbox_fallback)
        expand = float(config.VISION.apriltag_face_box_expand_scale)
        candidates, anno = self.apriltag._detect_tag_candidates(det_anno)
        if not candidates:
            return None, None, anno

        best: tuple[AprilTagInfo, dict, str] | None = None
        best_rank: tuple[float, float, float, float, float] | None = None
        best_reject: tuple[dict, str, str] | None = None
        best_reject_rank: tuple[float, float, float, float, float] | None = None
        saw_near_face = False
        saw_near_woodenbox = False
        for _, info, metrics in candidates:
            tag_x = metrics.get("tag_center_x_px")
            tag_y = metrics.get("tag_center_y_px")
            if tag_x is None or tag_y is None:
                continue

            near_face_dets = [
                det for det in face_candidates
                if self._point_in_expanded_box(float(tag_x), float(tag_y), det, expand)
            ]
            near_woodenbox_dets = [
                det for det in woodenbox_candidates
                if self._point_in_expanded_box(float(tag_x), float(tag_y), det, expand)
            ]
            if near_face_dets:
                anchor_dets = near_face_dets
                anchor_name = "FaceWoodenbox"
                saw_near_face = True
            elif allow_woodenbox_fallback and near_woodenbox_dets:
                anchor_dets = near_woodenbox_dets
                anchor_name = "woodenbox"
                saw_near_woodenbox = True
            else:
                continue

            center_err = float(metrics.get("tag_center_err_norm", 0.0))
            pose_yaw_deg = metrics.get("tag_pose_yaw_deg")
            practical_yaw_deg = metrics.get("tag_robot_yaw_practical_deg")
            yaw_deg = metrics.get("tag_effective_yaw_deg")
            yaw_source = str(metrics.get("tag_effective_yaw_source", "none"))
            tag_side_px = float(metrics.get("tag_side_px", 0.0))
            bearing_x_deg = metrics.get("tag_bearing_x_deg")
            front_score = float(metrics.get("tag_front_facing_score", 999.0))
            if anchor_name == "woodenbox":
                center_tol = float(config.VISION.apriltag_woodenbox_center_tol_norm)
                yaw_tol = float(config.VISION.apriltag_woodenbox_yaw_deg)
                min_side_px = float(config.VISION.apriltag_woodenbox_min_side_px)
                bearing_tol = float(config.VISION.apriltag_woodenbox_bearing_deg)
            else:
                center_tol = float(config.VISION.apriltag_accept_center_tol_norm)
                yaw_tol = float(config.VISION.apriltag_accept_yaw_deg)
                min_side_px = 0.0
                bearing_tol = None
            if anchor_name == "woodenbox" and bearing_x_deg is not None:
                center_ok = abs(float(bearing_x_deg)) <= bearing_tol
            else:
                center_ok = abs(center_err) <= center_tol
            yaw_ok = yaw_deg is not None and abs(float(yaw_deg)) <= yaw_tol
            size_ok = tag_side_px >= min_side_px
            anchor_dist = min(
                math.hypot(float(tag_x) - ((det.x1 + det.x2) * 0.5), float(tag_y) - ((det.y1 + det.y2) * 0.5))
                for det in anchor_dets
            )
            anchor_priority = 0.0 if anchor_name == "FaceWoodenbox" else 1.0
            bearing_abs = 999.0 if bearing_x_deg is None else abs(float(bearing_x_deg))
            rank = (anchor_priority, front_score, anchor_dist, bearing_abs, -tag_side_px)
            if not (center_ok and yaw_ok and size_ok):
                reasons: list[str] = []
                if not center_ok:
                    reasons.append("center")
                if not yaw_ok:
                    reasons.append("yaw")
                if not size_ok:
                    reasons.append("size")
                reason_text = ",".join(reasons) if reasons else "unknown"
                if best_reject is None or best_reject_rank is None or rank < best_reject_rank:
                    best_reject = (metrics, anchor_name, reason_text)
                    best_reject_rank = rank
                continue

            # Prefer FaceWoodenbox first, then the most front-facing tag, then nearest anchor distance.
            if best is None or best_rank is None or rank < best_rank:
                best = (info, metrics, anchor_name)
                best_rank = rank

        if best is not None:
            if best[2] == "woodenbox":
                self.status_text = "AprilTag accepted near woodenbox (FaceWoodenbox not visible yet)"
            return best[0], best[1], anno

        if (saw_near_face or saw_near_woodenbox) and best_reject is not None:
            sample_metrics, anchor_name, reason_text = best_reject
            center_err = float(sample_metrics.get("tag_center_err_norm", 0.0))
            pose_yaw_deg = sample_metrics.get("tag_pose_yaw_deg")
            practical_yaw_deg = sample_metrics.get("tag_robot_yaw_practical_deg")
            effective_yaw_deg = sample_metrics.get("tag_effective_yaw_deg")
            yaw_source = sample_metrics.get("tag_effective_yaw_source")
            side_px = float(sample_metrics.get("tag_side_px", 0.0))
            bearing_x_deg = sample_metrics.get("tag_bearing_x_deg")
            self.status_text = (
                f"AprilTag seen but rejected ({anchor_name}:{reason_text}) "
                f"(center={center_err:+.3f}, bearing={bearing_x_deg if bearing_x_deg is not None else 'na'}, "
                f"yaw={pose_yaw_deg if pose_yaw_deg is not None else 'na'}, "
                f"yaw2={practical_yaw_deg if practical_yaw_deg is not None else 'na'}, "
                f"yaw_eff={effective_yaw_deg if effective_yaw_deg is not None else 'na'}, "
                f"yaw_src={yaw_source if yaw_source is not None else 'na'}, side_px={side_px:.1f})"
            )
        else:
            self.status_text = (
                "AprilTag seen but rejected (not near FaceWoodenbox)"
                if not allow_woodenbox_fallback
                else "AprilTag seen but rejected (not near FaceWoodenbox/woodenbox)"
            )
        return None, None, anno

    def _cache_apriltag_info(self, found: AprilTagInfo | None, metrics: dict | None) -> bool:
        if found is None:
            return False

        self.april_info = found
        self._use_apriltag_distance_once = True
        self._plants_done_current_pair = 0
        tag_side_px = 0.0 if metrics is None else float(metrics.get("tag_side_px", 0.0))
        if (
            self._manual_cm_per_px is None
            and tag_side_px > 1.0
            and float(config.VISION.apriltag_size_cm) > 0.0
        ):
            self._cm_per_px = float(config.VISION.apriltag_size_cm) / tag_side_px
            self._cm_per_px_source = "apriltag(front)"
        self.status_text = (
            f"Tag {found.tag_id:05d} AB={found.planting_distance_cm} Cgap={found.spacing_gap_cm} "
            f"DE={found.cabbage_interval_cm} cm_per_px={self._cm_per_px:.4f} ({self._cm_per_px_source})"
        )
        return True

    def _run_apriltag_coarse_align(self) -> bool:
        """
        Practical coarse align step after AprilTag is read:
        rotate front camera view until tag center is near image center.

        This deliberately keeps the existing process structure unchanged.
        If align times out, caller may continue with the original flow.
        """
        if not bool(config.VISION.apriltag_coarse_align_enabled):
            return True

        deadline = time.monotonic() + max(0.5, float(config.VISION.apriltag_coarse_align_timeout_sec))
        hold_needed = max(1, int(config.VISION.apriltag_coarse_align_hold_frames))
        hold_count = 0
        target_heading_deg: float | None = None

        while not self._stop_event.is_set() and time.monotonic() < deadline:
            frame = self.cameras.get_raw_front_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            found, anno, metrics = self.apriltag.detect_and_annotate_with_metrics(frame)
            self._set_front_frame(anno)
            if found is None or metrics is None:
                self.drive.stop()
                self.status_text = "AprilTag coarse align lost tag"
                return False

            h, w = frame.shape[:2]
            tol_norm = float(config.VISION.apriltag_coarse_align_tol_px) / max(1.0, (w / 2.0))
            center_err = float(metrics.get("tag_center_err_norm", 0.0))
            bearing_x = metrics.get("tag_bearing_x_deg")
            current_heading = self.drive.get_heading_deg()
            heading_err_deg = None
            if current_heading is not None and bearing_x is not None:
                if target_heading_deg is None:
                    target_heading_deg = (float(current_heading) + float(bearing_x)) % 360.0
                heading_err_deg = (float(target_heading_deg) - float(current_heading) + 180.0) % 360.0 - 180.0
            if abs(center_err) <= tol_norm:
                hold_count += 1
            else:
                hold_count = 0

            if hold_count >= hold_needed:
                self.drive.stop()
                self.status_text = (
                    "AprilTag coarse align done "
                    f"(err={center_err:+.3f}, angle={bearing_x if bearing_x is not None else 'na'}, "
                    f"imu={heading_err_deg if heading_err_deg is not None else 'na'})"
                )
                return True

            if heading_err_deg is not None and abs(heading_err_deg) > float(config.IMU.heading_tolerance_deg):
                turn = self._clamp(
                    heading_err_deg * float(config.IMU.turn_gain),
                    -float(config.IMU.turn_norm_max),
                    float(config.IMU.turn_norm_max),
                )
            else:
                turn = self._clamp(
                    float(config.VISION.apriltag_coarse_align_turn_gain) * center_err,
                    -float(config.VISION.apriltag_coarse_align_turn_norm),
                    float(config.VISION.apriltag_coarse_align_turn_norm),
                )
            fwd = float(config.VISION.apriltag_coarse_align_forward_norm) if abs(center_err) <= (tol_norm * 2.0) else 0.0
            left_cmd = int(self._clamp(fwd + turn, -1.0, 1.0) * config.VISION.max_cmd)
            right_cmd = int(self._clamp(fwd - turn, -1.0, 1.0) * config.VISION.max_cmd)
            self.drive.send_velocity(left_cmd, right_cmd)
            self._push_pid_sample(center_err, turn, left_cmd, right_cmd, "front:tag_align")
            self.status_text = (
                "AprilTag coarse align "
                f"err={center_err:+.3f} angle={bearing_x if bearing_x is not None else 'na'} "
                f"imu={heading_err_deg if heading_err_deg is not None else 'na'} "
                f"hold={hold_count}/{hold_needed}"
            )
            time.sleep(config.RUNTIME.vision_loop_sec)

        self.drive.stop()
        self.status_text = "AprilTag coarse align timeout"
        return False

    @staticmethod
    def _wrap_deg(value: float) -> float:
        return (float(value) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _counts_to_signed_distance_cm(left_count: int, right_count: int) -> float:
        wheel_circ_cm = float(config.MOTION.wheel_diameter_inch) * 2.54 * math.pi
        cpr = max(1, int(config.MOTION.counts_per_rev))
        avg_counts = (float(left_count) + float(right_count)) * 0.5
        return (avg_counts / cpr) * wheel_circ_cm

    def _rotate_to_heading(
        self,
        target_heading_deg: float,
        label: str,
        *,
        timeout_sec: float | None = None,
        hold_needed: int = 3,
        turn_gain: float | None = None,
        turn_norm_min: float | None = None,
        turn_norm_max: float | None = None,
        heading_tolerance_deg: float | None = None,
        fine_band_deg: float | None = None,
        fine_turn_gain: float | None = None,
        fine_turn_norm_max: float | None = None,
    ) -> bool:
        deadline = time.monotonic() + max(
            0.5,
            float(timeout_sec if timeout_sec is not None else config.VISION.apriltag_triangle_turn_timeout_sec),
        )
        hold = 0
        gain = float(turn_gain if turn_gain is not None else config.IMU.turn_gain)
        norm_min = 0.0 if turn_norm_min is None else max(0.0, float(turn_norm_min))
        norm_max = float(turn_norm_max if turn_norm_max is not None else config.IMU.turn_norm_max)
        tolerance_deg = float(
            heading_tolerance_deg if heading_tolerance_deg is not None else config.IMU.heading_tolerance_deg
        )
        fine_band = None if fine_band_deg is None else float(fine_band_deg)
        fine_gain = gain if fine_turn_gain is None else float(fine_turn_gain)
        fine_norm = norm_max if fine_turn_norm_max is None else float(fine_turn_norm_max)
        last_heading = None
        last_err_deg = None

        while not self._stop_event.is_set() and time.monotonic() < deadline:
            current_heading = self.drive.get_heading_deg()
            if current_heading is None:
                self.drive.stop()
                self.status_text = f"{label}: no IMU heading"
                return False

            err_deg = self._wrap_deg(float(target_heading_deg) - float(current_heading))
            last_heading = float(current_heading)
            last_err_deg = float(err_deg)
            if abs(err_deg) <= tolerance_deg:
                hold += 1
            else:
                hold = 0

            if hold >= hold_needed:
                self.drive.stop()
                self.status_text = f"{label}: heading locked err={err_deg:+.1f}deg"
                return True

            active_gain = gain
            active_norm = norm_max
            if fine_band is not None and abs(err_deg) <= fine_band:
                active_gain = fine_gain
                active_norm = fine_norm

            turn = self._clamp(
                err_deg * active_gain,
                -active_norm,
                active_norm,
            )
            if norm_min > 0.0 and abs(err_deg) > tolerance_deg and abs(turn) < active_norm:
                enforced_min = min(active_norm, norm_min)
                if abs(turn) < enforced_min:
                    turn = math.copysign(enforced_min, turn if abs(turn) > 1e-6 else err_deg)
            cmd = int(turn * config.VISION.max_cmd)
            self.drive.send_velocity(cmd, -cmd)
            self.status_text = f"{label}: current={current_heading:.1f} target={target_heading_deg:.1f} err={err_deg:+.1f}"
            time.sleep(config.RUNTIME.vision_loop_sec)

        self.drive.stop()
        if last_heading is not None and last_err_deg is not None:
            self.status_text = (
                f"{label}: turn timeout current={last_heading:.1f} "
                f"target={float(target_heading_deg):.1f} err={last_err_deg:+.1f}"
            )
        else:
            self.status_text = f"{label}: turn timeout"
        return False

    def _drive_forward_distance(self, distance_cm: float, hold_heading_deg: float | None, label: str) -> bool:
        if distance_cm <= 0.0:
            return True

        start_dist = float(self.drive.get_telemetry().distance_cm)
        deadline = time.monotonic() + max(0.5, float(config.VISION.apriltag_triangle_move_timeout_sec))
        base_cmd = int(max(1, config.VISION.max_cmd * float(config.VISION.apriltag_triangle_move_norm)))

        while not self._stop_event.is_set() and time.monotonic() < deadline:
            tel = self.drive.get_telemetry()
            moved_cm = float(tel.distance_cm) - start_dist
            remaining = float(distance_cm) - moved_cm
            if remaining <= 0.0:
                self.drive.stop()
                self.status_text = f"{label}: move done {moved_cm:.1f}cm"
                return True

            turn_cmd = 0
            if hold_heading_deg is not None:
                current_heading = self.drive.get_heading_deg()
                if current_heading is not None:
                    err_deg = self._wrap_deg(float(hold_heading_deg) - float(current_heading))
                    turn_norm = self._clamp(
                        err_deg * float(config.IMU.turn_gain),
                        -float(config.IMU.turn_norm_max),
                        float(config.IMU.turn_norm_max),
                    )
                    turn_cmd = int(turn_norm * config.VISION.max_cmd)

            left_cmd = int(self._clamp((base_cmd + turn_cmd) / float(config.VISION.max_cmd), -1.0, 1.0) * config.VISION.max_cmd)
            right_cmd = int(self._clamp((base_cmd - turn_cmd) / float(config.VISION.max_cmd), -1.0, 1.0) * config.VISION.max_cmd)
            self.drive.send_velocity(left_cmd, right_cmd)
            self.status_text = f"{label}: moved={moved_cm:.1f}cm remain={remaining:.1f}cm"
            if not self._check_stuck_and_recover(True, self._last_track_error):
                return False
            time.sleep(config.RUNTIME.vision_loop_sec)

        self.drive.stop()
        self.status_text = f"{label}: move timeout"
        return False

    def _drive_relative_distance(
        self,
        distance_cm: float,
        hold_heading_deg: float | None,
        label: str,
        *,
        timeout_sec: float | None = None,
        drive_norm: float | None = None,
    ) -> bool:
        if abs(distance_cm) <= 0.0:
            return True

        start_tel = self.drive.get_telemetry()
        start_signed_cm = self._counts_to_signed_distance_cm(start_tel.left_count, start_tel.right_count)
        deadline = time.monotonic() + max(
            0.5,
            float(timeout_sec if timeout_sec is not None else config.VISION.manual_drive_distance_timeout_sec),
        )
        norm = abs(float(drive_norm if drive_norm is not None else self._get_manual_drive_distance_norm()))
        base_cmd_mag = int(max(1, config.VISION.max_cmd * norm))
        drive_dir = 1 if distance_cm >= 0.0 else -1
        last_moved_cm = 0.0
        last_remaining_cm = float(distance_cm)

        while not self._stop_event.is_set() and time.monotonic() < deadline:
            tel = self.drive.get_telemetry()
            current_signed_cm = self._counts_to_signed_distance_cm(tel.left_count, tel.right_count)
            moved_cm = current_signed_cm - start_signed_cm
            remaining = float(distance_cm) - moved_cm
            last_moved_cm = float(moved_cm)
            last_remaining_cm = float(remaining)
            if drive_dir * remaining <= 0.0:
                self.drive.stop()
                self.status_text = f"{label}: move done {moved_cm:.1f}cm"
                return True

            turn_cmd = 0
            if hold_heading_deg is not None:
                current_heading = self.drive.get_heading_deg()
                if current_heading is not None:
                    err_deg = self._wrap_deg(float(hold_heading_deg) - float(current_heading))
                    turn_norm = self._clamp(
                        err_deg * float(config.IMU.turn_gain),
                        -float(config.IMU.turn_norm_max),
                        float(config.IMU.turn_norm_max),
                    )
                    turn_cmd = int(turn_norm * config.VISION.max_cmd)

            base_cmd = drive_dir * base_cmd_mag
            left_cmd = int(
                self._clamp((base_cmd + turn_cmd) / float(config.VISION.max_cmd), -1.0, 1.0) * config.VISION.max_cmd
            )
            right_cmd = int(
                self._clamp((base_cmd - turn_cmd) / float(config.VISION.max_cmd), -1.0, 1.0) * config.VISION.max_cmd
            )
            self.drive.send_velocity(left_cmd, right_cmd)
            self.status_text = f"{label}: moved={moved_cm:.1f}cm remain={remaining:.1f}cm"
            time.sleep(config.RUNTIME.vision_loop_sec)

        self.drive.stop()
        if self._stop_event.is_set():
            self.status_text = f"{label}: stopped moved={last_moved_cm:.1f}cm"
        else:
            self.status_text = (
                f"{label}: move timeout moved={last_moved_cm:.1f}cm remain={last_remaining_cm:.1f}cm"
            )
        return False

    def _run_apriltag_triangle_align(self) -> bool:
        if not bool(config.VISION.apriltag_triangle_align_enabled):
            self.status_text = "Triangle align disabled"
            return False
        measurement = self._measure_apriltag_yaw(timeout_sec=6.0, min_samples=2)
        if measurement is None:
            self.status_text = "Triangle align skipped: no usable AprilTag yaw measurement"
            return False

        yaw_deg = measurement.get("yaw_deg")
        approx_dist_cm = measurement.get("approx_dist_cm")
        if yaw_deg is None or approx_dist_cm is None:
            self.status_text = "Triangle align skipped: missing approx_dist/yaw measurement"
            return False

        yaw_deg = float(yaw_deg)
        approx_dist_cm = float(approx_dist_cm)
        move1_cm = abs(approx_dist_cm * math.sin(math.radians(yaw_deg)))
        turn1_deg = (-90.0 - yaw_deg) if yaw_deg < 0.0 else (90.0 - yaw_deg)
        turn2_deg = 90.0 if yaw_deg < 0.0 else -90.0

        if move1_cm > float(config.VISION.apriltag_triangle_max_move_cm):
            self.status_text = f"Triangle align skipped: move too large ({move1_cm:.1f}cm)"
            return False
        if abs(turn1_deg) > float(config.VISION.apriltag_triangle_max_turn_deg):
            self.status_text = f"Triangle align skipped: turn1 too large ({turn1_deg:.1f}deg)"
            return False
        if abs(turn2_deg) > float(config.VISION.apriltag_triangle_max_turn_deg):
            self.status_text = f"Triangle align skipped: turn2 too large ({turn2_deg:.1f}deg)"
            return False

        logger.info(
            "Triangle plan(yaw-scan): approx=%.1fcm yaw=%.1fdeg turn1=%.1fdeg move1=%.1fcm turn2=%.1fdeg",
            approx_dist_cm,
            yaw_deg,
            turn1_deg,
            move1_cm,
            turn2_deg,
        )
        self.status_text = (
            f"Triangle plan: approx={approx_dist_cm:.1f} yaw={yaw_deg:.1f} "
            f"turn1={turn1_deg:.1f} move1={move1_cm:.1f} turn2={turn2_deg:.1f}"
        )
        if abs(turn1_deg) > 1.0 and not self._rotate_by_manual_degrees(turn1_deg, "triangle turn1"):
            return False

        hold_heading = self.drive.get_heading_deg()
        if move1_cm > 1.0 and not self._drive_relative_distance(
            move1_cm,
            hold_heading,
            "triangle move1",
            timeout_sec=float(config.VISION.manual_drive_distance_timeout_sec),
            drive_norm=self._get_manual_drive_distance_norm(),
        ):
            return False

        if abs(turn2_deg) > 1.0 and not self._rotate_by_manual_degrees(turn2_deg, "triangle turn2"):
            return False

        self.drive.stop()
        self.status_text = (
            f"Triangle align done approx={approx_dist_cm:.1f}cm yaw={yaw_deg:.1f}deg "
            f"turn1={turn1_deg:.1f} move1={move1_cm:.1f} turn2={turn2_deg:.1f}"
        )
        return True

    def _set_status_text(self, text: str) -> None:
        self.status_text = str(text)

    def _is_serial_mapping_swapped(self) -> tuple[bool, str]:
        """
        Detect swapped drive/actuator serial mapping.
        - Actuator stream should contain ACT_* lines.
        - Drive stream should contain DRV_* lines.
        """
        # Probe both sides once.
        seq_before_act = self.actuator.get_rx_seq()
        seq_before_drv = self.drive.get_rx_seq()
        self.actuator.request_status()
        self.drive.request_status()
        act_line = self.actuator.wait_for_status_line(0.5, after_seq=seq_before_act) or self.actuator.last_message
        drv_line = self.drive.wait_for_status_line(0.5, after_seq=seq_before_drv) or self.drive.get_telemetry().raw_line

        act_u = str(act_line).strip().upper()
        drv_u = str(drv_line).strip().upper()

        if act_u.startswith("DRV_") or drv_u.startswith("ACT_"):
            detail = f"actuator_last={act_u or '<none>'}, drive_last={drv_u or '<none>'}"
            return True, detail
        return False, ""

    def _update_servo_cam_tracking(self, bbox_bottom: int) -> None:
        """
        Smooth camera-servo tracking for vertical bbox trigger:
        - build a moving target based on overshoot
        - apply slew-rate limited commands for smooth motion
        """
        trigger = int(config.VISION.bbox_trigger_line_y)
        if bbox_bottom < trigger:
            return

        overshoot = max(0, int(bbox_bottom) - trigger)
        base_step = max(1.0, float(config.ACTUATOR.servo_cam_step_deg))
        threshold = max(1.0, float(config.ACTUATOR.servo_cam_fast_px_threshold))
        max_mul = max(1.0, float(config.ACTUATOR.servo_cam_fast_max_multiplier))
        speed_mul = 1.0 + min(max_mul - 1.0, overshoot / threshold)

        # Move target downward continuously; command path to target with slew limit.
        end_deg = float(config.ACTUATOR.servo_cam_end_deg)
        self._servo_cam_target_deg = max(end_deg, self._servo_cam_target_deg - (base_step * speed_mul))

        now = time.monotonic()
        if now - self._last_servo_step_ts < float(config.ACTUATOR.servo_cam_step_interval_sec):
            return

        cur_deg = float(self.actuator.servo_cam_deg)
        # Smooth but responsive: larger overshoot allows faster slew while staying continuous.
        slew_step = max(1.0, min(8.0, base_step * speed_mul * 0.7))
        next_deg = max(self._servo_cam_target_deg, cur_deg - slew_step)
        next_deg_i = int(round(next_deg))

        if next_deg_i < self.actuator.servo_cam_deg:
            self.actuator.set_servo_cam(next_deg_i)
        self._last_servo_step_ts = now

    # ---------- lifecycle ----------
    def start_background_services(self) -> None:
        self.cameras.start()
        self.drive.open()
        self.actuator.open()
        if not self.detector.enabled:
            logger.warning("Detector disabled: %s", self.detector.disabled_reason)
        logger.info("Background services started")

    def shutdown(self) -> None:
        self.stop_mission()
        self.drive.stop()
        self.actuator.stop_all()
        self.cameras.stop()
        self.drive.close()
        self.actuator.close()

    # ---------- command API ----------
    def start_mission(self) -> tuple[bool, str]:
        if self.running:
            return False, "mission already running"

        if not self.drive.link.connected and not self.drive.open():
            return False, "drive not connected"
        if not self.actuator.link.connected and not self.actuator.open():
            return False, "actuator not connected"

        # Always start mission with physical front/rear mapping.
        self.cameras.set_swap(False)
        self.running = True
        self._stop_event.clear()
        self._mission_thread = threading.Thread(target=self._mission_loop, daemon=True)
        self._mission_thread.start()
        return True, "mission started"

    def stop_mission(self) -> tuple[bool, str]:
        # Always raise stop-event so long operations (e.g. planting) can cancel.
        self._stop_event.set()
        self.drive.stop()
        self.actuator.stop_all()
        if not self.running:
            if self._planting_busy:
                self.state = MissionState.STOPPED
                self.status_text = "Stop requested while manual planting"
                return True, "manual planting stop requested"
            if self._measuring_busy:
                self.state = MissionState.STOPPED
                self.status_text = "Stop requested while manual measuring"
                return True, "manual measuring stop requested"
            self.state = MissionState.STOPPED
            self.status_text = "Stopped"
            return True, "already stopped"

        if self._mission_thread is not None:
            self._mission_thread.join(timeout=6.0)
            if self._mission_thread.is_alive():
                logger.warning("Mission thread still alive after stop timeout")
        self.running = False
        self.state = MissionState.STOPPED
        self.status_text = "Stopped by operator"
        return True, "mission stopped"

    def manual_move(self, direction: str) -> tuple[bool, str]:
        """
        Manual drive command from web.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before manual control"

        d = direction.strip().lower()
        drive_cmd = int(config.VISION.max_cmd * 0.35)
        turn_cmd = int(config.VISION.max_cmd * 0.28)

        with self._control_lock:
            if d == "forward":
                self.drive.send_velocity(drive_cmd, drive_cmd)
                self.status_text = f"Manual FORWARD ({drive_cmd})"
            elif d == "backward":
                self.drive.send_velocity(-drive_cmd, -drive_cmd)
                self.status_text = f"Manual BACKWARD ({drive_cmd})"
            elif d == "left":
                self.drive.send_velocity(-turn_cmd, turn_cmd)
                self.status_text = f"Manual LEFT ({turn_cmd})"
            elif d == "right":
                self.drive.send_velocity(turn_cmd, -turn_cmd)
                self.status_text = f"Manual RIGHT ({turn_cmd})"
            elif d == "stop":
                self.drive.stop()
                self.status_text = "Manual STOP"
            else:
                return False, f"unknown direction: {direction}"

        return True, "ok"

    def _rotate_by_manual_degrees(self, delta_deg: float, label: str) -> bool:
        current_heading = self.drive.get_heading_deg()
        if current_heading is None:
            self.status_text = f"{label}: no IMU heading"
            return False

        target_heading = (float(current_heading) - float(delta_deg)) % 360.0
        self.status_text = (
            f"{label}: current={float(current_heading):.1f} "
            f"delta={float(delta_deg):+.1f} target={target_heading:.1f}"
        )
        return self._rotate_to_heading(
            target_heading,
            label,
            timeout_sec=float(config.IMU.manual_rotate_timeout_sec),
            hold_needed=int(config.IMU.manual_rotate_hold_samples),
            turn_gain=float(config.IMU.manual_rotate_gain),
            turn_norm_min=self._get_manual_rotate_norm_min(),
            turn_norm_max=self._get_manual_rotate_norm_max(),
            heading_tolerance_deg=float(config.IMU.manual_rotate_tolerance_deg),
        )

    def manual_rotate(self, degrees: float) -> tuple[bool, str]:
        """
        Manual relative rotate command from web.
        Positive degrees are inverted relative to the IMU-positive heading direction.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before manual rotate"

        try:
            delta_deg = float(degrees)
        except (TypeError, ValueError):
            return False, f"invalid rotate degrees: {degrees}"

        if abs(delta_deg) < 0.1:
            return False, "rotate degrees too small"

        with self._control_lock:
            self._stop_event.clear()
            self.drive.stop()
            ok = self._rotate_by_manual_degrees(delta_deg, f"manual rotate {delta_deg:+.1f}deg")
            if ok:
                return True, f"manual rotate done delta={delta_deg:+.1f}deg"
            return False, self.status_text

    def manual_drive_distance(self, distance_cm: float) -> tuple[bool, str]:
        """
        Manual relative linear move from web in centimeters.
        Positive values move forward, negative values move backward.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before manual distance move"

        try:
            target_cm = float(distance_cm)
        except (TypeError, ValueError):
            return False, f"invalid drive distance: {distance_cm}"

        if abs(target_cm) < 0.1:
            return False, "drive distance too small"

        max_cm = float(config.VISION.manual_drive_distance_max_cm)
        if abs(target_cm) > max_cm:
            return False, f"drive distance too large: {target_cm:.1f}cm (max {max_cm:.1f}cm)"

        with self._control_lock:
            self._stop_event.clear()
            self.drive.stop()
            hold_heading = self.drive.get_heading_deg()
            heading_text = "na" if hold_heading is None else f"{float(hold_heading):.1f}"
            self.status_text = (
                f"Manual drive requested: distance={target_cm:+.1f}cm "
                f"heading={heading_text}"
            )
            ok = self._drive_relative_distance(
                target_cm,
                hold_heading,
                f"manual drive {target_cm:+.1f}cm",
                timeout_sec=float(config.VISION.manual_drive_distance_timeout_sec),
                drive_norm=self._get_manual_drive_distance_norm(),
            )
            if ok:
                return True, f"manual drive done target={target_cm:+.1f}cm"
            return False, self.status_text

    def set_manual_drive_distance_norm(self, value: float) -> tuple[bool, str]:
        try:
            norm = float(value)
        except (TypeError, ValueError):
            return False, f"invalid drive speed norm: {value}"

        if not 0.05 <= norm <= 1.0:
            return False, f"drive speed norm out of range: {norm:.3f} (use 0.05..1.00)"

        with self._control_lock:
            self._manual_drive_distance_norm = float(norm)
            self._floor_profile_name = "custom"
            self.status_text = (
                f"Manual drive speed set to {self._manual_drive_distance_norm:.2f} "
                f"(affects Drive By Distance + triangle move1)"
            )
        return True, self.status_text

    def set_floor_profile(self, profile: str) -> tuple[bool, str]:
        profile_name = str(profile or "").strip().lower()
        profiles = self._build_floor_profile_values()
        if profile_name not in profiles:
            return False, f"unknown floor profile: {profile}"

        values = profiles[profile_name]
        with self._control_lock:
            self._floor_profile_name = profile_name
            self._approach_min_norm = float(values["approach_min_norm"])
            self._approach_max_norm = float(values["approach_max_norm"])
            self._move_to_measure_speed_norm = float(values["move_to_measure_speed_norm"])
            self._preplant_adjust_slow_norm = float(values["preplant_adjust_slow_norm"])
            self._manual_drive_distance_norm = float(values["manual_drive_distance_norm"])
            self._manual_rotate_norm_min = float(values["manual_rotate_norm_min"])
            self._manual_rotate_norm_max = float(values["manual_rotate_norm_max"])
            self.status_text = (
                f"Floor profile set to {profile_name} "
                f"(approach {self._approach_min_norm:.2f}-{self._approach_max_norm:.2f}, "
                f"measure {self._move_to_measure_speed_norm:.2f}, "
                f"rotate {self._manual_rotate_norm_min:.2f}-{self._manual_rotate_norm_max:.2f})"
            )
        return True, self.status_text

    def set_runtime_drive_tuning(self, values: dict) -> tuple[bool, str]:
        def _read(name: str, lo: float = 0.05, hi: float = 1.0) -> float:
            raw = values.get(name)
            try:
                val = float(raw)
            except (TypeError, ValueError):
                raise ValueError(f"invalid {name}: {raw}")
            if not lo <= val <= hi:
                raise ValueError(f"{name} out of range: {val:.3f} (use {lo:.2f}..{hi:.2f})")
            return float(val)

        try:
            approach_min_norm = _read("approach_min_norm")
            approach_max_norm = _read("approach_max_norm")
            move_to_measure_speed_norm = _read("move_to_measure_speed_norm")
            preplant_adjust_slow_norm = _read("preplant_adjust_slow_norm")
            manual_drive_distance_norm = _read("manual_drive_distance_norm")
            manual_rotate_norm_min = _read("manual_rotate_norm_min")
            manual_rotate_norm_max = _read("manual_rotate_norm_max")
        except ValueError as exc:
            return False, str(exc)

        if approach_min_norm > approach_max_norm:
            return False, "approach_min_norm must be <= approach_max_norm"
        if manual_rotate_norm_min > manual_rotate_norm_max:
            return False, "manual_rotate_norm_min must be <= manual_rotate_norm_max"

        with self._control_lock:
            self._floor_profile_name = "custom"
            self._approach_min_norm = approach_min_norm
            self._approach_max_norm = approach_max_norm
            self._move_to_measure_speed_norm = move_to_measure_speed_norm
            self._preplant_adjust_slow_norm = preplant_adjust_slow_norm
            self._manual_drive_distance_norm = manual_drive_distance_norm
            self._manual_rotate_norm_min = manual_rotate_norm_min
            self._manual_rotate_norm_max = manual_rotate_norm_max
            self.status_text = (
                "Custom drive tuning applied "
                f"(approach {self._approach_min_norm:.2f}-{self._approach_max_norm:.2f}, "
                f"measure {self._move_to_measure_speed_norm:.2f}, "
                f"rotate {self._manual_rotate_norm_min:.2f}-{self._manual_rotate_norm_max:.2f})"
            )
        return True, self.status_text

    def get_runtime_tuning(self) -> dict:
        return {
            "floor_profile": self._floor_profile_name,
            "approach_min_norm": float(self._approach_min_norm),
            "approach_max_norm": float(self._approach_max_norm),
            "move_to_measure_speed_norm": float(self._move_to_measure_speed_norm),
            "preplant_adjust_slow_norm": float(self._preplant_adjust_slow_norm),
            "manual_drive_distance_norm": float(self._manual_drive_distance_norm),
            "manual_rotate_norm_min": float(self._manual_rotate_norm_min),
            "manual_rotate_norm_max": float(self._manual_rotate_norm_max),
        }

    def run_planting_process(self) -> tuple[bool, str]:
        """
        Trigger planting sequence from web.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before manual planting"
        # Clear stale stop requests before starting manual process.
        self._stop_event.clear()

        swapped, detail = self._is_serial_mapping_swapped()
        if swapped:
            msg = (
                "serial mapping swapped: set DRIVE_ESP32_PORT/ACT_ESP32_PORT correctly "
                f"({detail})"
            )
            self.status_text = msg
            return False, msg

        with self._control_lock:
            if self._planting_busy:
                return False, "planting process already running"
            self._planting_busy = True

        try:
            self.drive.stop()
            self.status_text = "Manual planting process running"
            ok = self.planting.run_once()
            if ok:
                self.status_text = "Manual planting process completed"
                return True, "planting completed"
            reason = self.planting.last_error or self.actuator.last_message or "unknown"
            self.status_text = f"Manual planting process failed: {reason}"
            return False, self.status_text
        finally:
            with self._control_lock:
                self._planting_busy = False

    def run_rear_measurement_process(self) -> tuple[bool, str]:
        """
        Trigger one rear-size measurement from web.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before manual measurement"

        with self._control_lock:
            if self._measuring_busy:
                return False, "measurement process already running"
            self._measuring_busy = True

        try:
            self._stop_event.clear()
            self.drive.stop()
            self.state = MissionState.MEASURING_REAR_SIZE
            self.status_text = "Manual rear-size measurement running"
            ok = self._measure_rear_size_once()
            if ok:
                self.state = MissionState.IDLE
                return True, "rear-size measurement completed"
            self.state = MissionState.ERROR
            return False, self.status_text
        finally:
            with self._control_lock:
                self._measuring_busy = False

    def calibrate_rear_measurement(self, actual_cm: float | int | str | None) -> tuple[bool, str]:
        """
        Manual rear-size calibration from web using a known real object size.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before rear calibration"
        try:
            actual_cm_f = float(actual_cm)
        except (TypeError, ValueError):
            return False, f"invalid actual size cm: {actual_cm}"
        if actual_cm_f <= 0.0:
            return False, "actual size cm must be > 0"

        with self._control_lock:
            if self._measuring_busy:
                return False, "measurement process already running"
            self._measuring_busy = True

        try:
            self._stop_event.clear()
            self.drive.stop()
            self.state = MissionState.MEASURING_REAR_SIZE
            self.status_text = f"Manual rear calibration running ({actual_cm_f:.2f} cm)"
            ok = self._calibrate_rear_size_once(actual_cm_f)
            if ok:
                self.state = MissionState.IDLE
                return True, "rear-size calibration completed"
            self.state = MissionState.ERROR
            return False, self.status_text
        finally:
            with self._control_lock:
                self._measuring_busy = False

    def save_rear_scale_to_env(self) -> tuple[bool, str]:
        cm_per_px = float(self._cm_per_px)
        if cm_per_px <= 0.0:
            return False, "rear scale is not set"

        env_path = Path(__file__).resolve().parents[2] / ".env"
        try:
            text = env_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            text = ""
        except OSError as exc:
            return False, f"failed to read .env: {exc}"

        key = "REAR_CM_PER_PX_DEFAULT"
        new_line = f"{key}={cm_per_px:.5f}"
        lines = text.splitlines()
        replaced = False
        for idx, line in enumerate(lines):
            if line.startswith(f"{key}="):
                lines[idx] = new_line
                replaced = True
                break
        if not replaced:
            lines.append(new_line)

        out = "\n".join(lines)
        if text.endswith("\n") or not lines:
            out += "\n"
        try:
            env_path.write_text(out, encoding="utf-8")
        except OSError as exc:
            return False, f"failed to write .env: {exc}"

        self.status_text = f"Saved rear scale {cm_per_px:.5f} cm/px to .env"
        return True, self.status_text

    def _measure_apriltag_yaw(self, *, timeout_sec: float, min_samples: int) -> dict | None:
        deadline = time.monotonic() + max(0.5, float(timeout_sec))
        yaw_samples: list[float] = []
        pose_yaw_samples: list[float] = []
        practical_yaw_samples: list[float] = []
        bearing_samples: list[float] = []
        dist_samples: list[float] = []
        tag_id: int | None = None
        last_anno: np.ndarray | None = None

        while time.monotonic() < deadline and not self._stop_event.is_set():
            frame = self.cameras.get_raw_front_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            info, anno, metrics = self.apriltag.detect_and_annotate_with_metrics(
                frame,
                prefer_largest_area=True,
            )
            last_anno = anno
            self._set_front_frame(anno)
            if info is not None and metrics is not None:
                tag_id = int(info.tag_id)
                pose_yaw = metrics.get("tag_pose_yaw_deg")
                practical_yaw = metrics.get("tag_robot_yaw_practical_deg")
                yaw = practical_yaw if practical_yaw is not None else pose_yaw
                bearing = metrics.get("tag_bearing_x_deg")
                dist = metrics.get("tag_approx_dist_cm")
                if pose_yaw is not None:
                    pose_yaw_samples.append(float(pose_yaw))
                if practical_yaw is not None:
                    practical_yaw_samples.append(float(practical_yaw))
                if yaw is not None:
                    yaw_samples.append(float(yaw))
                if bearing is not None:
                    bearing_samples.append(float(bearing))
                if dist is not None:
                    dist_samples.append(float(dist))
                if len(yaw_samples) >= max(1, int(min_samples)):
                    break
            time.sleep(config.RUNTIME.vision_loop_sec)

        if last_anno is not None:
            self._set_front_frame(last_anno)

        if not yaw_samples:
            self._apriltag_yaw_measurement = {
                "error": "no apriltag yaw measurement",
                "ts": time.time(),
            }
            return None

        yaw_deg = float(np.median(np.array(yaw_samples, dtype=np.float32)))
        pose_yaw_deg = (
            float(np.median(np.array(pose_yaw_samples, dtype=np.float32)))
            if pose_yaw_samples else None
        )
        practical_yaw_deg = (
            float(np.median(np.array(practical_yaw_samples, dtype=np.float32)))
            if practical_yaw_samples else None
        )
        bearing_deg = (
            float(np.median(np.array(bearing_samples, dtype=np.float32)))
            if bearing_samples else None
        )
        dist_cm = (
            float(np.median(np.array(dist_samples, dtype=np.float32)))
            if dist_samples else None
        )
        robot_forward_samples = []
        robot_lateral_samples = []
        latest_practical_yaw_deg = None
        frame = self.cameras.get_raw_front_frame()
        if frame is not None:
            _, _, m2 = self.apriltag.detect_and_annotate_with_metrics(
                frame,
                prefer_largest_area=True,
            )
            if m2 is not None:
                if m2.get("tag_robot_forward_cm") is not None:
                    robot_forward_samples.append(float(m2["tag_robot_forward_cm"]))
                if m2.get("tag_robot_lateral_cm") is not None:
                    robot_lateral_samples.append(float(m2["tag_robot_lateral_cm"]))
                if m2.get("tag_robot_yaw_practical_deg") is not None:
                    latest_practical_yaw_deg = float(m2["tag_robot_yaw_practical_deg"])
        self._apriltag_yaw_measurement = {
            "tag_id": tag_id,
            "yaw_deg": float(round(yaw_deg, 3)),
            "yaw_source": (
                "practical"
                if practical_yaw_deg is not None
                else "pose"
            ),
            "selection_mode": "largest_area",
            "yaw_pose_deg": (
                None if pose_yaw_deg is None else float(round(pose_yaw_deg, 3))
            ),
            "yaw_practical_deg": (
                None if practical_yaw_deg is None else float(round(practical_yaw_deg, 3))
            ),
            "bearing_x_deg": None if bearing_deg is None else float(round(bearing_deg, 3)),
            "approx_dist_cm": None if dist_cm is None else float(round(dist_cm, 3)),
            "robot_forward_cm": (
                None if not robot_forward_samples
                else float(round(float(np.median(np.array(robot_forward_samples, dtype=np.float32))), 3))
            ),
            "robot_lateral_cm": (
                None if not robot_lateral_samples
                else float(round(float(np.median(np.array(robot_lateral_samples, dtype=np.float32))), 3))
            ),
            "latest_yaw_practical_deg": (
                None if latest_practical_yaw_deg is None else float(round(latest_practical_yaw_deg, 3))
            ),
            "samples": int(len(yaw_samples)),
            "ts": time.time(),
        }
        return self._apriltag_yaw_measurement

    def run_apriltag_yaw_measurement(self) -> tuple[bool, str]:
        """
        Trigger one front-camera AprilTag yaw measurement from web.
        Allowed only when auto mission is not running.
        """
        if self.running:
            return False, "mission is running, stop mission before AprilTag yaw measurement"

        with self._control_lock:
            self._stop_event.clear()
            self.drive.stop()
            measurement = self._measure_apriltag_yaw(timeout_sec=8.0, min_samples=2)
            if measurement is None:
                self.status_text = "AprilTag yaw measurement failed: no usable tag"
                return False, self.status_text

            yaw_deg = float(measurement["yaw_deg"])
            bearing_deg = measurement.get("bearing_x_deg")
            self.status_text = (
                f"AprilTag yaw measured: yaw={yaw_deg:.1f}deg bearing={float(bearing_deg):.1f}deg"
                if bearing_deg is not None
                else f"AprilTag yaw measured: yaw={yaw_deg:.1f}deg"
            )
            return True, self.status_text

    def toggle_camera_swap(self) -> tuple[bool, str]:
        if self.running:
            self.status_text = "Camera swap disabled while mission running"
            return False, self.status_text
        swapped = self.cameras.toggle_swap()
        self.status_text = f"Camera swap {'ON' if swapped else 'OFF'}"
        return True, self.status_text

    def get_camera_info(self) -> dict:
        return self.cameras.get_camera_info()

    # ---------- image API ----------
    def get_front_frame(self) -> np.ndarray | None:
        with self._front_frame_lock:
            if self._front_annotated is not None:
                return self._front_annotated.copy()

        # Fallback for preview mode (before mission starts)
        raw = self.cameras.get_front_frame()
        if raw is None:
            return None
        if self.detector.enabled:
            now = time.monotonic()
            with self._front_frame_lock:
                interval = max(0.05, float(config.VISION.idle_preview_interval_sec))
                if (
                    self._front_idle_preview is not None
                    and (now - self._front_idle_preview_ts) < interval
                ):
                    return self._front_idle_preview.copy()

            _, annotated = self.detector.detect_best(raw, config.VISION.target_classes)
            cv2.putText(
                annotated,
                "Front preview (mission idle)",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (50, 220, 50),
                2,
            )
            with self._front_frame_lock:
                self._front_idle_preview = annotated.copy()
                self._front_idle_preview_ts = now
            return annotated

        frame = raw.copy()
        cv2.putText(frame, "Front preview", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 220, 50), 2)
        if not self.detector.enabled:
            cv2.putText(
                frame,
                f"YOLO disabled: {self.detector.disabled_reason}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )
        return frame

    def get_rear_frame(self) -> np.ndarray | None:
        with self._rear_frame_lock:
            if self._rear_annotated is not None:
                return self._rear_annotated.copy()

        # Fallback for preview mode (before mission starts)
        raw = self.cameras.get_rear_frame()
        if raw is None:
            return None
        if self.detector.enabled:
            now = time.monotonic()
            with self._rear_frame_lock:
                interval = max(0.05, float(config.VISION.idle_preview_interval_sec))
                if (
                    self._rear_idle_preview is not None
                    and (now - self._rear_idle_preview_ts) < interval
                ):
                    return self._rear_idle_preview.copy()

            _, annotated = self._annotate_rear_frame(raw)
            cv2.putText(
                annotated,
                "Rear preview (mission idle)",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (50, 220, 50),
                2,
            )
            with self._rear_frame_lock:
                self._rear_idle_preview = annotated.copy()
                self._rear_idle_preview_ts = now
            return annotated

        frame = raw.copy()
        cv2.putText(frame, "Rear preview", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 220, 50), 2)
        return frame

    # ---------- status API ----------
    def get_snapshot(self) -> MissionSnapshot:
        drive_tel = self.drive.get_telemetry()
        return MissionSnapshot(
            state=self.state.value,
            running=self.running,
            status_text=self.status_text,
            target_class=self.target_class,
            servo_cam_deg=self.actuator.servo_cam_deg,
            april=None if self.april_info is None else asdict(self.april_info),
            drive=asdict(drive_tel),
            imu=self.drive.get_imu_status(),
            pid={
                "kp": config.VISION.kp,
                "ki": config.VISION.ki,
                "kd": config.VISION.kd,
                "history": list(self._pid_history),
            },
            rear_measurement=self._rear_measurement,
            apriltag_yaw_measurement=self._apriltag_yaw_measurement,
            plant_map=list(self._plant_map),
        )

    # ---------- internal ----------
    def _set_front_frame(self, frame: np.ndarray | None) -> None:
        if frame is None:
            return
        with self._front_frame_lock:
            self._front_annotated = frame

    def _set_rear_frame(self, frame: np.ndarray | None) -> None:
        if frame is None:
            return
        with self._rear_frame_lock:
            self._rear_annotated = frame

    def _annotate_rear_frame(self, rear_raw: np.ndarray) -> tuple[Detection | None, np.ndarray]:
        det, rear_anno = self.detector.detect_best(rear_raw, config.VISION.target_classes)
        rear_has_box = det is not None
        cv2.putText(
            rear_anno,
            f"Rear woodenbox: {'YES' if rear_has_box else 'NO'}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if rear_has_box else (0, 0, 255),
            2,
        )
        return det, rear_anno

    def _collect_rear_size_samples(
        self,
        *,
        status_text: str,
        overlay_prefix: str,
        actual_cm: float | None = None,
    ) -> tuple[str, list[float], list[float], list[float], np.ndarray | None]:
        self.drive.stop()
        self._reset_stuck_watch()
        self.status_text = status_text
        deadline = time.monotonic() + max(0.5, float(config.VISION.rear_measure_timeout_sec))
        need_frames = max(1, int(config.VISION.rear_measure_frames))
        target_cls = str(config.VISION.rear_measure_target_class).strip()
        sizes_px: list[float] = []
        center_err_samples: list[float] = []
        selected_conf_samples: list[float] = []
        last_anno = None

        while time.monotonic() < deadline and not self._stop_event.is_set():
            rear_frame = self.cameras.get_raw_rear_frame()
            if rear_frame is None:
                time.sleep(0.05)
                continue
            dets, rear_anno = self.detector.detect_all(
                rear_frame,
                (target_cls,),
                max_per_class=max(1, int(config.VISION.rear_measure_max_per_class)),
                max_total=max(1, int(config.VISION.rear_measure_max_total)),
                use_priority=False,
            )
            det = self._select_rear_measure_detection(dets, rear_frame.shape) if dets else None
            if det is not None:
                size_px = self._bbox_size_px(det)
                sizes_px.append(size_px)
                center_err_norm = self._bbox_center_err_norm(det, rear_frame.shape)
                center_err_samples.append(center_err_norm)
                selected_conf_samples.append(float(det.conf))
                if actual_cm is None:
                    line = (
                        f"{overlay_prefix} {target_cls}: {size_px * float(self._cm_per_px):.2f} cm "
                        f"({len(dets)} det, cen={center_err_norm:+.2f})"
                    )
                else:
                    line = (
                        f"{overlay_prefix} {target_cls}: actual {actual_cm:.2f} cm "
                        f"px={size_px:.1f} ({len(dets)} det, cen={center_err_norm:+.2f})"
                    )
                cv2.rectangle(rear_anno, (det.x1, det.y1), (det.x2, det.y2), (0, 128, 255), 3)
                cv2.putText(
                    rear_anno,
                    line,
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
            else:
                cv2.putText(
                    rear_anno,
                    f"{overlay_prefix} {target_cls}: no detection",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
            last_anno = rear_anno
            self._set_rear_frame(rear_anno)
            if len(sizes_px) >= need_frames:
                break
            time.sleep(config.RUNTIME.vision_loop_sec)

        if last_anno is not None:
            self._set_rear_frame(last_anno)

        return target_cls, sizes_px, center_err_samples, selected_conf_samples, last_anno

    def _measure_rear_size_once(self, *, record_point: bool = True, phase: str = "measure") -> bool:
        target_cls, sizes_px, center_err_samples, selected_conf_samples, _ = self._collect_rear_size_samples(
            status_text="Measuring cabbage size (rear)" if record_point else "Adjusting cabbage size (rear)",
            overlay_prefix="Measure",
        )

        if record_point:
            self._measure_idx += 1
            idx = self._measure_idx
        else:
            idx = self._measure_idx

        if sizes_px:
            size_px_used = self._filter_rear_measure_samples(sizes_px)
            size_px_med = float(np.median(np.array(size_px_used, dtype=np.float32)))
            size_cm = size_px_med * float(self._cm_per_px)
            threshold_cm = float(config.VISION.rear_measure_size_threshold_cm)
            reached = size_cm >= threshold_cm
            point = {
                "idx": idx,
                "distance_cm": float(self.drive.get_telemetry().distance_cm),
                "target_class": target_cls,
                "size_cm": float(round(size_cm, 3)),
                "size_threshold_cm": float(threshold_cm),
                "reached": bool(reached),
                "cm_per_px": float(round(self._cm_per_px, 6)),
                "cm_per_px_source": self._cm_per_px_source,
                "samples": int(len(size_px_used)),
                "samples_raw": int(len(sizes_px)),
                "center_err_norm": (
                    None if not center_err_samples
                    else float(round(float(np.median(np.array(center_err_samples, dtype=np.float32))), 4))
                ),
                "selected_conf": (
                    None if not selected_conf_samples
                    else float(round(float(np.median(np.array(selected_conf_samples, dtype=np.float32))), 4))
                ),
                "phase": phase,
                "ts": time.time(),
            }
            self._rear_measurement = point
            if record_point:
                self._plant_map.append(point)
                self.status_text = (
                    f"Rear measure #{point['idx']}: {point['size_cm']:.2f} cm "
                    f"({'OK' if reached else 'LOW'})"
                )
            else:
                self.status_text = (
                    f"Rear adjust ({phase}): {point['size_cm']:.2f} cm "
                    f"({'OK' if reached else 'LOW'})"
                )
            return True

        self._rear_measurement = {
            "idx": idx,
            "target_class": target_cls,
            "size_cm": None,
            "size_threshold_cm": float(config.VISION.rear_measure_size_threshold_cm),
            "reached": False,
            "error": "no detection",
            "phase": phase,
            "ts": time.time(),
        }
        if record_point:
            self.status_text = f"Rear measure failed: no {target_cls} detection"
        else:
            self.status_text = f"Rear adjust failed: no {target_cls} detection"
        return False

    def _calibrate_rear_size_once(self, actual_cm: float) -> bool:
        target_cls, sizes_px, center_err_samples, selected_conf_samples, _ = self._collect_rear_size_samples(
            status_text=f"Calibrating rear size using actual {actual_cm:.2f} cm",
            overlay_prefix="Calibrate",
            actual_cm=float(actual_cm),
        )

        if not sizes_px:
            self._rear_measurement = {
                "target_class": target_cls,
                "actual_cm": float(round(actual_cm, 3)),
                "cm_per_px": float(round(self._cm_per_px, 6)),
                "cm_per_px_source": self._cm_per_px_source,
                "error": "no detection",
                "phase": "manual_calibration",
                "ts": time.time(),
            }
            self.status_text = f"Rear calibration failed: no {target_cls} detection"
            return False

        size_px_used = self._filter_rear_measure_samples(sizes_px)
        size_px_med = float(np.median(np.array(size_px_used, dtype=np.float32)))
        if size_px_med <= 0.0:
            self.status_text = "Rear calibration failed: invalid size_px"
            return False

        new_cm_per_px = float(actual_cm) / size_px_med
        self._manual_cm_per_px = new_cm_per_px
        self._manual_cm_per_px_source = f"manual({float(actual_cm):.2f}cm)"
        self._cm_per_px = new_cm_per_px
        self._cm_per_px_source = self._manual_cm_per_px_source
        self._rear_measurement = {
            "target_class": target_cls,
            "actual_cm": float(round(actual_cm, 3)),
            "size_px": float(round(size_px_med, 3)),
            "cm_per_px": float(round(new_cm_per_px, 6)),
            "cm_per_px_source": self._cm_per_px_source,
            "samples": int(len(size_px_used)),
            "samples_raw": int(len(sizes_px)),
            "center_err_norm": (
                None if not center_err_samples
                else float(round(float(np.median(np.array(center_err_samples, dtype=np.float32))), 4))
            ),
            "selected_conf": (
                None if not selected_conf_samples
                else float(round(float(np.median(np.array(selected_conf_samples, dtype=np.float32))), 4))
            ),
            "phase": "manual_calibration",
            "ts": time.time(),
        }
        self.status_text = (
            f"Rear calibration done: actual {actual_cm:.2f} cm -> "
            f"{self._cm_per_px:.5f} cm/px ({self._cm_per_px_source})"
        )
        return True

    def _configure_preplant_rear_adjust(self, dist0_cm: float, move_increment_cm: float, first_leg: bool) -> None:
        """
        Pre-plant rear adjust is disabled; rear-size scale is calibrated manually
        from the dashboard instead of interrupting motion during the mission.
        """
        self._preplant_adjust_pending = False
        self._preplant_adjust_search_start_cm = float(dist0_cm)
        self._preplant_adjust_fallback_cm = float(dist0_cm)
        self._preplant_adjust_goal_cm = float(dist0_cm)

    def _update_no_box_streak(self, front_has_box: bool, rear_has_box: bool) -> bool:
        if front_has_box or rear_has_box:
            self._no_box_streak = 0
            return False
        self._no_box_streak += 1
        return self._no_box_streak >= max(1, int(config.RUNTIME.no_box_stop_frames))

    def _compute_move_increment_cm(self) -> float:
        offset_cm = float(config.MOTION.front_to_plant_point_cm)
        if self.april_info is not None and self._use_apriltag_distance_once:
            self._use_apriltag_distance_once = False
            return float(self.april_info.planting_distance_cm) + offset_cm
        if self.april_info is not None:
            return float(self.april_info.planting_distance_cm)
        return offset_cm

    def _normalize_bbox_to_trigger_line(self) -> bool:
        """
        After entry alignment and before AprilTag scanning, normalize robot range so that
        bbox bottom is near trigger line:
        - bottom < line  -> move forward
        - bottom > line  -> move backward
        """
        trigger = int(config.VISION.bbox_trigger_line_y)
        tol = max(1, int(config.VISION.pre_april_bbox_tolerance_px))
        adjust_cmd = int(max(1, config.VISION.max_cmd * float(config.VISION.pre_april_adjust_norm)))
        deadline = time.monotonic() + max(0.5, float(config.VISION.pre_april_adjust_timeout_sec))

        while not self._stop_event.is_set() and time.monotonic() < deadline:
            front_raw = self.cameras.get_raw_front_frame()
            rear_raw = self.cameras.get_raw_rear_frame()
            if front_raw is None or rear_raw is None:
                time.sleep(0.05)
                continue

            det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
            det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
            self._set_front_frame(front_anno)
            self._set_rear_frame(rear_anno)

            active_source, active_det, _ = self._select_active_target(det_front, det_rear, front_raw, rear_raw)
            self._approach_source = active_source
            if active_det is None:
                self.drive.stop()
                self.status_text = "Pre-April adjust failed: target lost"
                return False

            err_px = int(active_det.bottom) - trigger
            if abs(err_px) <= tol:
                self.drive.stop()
                self.status_text = (
                    f"Pre-April adjust done ({active_source}) bottom={int(active_det.bottom)} "
                    f"target={trigger} tol={tol}"
                )
                return True

            # User-required direction policy (swapped):
            # bottom < line => forward, bottom > line => backward
            if int(active_det.bottom) < trigger:
                self.drive.send_velocity(adjust_cmd, adjust_cmd)
                direction = "forward"
            else:
                self.drive.send_velocity(-adjust_cmd, -adjust_cmd)
                direction = "backward"

            self.status_text = (
                f"Pre-April adjust ({active_source}) bottom={int(active_det.bottom)} "
                f"target={trigger} err={err_px:+d}px -> {direction}"
            )
            time.sleep(config.RUNTIME.vision_loop_sec)

        self.drive.stop()
        self.status_text = "Pre-April adjust timeout"
        return False

    def _push_pid_sample(self, error: float, turn: float, left_cmd: int, right_cmd: int, source: str) -> None:
        self._pid_seq += 1
        self._pid_history.append(
            {
                "seq": self._pid_seq,
                "error": float(error),
                "turn": float(turn),
                "left_cmd": int(left_cmd),
                "right_cmd": int(right_cmd),
                "source": source,
            }
        )

    @staticmethod
    def _bbox_size_px(det: Detection) -> float:
        w = max(1, det.x2 - det.x1)
        h = max(1, det.y2 - det.y1)
        return (float(w) + float(h)) * 0.5

    @staticmethod
    def _bbox_center_err_norm(det: Detection, frame_shape: tuple[int, int, int]) -> float:
        w = max(1.0, float(frame_shape[1]))
        return (float(det.cx) - (w * 0.5)) / (w * 0.5)

    def _select_rear_measure_detection(
        self,
        dets: list[Detection],
        frame_shape: tuple[int, int, int],
    ) -> Detection | None:
        if not dets:
            return None
        return min(
            dets,
            key=lambda det: (
                abs(self._bbox_center_err_norm(det, frame_shape)),
                -self._bbox_size_px(det),
                -float(det.conf),
            ),
        )

    @staticmethod
    def _filter_rear_measure_samples(samples_px: list[float]) -> list[float]:
        if len(samples_px) < 3:
            return list(samples_px)
        arr = np.array(samples_px, dtype=np.float32)
        med = float(np.median(arr))
        tol = max(3.0, med * 0.22)
        filtered = [float(v) for v in arr.tolist() if abs(float(v) - med) <= tol]
        return filtered if filtered else list(samples_px)

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _sleep_interruptible(self, sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(sec))
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return True

    def _select_active_target(
        self,
        det_front: Detection | None,
        det_rear: Detection | None,
        front_raw: np.ndarray,
        rear_raw: np.ndarray,
    ) -> tuple[str, Detection | None, np.ndarray]:
        active_source = "front"
        active_det = det_front
        active_frame = front_raw
        if det_front is None and det_rear is not None:
            active_source = "rear"
            active_det = det_rear
            active_frame = rear_raw
        elif det_front is not None:
            active_source = "front"
            active_det = det_front
            active_frame = front_raw
        return active_source, active_det, active_frame

    def _compute_lateral_heading_error(
        self,
        det_front: Detection | None,
        det_rear: Detection | None,
        active_det: Detection,
        frame_w: int,
    ) -> tuple[float, float]:
        center_x = frame_w / 2.0
        lateral_err = (active_det.cx - center_x) / center_x
        if det_front is not None and det_rear is not None:
            e_front = (det_front.cx - center_x) / center_x
            e_rear = (det_rear.cx - center_x) / center_x
            heading_err = e_front - e_rear
        else:
            heading_err = lateral_err
        return lateral_err, heading_err

    def _compute_approach_base_norm(self, det: Detection, lateral_err: float) -> float:
        trigger = float(config.VISION.bbox_trigger_line_y)
        bottom = float(det.bottom)
        near = self._clamp((bottom - trigger * 0.6) / max(1.0, trigger * 0.4), 0.0, 1.0)
        err_abs = abs(float(lateral_err))

        base_hi = self._get_approach_max_norm()
        base_lo = self._get_approach_min_norm()
        slow_err = self._clamp(1.0 - float(config.VISION.approach_err_slow_gain) * err_abs, 0.25, 1.0)
        slow_near = self._clamp(1.0 - float(config.VISION.approach_near_slow_gain) * near, 0.25, 1.0)
        dyn = base_hi * slow_err * slow_near
        return self._clamp(dyn, base_lo, base_hi)

    def _reset_stuck_watch(self) -> None:
        self._stuck_ref_ts = 0.0
        self._stuck_ref_dist_cm = 0.0
        self._stuck_retries = 0

    def _run_stuck_recovery(self, error_hint: float) -> bool:
        max_retry = max(1, int(config.VISION.stuck_max_retries))
        if self._stuck_retries >= max_retry:
            self.drive.stop()
            self.state = MissionState.ERROR
            self.status_text = "Stuck recovery exhausted"
            return False

        self._stuck_retries += 1
        self.status_text = f"Stuck recovery {self._stuck_retries}/{max_retry}"
        self.drive.stop()
        if not self._sleep_interruptible(0.12):
            return False

        max_cmd = int(config.VISION.max_cmd)
        back_cmd = int(max_cmd * float(config.VISION.stuck_backoff_norm))
        self.drive.send_velocity(-back_cmd, -back_cmd)
        if not self._sleep_interruptible(float(config.VISION.stuck_backoff_sec)):
            return False

        # Alternate turn direction while biasing by current tracking error sign.
        turn_dir = -1 if error_hint >= 0 else 1
        if self._stuck_retries % 2 == 0:
            turn_dir *= -1
        turn_cmd = int(max_cmd * float(config.VISION.stuck_turn_norm))
        self.drive.send_velocity(turn_dir * turn_cmd, -turn_dir * turn_cmd)
        if not self._sleep_interruptible(float(config.VISION.stuck_turn_sec)):
            return False

        self.drive.stop()
        self.pid.reset()
        self._stuck_ref_ts = 0.0
        return True

    def _check_stuck_and_recover(self, command_active: bool, error_hint: float) -> bool:
        if not command_active:
            self._stuck_ref_ts = 0.0
            return True

        now = time.monotonic()
        dist = float(self.drive.get_telemetry().distance_cm)
        window = max(0.3, float(config.VISION.stuck_window_sec))
        min_progress = max(0.05, float(config.VISION.stuck_min_progress_cm))

        if self._stuck_ref_ts <= 0.0:
            self._stuck_ref_ts = now
            self._stuck_ref_dist_cm = dist
            return True

        if now - self._stuck_ref_ts < window:
            return True

        progress = abs(dist - self._stuck_ref_dist_cm)
        self._stuck_ref_ts = now
        self._stuck_ref_dist_cm = dist

        if progress >= min_progress:
            return True

        logger.warning("Drive appears stuck (progress=%.3fcm < %.3fcm); running recovery", progress, min_progress)
        return self._run_stuck_recovery(error_hint)

    def _mission_loop(self) -> None:
        try:
            self._run_fsm()
        except Exception as exc:
            logger.exception("Mission loop crashed: %s", exc)
            self.state = MissionState.ERROR
            self.status_text = str(exc)
            self.drive.stop()
        finally:
            self.running = False

    def _run_fsm(self) -> None:
        self.state = MissionState.INITIALIZING
        self.status_text = "Initializing actuators (home)"

        swapped, detail = self._is_serial_mapping_swapped()
        if swapped:
            raise RuntimeError(
                "serial mapping swapped: set DRIVE_ESP32_PORT/ACT_ESP32_PORT correctly "
                f"({detail})"
            )

        init_seq = self.actuator.get_rx_seq()
        ok, msg = self.actuator.initialize_home()
        if not ok:
            raise RuntimeError(f"initialize_home failed: {msg}")
        home_deadline = time.monotonic() + float(config.ACTUATOR.plant_limit_timeout_sec)
        last_resend = time.monotonic()
        got_home = False
        while time.monotonic() < home_deadline and not self._stop_event.is_set():
            if self.actuator.wait_for_event(("ACT_EVENT PLANT_HIT_TOP",), 0.3, after_seq=init_seq):
                got_home = True
                break
            seq_before_status = self.actuator.get_rx_seq()
            ok, _ = self.actuator.request_status()
            if ok:
                status_line = self.actuator.wait_for_status_line(0.35, after_seq=seq_before_status)
                if self.actuator.top_limit_from_status(status_line) is True:
                    got_home = True
                    break
            now = time.monotonic()
            if now - last_resend >= 1.0:
                # Keep up-limit command alive in case first command was dropped.
                self.actuator.plant_up_until_top()
                last_resend = now
            time.sleep(0.03)
        if not got_home:
            raise RuntimeError(f"initialize_home timeout waiting PLANT_HIT_TOP (last={self.actuator.last_message})")

        self.status_text = "Initializing actuators (servo start)"
        ok, msg = self.actuator.set_servo_cam(config.ACTUATOR.servo_cam_start_deg)
        if not ok:
            raise RuntimeError(f"set_servo_cam failed: {msg}")
        ok, msg = self.actuator.set_servo_plant(config.ACTUATOR.servo_plant_start_deg)
        if not ok:
            raise RuntimeError(f"set_servo_plant failed: {msg}")

        self.drive.stop()
        ok, msg = self.drive.reset_odometry()
        if not ok:
            raise RuntimeError(f"reset_odometry failed: {msg}")
        self.pid.reset()
        self.april_info = None
        self._approach_source = "front"
        self._no_box_streak = 0
        self._use_apriltag_distance_once = False
        self._servo_cam_target_deg = float(config.ACTUATOR.servo_cam_start_deg)
        self._pid_history.clear()
        self._pid_seq = 0
        self._rear_measurement = None
        self._apriltag_yaw_measurement = None
        self._plant_map = []
        self._measure_idx = 0
        if self._manual_cm_per_px is not None:
            self._cm_per_px = float(self._manual_cm_per_px)
            self._cm_per_px_source = str(self._manual_cm_per_px_source or "manual")
        else:
            self._cm_per_px = float(config.VISION.rear_cm_per_px_default)
            self._cm_per_px_source = "default"
        self._align_hold_count = 0
        self._reset_stuck_watch()
        self._last_track_error = 0.0
        self._plants_done_current_pair = 0
        self._measure_interval_cm = 0.0
        self._preapproach_tag_attempted = False

        self.state = MissionState.READING_APRILTAG
        self.status_text = "Start mission, startup triangle scan"
        startup_triangle_ok = self._run_apriltag_triangle_align()
        startup_triangle_status = self.status_text
        if startup_triangle_ok:
            logger.info("Startup triangle align completed before search")
            self.status_text = "Startup triangle complete, searching target"
        else:
            logger.warning("Startup triangle align failed/skipped: %s", startup_triangle_status)
            self.status_text = f"Startup triangle skipped, searching target ({startup_triangle_status})"
        self.state = MissionState.SEARCHING_TARGET
        search_start = time.monotonic()

        while not self._stop_event.is_set():
            # Mission control must always use physical front/rear cameras.
            # Dashboard swap is for viewing only and must not alter autonomy.
            front_raw = self.cameras.get_raw_front_frame()
            rear_raw = self.cameras.get_raw_rear_frame()

            if front_raw is None:
                self.status_text = "Waiting front camera"
                time.sleep(config.RUNTIME.status_loop_sec)
                continue
            if rear_raw is None:
                self.status_text = "Waiting rear camera"
                time.sleep(config.RUNTIME.status_loop_sec)
                continue

            if self.state == MissionState.SEARCHING_TARGET:
                det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
                det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                front_has_box = det_front is not None
                rear_has_box = det_rear is not None

                self._set_front_frame(front_anno)
                self._set_rear_frame(rear_anno)

                if det_front is not None:
                    self._approach_source = "front"
                    self.target_class = det_front.class_name
                    if self.april_info is None:
                        self.state = MissionState.READING_APRILTAG
                        self.status_text = f"Target locked (front), scanning AprilTag first: {det_front.class_name}"
                        logger.info("Scanning AprilTag first after front target lock: %s", det_front.class_name)
                    else:
                        self.state = MissionState.APPROACHING_TARGET
                        self.status_text = f"Target locked (front): {det_front.class_name}"
                    self.pid.reset()
                    self._align_hold_count = 0
                    self._reset_stuck_watch()
                    continue

                if det_rear is not None:
                    self._approach_source = "rear"
                    self.target_class = det_rear.class_name
                    if self.april_info is None:
                        self.state = MissionState.READING_APRILTAG
                        self.status_text = f"Target locked (rear), scanning AprilTag first: {det_rear.class_name}"
                        logger.info("Scanning AprilTag first after rear target lock: %s", det_rear.class_name)
                    else:
                        self.state = MissionState.APPROACHING_TARGET
                        self.status_text = f"Target locked (rear): {det_rear.class_name}"
                    self.pid.reset()
                    self._align_hold_count = 0
                    self._reset_stuck_watch()
                    continue

                if self._update_no_box_streak(front_has_box, rear_has_box):
                    self.drive.stop()
                    self.state = MissionState.FINISHED
                    self.status_text = "No woodenbox on both cameras -> finish"
                    continue

                self.drive.stop()
                self._reset_stuck_watch()
                self.status_text = (
                    f"Searching target (no-box {self._no_box_streak}/{config.RUNTIME.no_box_stop_frames})"
                )
                if time.monotonic() - search_start > config.RUNTIME.search_timeout_sec:
                    self.state = MissionState.FINISHED
                    self.status_text = "Search timeout"
                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.APPROACHING_TARGET:
                det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
                det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                front_has_box = det_front is not None
                rear_has_box = det_rear is not None
                self._set_front_frame(front_anno)
                self._set_rear_frame(rear_anno)

                active_source, active_det, active_frame = self._select_active_target(
                    det_front, det_rear, front_raw, rear_raw
                )
                self._approach_source = active_source

                if active_det is None:
                    self.drive.stop()
                    self._reset_stuck_watch()
                    if self._update_no_box_streak(front_has_box, rear_has_box):
                        self.state = MissionState.FINISHED
                        self.status_text = "No woodenbox on both cameras -> finish"
                        continue
                    self.status_text = (
                        f"Target lost, back to search (no-box {self._no_box_streak}/{config.RUNTIME.no_box_stop_frames})"
                    )
                    self.state = MissionState.SEARCHING_TARGET
                    search_start = time.monotonic()
                    continue

                # Any valid target in approach path resets no-box streak.
                self._no_box_streak = 0

                h, w, _ = active_frame.shape
                lateral_err, heading_err = self._compute_lateral_heading_error(det_front, det_rear, active_det, w)
                error = lateral_err
                self._last_track_error = error
                turn = self.pid.update(error + (0.35 * heading_err))

                base_norm = self._compute_approach_base_norm(active_det, lateral_err)
                left_norm = self._clamp(base_norm + turn, -1.0, 1.0)
                right_norm = self._clamp(base_norm - turn, -1.0, 1.0)
                left_cmd = int(left_norm * config.VISION.max_cmd)
                right_cmd = int(right_norm * config.VISION.max_cmd)

                self.drive.send_velocity(left_cmd, right_cmd)
                self._push_pid_sample(error, turn, left_cmd, right_cmd, active_source)
                self.status_text = (
                    f"Approaching ({active_source}) {active_det.class_name}: L={left_cmd}, R={right_cmd} "
                    f"e={lateral_err:.3f} h={heading_err:.3f}"
                )
                if not self._check_stuck_and_recover(
                    command_active=(abs(left_cmd) + abs(right_cmd)) > 80,
                    error_hint=self._last_track_error,
                ):
                    continue

                if active_det.bottom >= config.VISION.bbox_trigger_line_y:
                    self._update_servo_cam_tracking(active_det.bottom)

                    if self.actuator.servo_cam_deg <= config.ACTUATOR.servo_cam_end_deg:
                        self.drive.stop()
                        self._reset_stuck_watch()
                        self._align_hold_count = 0
                        self.state = MissionState.ALIGNING_ENTRY
                        self.status_text = "BBox reached trigger line, aligning entry"
                        continue

                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.ALIGNING_ENTRY:
                det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
                det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                front_has_box = det_front is not None
                self._set_front_frame(front_anno)
                self._set_rear_frame(rear_anno)

                # ALIGNING_ENTRY uses front camera only by request.
                active_source = "front"
                active_det = det_front
                active_frame = front_raw
                self._approach_source = "front"

                if active_det is None:
                    self.drive.stop()
                    self._reset_stuck_watch()
                    self._align_hold_count = 0
                    if self._update_no_box_streak(front_has_box, False):
                        self.state = MissionState.FINISHED
                        self.status_text = "No woodenbox on front camera -> finish"
                        continue
                    self.state = MissionState.SEARCHING_TARGET
                    self.status_text = "Align lost front target, back to search"
                    search_start = time.monotonic()
                    continue

                self._no_box_streak = 0
                h, w, _ = active_frame.shape
                lateral_err, heading_err = self._compute_lateral_heading_error(det_front, None, active_det, w)
                self._last_track_error = lateral_err

                lateral_tol = float(config.VISION.entry_align_lateral_tol_px) / max(1.0, (w / 2.0))
                heading_tol = float(config.VISION.entry_align_heading_tol_norm)
                aligned = abs(lateral_err) <= lateral_tol and abs(heading_err) <= heading_tol
                if aligned:
                    self._align_hold_count += 1
                else:
                    self._align_hold_count = 0

                turn_raw = (
                    float(config.VISION.entry_align_turn_gain) * lateral_err
                    + float(config.VISION.entry_align_heading_gain) * heading_err
                )
                turn = self._clamp(
                    turn_raw,
                    -float(config.VISION.entry_align_turn_norm),
                    float(config.VISION.entry_align_turn_norm),
                )
                fwd = float(config.VISION.entry_align_forward_norm) if abs(lateral_err) <= (lateral_tol * 1.8) else 0.0
                left_cmd = int(self._clamp(fwd + turn, -1.0, 1.0) * config.VISION.max_cmd)
                right_cmd = int(self._clamp(fwd - turn, -1.0, 1.0) * config.VISION.max_cmd)

                self.drive.send_velocity(left_cmd, right_cmd)
                self._push_pid_sample(lateral_err, turn, left_cmd, right_cmd, f"{active_source}:align")
                self.status_text = (
                    f"Align entry ({active_source}) e={lateral_err:.3f} h={heading_err:.3f} "
                    f"hold={self._align_hold_count}/{max(1, int(config.VISION.entry_align_hold_frames))}"
                )

                if self._align_hold_count >= max(1, int(config.VISION.entry_align_hold_frames)):
                    self.drive.stop()
                    self._align_hold_count = 0
                    self._reset_stuck_watch()
                    if not self._normalize_bbox_to_trigger_line():
                        self.state = MissionState.SEARCHING_TARGET
                        self.status_text = "Pre-April adjust failed, back to search"
                        search_start = time.monotonic()
                        continue
                    if self.april_info is None:
                        self.state = MissionState.READING_APRILTAG
                        self.status_text = "Entry aligned, reading AprilTag"
                    else:
                        dist0 = self.drive.get_telemetry().distance_cm
                        first_leg = self.april_info is not None and self._use_apriltag_distance_once
                        move_increment_cm = self._compute_move_increment_cm()
                        self._configure_preplant_rear_adjust(dist0, move_increment_cm, first_leg)
                        self._distance_goal_cm = dist0 + move_increment_cm
                        self.state = MissionState.MOVING_TO_PLANT_POINT
                        self.status_text = "Entry aligned, use cached AprilTag"
                    continue

                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.READING_APRILTAG:
                if self.april_info is None:
                    found, metrics, tag_anno = self._try_read_apriltag_once(front_raw)
                    self._set_front_frame(tag_anno)
                    self._cache_apriltag_info(found, metrics)
                    if self.april_info is None:
                        det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
                        det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                        front_has_box = det_front is not None
                        rear_has_box = det_rear is not None
                        current_tag_status = self.status_text
                        show_tag_anno = current_tag_status.startswith("AprilTag seen but rejected")
                        self._set_front_frame(tag_anno if show_tag_anno else (front_anno if det_front is not None else tag_anno))
                        self._set_rear_frame(rear_anno)
                        if det_front is not None:
                            self.drive.stop()
                            self.target_class = det_front.class_name
                            self._approach_source = "front"
                            self._last_track_error = 0.0
                            if not current_tag_status.startswith("AprilTag seen but rejected"):
                                self.status_text = f"Seeking AprilTag: waiting on {det_front.class_name}"
                            time.sleep(config.RUNTIME.vision_loop_sec)
                            continue
                        self.drive.stop()
                        self._reset_stuck_watch()
                        if self._update_no_box_streak(front_has_box, rear_has_box):
                            self.state = MissionState.FINISHED
                            self.status_text = "No woodenbox on both cameras -> finish"
                            continue
                        self.state = MissionState.SEARCHING_TARGET
                        self.status_text = "Scanning AprilTag: no front woodenbox, searching target"
                        continue

                self.drive.stop()
                self._align_hold_count = 0
                self._reset_stuck_watch()
                self.state = MissionState.APPROACHING_TARGET
                self.status_text = "AprilTag read, continue PID approach"
                continue

            if self.state == MissionState.MOVING_TO_PLANT_POINT:
                tel = self.drive.get_telemetry()
                remaining = self._distance_goal_cm - tel.distance_cm
                self.status_text = f"Moving to plant point: remaining {remaining:.1f} cm"

                det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
                det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                front_has_box = det_front is not None
                rear_has_box = det_rear is not None
                self._set_front_frame(front_anno)
                self._set_rear_frame(rear_anno)

                if self._update_no_box_streak(front_has_box, rear_has_box):
                    self.drive.stop()
                    self._reset_stuck_watch()
                    self.state = MissionState.FINISHED
                    self.status_text = "No woodenbox on both cameras -> finish"
                    continue
                if not (front_has_box or rear_has_box):
                    self.drive.stop()
                    self._reset_stuck_watch()
                    self.status_text = (
                        f"Moving to plant point (no-box {self._no_box_streak}/{config.RUNTIME.no_box_stop_frames})"
                    )
                    time.sleep(config.RUNTIME.vision_loop_sec)
                    continue

                rear_apriltag_seen = False
                rear_apriltag_anno = None
                if self._preplant_adjust_pending and rear_raw is not None:
                    rear_apriltag_info, rear_apriltag_anno, rear_apriltag_metrics = (
                        self.apriltag.detect_and_annotate_with_metrics(
                            rear_raw,
                            prefer_largest_area=True,
                        )
                    )
                    rear_apriltag_seen = (
                        rear_apriltag_info is not None and rear_apriltag_metrics is not None
                    )
                    if rear_apriltag_seen and rear_apriltag_anno is not None:
                        self._set_rear_frame(rear_apriltag_anno)

                if self._preplant_adjust_pending and tel.distance_cm >= self._preplant_adjust_goal_cm:
                    self._preplant_adjust_pending = False

                if self._preplant_adjust_pending and rear_apriltag_seen:
                    self.drive.stop()
                    self._reset_stuck_watch()
                    pause_sec = max(0.0, float(config.VISION.preplant_adjust_apriltag_pause_sec))
                    self.status_text = (
                        f"Rear AprilTag seen, pause {pause_sec:.2f}s then adjust rear cabbage size"
                    )
                    if not self._sleep_interruptible(pause_sec):
                        self._preplant_adjust_pending = False
                        continue
                    self._measure_rear_size_once(record_point=False, phase="preplant_adjust")
                    self._preplant_adjust_pending = False
                    self._reset_stuck_watch()
                    continue

                if self._preplant_adjust_pending and tel.distance_cm >= self._preplant_adjust_fallback_cm:
                    self.drive.stop()
                    self._reset_stuck_watch()
                    pause_sec = max(0.0, float(config.VISION.preplant_adjust_apriltag_pause_sec))
                    self.status_text = (
                        f"Rear AprilTag not seen, fallback pause {pause_sec:.2f}s then adjust rear cabbage size"
                    )
                    if not self._sleep_interruptible(pause_sec):
                        self._preplant_adjust_pending = False
                        continue
                    self._measure_rear_size_once(record_point=False, phase="preplant_adjust_fallback")
                    self._preplant_adjust_pending = False
                    self._reset_stuck_watch()
                    continue

                if remaining <= 0:
                    self.drive.stop()
                    self._reset_stuck_watch()
                    self._preplant_adjust_pending = False
                    self.state = MissionState.PLANTING
                    continue

                move_norm = 0.35
                if self._preplant_adjust_pending and tel.distance_cm >= self._preplant_adjust_search_start_cm:
                    move_norm = self._get_preplant_adjust_slow_norm()
                forward_cmd = int(config.VISION.max_cmd * move_norm)
                self.drive.send_velocity(forward_cmd, forward_cmd)
                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.PLANTING:
                self.drive.stop()
                self._reset_stuck_watch()
                self.status_text = "Running planting sequence"
                ok = self.planting.run_once()
                if not ok:
                    self.state = MissionState.ERROR
                    reason = self.planting.last_error or self.actuator.last_message or "unknown"
                    self.status_text = f"Planting sequence failed: {reason}"
                    continue

                # Plant only 2 points per cycle:
                # 1) first point uses AB+offset
                # 2) second point uses AB
                if self.april_info is not None and self._plants_done_current_pair == 0:
                    self._plants_done_current_pair = 1
                    ab_cm = float(self.april_info.planting_distance_cm)
                    if ab_cm > 0:
                        dist0 = self.drive.get_telemetry().distance_cm
                        self._distance_goal_cm = dist0 + ab_cm
                        self.state = MissionState.MOVING_TO_PLANT_POINT
                        self.status_text = f"Plant #1 done, move to #2 by AB={ab_cm:.1f} cm"
                        continue
                    logger.warning("AB distance from AprilTag is <=0, skip 2nd planting move")

                if self.april_info is not None and self._plants_done_current_pair == 1:
                    self._plants_done_current_pair = 2
                    self._preplant_adjust_pending = False

                de_cm = 0.0
                c_cm = 0.0
                if self.april_info is not None:
                    c_cm = float(self.april_info.spacing_gap_cm)
                    de_cm = float(self.april_info.cabbage_interval_cm)
                self._measure_interval_cm = max(0.0, de_cm)
                first_measure_offset_cm = max(0.0, float(config.MOTION.plant_to_camback_point_cm))
                first_measure_base_cm = max(0.0, c_cm)
                first_measure_step_cm = first_measure_base_cm + first_measure_offset_cm
                self.state = MissionState.MOVING_TO_MEASURE_POINT
                if first_measure_step_cm > 0:
                    dist0 = self.drive.get_telemetry().distance_cm
                    self._distance_goal_cm = dist0 + first_measure_step_cm
                    self.status_text = (
                        "Move to first rear-measure point by "
                        f"C+CamBack={first_measure_step_cm:.1f} cm "
                        f"(C={first_measure_base_cm:.1f} + offset={first_measure_offset_cm:.1f})"
                    )
                else:
                    self._distance_goal_cm = self.drive.get_telemetry().distance_cm
                    self.status_text = "C invalid, measure at current position"
                continue

            if self.state == MissionState.MOVING_TO_MEASURE_POINT:
                tel = self.drive.get_telemetry()
                remaining = self._distance_goal_cm - tel.distance_cm
                self.status_text = f"Moving to rear measure point: remaining {remaining:.1f} cm"

                det_front, front_anno = self.detector.detect_best(front_raw, config.VISION.target_classes)
                det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                front_has_box = det_front is not None
                rear_has_box = det_rear is not None
                self._set_front_frame(front_anno)
                self._set_rear_frame(rear_anno)
                if self._update_no_box_streak(front_has_box, rear_has_box):
                    self.drive.stop()
                    self.state = MissionState.FINISHED
                    self.status_text = "No woodenbox on both cameras -> finish"
                    continue

                if remaining <= 0:
                    self.drive.stop()
                    self._reset_stuck_watch()
                    self.state = MissionState.MEASURING_REAR_SIZE
                    continue

                forward_cmd = int(config.VISION.max_cmd * self._get_move_to_measure_speed_norm())
                self.drive.send_velocity(forward_cmd, forward_cmd)
                if not self._check_stuck_and_recover(True, self._last_track_error):
                    continue
                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.MEASURING_REAR_SIZE:
                self._measure_rear_size_once()

                # After 2-plant cycle, keep measuring by DE repeatedly until out of plot.
                front_raw_now = self.cameras.get_raw_front_frame()
                rear_raw_now = self.cameras.get_raw_rear_frame()
                front_has_box = False
                rear_has_box = False
                if front_raw_now is not None:
                    det_front, front_anno = self.detector.detect_best(front_raw_now, config.VISION.target_classes)
                    front_has_box = det_front is not None
                    self._set_front_frame(front_anno)
                if rear_raw_now is not None:
                    det_rear, rear_anno = self._annotate_rear_frame(rear_raw_now)
                    rear_has_box = det_rear is not None
                    self._set_rear_frame(rear_anno)

                if self._update_no_box_streak(front_has_box, rear_has_box):
                    self.drive.stop()
                    self.state = MissionState.FINISHED
                    self.status_text = "Out of plot: no woodenbox on both cameras"
                    continue

                step_cm = max(0.0, float(self._measure_interval_cm))
                if step_cm <= 0.0:
                    self.drive.stop()
                    self.state = MissionState.FINISHED
                    self.status_text = "Rear measure done (DE unavailable)"
                    continue

                dist0 = self.drive.get_telemetry().distance_cm
                self._distance_goal_cm = dist0 + step_cm
                self.state = MissionState.MOVING_TO_MEASURE_POINT
                self.status_text = f"Next rear-measure point by DE={step_cm:.1f} cm"
                self._reset_stuck_watch()
                continue

            if self.state == MissionState.FINISHED:
                self.drive.stop()
                self.running = False
                return

            if self.state == MissionState.ERROR:
                self.drive.stop()
                self.running = False
                return

            time.sleep(config.RUNTIME.status_loop_sec)

        self.drive.stop()
        self.state = MissionState.STOPPED
        self.status_text = "Stopped"

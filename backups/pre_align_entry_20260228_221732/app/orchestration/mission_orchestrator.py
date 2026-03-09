"""Mission FSM for fully automatic operation."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import asdict

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

        self._distance_goal_cm = 0.0
        self._last_servo_step_ts = 0.0
        self._servo_cam_target_deg = float(config.ACTUATOR.servo_cam_start_deg)
        self._control_lock = threading.Lock()
        self._planting_busy = False
        self._approach_source = "front"
        self._no_box_streak = 0
        self._use_apriltag_distance_once = False
        self._pid_history = deque(maxlen=180)
        self._pid_seq = 0
        self._rear_measurement: dict | None = None
        self._plant_map: list[dict] = []
        self._measure_idx = 0
        self._cm_per_px = float(config.VISION.rear_cm_per_px_default)
        self._cm_per_px_source = "default"

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

    def toggle_camera_swap(self) -> tuple[bool, str]:
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
            pid={
                "kp": config.VISION.kp,
                "ki": config.VISION.ki,
                "kd": config.VISION.kd,
                "history": list(self._pid_history),
            },
            rear_measurement=self._rear_measurement,
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
        self._plant_map = []
        self._measure_idx = 0
        self._cm_per_px = float(config.VISION.rear_cm_per_px_default)
        self._cm_per_px_source = "default"

        self.state = MissionState.SEARCHING_TARGET
        search_start = time.monotonic()

        while not self._stop_event.is_set():
            # Mission control follows the current interface mapping.
            # Default control source is front; fallback to rear when front target is missing.
            front_raw = self.cameras.get_front_frame()
            rear_raw = self.cameras.get_rear_frame()

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
                    self.state = MissionState.APPROACHING_TARGET
                    self.status_text = f"Target locked (front): {det_front.class_name}"
                    self.pid.reset()
                    continue

                if det_rear is not None:
                    self._approach_source = "rear"
                    self.target_class = det_rear.class_name
                    self.state = MissionState.APPROACHING_TARGET
                    self.status_text = f"Target locked (rear): {det_rear.class_name}"
                    self.pid.reset()
                    continue

                if self._update_no_box_streak(front_has_box, rear_has_box):
                    self.drive.stop()
                    self.state = MissionState.FINISHED
                    self.status_text = "No woodenbox on both cameras -> finish"
                    continue

                self.drive.stop()
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

                active_source = "front"
                active_det = det_front
                active_frame = front_raw

                # Requirement: if rear sees woodenbox but front does not, drive by rear camera.
                if det_front is None and det_rear is not None:
                    active_source = "rear"
                    active_det = det_rear
                    active_frame = rear_raw
                elif det_front is not None:
                    active_source = "front"
                    active_det = det_front
                    active_frame = front_raw
                self._approach_source = active_source

                if active_det is None:
                    self.drive.stop()
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
                center_x = w / 2.0
                error = (active_det.cx - center_x) / center_x
                turn = self.pid.update(error)

                left_norm = max(-1.0, min(1.0, config.VISION.base_norm + turn))
                right_norm = max(-1.0, min(1.0, config.VISION.base_norm - turn))
                left_cmd = int(left_norm * config.VISION.max_cmd)
                right_cmd = int(right_norm * config.VISION.max_cmd)

                self.drive.send_velocity(left_cmd, right_cmd)
                self._push_pid_sample(error, turn, left_cmd, right_cmd, active_source)
                self.status_text = (
                    f"Approaching ({active_source}) {active_det.class_name}: L={left_cmd}, R={right_cmd}"
                )

                if active_det.bottom >= config.VISION.bbox_trigger_line_y:
                    self._update_servo_cam_tracking(active_det.bottom)

                    if self.actuator.servo_cam_deg <= config.ACTUATOR.servo_cam_end_deg:
                        self.drive.stop()
                        if self.april_info is None:
                            self.state = MissionState.READING_APRILTAG
                            self.status_text = "BBox reached trigger line, reading AprilTag"
                        else:
                            dist0 = self.drive.get_telemetry().distance_cm
                            move_increment_cm = self._compute_move_increment_cm()
                            self._distance_goal_cm = dist0 + move_increment_cm
                            self.state = MissionState.MOVING_TO_PLANT_POINT
                            self.status_text = "Use cached AprilTag, move to plant point"
                        continue

                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.READING_APRILTAG:
                if self.april_info is None:
                    self.status_text = "Scanning AprilTag 52h13 (wait until found)"
                    while not self._stop_event.is_set():
                        frame = self.cameras.get_front_frame()
                        if frame is None:
                            time.sleep(0.05)
                            continue
                        found, anno, metrics = self.apriltag.detect_and_annotate_with_metrics(frame)
                        self._set_front_frame(anno)
                        if found is not None:
                            self.april_info = found
                            self._use_apriltag_distance_once = True
                            tag_side_px = 0.0 if metrics is None else float(metrics.get("tag_side_px", 0.0))
                            if tag_side_px > 1.0 and float(config.VISION.apriltag_size_cm) > 0.0:
                                self._cm_per_px = float(config.VISION.apriltag_size_cm) / tag_side_px
                                self._cm_per_px_source = "apriltag(front)"
                            self.status_text = (
                                f"Tag {found.tag_id:05d} AB={found.planting_distance_cm} Cgap={found.spacing_gap_cm} "
                                f"DE={found.cabbage_interval_cm} cm_per_px={self._cm_per_px:.4f} ({self._cm_per_px_source})"
                            )
                            break
                        time.sleep(0.05)
                    if self._stop_event.is_set():
                        continue

                dist0 = self.drive.get_telemetry().distance_cm
                move_increment_cm = self._compute_move_increment_cm()
                self._distance_goal_cm = dist0 + move_increment_cm
                self.state = MissionState.MOVING_TO_PLANT_POINT
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
                    self.state = MissionState.FINISHED
                    self.status_text = "No woodenbox on both cameras -> finish"
                    continue
                if not (front_has_box or rear_has_box):
                    self.drive.stop()
                    self.status_text = (
                        f"Moving to plant point (no-box {self._no_box_streak}/{config.RUNTIME.no_box_stop_frames})"
                    )
                    time.sleep(config.RUNTIME.vision_loop_sec)
                    continue

                if remaining <= 0:
                    self.drive.stop()
                    self.state = MissionState.PLANTING
                    continue

                forward_cmd = int(config.VISION.max_cmd * 0.35)
                self.drive.send_velocity(forward_cmd, forward_cmd)
                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.PLANTING:
                self.drive.stop()
                self.status_text = "Running planting sequence"
                ok = self.planting.run_once()
                if not ok:
                    self.state = MissionState.ERROR
                    reason = self.planting.last_error or self.actuator.last_message or "unknown"
                    self.status_text = f"Planting sequence failed: {reason}"
                    continue

                de_cm = 0.0
                if self.april_info is not None:
                    de_cm = float(self.april_info.cabbage_interval_cm)
                if de_cm > 0:
                    dist0 = self.drive.get_telemetry().distance_cm
                    self._distance_goal_cm = dist0 + de_cm
                    self.state = MissionState.MOVING_TO_MEASURE_POINT
                    self.status_text = f"Move to rear-measure point by DE={de_cm:.1f} cm"
                    continue

                front_exists = False
                det_front, front_anno = self.detector.detect_best(front_raw, ("Woodenbox", "FaceWoodenbox"))
                if det_front is not None:
                    front_exists = True
                det_rear, rear_anno = self._annotate_rear_frame(rear_raw)
                rear_exists = det_rear is not None

                self._set_front_frame(front_anno)
                self._set_rear_frame(rear_anno)

                if not front_exists and not rear_exists:
                    self.state = MissionState.FINISHED
                    self.status_text = "No woodenbox on both cameras -> finish"
                else:
                    self.state = MissionState.SEARCHING_TARGET
                    search_start = time.monotonic()
                    self.status_text = "Continue next cycle"
                continue

            if self.state == MissionState.MOVING_TO_MEASURE_POINT:
                tel = self.drive.get_telemetry()
                remaining = self._distance_goal_cm - tel.distance_cm
                self.status_text = f"Moving to rear measure point: remaining {remaining:.1f} cm"

                if remaining <= 0:
                    self.drive.stop()
                    self.state = MissionState.MEASURING_REAR_SIZE
                    continue

                forward_cmd = int(config.VISION.max_cmd * float(config.VISION.move_to_measure_speed_norm))
                self.drive.send_velocity(forward_cmd, forward_cmd)
                time.sleep(config.RUNTIME.vision_loop_sec)
                continue

            if self.state == MissionState.MEASURING_REAR_SIZE:
                self.drive.stop()
                self.status_text = "Measuring cabbage size (rear)"
                deadline = time.monotonic() + max(0.5, float(config.VISION.rear_measure_timeout_sec))
                need_frames = max(1, int(config.VISION.rear_measure_frames))
                target_cls = str(config.VISION.rear_measure_target_class).strip()
                sizes_px: list[float] = []
                last_anno = None

                while time.monotonic() < deadline and not self._stop_event.is_set():
                    rear_frame = self.cameras.get_rear_frame()
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
                    det = max(dets, key=self._bbox_size_px) if dets else None
                    if det is not None:
                        size_px = self._bbox_size_px(det)
                        sizes_px.append(size_px)
                        size_est_cm = size_px * float(self._cm_per_px)
                        cv2.rectangle(rear_anno, (det.x1, det.y1), (det.x2, det.y2), (0, 128, 255), 3)
                        cv2.putText(
                            rear_anno,
                            f"Measure {target_cls}: {size_est_cm:.2f} cm ({len(dets)} det)",
                            (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2,
                        )
                    else:
                        cv2.putText(
                            rear_anno,
                            f"Measure {target_cls}: no detection",
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

                if sizes_px:
                    size_px_med = float(np.median(np.array(sizes_px, dtype=np.float32)))
                    size_cm = size_px_med * float(self._cm_per_px)
                    threshold_cm = float(config.VISION.rear_measure_size_threshold_cm)
                    reached = size_cm >= threshold_cm
                    self._measure_idx += 1
                    point = {
                        "idx": self._measure_idx,
                        "distance_cm": float(self.drive.get_telemetry().distance_cm),
                        "target_class": target_cls,
                        "size_cm": float(round(size_cm, 3)),
                        "size_threshold_cm": float(threshold_cm),
                        "reached": bool(reached),
                        "cm_per_px": float(round(self._cm_per_px, 6)),
                        "cm_per_px_source": self._cm_per_px_source,
                        "samples": int(len(sizes_px)),
                        "ts": time.time(),
                    }
                    self._rear_measurement = point
                    self._plant_map.append(point)
                    self.status_text = (
                        f"Rear measure #{point['idx']}: {point['size_cm']:.2f} cm "
                        f"({'OK' if reached else 'LOW'})"
                    )
                else:
                    self._rear_measurement = {
                        "idx": self._measure_idx + 1,
                        "target_class": target_cls,
                        "size_cm": None,
                        "size_threshold_cm": float(config.VISION.rear_measure_size_threshold_cm),
                        "reached": False,
                        "error": "no detection",
                        "ts": time.time(),
                    }
                    self.status_text = f"Rear measure failed: no {target_cls} detection"

                self.state = MissionState.SEARCHING_TARGET
                search_start = time.monotonic()
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

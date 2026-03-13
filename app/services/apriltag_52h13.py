"""AprilTag 52h13 detector + AB/C/DE decoder."""

from __future__ import annotations

import logging
import math
from typing import Optional

import cv2
import numpy as np

from app import config
from app.models import AprilTagInfo

logger = logging.getLogger(__name__)

try:
    from pupil_apriltags import Detector
    APRIL_AVAILABLE = True
except Exception:
    APRIL_AVAILABLE = False


C_TO_SPACING_MAP = {
    1: 5,
    2: 10,
    3: 15,
    4: 20,
    5: 25,
}


class AprilTag52h13Service:
    def __init__(self):
        self.enabled = False
        self.detector = None
        self._filtered_forward_cm = 0.0
        self._filtered_lateral_cm = 0.0
        self._filtered_yaw_deg = 0.0
        self._first_pose_measurement = True
        self._last_filter_tag_id: int | None = None
        if APRIL_AVAILABLE:
            try:
                self.detector = Detector(
                    families="tagStandard52h13",
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0,
                )
                self.enabled = True
                logger.info("AprilTag detector initialized (52h13)")
            except Exception as exc:
                logger.warning("AprilTag init failed: %s", exc)
        else:
            logger.warning("pupil-apriltags not installed; tag reading disabled")

    def _decode_tag_id(self, tag_id: int) -> AprilTagInfo:
        tag_str = f"{tag_id:05d}"
        ab = int(tag_str[:2])
        c = int(tag_str[2])
        de = int(tag_str[3:])
        spacing = C_TO_SPACING_MAP.get(c, 0)
        return AprilTagInfo(
            tag_id=tag_id,
            planting_distance_cm=ab,
            spacing_gap_cm=spacing,
            cabbage_interval_cm=de,
        )

    @staticmethod
    def _derive_effective_yaw_deg(
        practical_yaw_deg: float | None,
        pose_yaw_deg: float | None,
        bearing_x_deg: float | None,
    ) -> tuple[float | None, str]:
        if practical_yaw_deg is not None:
            return float(practical_yaw_deg), "practical"
        if pose_yaw_deg is not None:
            # pose yaw alone tends to underestimate side-facing tags; keep a conservative penalty.
            return float(pose_yaw_deg) * 2.5, "pose_penalized"
        if bearing_x_deg is not None:
            return float(bearing_x_deg), "bearing_proxy"
        return None, "none"

    def _detect_tag_candidates(
        self, frame: np.ndarray
    ) -> tuple[list[tuple[object, AprilTagInfo, dict]], np.ndarray]:
        annotated = frame.copy()
        if not self.enabled:
            return [], annotated

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced_gray = cv2.convertScaleAbs(gray, alpha=1.4, beta=20)
        enhanced_gray = cv2.medianBlur(enhanced_gray, 3)
        pose_enabled = all(
            float(v) > 0.0
            for v in (
                config.VISION.apriltag_fx,
                config.VISION.apriltag_fy,
                config.VISION.apriltag_cx,
                config.VISION.apriltag_cy,
                config.VISION.apriltag_size_cm,
            )
        )
        camera_params = None
        tag_size_m = None
        if pose_enabled:
            camera_params = (
                float(config.VISION.apriltag_fx),
                float(config.VISION.apriltag_fy),
                float(config.VISION.apriltag_cx),
                float(config.VISION.apriltag_cy),
            )
            tag_size_m = float(config.VISION.apriltag_size_cm) / 100.0

        tags = self.detector.detect(
            enhanced_gray,
            estimate_tag_pose=pose_enabled,
            camera_params=camera_params,
            tag_size=tag_size_m,
        )
        if len(tags) == 0:
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=pose_enabled,
                camera_params=camera_params,
                tag_size=tag_size_m,
            )
        if not tags:
            return [], annotated

        h, w = frame.shape[:2]
        candidates: list[tuple[object, AprilTagInfo, dict]] = []
        tags = sorted(tags, key=self._tag_visible_area, reverse=True)
        for tag in tags:
            info = self._decode_tag_id(int(tag.tag_id))
            c = tag.corners.astype(np.float32)
            edge_lens = [
                float(np.linalg.norm(c[0] - c[1])),
                float(np.linalg.norm(c[1] - c[2])),
                float(np.linalg.norm(c[2] - c[3])),
                float(np.linalg.norm(c[3] - c[0])),
            ]
            side_px = float(np.mean(edge_lens))
            center_x = float(tag.center[0])
            center_y = float(tag.center[1])
            center_err_norm = 0.0 if w <= 0 else (center_x - (w / 2.0)) / (w / 2.0)
            bearing_x_deg, bearing_y_deg = self._compute_bearing_deg(center_x, center_y, frame.shape)
            approx_dist_cm = None
            if float(config.VISION.apriltag_size_cm) > 0.0 and side_px > 1.0 and float(config.VISION.apriltag_fx) > 0.0:
                approx_dist_cm = float(config.VISION.apriltag_size_cm) * float(config.VISION.apriltag_fx) / side_px
            pose_x_cm = None
            pose_y_cm = None
            pose_z_cm = None
            pose_yaw_deg = None
            robot_forward_cm = None
            robot_lateral_cm = None
            practical_yaw_deg = None
            pixel_error_x = center_err_norm
            if pose_enabled and getattr(tag, "pose_t", None) is not None and getattr(tag, "pose_R", None) is not None:
                t = np.array(tag.pose_t).reshape(-1)
                pose_x_cm = float(t[0]) * 100.0
                pose_y_cm = float(t[1]) * 100.0
                pose_z_cm = float(t[2]) * 100.0
                _, pose_yaw_deg, _ = self._rotation_to_euler_deg(np.array(tag.pose_R))

            practical_pose = self._compute_robot_pose_metrics(tag, frame.shape)
            if practical_pose is not None:
                robot_forward_cm, robot_lateral_cm, practical_yaw_deg = practical_pose

            effective_yaw_deg, effective_yaw_source = self._derive_effective_yaw_deg(
                practical_yaw_deg,
                pose_yaw_deg,
                bearing_x_deg,
            )
            front_facing_score = (
                999.0 if effective_yaw_deg is None else abs(float(effective_yaw_deg))
            )

            metrics = {
                "tag_side_px": side_px,
                "tag_center_x_px": center_x,
                "tag_center_y_px": center_y,
                "tag_center_err_norm": center_err_norm,
                "tag_bearing_x_deg": bearing_x_deg,
                "tag_bearing_y_deg": bearing_y_deg,
                "tag_approx_dist_cm": approx_dist_cm,
                "tag_pose_x_cm": pose_x_cm,
                "tag_pose_y_cm": pose_y_cm,
                "tag_pose_z_cm": pose_z_cm,
                "tag_pose_yaw_deg": pose_yaw_deg,
                "tag_pixel_error_x": pixel_error_x,
                "tag_robot_forward_cm": robot_forward_cm,
                "tag_robot_lateral_cm": robot_lateral_cm,
                "tag_robot_yaw_practical_deg": practical_yaw_deg,
                "tag_effective_yaw_deg": effective_yaw_deg,
                "tag_effective_yaw_source": effective_yaw_source,
                "tag_front_facing_score": front_facing_score,
            }
            candidates.append((tag, info, metrics))
        return candidates, annotated

    @staticmethod
    def _tag_visible_area(tag) -> float:
        return float(cv2.contourArea(tag.corners.astype(np.float32)))

    def _compute_robot_pose_metrics(
        self, tag, frame_shape: tuple[int, int, int]
    ) -> tuple[float, float, float] | None:
        fx = float(config.VISION.apriltag_fx)
        fy = float(config.VISION.apriltag_fy)
        cx = float(config.VISION.apriltag_cx)
        cy = float(config.VISION.apriltag_cy)
        tag_size_cm = float(config.VISION.apriltag_size_cm)
        if min(fx, fy, tag_size_cm) <= 0.0:
            return None

        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        half_size_m = (tag_size_cm / 100.0) * 0.5
        obj_pts = np.array(
            [
                [-half_size_m, -half_size_m, 0.0],
                [half_size_m, -half_size_m, 0.0],
                [half_size_m, half_size_m, 0.0],
                [-half_size_m, half_size_m, 0.0],
            ],
            dtype=np.float32,
        )
        success, rvec, tvec = cv2.solvePnP(
            obj_pts,
            tag.corners.astype(np.float32),
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None

        rot, _ = cv2.Rodrigues(rvec)
        rot_inv = rot.T
        t_inv = -rot_inv @ tvec
        raw_forward_cm = float(-t_inv[2][0] * 100.0)
        raw_lateral_cm = float(-t_inv[0][0] * 100.0)
        center_x = float(tag.center[0])
        w = frame_shape[1]
        pixel_error_x = 0.0 if w <= 0 else (center_x - (w / 2.0)) / (w / 2.0)

        robot_forward_cm = raw_forward_cm - float(config.VISION.apriltag_cam_offset_x_cm)
        robot_lateral_cm = raw_lateral_cm - float(config.VISION.apriltag_cam_offset_y_cm)
        robot_lateral_cm -= pixel_error_x * raw_forward_cm * float(config.VISION.apriltag_pixel_error_y_gain)

        raw_yaw_deg = math.degrees(math.atan2(rot[0, 2], rot[2, 2]))
        if raw_yaw_deg > 90.0:
            raw_yaw_deg -= 180.0
        elif raw_yaw_deg < -90.0:
            raw_yaw_deg += 180.0

        if self._first_pose_measurement or self._last_filter_tag_id != int(tag.tag_id):
            self._filtered_forward_cm = robot_forward_cm
            self._filtered_lateral_cm = robot_lateral_cm
            self._filtered_yaw_deg = raw_yaw_deg
            self._first_pose_measurement = False
            self._last_filter_tag_id = int(tag.tag_id)
        else:
            ax = float(config.VISION.apriltag_alpha_x)
            ay = float(config.VISION.apriltag_alpha_y)
            ayaw = float(config.VISION.apriltag_alpha_yaw)
            self._filtered_forward_cm = (ax * self._filtered_forward_cm) + ((1.0 - ax) * robot_forward_cm)
            self._filtered_lateral_cm = (ay * self._filtered_lateral_cm) + ((1.0 - ay) * robot_lateral_cm)
            self._filtered_yaw_deg = (ayaw * self._filtered_yaw_deg) + ((1.0 - ayaw) * raw_yaw_deg)

        return (
            float(self._filtered_forward_cm),
            float(self._filtered_lateral_cm),
            float(self._filtered_yaw_deg),
        )

    @staticmethod
    def _candidate_measure_rank(metrics: dict) -> tuple[float, float, float]:
        bearing_x = metrics.get("tag_bearing_x_deg")
        side_px = float(metrics.get("tag_side_px", 0.0))
        yaw_abs = float(metrics.get("tag_front_facing_score", 999.0))
        bearing_abs = 999.0 if bearing_x is None else abs(float(bearing_x))
        return (yaw_abs, bearing_abs, -side_px)

    def _select_measure_candidate(
        self,
        candidates: list[tuple[object, AprilTagInfo, dict]],
        *,
        prefer_largest_area: bool,
    ) -> tuple[object, AprilTagInfo, dict]:
        if prefer_largest_area:
            return max(candidates, key=lambda item: self._tag_visible_area(item[0]))
        return min(candidates, key=lambda item: self._candidate_measure_rank(item[2]))

    @staticmethod
    def _compute_bearing_deg(center_x: float, center_y: float, frame_shape: tuple[int, int, int]) -> tuple[float | None, float | None]:
        h, w = frame_shape[:2]

        fx = float(config.VISION.apriltag_fx)
        fy = float(config.VISION.apriltag_fy)
        cx = float(config.VISION.apriltag_cx)
        cy = float(config.VISION.apriltag_cy)

        if fx > 0.0 and cx > 0.0:
            bearing_x = math.degrees(math.atan2(center_x - cx, fx))
        elif float(config.VISION.apriltag_hfov_deg) > 0.0 and w > 0:
            bearing_x = ((center_x - (w / 2.0)) / (w / 2.0)) * (float(config.VISION.apriltag_hfov_deg) / 2.0)
        else:
            bearing_x = None

        if fy > 0.0 and cy > 0.0:
            bearing_y = math.degrees(math.atan2(center_y - cy, fy))
        elif float(config.VISION.apriltag_vfov_deg) > 0.0 and h > 0:
            bearing_y = ((center_y - (h / 2.0)) / (h / 2.0)) * (float(config.VISION.apriltag_vfov_deg) / 2.0)
        else:
            bearing_y = None

        return bearing_x, bearing_y

    @staticmethod
    def _rotation_to_euler_deg(rot: np.ndarray) -> tuple[float, float, float]:
        sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(rot[2, 1], rot[2, 2])
            pitch = math.atan2(-rot[2, 0], sy)
            yaw = math.atan2(rot[1, 0], rot[0, 0])
        else:
            roll = math.atan2(-rot[1, 2], rot[1, 1])
            pitch = math.atan2(-rot[2, 0], sy)
            yaw = 0.0

        return tuple(math.degrees(v) for v in (roll, pitch, yaw))

    def detect_and_annotate_with_metrics(
        self,
        frame: np.ndarray,
        *,
        prefer_largest_area: bool = False,
    ) -> tuple[Optional[AprilTagInfo], np.ndarray, Optional[dict]]:
        try:
            candidates, annotated = self._detect_tag_candidates(frame)
            if not candidates:
                return None, annotated, None

            tag, info, metrics = self._select_measure_candidate(
                candidates,
                prefer_largest_area=prefer_largest_area,
            )

            corners = tag.corners.astype(int)
            for i in range(4):
                p1 = tuple(corners[i])
                p2 = tuple(corners[(i + 1) % 4])
                cv2.line(annotated, p1, p2, (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.circle(annotated, center, 5, (0, 0, 255), -1)

            cv2.putText(annotated, f"TagID: {info.tag_id:05d}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"AB={info.planting_distance_cm}cm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(annotated, f"C gap={info.spacing_gap_cm}cm", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            cv2.putText(annotated, f"DE={info.cabbage_interval_cm}cm", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            bearing_x_deg = metrics.get("tag_bearing_x_deg")
            approx_dist_cm = metrics.get("tag_approx_dist_cm")
            pose_x_cm = metrics.get("tag_pose_x_cm")
            pose_z_cm = metrics.get("tag_pose_z_cm")
            pose_yaw_deg = metrics.get("tag_pose_yaw_deg")
            robot_forward_cm = metrics.get("tag_robot_forward_cm")
            robot_lateral_cm = metrics.get("tag_robot_lateral_cm")
            practical_yaw_deg = metrics.get("tag_robot_yaw_practical_deg")

            if bearing_x_deg is not None or approx_dist_cm is not None:
                bx = "na" if bearing_x_deg is None else f"{bearing_x_deg:.1f}"
                dist = "na" if approx_dist_cm is None else f"{approx_dist_cm:.1f}"
                cv2.putText(
                    annotated,
                    f"angleX={bx}deg dist~={dist}cm",
                    (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                )
            if pose_x_cm is not None and pose_z_cm is not None:
                cv2.putText(
                    annotated,
                    f"x={pose_x_cm:.1f}cm z={pose_z_cm:.1f}cm yaw={pose_yaw_deg:.1f}deg",
                    (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 200, 50),
                    2,
                )
            if robot_forward_cm is not None and robot_lateral_cm is not None and practical_yaw_deg is not None:
                cv2.putText(
                    annotated,
                    f"robotX={robot_forward_cm:.1f}cm robotY={robot_lateral_cm:.1f}cm yaw2={practical_yaw_deg:.1f}deg",
                    (10, 175),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 220, 120),
                    2,
                )

            return info, annotated, metrics
        except Exception as exc:
            logger.error("AprilTag detection error: %s", exc)
            return None, annotated, None

    def detect_and_annotate(
        self,
        frame: np.ndarray,
        *,
        prefer_largest_area: bool = False,
    ) -> tuple[Optional[AprilTagInfo], np.ndarray]:
        info, annotated, _ = self.detect_and_annotate_with_metrics(
            frame,
            prefer_largest_area=prefer_largest_area,
        )
        return info, annotated

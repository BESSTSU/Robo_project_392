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
        self, frame: np.ndarray
    ) -> tuple[Optional[AprilTagInfo], np.ndarray, Optional[dict]]:
        annotated = frame.copy()
        if not self.enabled:
            return None, annotated, None

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
                gray,
                estimate_tag_pose=pose_enabled,
                camera_params=camera_params,
                tag_size=tag_size_m,
            )
            if not tags:
                return None, annotated, None

            tag = tags[0]
            info = self._decode_tag_id(int(tag.tag_id))

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

            # Approximate side length in pixels (mean of 4 edges).
            c = tag.corners.astype(np.float32)
            edge_lens = [
                float(np.linalg.norm(c[0] - c[1])),
                float(np.linalg.norm(c[1] - c[2])),
                float(np.linalg.norm(c[2] - c[3])),
                float(np.linalg.norm(c[3] - c[0])),
            ]
            side_px = float(np.mean(edge_lens))
            h, w = frame.shape[:2]
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
            if pose_enabled and getattr(tag, "pose_t", None) is not None and getattr(tag, "pose_R", None) is not None:
                t = np.array(tag.pose_t).reshape(-1)
                pose_x_cm = float(t[0]) * 100.0
                pose_y_cm = float(t[1]) * 100.0
                pose_z_cm = float(t[2]) * 100.0
                _, _, pose_yaw_deg = self._rotation_to_euler_deg(np.array(tag.pose_R))

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
            }
            return info, annotated, metrics
        except Exception as exc:
            logger.error("AprilTag detection error: %s", exc)
            return None, annotated, None

    def detect_and_annotate(self, frame: np.ndarray) -> tuple[Optional[AprilTagInfo], np.ndarray]:
        info, annotated, _ = self.detect_and_annotate_with_metrics(frame)
        return info, annotated

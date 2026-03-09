"""AprilTag 52h13 detector + AB/C/DE decoder."""

from __future__ import annotations

import logging
from typing import Optional

import cv2
import numpy as np

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

    def detect_and_annotate_with_metrics(
        self, frame: np.ndarray
    ) -> tuple[Optional[AprilTagInfo], np.ndarray, Optional[dict]]:
        annotated = frame.copy()
        if not self.enabled:
            return None, annotated, None

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)
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
            metrics = {"tag_side_px": side_px}
            return info, annotated, metrics
        except Exception as exc:
            logger.error("AprilTag detection error: %s", exc)
            return None, annotated, None

    def detect_and_annotate(self, frame: np.ndarray) -> tuple[Optional[AprilTagInfo], np.ndarray]:
        info, annotated, _ = self.detect_and_annotate_with_metrics(frame)
        return info, annotated

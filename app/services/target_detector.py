"""YOLO target detection with class priority and single-target output."""

from __future__ import annotations

import logging
from typing import Optional

import cv2
import numpy as np

from app import config
from app.models import Detection

logger = logging.getLogger(__name__)

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:
    YOLO_AVAILABLE = False


class TargetDetector:
    def __init__(self):
        self.model = None
        self.enabled = False
        self.disabled_reason = ""
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(config.VISION.model_path)
                self.enabled = True
                logger.info("YOLO loaded: %s", config.VISION.model_path)
            except Exception as exc:
                self.disabled_reason = str(exc)
                logger.warning("YOLO unavailable at runtime: %s", exc)
        else:
            self.disabled_reason = "ultralytics not installed"
            logger.warning("ultralytics not installed; detection disabled")

    @staticmethod
    def _draw_common_overlay(annotated: np.ndarray) -> None:
        h, w, _ = annotated.shape
        center_x = w / 2
        cv2.line(
            annotated,
            (0, config.VISION.bbox_trigger_line_y),
            (w, config.VISION.bbox_trigger_line_y),
            (255, 200, 0),
            2,
        )
        cv2.line(annotated, (int(center_x), 0), (int(center_x), h), (255, 0, 0), 1)

    def detect_all(
        self,
        frame: np.ndarray,
        classes: tuple[str, ...],
        max_per_class: int = 3,
        max_total: int = 10,
        use_priority: bool = True,
    ) -> tuple[list[Detection], np.ndarray]:
        """
        Multi-target detector for measurement/planning stages.
        Returns sorted detections + annotated frame.
        """
        annotated = frame.copy()
        self._draw_common_overlay(annotated)

        if not self.enabled:
            cv2.putText(
                annotated,
                f"YOLO disabled ({self.disabled_reason})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )
            return [], annotated

        try:
            target_classes_norm = {c.strip().lower() for c in classes}
            priority_norm = config.VISION.priority_class.strip().lower()
            result = self.model(frame, verbose=False, max_det=int(config.VISION.yolo_max_det))[0]
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                return [], annotated

            by_class: dict[str, list[Detection]] = {}
            for box in boxes:
                cls_id = int(box.cls[0].item())
                class_name = self.model.names[cls_id]
                class_name_norm = str(class_name).strip().lower()
                conf = float(box.conf[0].item())
                if class_name_norm not in target_classes_norm or conf < config.VISION.confidence_threshold:
                    continue

                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                cand = Detection(class_name=class_name, conf=conf, x1=x1, y1=y1, x2=x2, y2=y2)
                by_class.setdefault(class_name_norm, []).append(cand)

            if not by_class:
                return [], annotated

            selected: list[Detection] = []
            for cls, items in by_class.items():
                items.sort(key=lambda d: d.conf, reverse=True)
                selected.extend(items[: max(1, int(max_per_class))])

            # Tracking mode: priority class first, then confidence.
            # Measurement mode: confidence-only ranking.
            if use_priority:
                selected.sort(
                    key=lambda d: (
                        0 if d.class_name.strip().lower() == priority_norm else 1,
                        -d.conf,
                    )
                )
            else:
                selected.sort(key=lambda d: -d.conf)
            selected = selected[: max(1, int(max_total))]

            for det in selected:
                pri = det.class_name.strip().lower() == priority_norm
                color = (0, 255, 0) if pri else (0, 255, 255)
                cv2.rectangle(annotated, (det.x1, det.y1), (det.x2, det.y2), color, 2)
                cv2.putText(
                    annotated,
                    f"{det.class_name} {det.conf:.2f}",
                    (det.x1, max(10, det.y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

            return selected, annotated
        except Exception as exc:
            logger.error("YOLO detection error: %s", exc)
            cv2.putText(annotated, f"Detect error: {exc}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            return [], annotated

    def detect_best(self, frame: np.ndarray, classes: tuple[str, ...]) -> tuple[Optional[Detection], np.ndarray]:
        detections, annotated = self.detect_all(frame, classes, max_per_class=1, max_total=1)
        if not detections:
            return None, annotated
        best = detections[0]
        cv2.circle(annotated, (int(best.cx), int((best.y1 + best.y2) / 2)), 5, (0, 0, 255), -1)
        return best, annotated

    def detect_woodenbox_exists(self, frame: np.ndarray) -> bool:
        det, _ = self.detect_best(frame, ("woodenbox", "facewoodenbox"))
        return det is not None

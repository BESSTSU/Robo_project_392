"""Dual camera capture with thread-safe frame access."""

from __future__ import annotations

import logging
import threading
import time

import cv2
import numpy as np

from app import config

logger = logging.getLogger(__name__)


class _CameraWorker:
    def __init__(self, camera_index: int, width: int, height: int, label: str):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.label = label

        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        while self._running:
            cap = cv2.VideoCapture(self.camera_index)
            if not cap.isOpened():
                logger.warning("camera %s index %d open failed", self.label, self.camera_index)
                time.sleep(1.0)
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            logger.info("camera %s opened (%dx%d)", self.label, self.width, self.height)

            while self._running and cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    break
                with self._lock:
                    self._frame = frame
                time.sleep(config.RUNTIME.camera_loop_sec)

            cap.release()

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame is None else self._frame.copy()


class DualCameraManager:
    def __init__(self):
        self.front = _CameraWorker(
            config.CAMERA.front_index,
            config.CAMERA.width,
            config.CAMERA.height,
            "front",
        )
        self.rear = _CameraWorker(
            config.CAMERA.rear_index,
            config.CAMERA.width,
            config.CAMERA.height,
            "rear",
        )
        self._swap_lock = threading.Lock()
        self._swap_runtime = bool(config.CAMERA.swap)
        logger.info(
            "camera mapping configured front=%d rear=%d (initial swap=%s)",
            config.CAMERA.front_index,
            config.CAMERA.rear_index,
            "on" if self._swap_runtime else "off",
        )
        logger.info(
            "camera effective front=%s rear=%s",
            "rear" if self._swap_runtime else "front",
            "front" if self._swap_runtime else "rear",
        )

    def set_swap(self, enabled: bool) -> None:
        with self._swap_lock:
            self._swap_runtime = bool(enabled)
        logger.info(
            "camera swap changed: %s (effective front=%s rear=%s)",
            "on" if self._swap_runtime else "off",
            "rear" if self._swap_runtime else "front",
            "front" if self._swap_runtime else "rear",
        )

    def toggle_swap(self) -> bool:
        with self._swap_lock:
            self._swap_runtime = not self._swap_runtime
            swapped = self._swap_runtime
        logger.info(
            "camera swap toggled: %s (effective front=%s rear=%s)",
            "on" if swapped else "off",
            "rear" if swapped else "front",
            "front" if swapped else "rear",
        )
        return swapped

    def is_swapped(self) -> bool:
        with self._swap_lock:
            return self._swap_runtime

    def _use_swapped(self) -> bool:
        with self._swap_lock:
            return self._swap_runtime

    def _logical_front_source(self) -> _CameraWorker:
        return self.rear if self._use_swapped() else self.front

    def _logical_rear_source(self) -> _CameraWorker:
        return self.front if self._use_swapped() else self.rear

    def get_front_frame(self):
        return self._logical_front_source().get_frame()

    def get_rear_frame(self):
        return self._logical_rear_source().get_frame()

    def get_raw_front_frame(self):
        return self.front.get_frame()

    def get_raw_rear_frame(self):
        return self.rear.get_frame()

    def get_camera_info(self) -> dict:
        return {
            "configured_front_index": config.CAMERA.front_index,
            "configured_rear_index": config.CAMERA.rear_index,
            "swap": self.is_swapped(),
            "effective_front_source": "rear" if self.is_swapped() else "front",
            "effective_rear_source": "front" if self.is_swapped() else "rear",
        }

    def start(self) -> None:
        self.front.start()
        self.rear.start()

    def stop(self) -> None:
        self.front.stop()
        self.rear.stop()

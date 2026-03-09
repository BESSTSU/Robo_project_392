"""Thread-safe serial link abstraction for ESP32 devices."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Callable

import serial

logger = logging.getLogger(__name__)


class SerialLink:
    def __init__(self, port: str, baud: int, name: str):
        self.port = port
        self.baud = baud
        self.name = name

        self._ser: serial.Serial | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        self._listeners: list[Callable[[str], None]] = []
        self._recent_lines = deque(maxlen=200)
        self.connected = False

    def open(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.connected = True
            self._running = True
            self._thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()
            logger.info("[%s] connected at %s @ %d", self.name, self.port, self.baud)
            return True
        except Exception as exc:
            logger.warning("[%s] open failed (%s): %s", self.name, self.port, exc)
            self.connected = False
            return False

    def add_listener(self, listener: Callable[[str], None]) -> None:
        self._listeners.append(listener)

    def _reader_loop(self) -> None:
        while self._running:
            try:
                if self._ser is None:
                    time.sleep(0.2)
                    continue

                line = self._ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                self._recent_lines.append(line)
                for listener in self._listeners:
                    try:
                        listener(line)
                    except Exception as exc:
                        logger.debug("[%s] listener error: %s", self.name, exc)
            except Exception as exc:
                logger.warning("[%s] read error: %s", self.name, exc)
                self.connected = False
                time.sleep(0.2)

    def send_line(self, line: str) -> tuple[bool, str]:
        if self._ser is None or not self.connected:
            return False, "not connected"

        try:
            with self._lock:
                self._ser.write((line + "\n").encode())
            return True, "ok"
        except Exception as exc:
            self.connected = False
            return False, str(exc)

    def recent_lines(self) -> list[str]:
        return list(self._recent_lines)

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None:
            self._ser.close()
        self.connected = False
        logger.info("[%s] closed", self.name)

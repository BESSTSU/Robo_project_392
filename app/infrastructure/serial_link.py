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
        self._last_open_error = ""

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.connected = True
            self._last_open_error = ""
            return True
        except Exception as exc:
            self._ser = None
            self.connected = False
            self._last_open_error = str(exc)
            return False

    def open(self) -> bool:
        ok = self._open_serial()
        if not ok:
            logger.warning("[%s] open failed (%s): %s", self.name, self.port, self._last_open_error)
            return False

        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        logger.info("[%s] connected at %s @ %d", self.name, self.port, self.baud)
        return True

    def add_listener(self, listener: Callable[[str], None]) -> None:
        self._listeners.append(listener)

    def ensure_open(self) -> bool:
        if self.connected and self._ser is not None:
            return True
        ok = self._open_serial()
        if ok:
            logger.info("[%s] reconnected at %s @ %d", self.name, self.port, self.baud)
        return ok

    def _reader_loop(self) -> None:
        while self._running:
            try:
                if self._ser is None:
                    self.ensure_open()
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
                try:
                    if self._ser is not None:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(0.2)

    def send_line(self, line: str) -> tuple[bool, str]:
        if self._ser is None or not self.connected:
            if not self.ensure_open():
                return False, f"not connected ({self._last_open_error or 'open failed'})"

        try:
            with self._lock:
                self._ser.write((line + "\n").encode())
            return True, "ok"
        except Exception as exc:
            self.connected = False
            return False, str(exc)

    def recent_lines(self) -> list[str]:
        return list(self._recent_lines)

    def reset_input_buffer(self) -> None:
        if self._ser is None:
            return
        try:
            with self._lock:
                self._ser.reset_input_buffer()
        except Exception as exc:
            logger.debug("[%s] reset_input_buffer failed: %s", self.name, exc)

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None:
            self._ser.close()
        self.connected = False
        logger.info("[%s] closed", self.name)

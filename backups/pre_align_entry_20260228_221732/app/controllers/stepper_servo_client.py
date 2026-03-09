"""Actuator ESP32 client for stepper + servo commands."""

from __future__ import annotations

import logging
import re
import threading
import time
from collections import deque

from app import config
from app.infrastructure.serial_link import SerialLink

logger = logging.getLogger(__name__)


class StepperServoClient:
    def __init__(self):
        self.link = SerialLink(config.SERIAL.actuator_port, config.SERIAL.actuator_baud, "actuator")
        self.link.add_listener(self._on_line)
        self._lock = threading.Lock()
        self.last_message = ""
        self.servo_cam_deg = config.ACTUATOR.servo_cam_start_deg
        self.servo_plant_deg = config.ACTUATOR.servo_plant_start_deg
        self._rx_seq = 0
        self._rx_history: deque[tuple[int, str]] = deque(maxlen=400)
        self.protocol_version = "legacy"
        self.protocol_caps: set[str] = set()
        self.protocol_raw = ""
        self._caps_known = False

    def _on_line(self, line: str) -> None:
        with self._lock:
            self._rx_seq += 1
            self.last_message = line
            self._rx_history.append((self._rx_seq, line))

    def open(self) -> bool:
        ok = self.link.open()
        if ok:
            self._probe_protocol()
        return ok

    def _find_line_after(self, after_seq: int, contains: tuple[str, ...]) -> str | None:
        needles = tuple(n.upper() for n in contains if n)
        if not needles:
            return None
        with self._lock:
            for seq, line in reversed(self._rx_history):
                if seq <= after_seq:
                    break
                upper = line.upper()
                if any(n in upper for n in needles):
                    return line
        return None

    @staticmethod
    def _parse_caps_csv(value: str) -> set[str]:
        return {tok.strip().upper() for tok in value.replace(" ", "").split(",") if tok.strip()}

    def _caps_from_help_lines(self, after_seq: int) -> set[str]:
        caps: set[str] = set()
        with self._lock:
            lines = [line.upper() for seq, line in self._rx_history if seq > after_seq]
        joined = " ".join(lines)
        if "INIT_HOME" in joined:
            caps.add("INIT_HOME")
        if "SERVO CAM" in joined:
            caps.add("SERVO_CAM")
        if "SERVO PLANT" in joined:
            caps.add("SERVO_PLANT")
        if "FEED STEP" in joined:
            caps.add("FEED_STEP")
        if "PLANT DOWN_LIMIT" in joined:
            caps.add("PLANT_DOWN_LIMIT")
        if "PLANT UP_LIMIT" in joined:
            caps.add("PLANT_UP_LIMIT")
        if "PLANT UP_STEP" in joined:
            caps.add("PLANT_UP_STEP")
        if "PLANT DOWN_STEP" in joined:
            caps.add("PLANT_DOWN_STEP")
        if "ACT_STATUS" in joined:
            caps.add("ACT_STATUS")
        if "ACT_STOP" in joined:
            caps.add("ACT_STOP")
        return caps

    def _probe_protocol(self) -> None:
        """
        Best-effort protocol/capability probe.
        Backward compatible: if probe fails, keep legacy mode and do not block commands.
        """
        self.protocol_version = "legacy"
        self.protocol_caps = set()
        self.protocol_raw = ""
        self._caps_known = False

        seq0 = self.get_rx_seq()
        ok, _ = self.link.send_line("PROTO_VER")
        if ok and self.wait_for_event(("ACT_PROTO", "ACT_ERR UNKNOWN:PROTO_VER"), 0.6, after_seq=seq0):
            line = self._find_line_after(seq0, ("ACT_PROTO", "ACT_ERR UNKNOWN:PROTO_VER"))
            if line and line.upper().startswith("ACT_PROTO"):
                self.protocol_raw = line
                m_ver = re.search(r"VER[:=]\s*([0-9A-Z._-]+)", line, flags=re.IGNORECASE)
                if m_ver:
                    self.protocol_version = m_ver.group(1)
                m_caps = re.search(r"CAPS[:=]\s*(.+)$", line, flags=re.IGNORECASE)
                if m_caps:
                    self.protocol_caps = self._parse_caps_csv(m_caps.group(1))
                    self._caps_known = True
                logger.info("Actuator protocol detected: ver=%s caps=%s", self.protocol_version, sorted(self.protocol_caps))
                return

        # Fallback: HELP parsing for older firmware without explicit PROTO_VER.
        seq1 = self.get_rx_seq()
        ok, _ = self.link.send_line("HELP")
        if ok:
            self.wait_for_event(("COMMANDS:", "ACT_ERR UNKNOWN:HELP"), 0.6, after_seq=seq1)
            caps = self._caps_from_help_lines(seq1)
            if caps:
                self.protocol_version = "legacy-help"
                self.protocol_caps = caps
                self._caps_known = True
                logger.info("Actuator capabilities inferred from HELP: %s", sorted(self.protocol_caps))
                return

        logger.info("Actuator protocol probe fallback: legacy (capabilities unknown)")

    def supports(self, capability: str) -> bool:
        """
        Capability check with backward-compatible default.
        If capabilities are unknown (legacy), return True to avoid breaking existing flows.
        """
        cap = capability.strip().upper()
        if not cap:
            return True
        if not self._caps_known:
            return True
        return cap in self.protocol_caps

    def initialize_home(self) -> tuple[bool, str]:
        return self.link.send_line(
            f"INIT_HOME CAM:{config.ACTUATOR.servo_cam_start_deg} PLANT:{config.ACTUATOR.servo_plant_start_deg}"
        )

    def set_servo_cam(self, deg: int) -> tuple[bool, str]:
        deg = max(0, min(180, int(deg)))
        ok, msg = self.link.send_line(f"SERVO CAM {deg}")
        if ok:
            self.servo_cam_deg = deg
        return ok, msg

    def set_servo_plant(self, deg: int) -> tuple[bool, str]:
        deg = max(0, min(180, int(deg)))
        ok, msg = self.link.send_line(f"SERVO PLANT {deg}")
        if ok:
            self.servo_plant_deg = deg
        return ok, msg

    def feed_step(self, deg: int) -> tuple[bool, str]:
        return self.link.send_line(f"FEED STEP {int(deg)}")

    def plant_down_until_bottom(self) -> tuple[bool, str]:
        return self.link.send_line("PLANT DOWN_LIMIT")

    def plant_up_until_top(self) -> tuple[bool, str]:
        return self.link.send_line("PLANT UP_LIMIT")

    def plant_up_step(self, deg: int) -> tuple[bool, str]:
        return self.link.send_line(f"PLANT UP_STEP {int(deg)}")

    def plant_down_step(self, deg: int) -> tuple[bool, str]:
        return self.link.send_line(f"PLANT DOWN_STEP {int(deg)}")

    def stop_all(self) -> tuple[bool, str]:
        return self.link.send_line("ACT_STOP")

    def request_status(self) -> tuple[bool, str]:
        return self.link.send_line("ACT_STATUS")

    def get_rx_seq(self) -> int:
        with self._lock:
            return self._rx_seq

    def wait_for_status_line(self, timeout_sec: float, after_seq: int | None = None) -> str | None:
        if after_seq is None:
            with self._lock:
                start_seq = self._rx_seq
        else:
            start_seq = int(after_seq)

        deadline = time.monotonic() + max(0.05, float(timeout_sec))
        while time.monotonic() < deadline:
            with self._lock:
                for seq, line in reversed(self._rx_history):
                    if seq <= start_seq:
                        break
                    if "ACT_STAT" in line:
                        return line
            time.sleep(0.02)
        return None

    @staticmethod
    def top_limit_from_status(line: str | None) -> bool | None:
        if not line:
            return None
        m = re.search(r"TOP:(\d+)", line)
        if not m:
            return None
        return m.group(1) == "1"

    @staticmethod
    def bottom_limit_from_status(line: str | None) -> bool | None:
        if not line:
            return None
        m = re.search(r"BOTTOM:(\d+)", line)
        if not m:
            return None
        return m.group(1) == "1"

    @staticmethod
    def plant_pos_from_status(line: str | None) -> int | None:
        if not line:
            return None
        m = re.search(r"PLANT_POS:(-?\d+)", line)
        if not m:
            return None
        return int(m.group(1))

    def wait_for_plant_motion_complete(
        self,
        timeout_sec: float,
        poll_sec: float = 0.12,
        stable_samples: int = 3,
        require_motion: bool = True,
        initial_pos: int | None = None,
    ) -> bool:
        """
        Poll ACT_STATUS and wait until PLANT_POS becomes stable, indicating move completion.
        For step moves, callers usually require actual motion to be observed first to avoid
        false-complete when actuator has not started moving yet.
        """
        deadline = time.monotonic() + max(0.2, float(timeout_sec))
        poll_sec = max(0.05, float(poll_sec))
        stable_samples = max(1, int(stable_samples))

        last_pos: int | None = None
        stable_count = 0
        saw_motion = False

        while time.monotonic() < deadline:
            seq_before = self.get_rx_seq()
            ok, _ = self.request_status()
            if not ok:
                time.sleep(poll_sec)
                continue

            line = self.wait_for_status_line(max(0.25, poll_sec * 2), after_seq=seq_before)
            pos = self.plant_pos_from_status(line)
            if pos is None:
                time.sleep(poll_sec)
                continue

            if last_pos is None:
                last_pos = pos
                stable_count = 1
                time.sleep(poll_sec)
                continue

            if pos != last_pos:
                saw_motion = True
                stable_count = 0
                last_pos = pos
            else:
                stable_count += 1

            # If first sampled position is already different from pre-command position,
            # treat as motion observed (move may have completed before next poll).
            if initial_pos is not None and pos != initial_pos:
                saw_motion = True

            # Accept "done" when position has settled.
            if stable_count >= stable_samples and (saw_motion or not require_motion):
                return True

            time.sleep(poll_sec)

        return False

    def wait_for_plant_motion_start(
        self,
        timeout_sec: float,
        initial_pos: int | None = None,
        min_delta: int = 2,
        poll_sec: float = 0.1,
    ) -> bool:
        """
        Wait until PLANT_POS changes by at least min_delta from initial_pos.
        Returns False on timeout or when position is unavailable.
        """
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        poll_sec = max(0.05, float(poll_sec))
        min_delta = max(1, int(min_delta))
        base_pos = initial_pos

        while time.monotonic() < deadline:
            seq_before = self.get_rx_seq()
            ok, _ = self.request_status()
            if not ok:
                time.sleep(poll_sec)
                continue

            line = self.wait_for_status_line(max(0.25, poll_sec * 2), after_seq=seq_before)
            pos = self.plant_pos_from_status(line)
            if pos is None:
                time.sleep(poll_sec)
                continue

            if base_pos is None:
                base_pos = pos
            elif abs(pos - base_pos) >= min_delta:
                return True

            time.sleep(poll_sec)

        return False

    def wait_for_event(self, keywords: tuple[str, ...], timeout_sec: float, after_seq: int | None = None) -> bool:
        """
        Wait for any incoming actuator line containing one of the keywords.
        If after_seq is provided, only lines with seq > after_seq are considered.
        """
        keys = tuple(k for k in keywords if k)
        if not keys:
            return False

        if after_seq is None:
            with self._lock:
                start_seq = self._rx_seq
        else:
            start_seq = int(after_seq)

        deadline = time.monotonic() + max(0.05, float(timeout_sec))
        while time.monotonic() < deadline:
            with self._lock:
                for seq, line in reversed(self._rx_history):
                    if seq <= start_seq:
                        break
                    if any(k in line for k in keys):
                        return True
            time.sleep(0.02)
        return False

    def get_status(self) -> dict:
        with self._lock:
            return {
                "connected": self.link.connected,
                "port": self.link.port,
                "last_message": self.last_message,
                "servo_cam_deg": self.servo_cam_deg,
                "servo_plant_deg": self.servo_plant_deg,
                "protocol_version": self.protocol_version,
                "protocol_caps_known": self._caps_known,
                "protocol_caps": sorted(self.protocol_caps) if self._caps_known else [],
            }

    def close(self) -> None:
        self.link.close()

"""Drive ESP32 client: send velocity + parse encoder telemetry."""

from __future__ import annotations

import logging
import threading
import math
import time
import re
from collections import deque

from app import config
from app.infrastructure.serial_link import SerialLink
from app.models import DriveTelemetry

logger = logging.getLogger(__name__)


class DriveClient:
    def __init__(self):
        self.link = SerialLink(config.SERIAL.drive_port, config.SERIAL.drive_baud, "drive")
        self.telemetry = DriveTelemetry()
        self._lock = threading.Lock()
        self._null_line_count = 0
        self._stat_line_count = 0
        self._last_stat_ts = 0.0
        self._rx_seq = 0
        self._rx_history: deque[tuple[int, str]] = deque(maxlen=400)
        self.protocol_version = "legacy"
        self.protocol_caps: set[str] = set()
        self.protocol_raw = ""
        self._caps_known = False
        self.link.add_listener(self._on_line)

    def open(self) -> bool:
        ok = self.link.open()
        if ok:
            self._probe_protocol()
        return ok

    def _find_line_after(
        self,
        after_seq: int,
        contains: tuple[str, ...],
        timeout_sec: float,
        *,
        startswith: tuple[str, ...] = (),
        exclude_contains: tuple[str, ...] = (),
    ) -> str | None:
        needles = tuple(c.upper() for c in contains if c)
        prefixes = tuple(c.upper() for c in startswith if c)
        excluded = tuple(c.upper() for c in exclude_contains if c)
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline:
            with self._lock:
                for seq, line in reversed(self._rx_history):
                    if seq <= after_seq:
                        break
                    upper = line.upper()
                    if prefixes and not any(upper.startswith(prefix) for prefix in prefixes):
                        continue
                    if excluded and any(token in upper for token in excluded):
                        continue
                    if any(n in upper for n in needles):
                        return line
            time.sleep(0.02)
        return None

    @staticmethod
    def _parse_caps_csv(value: str) -> set[str]:
        known = {"VEL", "ODO_RESET", "STOP", "STATUS", "IMU_STATUS", "PROTO_VER", "HELP"}
        caps = set()
        for tok in value.replace(" ", "").split(","):
            norm = tok.strip().upper()
            if norm in known:
                caps.add(norm)
        return caps

    def _probe_protocol(self) -> None:
        """
        Best-effort protocol/capability probe.
        Backward compatible: unknown legacy firmware keeps previous behavior.
        """
        self.protocol_version = "legacy"
        self.protocol_caps = set()
        self.protocol_raw = ""
        self._caps_known = False

        self.link.reset_input_buffer()
        seq0 = self.get_rx_seq()
        ok, _ = self.link.send_line("PROTO_VER")
        if ok:
            line = self._find_line_after(
                seq0,
                ("DRV_PROTO", "DRV_ERR UNKNOWN:PROTO_VER"),
                3.0,
                startswith=("DRV_PROTO", "DRV_ERR"),
                exclude_contains=("DRV_STAT",),
            )
            if line and line.upper().startswith("DRV_PROTO"):
                self.protocol_raw = line
                m_ver = re.search(r"VER[:=]\s*([0-9A-Z._-]+)", line, flags=re.IGNORECASE)
                if m_ver:
                    self.protocol_version = m_ver.group(1)
                m_caps = re.search(r"CAPS[:=]\s*(.+)$", line, flags=re.IGNORECASE)
                if m_caps:
                    self.protocol_caps = self._parse_caps_csv(m_caps.group(1))
                    self._caps_known = True
                logger.info("Drive protocol detected: ver=%s caps=%s", self.protocol_version, sorted(self.protocol_caps))
                return

        self.link.reset_input_buffer()
        seq1 = self.get_rx_seq()
        ok, _ = self.link.send_line("HELP")
        if ok:
            line = self._find_line_after(
                seq1,
                ("COMMANDS:", "DRV_ERR UNKNOWN:HELP"),
                3.0,
                startswith=("COMMANDS:", "DRV_ERR"),
                exclude_contains=("DRV_STAT",),
            )
            if line and "COMMANDS:" in line.upper():
                up = line.upper()
                caps: set[str] = set()
                if "VEL" in up:
                    caps.add("VEL")
                if "ODO RESET" in up:
                    caps.add("ODO_RESET")
                if "STOP" in up:
                    caps.add("STOP")
                if "STATUS" in up:
                    caps.add("STATUS")
                if "IMU STATUS" in up:
                    caps.add("IMU_STATUS")
                if caps:
                    self.protocol_version = "legacy-help"
                    self.protocol_caps = caps
                    self._caps_known = True
                    logger.info("Drive capabilities inferred from HELP: %s", sorted(self.protocol_caps))
                    return

        logger.info("Drive protocol probe fallback: legacy (capabilities unknown)")

    def supports(self, capability: str) -> bool:
        cap = capability.strip().upper()
        if not cap:
            return True
        if not self._caps_known:
            return True
        return cap in self.protocol_caps

    @staticmethod
    def _parse_stat_fields(line: str) -> dict[str, str]:
        """
        Parse key:value fields from lines like:
        DRV_STAT L_RPM:1.23 R_RPM:2.34 DIST_CM:12.3 L_COUNT:123 R_COUNT:456
        """
        idx = line.find("DRV_STAT")
        if idx < 0:
            return {}

        payload = line[idx:]
        fields: dict[str, str] = {}
        tokens = payload.replace(",", " ").split()
        for tok in tokens[1:]:
            sep = ":" if ":" in tok else "=" if "=" in tok else None
            if sep is None:
                continue
            key, value = tok.split(sep, 1)
            fields[key.strip().upper()] = value.strip()
        return fields

    @staticmethod
    def _safe_float(value: str | None, default: float) -> float:
        if value is None:
            return default
        try:
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _safe_int(value: str | None, default: int) -> int:
        if value is None:
            return default
        try:
            return int(float(value))
        except Exception:
            return default

    @staticmethod
    def _normalize_heading_deg(value: float) -> float:
        return float(value) % 360.0

    def _on_line(self, line: str) -> None:
        normalized = line.replace("\x00", "").strip()

        if not normalized:
            with self._lock:
                self._null_line_count += 1
                self.telemetry.raw_line = "<NUL_STREAM>"
            return

        with self._lock:
            self._rx_seq += 1
            self._rx_history.append((self._rx_seq, normalized))
            self.telemetry.raw_line = normalized

        fields = self._parse_stat_fields(normalized)
        if not fields:
            return

        with self._lock:
            left_rpm = self._safe_float(fields.get("L_RPM"), self.telemetry.left_rpm)
            right_rpm = self._safe_float(fields.get("R_RPM"), self.telemetry.right_rpm)
            distance_cm = self._safe_float(fields.get("DIST_CM"), self.telemetry.distance_cm)
            left_count = self._safe_int(fields.get("L_COUNT"), self.telemetry.left_count)
            right_count = self._safe_int(fields.get("R_COUNT"), self.telemetry.right_count)
            imu_connected = self._safe_int(fields.get("IMU"), 1 if self.telemetry.imu_connected else 0) != 0
            heading_deg_raw = fields.get("HEAD_DEG")
            heading_deg = self.telemetry.heading_deg
            if heading_deg_raw is not None:
                parsed_heading = self._safe_float(heading_deg_raw, -1.0)
                if parsed_heading < 0.0:
                    heading_deg = None
                else:
                    heading_deg = self._normalize_heading_deg(parsed_heading + float(config.IMU.heading_offset_deg))
                    if bool(config.IMU.invert_heading):
                        heading_deg = self._normalize_heading_deg(360.0 - heading_deg)
            imu_cal_sys = self._safe_int(fields.get("CAL_SYS"), self.telemetry.imu_cal_sys)
            imu_cal_gyro = self._safe_int(fields.get("CAL_GYRO"), self.telemetry.imu_cal_gyro)
            imu_cal_accel = self._safe_int(fields.get("CAL_ACCEL"), self.telemetry.imu_cal_accel)
            imu_cal_mag = self._safe_int(fields.get("CAL_MAG"), self.telemetry.imu_cal_mag)

            # Fallback:
            # If DIST_CM is near zero due left/right sign cancellation,
            # derive traveled distance from absolute encoder counts.
            wheel_circ_cm = float(config.MOTION.wheel_diameter_inch) * 2.54 * math.pi
            cpr = max(1, int(config.MOTION.counts_per_rev))
            dist_abs_cm = ((abs(left_count) + abs(right_count)) * 0.5 / cpr) * wheel_circ_cm
            if abs(distance_cm) < 0.5 and dist_abs_cm > 0.5:
                distance_cm = dist_abs_cm

            self.telemetry.left_rpm = left_rpm
            self.telemetry.right_rpm = right_rpm
            self.telemetry.distance_cm = distance_cm
            self.telemetry.left_count = left_count
            self.telemetry.right_count = right_count
            self.telemetry.imu_connected = imu_connected
            self.telemetry.heading_deg = heading_deg
            self.telemetry.imu_cal_sys = imu_cal_sys
            self.telemetry.imu_cal_gyro = imu_cal_gyro
            self.telemetry.imu_cal_accel = imu_cal_accel
            self.telemetry.imu_cal_mag = imu_cal_mag
            self._stat_line_count += 1
            self._last_stat_ts = time.monotonic()

    def send_velocity(self, left_cmd: int, right_cmd: int) -> tuple[bool, str]:
        left_cmd = max(-config.VISION.max_cmd, min(config.VISION.max_cmd, int(left_cmd)))
        right_cmd = max(-config.VISION.max_cmd, min(config.VISION.max_cmd, int(right_cmd)))
        return self.link.send_line(f"VEL L:{left_cmd} R:{right_cmd}")

    def stop(self) -> tuple[bool, str]:
        return self.send_velocity(0, 0)

    def reset_odometry(self) -> tuple[bool, str]:
        return self.link.send_line("ODO RESET")

    def request_status(self) -> tuple[bool, str]:
        # Prefer explicit STATUS when supported.
        if self.supports("STATUS"):
            return self.link.send_line("STATUS")
        # Legacy fallback: rely on periodic DRV_STAT streaming.
        return True, "status-stream-only"

    def get_rx_seq(self) -> int:
        with self._lock:
            return self._rx_seq

    def wait_for_status_line(self, timeout_sec: float, after_seq: int | None = None) -> str | None:
        if after_seq is None:
            with self._lock:
                start_seq = self._rx_seq
        else:
            start_seq = int(after_seq)
        return self._find_line_after(start_seq, ("DRV_STAT",), timeout_sec, startswith=("DRV_STAT",))

    def get_telemetry(self) -> DriveTelemetry:
        with self._lock:
            return DriveTelemetry(
                left_rpm=self.telemetry.left_rpm,
                right_rpm=self.telemetry.right_rpm,
                distance_cm=self.telemetry.distance_cm,
                left_count=self.telemetry.left_count,
                right_count=self.telemetry.right_count,
                imu_connected=self.telemetry.imu_connected,
                heading_deg=self.telemetry.heading_deg,
                imu_cal_sys=self.telemetry.imu_cal_sys,
                imu_cal_gyro=self.telemetry.imu_cal_gyro,
                imu_cal_accel=self.telemetry.imu_cal_accel,
                imu_cal_mag=self.telemetry.imu_cal_mag,
                raw_line=self.telemetry.raw_line,
            )

    def get_heading_deg(self) -> float | None:
        with self._lock:
            return self.telemetry.heading_deg

    def get_imu_status(self) -> dict:
        with self._lock:
            return {
                "source": "drive_esp32",
                "connected": bool(self.telemetry.imu_connected),
                "heading_deg": self.telemetry.heading_deg,
                "calibration_status": {
                    "sys": int(self.telemetry.imu_cal_sys),
                    "gyro": int(self.telemetry.imu_cal_gyro),
                    "accel": int(self.telemetry.imu_cal_accel),
                    "mag": int(self.telemetry.imu_cal_mag),
                },
            }

    def get_status(self) -> dict:
        with self._lock:
            stat_age_sec = None
            if self._last_stat_ts > 0:
                stat_age_sec = max(0.0, time.monotonic() - self._last_stat_ts)
            return {
                "connected": self.link.connected,
                "port": self.link.port,
                "raw_line": self.telemetry.raw_line,
                "recent_lines": self.link.recent_lines()[-10:],
                "stat_line_count": self._stat_line_count,
                "null_line_count": self._null_line_count,
                "last_stat_age_sec": stat_age_sec,
                "protocol_version": self.protocol_version,
                "protocol_caps_known": self._caps_known,
                "protocol_caps": sorted(self.protocol_caps) if self._caps_known else [],
                "imu_connected": bool(self.telemetry.imu_connected),
                "heading_deg": self.telemetry.heading_deg,
                "imu_cal_sys": int(self.telemetry.imu_cal_sys),
                "imu_cal_gyro": int(self.telemetry.imu_cal_gyro),
                "imu_cal_accel": int(self.telemetry.imu_cal_accel),
                "imu_cal_mag": int(self.telemetry.imu_cal_mag),
            }

    def close(self) -> None:
        self.link.close()

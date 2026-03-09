"""Planting sequence orchestration on top of StepperServoClient."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable

from app import config
from app.controllers.stepper_servo_client import StepperServoClient

logger = logging.getLogger(__name__)


class PlantingSequence:
    def __init__(
        self,
        actuator: StepperServoClient,
        set_status: Callable[[str], None] | None = None,
        should_stop: Callable[[], bool] | None = None,
    ):
        self.actuator = actuator
        self.last_error = ""
        self._set_status_cb = set_status
        self._should_stop_cb = should_stop

    def _set_status(self, text: str) -> None:
        if self._set_status_cb is not None:
            self._set_status_cb(str(text))

    def _fail(self, reason: str) -> bool:
        self.last_error = reason
        logger.error("Planting failed: %s", reason)
        return False

    def _is_cancelled(self) -> bool:
        if self._should_stop_cb is None:
            return False
        try:
            return bool(self._should_stop_cb())
        except Exception:
            return False

    def _sleep_with_cancel(self, total_sec: float) -> bool:
        """
        Sleep in short slices and abort early when stop is requested.
        """
        deadline = time.monotonic() + max(0.0, float(total_sec))
        while time.monotonic() < deadline:
            if self._is_cancelled():
                self.actuator.stop_all()
                return self._fail("cancelled by operator")
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return True

    def _read_plant_pos(self) -> int | None:
        seq_before_status = self.actuator.get_rx_seq()
        ok, _ = self.actuator.request_status()
        if not ok:
            return None
        status_line = self.actuator.wait_for_status_line(0.45, after_seq=seq_before_status)
        return self.actuator.plant_pos_from_status(status_line)

    def _wait_bottom_limit(self, context: str) -> bool:
        timeout_sec = float(config.ACTUATOR.plant_limit_timeout_sec)
        grace_sec = max(0.0, float(config.ACTUATOR.plant_limit_grace_sec))
        seq_before_down = self.actuator.get_rx_seq()
        ok, msg = self.actuator.plant_down_until_bottom()
        if not ok:
            return self._fail(f"{context}: plant_down_until_bottom failed: {msg}")
        deadline = time.monotonic() + timeout_sec
        stuck_since = None
        switched_once = False
        while time.monotonic() < deadline:
            if self._is_cancelled():
                self.actuator.stop_all()
                return self._fail("cancelled by operator")
            if self.actuator.wait_for_event(("ACT_EVENT PLANT_HIT_BOTTOM",), 0.25, after_seq=seq_before_down):
                return True
            seq_before_status = self.actuator.get_rx_seq()
            ok, _ = self.actuator.request_status()
            if ok:
                status_line = self.actuator.wait_for_status_line(0.35, after_seq=seq_before_status)
                if self.actuator.bottom_limit_from_status(status_line) is True:
                    logger.warning("%s: no event but BOTTOM=1 from ACT_STATUS -> continue", context)
                    return True
                # If TOP keeps active while trying to go DOWN, likely direction mismatch.
                if self.actuator.top_limit_from_status(status_line) is True and not switched_once:
                    now = time.monotonic()
                    if stuck_since is None:
                        stuck_since = now
                    elif now - stuck_since >= 1.5:
                        logger.warning(
                            "%s: TOP stayed active while seeking BOTTOM, retry with PLANT UP_LIMIT once",
                            context,
                        )
                        self.actuator.plant_up_until_top()
                        switched_once = True
                else:
                    stuck_since = None
            time.sleep(0.03)

        # Fallback 1: event may be missed/late; verify live limit state once.
        seq_before_status = self.actuator.get_rx_seq()
        ok, msg = self.actuator.request_status()
        if ok:
            status_line = self.actuator.wait_for_status_line(0.6, after_seq=seq_before_status)
            if self.actuator.bottom_limit_from_status(status_line) is True:
                logger.warning("%s: timeout on event, but BOTTOM=1 from ACT_STATUS -> continue", context)
                return True

        # Fallback 2: grace wait window for slow travel to limit.
        if grace_sec > 0:
            logger.warning("%s: bottom not reached in %.1fs, entering grace %.1fs", context, timeout_sec, grace_sec)
            deadline = time.monotonic() + grace_sec
            while time.monotonic() < deadline:
                if self._is_cancelled():
                    self.actuator.stop_all()
                    return self._fail("cancelled by operator")
                if self.actuator.wait_for_event(("ACT_EVENT PLANT_HIT_BOTTOM",), 0.25, after_seq=seq_before_down):
                    logger.warning("%s: reached bottom during grace window", context)
                    return True
                seq_before_status = self.actuator.get_rx_seq()
                ok, msg = self.actuator.request_status()
                if ok:
                    status_line = self.actuator.wait_for_status_line(0.35, after_seq=seq_before_status)
                    if self.actuator.bottom_limit_from_status(status_line) is True:
                        logger.warning("%s: BOTTOM=1 during grace window", context)
                        return True
                time.sleep(0.05)

        self.actuator.stop_all()
        return self._fail(
            f"{context}: wait bottom limit timeout after {timeout_sec + grace_sec:.1f}s "
            f"(last={self.actuator.last_message})"
        )

    def _wait_top_limit(self, context: str) -> bool:
        timeout_sec = float(config.ACTUATOR.plant_limit_timeout_sec)
        grace_sec = max(0.0, float(config.ACTUATOR.plant_limit_grace_sec))
        seq_before_top = self.actuator.get_rx_seq()
        ok, msg = self.actuator.plant_up_until_top()
        if not ok:
            return self._fail(f"{context}: plant_up_until_top failed: {msg}")
        deadline = time.monotonic() + timeout_sec
        stuck_since = None
        switched_once = False
        while time.monotonic() < deadline:
            if self._is_cancelled():
                self.actuator.stop_all()
                return self._fail("cancelled by operator")
            if self.actuator.wait_for_event(("ACT_EVENT PLANT_HIT_TOP",), 0.25, after_seq=seq_before_top):
                return True
            seq_before_status = self.actuator.get_rx_seq()
            ok, _ = self.actuator.request_status()
            if ok:
                status_line = self.actuator.wait_for_status_line(0.35, after_seq=seq_before_status)
                if self.actuator.top_limit_from_status(status_line) is True:
                    logger.warning("%s: no event but TOP=1 from ACT_STATUS -> continue", context)
                    return True
                # If BOTTOM keeps active while trying to go UP, likely direction mismatch.
                if self.actuator.bottom_limit_from_status(status_line) is True and not switched_once:
                    now = time.monotonic()
                    if stuck_since is None:
                        stuck_since = now
                    elif now - stuck_since >= 1.5:
                        logger.warning(
                            "%s: BOTTOM stayed active while seeking TOP, retry with PLANT DOWN_LIMIT once",
                            context,
                        )
                        self.actuator.plant_down_until_bottom()
                        switched_once = True
                else:
                    stuck_since = None
            time.sleep(0.03)

        # Fallback 1: event may be missed/late; verify live limit state once.
        seq_before_status = self.actuator.get_rx_seq()
        ok, msg = self.actuator.request_status()
        if ok:
            status_line = self.actuator.wait_for_status_line(0.6, after_seq=seq_before_status)
            if self.actuator.top_limit_from_status(status_line) is True:
                logger.warning("%s: timeout on event, but TOP=1 from ACT_STATUS -> continue", context)
                return True

        # Fallback 2: grace wait window for slow travel to limit.
        if grace_sec > 0:
            logger.warning("%s: top not reached in %.1fs, entering grace %.1fs", context, timeout_sec, grace_sec)
            deadline = time.monotonic() + grace_sec
            while time.monotonic() < deadline:
                if self._is_cancelled():
                    self.actuator.stop_all()
                    return self._fail("cancelled by operator")
                if self.actuator.wait_for_event(("ACT_EVENT PLANT_HIT_TOP",), 0.25, after_seq=seq_before_top):
                    logger.warning("%s: reached top during grace window", context)
                    return True
                seq_before_status = self.actuator.get_rx_seq()
                ok, msg = self.actuator.request_status()
                if ok:
                    status_line = self.actuator.wait_for_status_line(0.35, after_seq=seq_before_status)
                    if self.actuator.top_limit_from_status(status_line) is True:
                        logger.warning("%s: TOP=1 during grace window", context)
                        return True
                time.sleep(0.05)

        self.actuator.stop_all()
        return self._fail(
            f"{context}: wait top limit timeout after {timeout_sec + grace_sec:.1f}s "
            f"(last={self.actuator.last_message})"
        )

    def _up_step(self, context: str) -> bool:
        if self._is_cancelled():
            self.actuator.stop_all()
            return self._fail("cancelled by operator")
        if not self.actuator.supports("PLANT_UP_STEP"):
            logger.warning("%s: firmware capability PLANT_UP_STEP unavailable, fallback to UP_LIMIT", context)
            return self._wait_top_limit(f"{context} unsupported-fallback")

        initial_pos = self._read_plant_pos()
        ok, msg = self.actuator.plant_up_step(config.ACTUATOR.step1_after_plant_deg)
        if not ok:
            return self._fail(f"{context}: plant_up_step failed: {msg}")

        # Fast fallback: if no movement starts shortly after command, do not wait full timeout.
        start_timeout = min(2.0, max(0.5, float(config.ACTUATOR.plant_up_step_timeout_sec) * 0.15))
        if not self.actuator.wait_for_plant_motion_start(start_timeout, initial_pos=initial_pos):
            logger.warning(
                "%s: no visible plant motion after UP_STEP (%.1fs), fallback to UP_LIMIT (last=%s)",
                context,
                start_timeout,
                self.actuator.last_message,
            )
            return self._wait_top_limit(f"{context} no-start-fallback")

        # Ensure UP_STEP completes before moving to the next process.
        if self.actuator.wait_for_plant_motion_complete(
            config.ACTUATOR.plant_up_step_timeout_sec,
            initial_pos=initial_pos,
        ):
            return True

        # Fallback: if top limit is already hit, accept as completed.
        seq_before_status = self.actuator.get_rx_seq()
        ok, _ = self.actuator.request_status()
        if ok:
            status_line = self.actuator.wait_for_status_line(0.6, after_seq=seq_before_status)
            if self.actuator.top_limit_from_status(status_line) is True:
                logger.warning("%s: UP_STEP timeout but TOP=1, treat as complete", context)
                return True

        logger.warning(
            "%s: UP_STEP completion timeout, fallback to UP_LIMIT (last=%s)",
            context,
            self.actuator.last_message,
        )
        return self._wait_top_limit(f"{context} timeout-fallback")

    def _down_step(self, context: str) -> bool:
        if self._is_cancelled():
            self.actuator.stop_all()
            return self._fail("cancelled by operator")
        if not self.actuator.supports("PLANT_DOWN_STEP"):
            logger.warning("%s: firmware capability PLANT_DOWN_STEP unavailable, fallback to DOWN_LIMIT", context)
            return self._wait_bottom_limit(f"{context} unsupported-fallback")

        initial_pos = self._read_plant_pos()
        seq_before_cmd = self.actuator.get_rx_seq()
        ok, msg = self.actuator.plant_down_step(config.ACTUATOR.step1_after_burried_deg)
        if not ok:
            return self._fail(f"{context}: plant_down_step failed: {msg}")

        # Immediate compatibility check: some firmwares do not implement PLANT DOWN_STEP.
        if self.actuator.wait_for_event(("ACT_ERR UNKNOWN:PLANT DOWN_STEP",), 0.45, after_seq=seq_before_cmd):
            logger.warning(
                "%s: firmware reports UNKNOWN for DOWN_STEP, fallback to DOWN_LIMIT",
                context,
            )
            return self._wait_bottom_limit(f"{context} unsupported-fallback")

        # Fast fallback: if no movement starts shortly after command, do not wait full timeout.
        start_timeout = min(2.0, max(0.5, float(config.ACTUATOR.plant_up_step_timeout_sec) * 0.15))
        if not self.actuator.wait_for_plant_motion_start(start_timeout, initial_pos=initial_pos):
            logger.warning(
                "%s: no visible plant motion after DOWN_STEP (%.1fs), fallback to DOWN_LIMIT (last=%s)",
                context,
                start_timeout,
                self.actuator.last_message,
            )
            return self._wait_bottom_limit(f"{context} no-start-fallback")

        # Primary: wait finite down-step completion.
        if self.actuator.wait_for_plant_motion_complete(
            config.ACTUATOR.plant_up_step_timeout_sec,
            initial_pos=initial_pos,
        ):
            return True

        # If no motion-complete signal, check if bottom is already active.
        seq_before_status = self.actuator.get_rx_seq()
        ok, _ = self.actuator.request_status()
        if ok:
            status_line = self.actuator.wait_for_status_line(0.6, after_seq=seq_before_status)
            if self.actuator.bottom_limit_from_status(status_line) is True:
                logger.warning("%s: down-step timeout but BOTTOM=1, treat as complete", context)
                return True

        # Backward compatibility: old firmware may not support PLANT DOWN_STEP.
        if "UNKNOWN:PLANT DOWN_STEP" in str(self.actuator.last_message).upper():
            logger.warning("%s: firmware does not support PLANT DOWN_STEP, fallback to DOWN_LIMIT", context)
            return self._wait_bottom_limit(f"{context} fallback")

        # Runtime fallback: command accepted but no completion observed.
        logger.warning(
            "%s: down-step timeout without bottom hit, fallback to DOWN_LIMIT (last=%s)",
            context,
            self.actuator.last_message,
        )
        return self._wait_bottom_limit(f"{context} timeout-fallback")

    def run_once(self) -> bool:
        logger.info("Planting sequence started")
        self.last_error = ""
        if self._is_cancelled():
            self.actuator.stop_all()
            return self._fail("cancelled by operator")

        if str(self.actuator.last_message).strip().upper().startswith("DRV_"):
            return self._fail(
                "actuator stream looks like DRIVE stream; check serial mapping of drive/actuator ESP32"
            )

        self._set_status("Planting: precheck top limit")
        # 0) Ensure plant axis is at top limit before starting this cycle.
        seq_before_status = self.actuator.get_rx_seq()
        ok, msg = self.actuator.request_status()
        if not ok:
            return self._fail(f"request_status failed: {msg}")

        status_line = self.actuator.wait_for_status_line(0.5, after_seq=seq_before_status)
        top_pressed = self.actuator.top_limit_from_status(status_line)
        if top_pressed is not True:
            if not self._wait_top_limit("precheck"):
                return False

        self._set_status("Planting: feed")
        # 1) Feed step + delay
        ok, msg = self.actuator.feed_step(config.ACTUATOR.feed_step_deg)
        if not ok:
            return self._fail(f"feed_step failed: {msg}")
        if not self._sleep_with_cancel(config.ACTUATOR.feed_delay_sec):
            return False

        self._set_status("Planting: phase1 down")
        # 2) Plant axis down to bottom limit
        if not self._wait_bottom_limit("phase1"):
            return False

        self._set_status("Planting: phase1 up step")
        # 3) Up by configured step + delay 2s
        if not self._up_step("phase1"):
            return False
        if not self._sleep_with_cancel(config.ACTUATOR.plant_retract_delay_sec):
            return False

        self._set_status("Planting: servo open")
        # 4) Open servo -> delay -> down to bottom -> close -> open -> up by step -> close servo
        ok, msg = self.actuator.set_servo_plant(config.ACTUATOR.servo_plant_degree)
        if not ok:
            return self._fail(f"set_servo_plant(open) failed: {msg}")
        if not self._sleep_with_cancel(config.ACTUATOR.plant_servo_delay_sec):
            return False

        self._set_status("Planting: phase2 down step")
        if not self._down_step("phase2"):
            return False

        self._set_status("Planting: servo close/open")
        ok, msg = self.actuator.set_servo_plant(config.ACTUATOR.servo_plant_after_deg)
        if not ok:
            return self._fail(f"set_servo_plant(close@bottom) failed: {msg}")
        if not self._sleep_with_cancel(config.ACTUATOR.plant_servo_delay_sec):
            return False

        ok, msg = self.actuator.set_servo_plant(config.ACTUATOR.servo_plant_degree)
        if not ok:
            return self._fail(f"set_servo_plant(reopen) failed: {msg}")

        self._set_status("Planting: phase2 up step")
        if not self._up_step("phase2"):
            return False
        # Let phase2 up-step finish before issuing final up-to-top command.
        if not self._sleep_with_cancel(config.ACTUATOR.plant_retract_delay_sec):
            return False

        self._set_status("Planting: final close and return top")
        ok, msg = self.actuator.set_servo_plant(config.ACTUATOR.servo_plant_start_deg)
        if not ok:
            return self._fail(f"set_servo_plant(close) failed: {msg}")

        # 5) Return plant axis to top limit
        if not self._wait_top_limit("finish"):
            return False

        logger.info("Planting sequence completed")
        return True

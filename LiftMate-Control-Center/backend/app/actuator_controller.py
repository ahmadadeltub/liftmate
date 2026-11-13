"""Upper (100mm) and Lower (30mm) Linear Actuator Group Controller."""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Deque, Dict, Tuple

from .config import ConfigManager
from .diagnostics import DiagnosticsManager
from .imu_controller import IMUController
from .safety_controller import SafetyController
from .serial_controller import SerialController


class ActuatorController:
    """Controls the Upper (4x 100mm) and Lower (4x 30mm) grouped linear actuators safely."""

    def __init__(
        self,
        config: ConfigManager,
        diagnostics: DiagnosticsManager,
        safety: SafetyController,
        serial_ctrl: SerialController,
        imu: IMUController,
    ) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics
        self.safety = safety
        self.serial = serial_ctrl
        self.imu = imu

        self.upper_pwm: int = 0
        self.lower_pwm: int = 0
        self.upper_direction: str = "STOPPED"
        self.lower_direction: str = "STOPPED"
        self.upper_last_cmd_ts: float = 0.0
        self.lower_last_cmd_ts: float = 0.0
        self.upper_start_ts: float = 0.0
        self.lower_start_ts: float = 0.0

        self._upper_history: Deque[Dict[str, Any]] = deque(maxlen=30)
        self._lower_history: Deque[Dict[str, Any]] = deque(maxlen=30)
        self._pulse_timers: Dict[str, threading.Timer | None] = {"UPPER": None, "LOWER": None}

        self.safety.register_estop_callback(self.stop_all_actuators)
        self.serial.register_timeout_callback(self._on_firmware_timeout)

        self._wdt_running = True
        self._wdt_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._wdt_thread.start()

    def _on_firmware_timeout(self) -> None:
        with self._lock:
            self.upper_pwm = 0
            self.lower_pwm = 0
            self.upper_direction = "STOPPED"
            self.lower_direction = "STOPPED"

    def _watchdog_loop(self) -> None:
        while self._wdt_running:
            time.sleep(0.05)
            now = time.time()
            with self._lock:
                wdt_ms = float(self.config.get_section("safety").get("actuator_watchdog_ms", 600))
                if self.upper_pwm != 0 and (now - self.upper_last_cmd_ts) * 1000.0 >= wdt_ms:
                    self.diagnostics.log_event("WARNING", "ACTUATORS", "UPPER_WATCHDOG_STOP", "Hold release/timeout")
                    self.stop_group("UPPER")
                if self.lower_pwm != 0 and (now - self.lower_last_cmd_ts) * 1000.0 >= wdt_ms:
                    self.diagnostics.log_event("WARNING", "ACTUATORS", "LOWER_WATCHDOG_STOP", "Hold release/timeout")
                    self.stop_group("LOWER")

    def _cancel_pulse_timer(self, group: str) -> None:
        t = self._pulse_timers.get(group)
        if t is not None:
            t.cancel()
            self._pulse_timers[group] = None

    def stop_group(self, group: str) -> Tuple[bool, str]:
        grp = group.upper()
        if grp not in ("UPPER", "LOWER"):
            return False, "Group must be UPPER or LOWER."
        with self._lock:
            self._cancel_pulse_timer(grp)
            if grp == "UPPER":
                self.upper_pwm = 0
                self.upper_direction = "STOPPED"
                self.upper_start_ts = 0.0
            else:
                self.lower_pwm = 0
                self.lower_direction = "STOPPED"
                self.lower_start_ts = 0.0

            ok, resp = False, "DISCONNECTED"
            if self.serial.connected:
                ok, resp = self.serial.send_command("JACK", grp, 0)
            if self.upper_pwm == 0 and self.lower_pwm == 0:
                self.safety.notify_motion_stopped(f"{grp} actuator stopped")
            return ok, resp

    def stop_all_actuators(self) -> Tuple[bool, str]:
        with self._lock:
            self._cancel_pulse_timer("UPPER")
            self._cancel_pulse_timer("LOWER")
            self.upper_pwm = 0
            self.lower_pwm = 0
            self.upper_direction = "STOPPED"
            self.lower_direction = "STOPPED"
            self.upper_start_ts = 0.0
            self.lower_start_ts = 0.0
            ok, resp = False, "DISCONNECTED"
            if self.serial.connected:
                ok, resp = self.serial.send_command("ACT", 0, 0)
            self.safety.notify_motion_stopped("All actuators stopped")
            return ok, resp

    def command_group(
        self,
        group: str,
        action: str,
        pwm: int = 120,
        duration_ms: int | None = None,
    ) -> Tuple[bool, str]:
        """Commands UPPER or LOWER actuator group (EXTEND, RETRACT, STOP)."""
        grp = group.strip().upper()
        act = action.strip().upper()
        if grp not in ("UPPER", "LOWER"):
            return False, "Actuator group must be UPPER or LOWER."

        if act == "STOP" or pwm == 0:
            return self.stop_group(grp)

        if act not in ("EXTEND", "RETRACT"):
            return False, "Action must be EXTEND, RETRACT, or STOP."

        allowed, reason = self.safety.can_move_actuator(grp, imu_status=self.imu.get_status())
        if not allowed:
            return False, reason

        act_cfg = self.config.get_section("actuators")
        grp_cfg = act_cfg.get(grp.lower(), {})
        max_pwm = int(grp_cfg.get("max_pwm", 255))
        inverted = bool(grp_cfg.get("inverted", False))

        mag = max(0, min(abs(int(pwm)), max_pwm, 255))
        sign = 1 if act == "EXTEND" else -1
        if inverted:
            sign *= -1
        signed_pwm = sign * mag

        now = time.time()
        with self._lock:
            self._cancel_pulse_timer(grp)

            # Enforce single-group mutual exclusion unless explicitly unlocked
            if not act_cfg.get("allow_simultaneous_groups", False):
                if grp == "UPPER" and self.lower_pwm != 0:
                    self.stop_group("LOWER")
                elif grp == "LOWER" and self.upper_pwm != 0:
                    self.stop_group("UPPER")

            ok, resp = self.serial.send_command("JACK", grp, signed_pwm)
            if ok:
                if grp == "UPPER":
                    if self.upper_pwm == 0:
                        self.upper_start_ts = now
                    self.upper_pwm = signed_pwm
                    self.upper_direction = act
                    self.upper_last_cmd_ts = now
                    self._upper_history.appendleft({
                        "ts": time.strftime("%H:%M:%S"),
                        "action": act,
                        "pwm": signed_pwm,
                        "pulse_ms": duration_ms,
                    })
                else:
                    if self.lower_pwm == 0:
                        self.lower_start_ts = now
                    self.lower_pwm = signed_pwm
                    self.lower_direction = act
                    self.lower_last_cmd_ts = now
                    self._lower_history.appendleft({
                        "ts": time.strftime("%H:%M:%S"),
                        "action": act,
                        "pwm": signed_pwm,
                        "pulse_ms": duration_ms,
                    })

                self.safety.notify_actuating_started(grp)
                if duration_ms is not None and duration_ms > 0:
                    pulse_s = max(0.08, min(float(duration_ms) / 1000.0, 2.0))
                    timer = threading.Timer(pulse_s, self.stop_group, args=(grp,))
                    timer.daemon = True
                    self._pulse_timers[grp] = timer
                    timer.start()
            return ok, resp

    def update_settings(
        self,
        driver_electrical_verified: bool | None = None,
        allow_simultaneous_groups: bool | None = None,
        require_imu_for_lifting: bool | None = None,
        upper_inverted: bool | None = None,
        lower_inverted: bool | None = None,
    ) -> Dict[str, Any]:
        act_cfg = self.config.get_section("actuators")
        if driver_electrical_verified is not None:
            act_cfg["driver_electrical_verified"] = bool(driver_electrical_verified)
        if allow_simultaneous_groups is not None:
            act_cfg["allow_simultaneous_groups"] = bool(allow_simultaneous_groups)
            if self.serial.connected:
                self.serial.send_command("DUAL_JACK", 1 if allow_simultaneous_groups else 0)
        if require_imu_for_lifting is not None:
            act_cfg["require_imu_for_lifting"] = bool(require_imu_for_lifting)
        if upper_inverted is not None:
            act_cfg.setdefault("upper", {})["inverted"] = bool(upper_inverted)
        if lower_inverted is not None:
            act_cfg.setdefault("lower", {})["inverted"] = bool(lower_inverted)

        self.config.update_section("actuators", act_cfg)
        self.diagnostics.log_event("INFO", "ACTUATORS", "SETTINGS_UPDATED", str(act_cfg))
        return self.get_status()

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            now = time.time()
            act_cfg = self.config.get_section("actuators")
            u_cfg = act_cfg.get("upper", {})
            l_cfg = act_cfg.get("lower", {})
            pins = self.config.get_section("pins")

            u_dur = round(now - self.upper_start_ts, 2) if self.upper_pwm != 0 and self.upper_start_ts > 0 else 0.0
            l_dur = round(now - self.lower_start_ts, 2) if self.lower_pwm != 0 and self.lower_start_ts > 0 else 0.0

            return {
                "driver_electrical_verified": bool(act_cfg.get("driver_electrical_verified", False)),
                "allow_simultaneous_groups": bool(act_cfg.get("allow_simultaneous_groups", False)),
                "require_imu_for_lifting": bool(act_cfg.get("require_imu_for_lifting", False)),
                "upper": {
                    "name": u_cfg.get("name", "UPPER JACK – 100mm"),
                    "stroke_mm": 100,
                    "rated_force_n": 50,
                    "actuator_count": 4,
                    "grouped_control": True,
                    "pins": pins.get("upper_actuator_group", {"pwm": 3, "in1": 24, "in2": 25}),
                    "commanded_pwm": self.upper_pwm,
                    "direction": self.upper_direction,
                    "inverted": bool(u_cfg.get("inverted", False)),
                    "default_pwm": int(u_cfg.get("default_pwm", 120)),
                    "jog_pulse_ms": int(u_cfg.get("jog_pulse_ms", 300)),
                    "active_duration_s": u_dur,
                    "max_continuous_run_ms": int(u_cfg.get("max_continuous_run_ms", 10000)),
                    "position_feedback": "Position feedback unavailable",
                    "history": list(self._upper_history)[:10],
                },
                "lower": {
                    "name": l_cfg.get("name", "LOWER BASE – 30mm"),
                    "stroke_mm": 30,
                    "rated_force_n": "Unconfirmed",
                    "actuator_count": 4,
                    "grouped_control": True,
                    "pins": pins.get("lower_actuator_group", {"pwm": 2, "in1": 22, "in2": 23}),
                    "commanded_pwm": self.lower_pwm,
                    "direction": self.lower_direction,
                    "inverted": bool(l_cfg.get("inverted", False)),
                    "default_pwm": int(l_cfg.get("default_pwm", 120)),
                    "jog_pulse_ms": int(l_cfg.get("jog_pulse_ms", 250)),
                    "active_duration_s": l_dur,
                    "max_continuous_run_ms": int(l_cfg.get("max_continuous_run_ms", 5000)),
                    "position_feedback": "Position feedback unavailable",
                    "history": list(self._lower_history)[:10],
                },
            }

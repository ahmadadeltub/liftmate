"""Servo Motor Controller (Arduino Pin 8) with Safe Travel Limits and Manual Enable."""
from __future__ import annotations

import threading
from typing import Any, Dict, Tuple

from .config import ConfigManager
from .diagnostics import DiagnosticsManager
from .safety_controller import SafetyController
from .serial_controller import SerialController


class ServoController:
    """Controls the pin 8 servo with strict lazy-enable and configurable travel limits."""

    def __init__(
        self,
        config: ConfigManager,
        diagnostics: DiagnosticsManager,
        safety: SafetyController,
        serial_ctrl: SerialController,
    ) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics
        self.safety = safety
        self.serial = serial_ctrl

        # Do NOT enable or move the servo on startup!
        self.enabled: bool = False
        self.commanded_angle: int = int(
            self.config.get_section("servo").get("initial_center_deg", 68)
        )
        self.hardware_attached: bool = False

    def set_enabled(self, enabled: bool) -> Tuple[bool, str]:
        with self._lock:
            self.enabled = bool(enabled)
            if not self.enabled and self.serial.connected:
                # Best-effort detach if supported by RobotJackSafe firmware
                self.serial.send_command("SERVO_DETACH")
                self.hardware_attached = False
            self.diagnostics.log_event(
                "INFO",
                "SERVO",
                "SERVO_ENABLE_CHANGED",
                f"enabled={self.enabled}",
            )
            return True, f"Servo control {'ENABLED' if self.enabled else 'DISABLED'}."

    def set_angle(self, angle_deg: int) -> Tuple[bool, str]:
        with self._lock:
            if not self.enabled:
                return False, "Servo control is disabled. Enable servo control first."

            allowed, reason = self.safety.can_move_servo()
            if not allowed:
                return False, reason

            srv_cfg = self.config.get_section("servo")
            safe_min = int(srv_cfg.get("safe_min_deg", 6))
            safe_max = int(srv_cfg.get("safe_max_deg", 130))
            inverted = bool(srv_cfg.get("inverted", False))

            req_angle = int(angle_deg)
            if req_angle < safe_min or req_angle > safe_max:
                return (
                    False,
                    f"Requested angle {req_angle}° exceeds safe travel limits ({safe_min}°–{safe_max}°).",
                )

            hw_angle = (136 - req_angle) if inverted else req_angle
            hw_angle = max(6, min(130, hw_angle))

            ok, resp = self.serial.send_command("SERVO", hw_angle)
            if ok:
                self.commanded_angle = req_angle
                self.hardware_attached = True
                self.diagnostics.log_event(
                    "INFO",
                    "SERVO",
                    "SERVO_ANGLE_SET",
                    f"ui_angle={req_angle} deg, hw_angle={hw_angle} deg",
                )
            return ok, resp

    def center(self) -> Tuple[bool, str]:
        center_deg = int(self.config.get_section("servo").get("initial_center_deg", 68))
        return self.set_angle(center_deg)

    def update_limits(
        self,
        safe_min_deg: int,
        safe_max_deg: int,
        inverted: bool = False,
    ) -> Tuple[bool, str]:
        s_min = int(safe_min_deg)
        s_max = int(safe_max_deg)
        if s_min < 0 or s_max > 180 or s_min >= s_max:
            return False, "Safe limits must satisfy 0 <= min < max <= 180."

        self.config.update_section("servo", {
            "safe_min_deg": s_min,
            "safe_max_deg": s_max,
            "inverted": bool(inverted),
        })
        if self.serial.connected:
            self.serial.send_command("SERVO_LIMITS", s_min, s_max)
        self.diagnostics.log_event(
            "INFO",
            "SERVO",
            "SERVO_LIMITS_UPDATED",
            f"min={s_min}, max={s_max}, inverted={inverted}",
        )
        return True, f"Servo safe limits updated to {s_min}°–{s_max}°."

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            srv_cfg = self.config.get_section("servo")
            return {
                "pin": int(srv_cfg.get("pin", 8)),
                "enabled": self.enabled,
                "hardware_attached": self.hardware_attached,
                "commanded_angle": self.commanded_angle,
                "initial_center_deg": int(srv_cfg.get("initial_center_deg", 68)),
                "safe_min_deg": int(srv_cfg.get("safe_min_deg", 6)),
                "safe_max_deg": int(srv_cfg.get("safe_max_deg", 130)),
                "hard_min_deg": 6,
                "hard_max_deg": 130,
                "inverted": bool(srv_cfg.get("inverted", False)),
                "presets": [6, 35, 68, 100, 130],
            }

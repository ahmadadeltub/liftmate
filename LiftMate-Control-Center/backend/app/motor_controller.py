"""Wheel Motor Testing, Calibration Wizard, and Mecanum Kinematics Controller."""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Tuple

from .config import ConfigManager
from .diagnostics import DiagnosticsManager
from .safety_controller import SafetyController
from .serial_controller import SerialController

VALID_POSITIONS = ("front_left", "front_right", "rear_left", "rear_right")

DIRECTION_VECTORS: Dict[str, Tuple[float, float, float]] = {
    "FORWARD": (1.0, 0.0, 0.0),
    "BACKWARD": (-1.0, 0.0, 0.0),
    "LEFT_STRAFE": (0.0, -1.0, 0.0),
    "RIGHT_STRAFE": (0.0, 1.0, 0.0),
    "ROTATE_LEFT": (0.0, 0.0, -1.0),
    "ROTATE_RIGHT": (0.0, 0.0, 1.0),
    "FORWARD_LEFT": (1.0, -1.0, 0.0),
    "FORWARD_RIGHT": (1.0, 1.0, 0.0),
    "BACKWARD_LEFT": (-1.0, -1.0, 0.0),
    "BACKWARD_RIGHT": (-1.0, 1.0, 0.0),
    "STOP": (0.0, 0.0, 0.0),
}


class MotorController:
    """Controls the 4 mecanum wheel motors, individual jog tests, calibration, and kinematics."""

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

        # Commanded speeds (raw signed PWM sent to Arduino motor channels 1..4)
        self.commanded_pwm: Dict[int, int] = {1: 0, 2: 0, 3: 0, 4: 0}
        self.test_results: Dict[int, str] = {1: "UNTESTED", 2: "UNTESTED", 3: "UNTESTED", 4: "UNTESTED"}
        self.last_drive_direction: str = "STOP"
        self.last_command_ts: float = 0.0
        self._jog_timer: threading.Timer | None = None

        self.safety.register_estop_callback(self.stop_all_wheels)
        self.serial.register_timeout_callback(self._on_firmware_timeout)

        self._wdt_running = True
        self._wdt_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._wdt_thread.start()

    def _on_firmware_timeout(self) -> None:
        with self._lock:
            for idx in (1, 2, 3, 4):
                self.commanded_pwm[idx] = 0
            self.last_drive_direction = "STOP"

    def _watchdog_loop(self) -> None:
        while self._wdt_running:
            time.sleep(0.05)
            with self._lock:
                active = any(v != 0 for v in self.commanded_pwm.values())
                if not active:
                    continue
                wdt_ms = float(self.config.get_section("safety").get("wheel_watchdog_ms", 450))
                elapsed_ms = (time.time() - self.last_command_ts) * 1000.0
                if elapsed_ms >= wdt_ms:
                    self.diagnostics.log_event(
                        "WARNING",
                        "MOTORS",
                        "SOFTWARE_WHEEL_WATCHDOG_EXPIRED",
                        f"elapsed={round(elapsed_ms, 1)}ms >= {wdt_ms}ms",
                    )
                    self.stop_all_wheels()

    def _cancel_jog_timer(self) -> None:
        if self._jog_timer is not None:
            self._jog_timer.cancel()
            self._jog_timer = None

    def stop_all_wheels(self) -> Tuple[bool, str]:
        with self._lock:
            self._cancel_jog_timer()
            for i in (1, 2, 3, 4):
                self.commanded_pwm[i] = 0
            self.last_drive_direction = "STOP"
            ok, msg = False, "DISCONNECTED"
            if self.serial.connected:
                ok, msg = self.serial.send_command("DRIVE", 0, 0, 0, 0)
            self.safety.notify_motion_stopped("Wheels stopped")
            return ok, msg

    def command_single_motor(
        self,
        motor_id: int,
        pwm: int,
        direction: str = "FORWARD",
        duration_ms: int | None = None,
    ) -> Tuple[bool, str]:
        """Commands a single wheel motor (1..4) with optional auto-stop jog timer (200..500ms)."""
        if motor_id not in (1, 2, 3, 4):
            return False, "Motor ID must be 1, 2, 3, or 4."

        if pwm == 0 or direction.upper() == "STOP":
            with self._lock:
                self._cancel_jog_timer()
                self.commanded_pwm[motor_id] = 0
                ok, resp = self.serial.send_command("MOTOR", motor_id, 0)
                if all(v == 0 for v in self.commanded_pwm.values()):
                    self.safety.notify_motion_stopped(f"Motor {motor_id} stopped")
                return ok, resp

        allowed, reason = self.safety.can_drive_wheels()
        if not allowed:
            return False, reason

        cal = self.config.get_section("wheel_calibration")
        max_limit = int(cal.get("max_pwm_limit", 200))
        clamped_mag = max(0, min(abs(int(pwm)), max_limit, 255))

        sign = -1 if direction.upper() in ("REVERSE", "BACKWARD") else 1
        inverted = bool(cal.get("inverted", {}).get(str(motor_id), False))
        if inverted:
            sign *= -1

        signed_pwm = sign * clamped_mag

        with self._lock:
            self._cancel_jog_timer()
            ok, resp = self.serial.send_command("MOTOR", motor_id, signed_pwm)
            if ok:
                self.commanded_pwm[motor_id] = signed_pwm
                self.last_command_ts = time.time()
                self.safety.notify_driving_started()
                self.diagnostics.log_event(
                    "INFO",
                    "MOTORS",
                    f"MOTOR_{motor_id}_CMD",
                    f"dir={direction.upper()}, pwm={signed_pwm}, duration_ms={duration_ms}",
                )
                if duration_ms is not None:
                    jog_s = max(0.1, min(float(duration_ms) / 1000.0, 0.5))
                    self._jog_timer = threading.Timer(jog_s, self._auto_stop_motor, args=(motor_id,))
                    self._jog_timer.daemon = True
                    self._jog_timer.start()
            return ok, resp

    def _auto_stop_motor(self, motor_id: int) -> None:
        with self._lock:
            self.commanded_pwm[motor_id] = 0
            if self.serial.connected:
                self.serial.send_command("MOTOR", motor_id, 0)
            if all(v == 0 for v in self.commanded_pwm.values()):
                self.safety.notify_motion_stopped(f"Motor {motor_id} timed jog completed")

    def compute_mecanum_wheel_pwms(
        self,
        vy: float,
        vx: float,
        omega: float,
        base_pwm: int,
    ) -> Dict[int, int]:
        """Computes signed PWM values for Arduino motor channels 1..4 from (vy, vx, omega).

        vy:    +1.0 forward, -1.0 backward
        vx:    +1.0 right strafe, -1.0 left strafe
        omega: +1.0 clockwise (rotate right), -1.0 counter-clockwise (rotate left)
        """
        cal = self.config.get_section("wheel_calibration")
        max_limit = int(cal.get("max_pwm_limit", 200))
        speed = max(0, min(int(base_pwm), max_limit, 255))

        raw_pos = {
            "front_left": vy + vx + omega,
            "front_right": vy - vx - omega,
            "rear_left": vy - vx + omega,
            "rear_right": vy + vx - omega,
        }
        peak = max(abs(v) for v in raw_pos.values())
        if peak > 1.0:
            for k in raw_pos:
                raw_pos[k] /= peak

        positions_map = cal.get("positions", {})
        inverted_map = cal.get("inverted", {})

        channel_pwms: Dict[int, int] = {1: 0, 2: 0, 3: 0, 4: 0}
        for ch_idx in (1, 2, 3, 4):
            pos_name = positions_map.get(str(ch_idx), "")
            factor = raw_pos.get(pos_name, 0.0)
            val = int(round(factor * speed))
            if bool(inverted_map.get(str(ch_idx), False)):
                val = -val
            channel_pwms[ch_idx] = max(-255, min(255, val))
        return channel_pwms

    def drive_direction(self, direction: str, speed_pwm: int) -> Tuple[bool, str]:
        dir_key = direction.strip().upper().replace("-", "_")
        if dir_key == "STOP" or speed_pwm == 0:
            return self.stop_all_wheels()

        if dir_key not in DIRECTION_VECTORS:
            return False, f"Unknown direction '{direction}'."

        allowed, reason = self.safety.can_drive_wheels()
        if not allowed:
            return False, reason

        vy, vx, omega = DIRECTION_VECTORS[dir_key]
        return self.drive_vector(vy, vx, omega, speed_pwm, label=dir_key)

    def drive_vector(
        self,
        vy: float,
        vx: float,
        omega: float,
        speed_pwm: int,
        label: str = "VECTOR",
    ) -> Tuple[bool, str]:
        if abs(vy) < 1e-3 and abs(vx) < 1e-3 and abs(omega) < 1e-3 or speed_pwm == 0:
            return self.stop_all_wheels()

        allowed, reason = self.safety.can_drive_wheels()
        if not allowed:
            return False, reason

        channel_pwms = self.compute_mecanum_wheel_pwms(vy, vx, omega, speed_pwm)
        with self._lock:
            self._cancel_jog_timer()
            ok, resp = self.serial.send_command(
                "DRIVE",
                channel_pwms[1],
                channel_pwms[2],
                channel_pwms[3],
                channel_pwms[4],
            )
            if ok:
                self.commanded_pwm = channel_pwms
                self.last_drive_direction = label
                self.last_command_ts = time.time()
                self.safety.notify_driving_started()
            return ok, resp

    def update_calibration(
        self,
        positions: Dict[str, str],
        inverted: Dict[str, bool],
        max_pwm_limit: int | None = None,
    ) -> Tuple[bool, str]:
        """Saves wheel position mapping and polarity inversions to robot_config.yaml."""
        assigned = [positions.get(str(i), "") for i in (1, 2, 3, 4)]
        if set(assigned) != set(VALID_POSITIONS):
            return (
                False,
                "Each physical wheel position (front_left, front_right, rear_left, rear_right) "
                "must be assigned to exactly one motor (1–4).",
            )

        updates: Dict[str, Any] = {
            "calibrated": True,
            "positions": {str(i): positions[str(i)] for i in (1, 2, 3, 4)},
            "inverted": {str(i): bool(inverted.get(str(i), False)) for i in (1, 2, 3, 4)},
        }
        if max_pwm_limit is not None:
            updates["max_pwm_limit"] = max(30, min(int(max_pwm_limit), 255))

        self.config.update_section("wheel_calibration", updates)
        self.diagnostics.log_event("INFO", "MOTORS", "WHEEL_CALIBRATION_SAVED", str(updates))
        return True, "Wheel mapping calibration saved to config/robot_config.yaml."

    def record_test_result(self, motor_id: int, result: str) -> None:
        if motor_id in (1, 2, 3, 4):
            self.test_results[motor_id] = result.upper()
            self.diagnostics.log_event("INFO", "MOTORS", f"MOTOR_{motor_id}_TEST_RESULT", result.upper())

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            cal = self.config.get_section("wheel_calibration")
            pins = self.config.get_section("pins").get("wheel_motors", {})
            motors_list: List[Dict[str, Any]] = []
            for idx in (1, 2, 3, 4):
                s_idx = str(idx)
                pin_info = pins.get(s_idx, {})
                motors_list.append({
                    "motor_id": idx,
                    "calibrated_position": cal.get("positions", {}).get(s_idx, "unassigned"),
                    "inverted": bool(cal.get("inverted", {}).get(s_idx, False)),
                    "commanded_pwm": self.commanded_pwm[idx],
                    "measured_speed": "No encoder feedback (Open-Loop)",
                    "pins": pin_info,
                    "comm_status": "ONLINE" if self.serial.connected else "OFFLINE",
                    "test_result": self.test_results[idx],
                })
            return {
                "calibrated": bool(cal.get("calibrated", False)),
                "default_test_pwm": int(cal.get("default_test_pwm", 50)),
                "default_drive_pwm": int(cal.get("default_drive_pwm", 75)),
                "max_pwm_limit": int(cal.get("max_pwm_limit", 200)),
                "jog_duration_ms": int(cal.get("jog_duration_ms", 350)),
                "last_drive_direction": self.last_drive_direction,
                "motors": motors_list,
            }

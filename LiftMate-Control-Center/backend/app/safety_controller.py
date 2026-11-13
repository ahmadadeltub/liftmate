"""Software safety state machine and interlock controller for LiftMate."""
from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Any, Callable, Dict, Tuple

from .config import ConfigManager
from .diagnostics import DiagnosticsManager


class SafetyState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    IDLE = "IDLE"
    ARMED = "ARMED"
    DRIVING = "DRIVING"
    ACTUATING_UPPER = "ACTUATING_UPPER"
    ACTUATING_LOWER = "ACTUATING_LOWER"
    FAULT = "FAULT"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class SafetyController:
    """Enforces software safety states, arming rules, watchdogs, and interlocks.

    NOTE: Software emergency-stop is NOT a substitute for a hardwired physical
    emergency-stop switch that disconnects 12V motor/actuator power.
    """

    def __init__(self, config: ConfigManager, diagnostics: DiagnosticsManager) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics
        self.state: SafetyState = SafetyState.DISCONNECTED
        self.estop_latched: bool = False
        self.estop_reason: str = ""
        self.fault_reason: str = ""
        self.connected: bool = False
        self.last_state_change_ts: float = time.time()
        self._emergency_stop_callbacks: list[Callable[[], None]] = []

    def register_estop_callback(self, cb: Callable[[], None]) -> None:
        self._emergency_stop_callbacks.append(cb)

    def _set_state(self, new_state: SafetyState, reason: str = "") -> None:
        with self._lock:
            old_state = self.state
            self.state = new_state
            self.last_state_change_ts = time.time()
            if old_state != new_state:
                self.diagnostics.log_event(
                    "WARNING" if new_state in (SafetyState.FAULT, SafetyState.EMERGENCY_STOP) else "INFO",
                    "SAFETY",
                    f"STATE_{old_state.value}_TO_{new_state.value}",
                    reason,
                )

    def on_serial_connected(self) -> None:
        with self._lock:
            self.connected = True
            if self.estop_latched:
                self._set_state(SafetyState.EMERGENCY_STOP, "Connected while E-STOP latched")
            else:
                # Always enter IDLE (disarmed) on connection or reconnection
                self._set_state(SafetyState.IDLE, "Serial connected; manual arming required")

    def on_serial_disconnected(self, reason: str = "Serial connection closed") -> None:
        with self._lock:
            self.connected = False
            if not self.estop_latched:
                self._set_state(SafetyState.DISCONNECTED, reason)
            else:
                self.diagnostics.log_event("WARNING", "SAFETY", "DISCONNECTED_DURING_ESTOP", reason)

    def arm(self) -> Tuple[bool, str]:
        with self._lock:
            if self.estop_latched or self.state == SafetyState.EMERGENCY_STOP:
                return False, "Cannot arm while EMERGENCY_STOP is latched. Reset E-STOP first."
            if not self.connected:
                return False, "Cannot arm while controller is DISCONNECTED."
            if self.state == SafetyState.FAULT:
                return False, f"Cannot arm while in FAULT state ({self.fault_reason}). Clear fault first."
            self._set_state(SafetyState.ARMED, "Operator manually armed system")
            return True, "System ARMED."

    def disarm(self, reason: str = "Operator disarmed system") -> Tuple[bool, str]:
        with self._lock:
            for cb in self._emergency_stop_callbacks:
                try:
                    cb()
                except Exception:
                    pass
            if self.estop_latched:
                return True, "System outputs stopped (E-STOP remains latched)."
            if self.connected:
                self._set_state(SafetyState.IDLE, reason)
            else:
                self._set_state(SafetyState.DISCONNECTED, reason)
            return True, "System DISARMED (IDLE)."

    def trigger_emergency_stop(self, reason: str = "Operator pressed Emergency Stop") -> Dict[str, Any]:
        with self._lock:
            self.estop_latched = True
            self.estop_reason = reason
            self._set_state(SafetyState.EMERGENCY_STOP, reason)
            self.diagnostics.log_event("CRITICAL", "SAFETY", "EMERGENCY_STOP_LATCHED", reason)
        for cb in self._emergency_stop_callbacks:
            try:
                cb()
            except Exception:
                pass
        return self.get_status()

    def reset_emergency_stop(self) -> Tuple[bool, str]:
        with self._lock:
            self.estop_latched = False
            self.estop_reason = ""
            self.fault_reason = ""
            if self.connected:
                self._set_state(SafetyState.IDLE, "E-STOP reset by operator; system in IDLE (require manual re-arm)")
                return True, "Emergency stop reset. System is in IDLE — manually ARM when ready."
            else:
                self._set_state(SafetyState.DISCONNECTED, "E-STOP reset while disconnected")
                return True, "Emergency stop reset while disconnected."

    def trigger_fault(self, reason: str) -> None:
        with self._lock:
            self.fault_reason = reason
            if not self.estop_latched:
                self._set_state(SafetyState.FAULT, reason)
            self.diagnostics.log_event("ERROR", "SAFETY", "SAFETY_FAULT", reason)
        for cb in self._emergency_stop_callbacks:
            try:
                cb()
            except Exception:
                pass

    def clear_fault(self) -> Tuple[bool, str]:
        with self._lock:
            if self.estop_latched:
                return False, "Emergency stop is active. Reset E-STOP first."
            self.fault_reason = ""
            if self.connected:
                self._set_state(SafetyState.IDLE, "Fault cleared by operator; manual re-arm required")
                return True, "Fault cleared. System is in IDLE."
            self._set_state(SafetyState.DISCONNECTED, "Fault cleared while disconnected")
            return True, "Fault cleared."

    def can_drive_wheels(self) -> Tuple[bool, str]:
        with self._lock:
            if self.estop_latched or self.state == SafetyState.EMERGENCY_STOP:
                return False, "Blocked: EMERGENCY_STOP is active."
            if not self.connected:
                return False, "Blocked: Arduino controller is DISCONNECTED."
            if self.state == SafetyState.IDLE:
                return False, "Blocked: System is IDLE (Disarmed). Click ARM SYSTEM first."
            if self.state == SafetyState.FAULT:
                return False, f"Blocked: System FAULT ({self.fault_reason})."
            if self.state in (SafetyState.ACTUATING_UPPER, SafetyState.ACTUATING_LOWER):
                return False, f"Blocked: Cannot drive wheels while {self.state.value} is active."
            return True, "OK"

    def can_move_actuator(self, group: str, imu_status: Dict[str, Any] | None = None) -> Tuple[bool, str]:
        group_upper = group.upper()
        with self._lock:
            if self.estop_latched or self.state == SafetyState.EMERGENCY_STOP:
                return False, "Blocked: EMERGENCY_STOP is active."
            if not self.connected:
                return False, "Blocked: Arduino controller is DISCONNECTED."
            if self.state == SafetyState.IDLE:
                return False, "Blocked: System is IDLE (Disarmed). Click ARM SYSTEM first."
            if self.state == SafetyState.FAULT:
                return False, f"Blocked: System FAULT ({self.fault_reason})."
            if self.state == SafetyState.DRIVING:
                return False, "Blocked: Cannot operate linear actuators while robot wheels are DRIVING."

            act_cfg = self.config.get_section("actuators")
            if not act_cfg.get("driver_electrical_verified", False):
                return (
                    False,
                    "Blocked: Motor driver & electrical rating verification must be confirmed before actuator operation.",
                )

            allow_dual = bool(act_cfg.get("allow_simultaneous_groups", False))
            if not allow_dual:
                if group_upper == "UPPER" and self.state == SafetyState.ACTUATING_LOWER:
                    return False, "Blocked: Lower actuator group is currently active (simultaneous actuation prohibited)."
                if group_upper == "LOWER" and self.state == SafetyState.ACTUATING_UPPER:
                    return False, "Blocked: Upper actuator group is currently active (simultaneous actuation prohibited)."

            if act_cfg.get("require_imu_for_lifting", False) and imu_status is not None:
                if not imu_status.get("connected", False):
                    return False, "Blocked: IMU is disconnected and 'Require IMU for Lifting' safety lock is enabled."
                if imu_status.get("tilt_critical", False):
                    return False, "Blocked: Critical chassis tilt angle exceeded!"

            return True, "OK"

    def can_move_servo(self) -> Tuple[bool, str]:
        with self._lock:
            if self.estop_latched or self.state == SafetyState.EMERGENCY_STOP:
                return False, "Blocked: EMERGENCY_STOP is active."
            if not self.connected:
                return False, "Blocked: Arduino controller is DISCONNECTED."
            if self.state == SafetyState.IDLE:
                return False, "Blocked: System is IDLE (Disarmed). Click ARM SYSTEM first."
            if self.state == SafetyState.FAULT:
                return False, f"Blocked: System FAULT ({self.fault_reason})."
            return True, "OK"

    def notify_driving_started(self) -> None:
        with self._lock:
            if self.state == SafetyState.ARMED:
                self._set_state(SafetyState.DRIVING, "Wheel motion active")

    def notify_actuating_started(self, group: str) -> None:
        with self._lock:
            if self.state in (SafetyState.ARMED, SafetyState.ACTUATING_UPPER, SafetyState.ACTUATING_LOWER):
                target = SafetyState.ACTUATING_UPPER if group.upper() == "UPPER" else SafetyState.ACTUATING_LOWER
                self._set_state(target, f"{group.upper()} actuator motion active")

    def notify_motion_stopped(self, reason: str = "Motion stopped") -> None:
        with self._lock:
            if self.state in (SafetyState.DRIVING, SafetyState.ACTUATING_UPPER, SafetyState.ACTUATING_LOWER):
                self._set_state(SafetyState.ARMED, reason)

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            safety_cfg = self.config.get_section("safety")
            act_cfg = self.config.get_section("actuators")
            return {
                "state": self.state.value,
                "connected": self.connected,
                "armed": self.state in (
                    SafetyState.ARMED,
                    SafetyState.DRIVING,
                    SafetyState.ACTUATING_UPPER,
                    SafetyState.ACTUATING_LOWER,
                ),
                "estop_latched": self.estop_latched,
                "estop_reason": self.estop_reason,
                "fault_reason": self.fault_reason,
                "wheel_watchdog_ms": safety_cfg.get("wheel_watchdog_ms", 450),
                "actuator_watchdog_ms": safety_cfg.get("actuator_watchdog_ms", 600),
                "driver_electrical_verified": act_cfg.get("driver_electrical_verified", False),
                "allow_simultaneous_groups": act_cfg.get("allow_simultaneous_groups", False),
                "require_imu_for_lifting": act_cfg.get("require_imu_for_lifting", False),
                "physical_estop_notice": (
                    "WARNING: Software STOP ALL is not a fail-safe substitute for a hardwired "
                    "physical emergency-stop switch in series with the 12V motor/actuator supply."
                ),
            }

"""Guarded Autonomous Obstacle-Avoidance Controller for LiftMate."""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

from .actuator_controller import ActuatorController
from .ai_vision_controller import AIVisionController
from .camera_controller import CameraController
from .config import ConfigManager
from .diagnostics import DiagnosticsManager
from .motor_controller import MotorController
from .navigation_controller import NavigationController
from .safety_controller import SafetyController, SafetyState
from .serial_controller import SerialController

MIN_AUTO_SPEED_PWM = 55
MAX_AUTO_SPEED_PWM = 125
DEFAULT_AUTO_SPEED_PWM = 90
CONTROL_INTERVAL_S = 0.10
MAX_CAMERA_FRAME_AGE_S = 1.8
BRAKE_HOLD_S = 0.25
TURN_LATCH_S = 0.60
MAX_SCAN_ROTATE_S = 2.50


class AutonomousController:
    """Runs guarded low-speed obstacle avoidance using live AI Vision detections.

    Never drives when:
    - Arduino Mega serial link is disconnected
    - Software E-STOP is latched or system is in FAULT state
    - Any linear actuator group (Jack/Holder) is active
    - Camera stream is inactive or frames are stale
    - AI Vision model is not ready or perception state is UNKNOWN/UNAVAILABLE
    - Operator issues any manual drive, actuator, stop, or reset command
    """

    def __init__(
        self,
        config: ConfigManager,
        diagnostics: DiagnosticsManager,
        safety: SafetyController,
        serial_ctrl: SerialController,
        motors: MotorController,
        actuators: ActuatorController,
        camera: CameraController,
        ai_vision: AIVisionController,
        navigation: NavigationController,
    ) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics
        self.safety = safety
        self.serial = serial_ctrl
        self.motors = motors
        self.actuators = actuators
        self.camera = camera
        self.ai_vision = ai_vision
        self.navigation = navigation

        self.enabled: bool = False
        self.speed_pwm: int = DEFAULT_AUTO_SPEED_PWM
        self.action: str = "MANUAL"
        self.reason: str = "Manual operator control active"
        self.last_decision_ts: float = 0.0
        self.last_vector: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_channel_pwms: Dict[int, int] = {1: 0, 2: 0, 3: 0, 4: 0}
        self._decision_history: Deque[Dict[str, Any]] = deque(maxlen=20)

        self._turn_latch_dir: str = ""
        self._turn_latch_until: float = 0.0
        self._brake_until: float = 0.0
        self._scan_started_ts: float = 0.0
        self._was_forward: bool = False

        self.safety.register_estop_callback(self._on_safety_estop)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def _on_safety_estop(self) -> None:
        with self._lock:
            if self.enabled:
                self.enabled = False
                self.action = "MANUAL"
                self.reason = "Disengaged by Emergency Stop"
                self.last_vector = (0.0, 0.0, 0.0)
                self.last_channel_pwms = {1: 0, 2: 0, 3: 0, 4: 0}
                self._reset_motion_state()

    def _reset_motion_state(self) -> None:
        self._turn_latch_dir = ""
        self._turn_latch_until = 0.0
        self._brake_until = 0.0
        self._scan_started_ts = 0.0
        self._was_forward = False

    def _stop_wheels(self, reason: str, action: str) -> None:
        self.motors.stop_all_wheels()
        self.navigation.notify_wheel_command(0, 0, 0)
        if self.serial.connected:
            self.serial.send_command("DRIVE", 0, 0, 0, 0)
        self.safety.notify_motion_stopped(reason)
        self.action = action
        self.reason = reason
        self.last_vector = (0.0, 0.0, 0.0)
        self.last_channel_pwms = {1: 0, 2: 0, 3: 0, 4: 0}
        self.last_decision_ts = time.time()

    def _check_preconditions(self) -> Tuple[bool, str, str]:
        if self.safety.estop_latched or self.safety.state == SafetyState.EMERGENCY_STOP:
            return False, "PAUSED_ESTOP", "Emergency Stop is active."
        if self.safety.state == SafetyState.FAULT:
            return False, "PAUSED_FAULT", f"Safety fault active ({self.safety.fault_reason})."
        if not self.serial.connected or not self.safety.connected:
            return False, "PAUSED_DISCONNECTED", "Arduino Mega USB serial is disconnected."
        if self.actuators.upper_pwm != 0 or self.actuators.lower_pwm != 0:
            return False, "PAUSED_ACTUATOR", "Wheels locked while linear actuators are moving."

        cam_status = self.camera.get_status()
        if not cam_status.get("streaming") or not cam_status.get("connected"):
            return False, "PAUSED_NO_CAMERA", "Camera stream is offline."

        last_ts = float(getattr(self.camera, "last_frame_ts", 0.0) or 0.0)
        if last_ts <= 0.0 or (time.time() - last_ts) > MAX_CAMERA_FRAME_AGE_S:
            return False, "PAUSED_CAMERA_STALE", "Camera frames are stale; holding position safely."

        ai_status = self.ai_vision.get_status()
        if not ai_status.get("model_ready", False):
            return False, "PAUSED_NO_MODEL", "AI obstacle model is not ready."

        return True, "OK", "Preconditions satisfied"

    def set_enabled(
        self,
        enabled: bool,
        speed_pwm: Optional[int] = None,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        """Enables or disables guarded autonomous obstacle-avoidance driving."""
        with self._lock:
            if speed_pwm is not None:
                self.speed_pwm = max(MIN_AUTO_SPEED_PWM, min(int(speed_pwm), MAX_AUTO_SPEED_PWM))

            if not enabled:
                was_enabled = self.enabled
                self.enabled = False
                self._reset_motion_state()
                self._stop_wheels("Switched to Manual mode", "MANUAL")
                if was_enabled:
                    self.diagnostics.log_event("INFO", "AUTONOMOUS", "AUTO_DISABLED", "Manual control restored")
                return True, "Manual mode active.", self.get_status()

            ok_pre, fail_action, fail_reason = self._check_preconditions()
            if not ok_pre:
                self.enabled = False
                self._reset_motion_state()
                self.action = fail_action
                self.reason = fail_reason
                self.diagnostics.log_event("WARNING", "AUTONOMOUS", "AUTO_START_REJECTED", fail_reason)
                return False, f"Cannot start Autonomous Mode: {fail_reason}", self.get_status()

            if self.safety.state == SafetyState.IDLE:
                armed_ok, armed_msg = self.safety.arm()
                if not armed_ok:
                    return False, armed_msg, self.get_status()

            # Ensure AI vision detection is enabled so regions update continuously
            if not self.ai_vision.enabled:
                self.ai_vision.set_enabled(True)
            self.ai_vision.notify_motor_activity()

            if self._stop_event.is_set() or not self._thread.is_alive():
                self._stop_event.clear()
                self._thread = threading.Thread(target=self._control_loop, daemon=True)
                self._thread.start()

            self._reset_motion_state()
            self.enabled = True
            self.action = "ENGAGED"
            self.reason = f"Autonomous obstacle avoidance active (speed {self.speed_pwm} PWM)"
            self.last_decision_ts = time.time()
            self.diagnostics.log_event(
                "INFO",
                "AUTONOMOUS",
                "AUTO_ENABLED",
                f"speed_pwm={self.speed_pwm}",
            )
            return True, "Autonomous obstacle-avoidance mode enabled.", self.get_status()

    def manual_override(self, reason: str = "Operator manual control") -> bool:
        """Immediately disengages Autonomous Mode when manual control arrives."""
        with self._lock:
            if not self.enabled and self.action == "MANUAL":
                return False
            was_enabled = self.enabled
            self.enabled = False
            self._reset_motion_state()
            self.action = "MANUAL"
            self.reason = reason
            self.last_vector = (0.0, 0.0, 0.0)
            self.last_channel_pwms = {1: 0, 2: 0, 3: 0, 4: 0}
            self.last_decision_ts = time.time()
            if was_enabled:
                self.diagnostics.log_event("INFO", "AUTONOMOUS", "MANUAL_OVERRIDE", reason)
            return was_enabled

    def stop(self) -> None:
        """Stops the background autonomous loop and halts wheel outputs if active."""
        with self._lock:
            if self.enabled:
                self.enabled = False
                self._stop_wheels("Autonomous controller shutdown", "MANUAL")
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    @staticmethod
    def _region_clear_score(region: Dict[str, Any]) -> int:
        status = str(region.get("status", "UNKNOWN")).upper()
        risk = str(region.get("risk", "UNKNOWN")).upper()
        blocked = bool(region.get("blocked", False))
        if (
            "UNKNOWN" in status
            or "UNRELIABLE" in status
            or "AI OFF" in status
            or risk == "UNKNOWN"
        ):
            return -1
        if blocked or risk == "HIGH" or "BLOCKED" in status:
            return 0
        if risk == "MEDIUM" or "CAUTION" in status or "POTENTIAL" in status or "OBSTACLE" in status:
            return 1
        return 2

    def _decide_motion(
        self,
        ai_status: Dict[str, Any],
        now: float,
    ) -> Tuple[str, str, float, float, float]:
        vision_state = str(ai_status.get("vision_status", "UNKNOWN")).upper()
        uncertain_tokens = (
            "UNAVAILABLE",
            "NOT READY",
            "NOT_READY",
            "AI OFF",
            "DISABLED",
            "UNKNOWN",
            "UNRELIABLE",
        )
        if any(tok in vision_state for tok in uncertain_tokens):
            return "PAUSED_UNCERTAIN", f"Vision uncertain ({vision_state}); stopping safely", 0.0, 0.0, 0.0

        regions = ai_status.get("regions", {})
        left_r = regions.get("LEFT", {})
        center_r = regions.get("CENTER", {})
        right_r = regions.get("RIGHT", {})

        left_score = self._region_clear_score(left_r)
        center_score = self._region_clear_score(center_r)
        right_score = self._region_clear_score(right_r)

        if left_score < 0 or center_score < 0 or right_score < 0:
            return "PAUSED_UNCERTAIN", "Region confidence unknown; stopping safely", 0.0, 0.0, 0.0

        # Active obstacle straight ahead -> brake briefly then rotate away
        if center_score == 0:
            if self._was_forward and now >= self._brake_until:
                self._brake_until = now + BRAKE_HOLD_S
            self._was_forward = False

            if now < self._brake_until:
                return "BRAKING", "Obstacle ahead — braking before turn", 0.0, 0.0, 0.0

            # Decide turn direction or respect active turn latch while clearing obstacle
            if now < self._turn_latch_until and self._turn_latch_dir in ("LEFT", "RIGHT"):
                chosen_dir = self._turn_latch_dir
            elif left_score > right_score:
                chosen_dir = "LEFT"
            elif right_score > left_score:
                chosen_dir = "RIGHT"
            elif left_score >= 1 and right_score >= 1:
                chosen_dir = self._turn_latch_dir if self._turn_latch_dir in ("LEFT", "RIGHT") else "RIGHT"
            else:
                chosen_dir = ""

            if chosen_dir == "LEFT":
                self._scan_started_ts = 0.0
                self._turn_latch_dir = "LEFT"
                self._turn_latch_until = now + TURN_LATCH_S
                return "AVOID_LEFT", "Center blocked, Left clearer — rotating left", 0.0, 0.0, -0.80

            if chosen_dir == "RIGHT":
                self._scan_started_ts = 0.0
                self._turn_latch_dir = "RIGHT"
                self._turn_latch_until = now + TURN_LATCH_S
                return "AVOID_RIGHT", "Center blocked, Right clearer — rotating right", 0.0, 0.0, 0.80

            # All 3 zones blocked: run a bounded low-speed scan rotation, then stop
            if self._scan_started_ts <= 0.0:
                self._scan_started_ts = now
            if (now - self._scan_started_ts) < MAX_SCAN_ROTATE_S:
                scan_dir = -0.65 if self._turn_latch_dir == "LEFT" else 0.65
                return "SCAN_ROTATE", "All zones blocked — scanning for clear exit", 0.0, 0.0, scan_dir
            return "BLOCKED_STOP", "Surrounded by obstacles — holding position safely", 0.0, 0.0, 0.0

        self._scan_started_ts = 0.0
        if center_score == 1:
            self._was_forward = True
            if left_score < right_score:
                return "VEER_RIGHT", "Center caution — slowing & steering right", 0.55, 0.25, 0.45
            if right_score < left_score:
                return "VEER_LEFT", "Center caution — slowing & steering left", 0.55, -0.25, -0.45
            return "CRUISE_CAUTION", "Center caution — moving forward slowly", 0.55, 0.0, 0.0

        # Center is CLEAR
        self._was_forward = True
        self._turn_latch_until = 0.0
        if left_score == 0 and right_score >= 1:
            return "FORWARD_BIAS_RIGHT", "Path clear ahead, avoiding left side obstacle", 0.85, 0.30, 0.15
        if right_score == 0 and left_score >= 1:
            return "FORWARD_BIAS_LEFT", "Path clear ahead, avoiding right side obstacle", 0.85, -0.30, -0.15
        return "FORWARD", "Path clear — moving forward", 1.0, 0.0, 0.0

    def step_once(self) -> Dict[str, Any]:
        """Executes a single autonomous perception-and-control cycle."""
        with self._lock:
            if not self.enabled:
                return self.get_status()

            now = time.time()
            ok_pre, fail_action, fail_reason = self._check_preconditions()
            if not ok_pre:
                self._stop_wheels(fail_reason, fail_action)
                return self.get_status()

            if not self.ai_vision.enabled:
                self.ai_vision.set_enabled(True)
            self.ai_vision.notify_motor_activity()
            ai_status = self.ai_vision.get_status()

            action, reason, vy, vx, omega = self._decide_motion(ai_status, now)
            prev_action = self.action
            self.last_decision_ts = now

            if abs(vy) < 1e-3 and abs(vx) < 1e-3 and abs(omega) < 1e-3:
                self._stop_wheels(reason, action)
            else:
                channel_pwms = self.motors.compute_mecanum_wheel_pwms(vy, vx, omega, self.speed_pwm)
                ok_tx, resp = self.serial.send_command(
                    "DRIVE",
                    channel_pwms[1],
                    channel_pwms[2],
                    channel_pwms[3],
                    channel_pwms[4],
                )
                if not ok_tx:
                    self._stop_wheels(f"Serial command failed: {resp}", "PAUSED_SERIAL_ERR")
                    return self.get_status()

                self.motors.commanded_pwm = dict(channel_pwms)
                self.motors.last_command_ts = now
                self.motors.last_drive_direction = f"AUTO_{action}"
                self.safety.notify_driving_started()
                self.navigation.notify_wheel_command(
                    int(round(vx * self.speed_pwm)),
                    int(round(vy * self.speed_pwm)),
                    int(round(omega * self.speed_pwm)),
                )
                self.action = action
                self.reason = reason
                self.last_vector = (round(vy, 2), round(vx, 2), round(omega, 2))
                self.last_channel_pwms = dict(channel_pwms)

            if action != prev_action:
                self._decision_history.appendleft({
                    "ts": time.strftime("%H:%M:%S"),
                    "action": self.action,
                    "reason": self.reason,
                })
                self.diagnostics.log_event("INFO", "AUTONOMOUS", f"AUTO_{self.action}", self.reason)

            return self.get_status()

    def _control_loop(self) -> None:
        while not self._stop_event.wait(CONTROL_INTERVAL_S):
            try:
                if self.enabled:
                    self.step_once()
            except Exception as exc:
                with self._lock:
                    self._stop_wheels(f"Autonomous loop error: {exc}", "PAUSED_ERROR")
                    self.diagnostics.log_event("ERROR", "AUTONOMOUS", "AUTO_LOOP_ERROR", str(exc))

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "enabled": self.enabled,
                "mode": "AUTONOMOUS" if self.enabled else "MANUAL",
                "action": self.action,
                "reason": self.reason,
                "speed_pwm": self.speed_pwm,
                "min_speed_pwm": MIN_AUTO_SPEED_PWM,
                "max_speed_pwm": MAX_AUTO_SPEED_PWM,
                "last_decision_ts": self.last_decision_ts,
                "vector": {
                    "vy": self.last_vector[0],
                    "vx": self.last_vector[1],
                    "omega": self.last_vector[2],
                },
                "channel_pwms": dict(self.last_channel_pwms),
                "history": list(self._decision_history)[:8],
            }

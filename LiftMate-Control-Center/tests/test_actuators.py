"""Automated tests for Upper and Lower Linear Actuator safety interlocks."""
from __future__ import annotations

from pathlib import Path

from backend.app.actuator_controller import ActuatorController
from backend.app.config import ConfigManager
from backend.app.diagnostics import DiagnosticsManager
from backend.app.imu_controller import IMUController
from backend.app.motor_controller import MotorController
from backend.app.safety_controller import SafetyController
from backend.app.serial_controller import SerialController


def _make_actuator_stack(tmp_path: Path):
    cfg = ConfigManager(tmp_path / "robot_config.yaml")
    diag = DiagnosticsManager(log_file=tmp_path / "diag.csv")
    safety = SafetyController(cfg, diag)
    imu = IMUController(cfg, diag)
    ser = SerialController(cfg, diag, safety, imu)
    ser.set_simulation_mode(True)
    motors = MotorController(cfg, diag, safety, ser)
    actuators = ActuatorController(cfg, diag, safety, ser, imu)
    return cfg, safety, motors, actuators


def test_actuator_requires_driver_verification_and_arming(tmp_path: Path) -> None:
    _, safety, _, actuators = _make_actuator_stack(tmp_path)

    # Blocked while IDLE (Disarmed)
    ok, msg = actuators.command_group("UPPER", "EXTEND", 100)
    assert ok is False and "IDLE" in msg

    safety.arm()
    # Still blocked because driver_electrical_verified defaults to False
    ok, msg = actuators.command_group("UPPER", "EXTEND", 100)
    assert ok is False and "verification" in msg.lower()

    # Verify driver and retry
    actuators.update_settings(driver_electrical_verified=True)
    ok, _ = actuators.command_group("UPPER", "EXTEND", 110)
    assert ok is True
    status = actuators.get_status()
    assert status["upper"]["commanded_pwm"] == 110
    assert status["upper"]["position_feedback"] == "Position feedback unavailable"
    actuators.stop_all_actuators()


def test_prohibit_simultaneous_upper_and_lower_by_default(tmp_path: Path) -> None:
    _, safety, motors, actuators = _make_actuator_stack(tmp_path)
    safety.arm()
    actuators.update_settings(driver_electrical_verified=True, allow_simultaneous_groups=False)

    ok1, _ = actuators.command_group("UPPER", "EXTEND", 100)
    assert ok1 is True

    # Attempting LOWER while UPPER is active must be rejected by safety interlock
    ok2, msg2 = actuators.command_group("LOWER", "EXTEND", 100)
    assert ok2 is False
    assert "simultaneous" in msg2.lower()

    # Attempting to drive wheels while UPPER is active must also be rejected
    ok_drive, msg_drive = motors.drive_direction("FORWARD", 60)
    assert ok_drive is False
    assert "ACTUATING_UPPER" in msg_drive

    actuators.stop_all_actuators()

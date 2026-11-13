"""Automated tests for Wheel Motor output limits, calibration, and Mecanum kinematics."""
from __future__ import annotations

import time
from pathlib import Path

from backend.app.config import ConfigManager
from backend.app.diagnostics import DiagnosticsManager
from backend.app.imu_controller import IMUController
from backend.app.motor_controller import MotorController
from backend.app.safety_controller import SafetyController
from backend.app.serial_controller import SerialController


def _make_motor_stack(tmp_path: Path):
    cfg = ConfigManager(tmp_path / "robot_config.yaml")
    diag = DiagnosticsManager(log_file=tmp_path / "diag.csv")
    safety = SafetyController(cfg, diag)
    imu = IMUController(cfg, diag)
    ser = SerialController(cfg, diag, safety, imu)
    ser.set_simulation_mode(True)
    motors = MotorController(cfg, diag, safety, ser)
    return cfg, safety, ser, motors


def test_motor_calibration_validation_and_persistence(tmp_path: Path) -> None:
    _, _, _, motors = _make_motor_stack(tmp_path)

    # Duplicate positions must be rejected
    bad_pos = {"1": "front_left", "2": "front_left", "3": "rear_left", "4": "rear_right"}
    ok, _ = motors.update_calibration(bad_pos, {"1": False, "2": False, "3": False, "4": False})
    assert ok is False

    # Valid unique positions must succeed
    good_pos = {
        "1": "front_left",
        "2": "front_right",
        "3": "rear_left",
        "4": "rear_right",
    }
    inv = {"1": False, "2": True, "3": False, "4": True}
    ok, _ = motors.update_calibration(good_pos, inv, max_pwm_limit=160)
    assert ok is True
    status = motors.get_status()
    assert status["calibrated"] is True
    assert status["max_pwm_limit"] == 160


def test_mecanum_kinematics_all_directions(tmp_path: Path) -> None:
    _, safety, _, motors = _make_motor_stack(tmp_path)
    motors.update_calibration(
        {"1": "front_left", "2": "front_right", "3": "rear_left", "4": "rear_right"},
        {"1": False, "2": False, "3": False, "4": False},
        max_pwm_limit=200,
    )
    safety.arm()

    # Forward: all 4 positive
    ok, _ = motors.drive_direction("FORWARD", 80)
    assert ok is True
    assert motors.commanded_pwm == {1: 80, 2: 80, 3: 80, 4: 80}

    # Right strafe: FL+, FR-, RL-, RR+
    ok, _ = motors.drive_direction("RIGHT_STRAFE", 80)
    assert ok is True
    assert motors.commanded_pwm == {1: 80, 2: -80, 3: -80, 4: 80}

    # Rotate right (CW): FL+, FR-, RL+, RR-
    ok, _ = motors.drive_direction("ROTATE_RIGHT", 80)
    assert ok is True
    assert motors.commanded_pwm == {1: 80, 2: -80, 3: 80, 4: -80}

    # Diagonal Forward-Right: FL+, FR=0, RL=0, RR+
    ok, _ = motors.drive_direction("FORWARD_RIGHT", 80)
    assert ok is True
    assert motors.commanded_pwm == {1: 80, 2: 0, 3: 0, 4: 80}

    motors.stop_all_wheels()
    assert motors.commanded_pwm == {1: 0, 2: 0, 3: 0, 4: 0}


def test_pwm_clamping_and_timed_jog_autostop(tmp_path: Path) -> None:
    _, safety, _, motors = _make_motor_stack(tmp_path)
    motors.update_calibration(
        {"1": "rear_left", "2": "rear_right", "3": "front_left", "4": "front_right"},
        {"1": False, "2": False, "3": False, "4": False},
        max_pwm_limit=120,
    )
    safety.arm()

    # Request 250 PWM when max_pwm_limit=120 -> clamped to 120, auto-stops after 200ms
    ok, _ = motors.command_single_motor(1, pwm=250, direction="FORWARD", duration_ms=200)
    assert ok is True
    assert motors.commanded_pwm[1] == 120
    time.sleep(0.32)
    assert motors.commanded_pwm[1] == 0

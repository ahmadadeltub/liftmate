"""Automated tests for SafetyController state machine, Emergency Stop, Servo, and Camera."""
from __future__ import annotations

from pathlib import Path

from backend.app.camera_controller import CameraController
from backend.app.config import ConfigManager
from backend.app.diagnostics import DiagnosticsManager
from backend.app.imu_controller import IMUController
from backend.app.motor_controller import MotorController
from backend.app.safety_controller import SafetyController, SafetyState
from backend.app.serial_controller import SerialController
from backend.app.servo_controller import ServoController


def test_emergency_stop_latch_and_recovery(tmp_path: Path) -> None:
    cfg = ConfigManager(tmp_path / "robot_config.yaml")
    diag = DiagnosticsManager(log_file=tmp_path / "diag.csv")
    safety = SafetyController(cfg, diag)
    imu = IMUController(cfg, diag)
    ser = SerialController(cfg, diag, safety, imu)
    ser.set_simulation_mode(True)
    motors = MotorController(cfg, diag, safety, ser)

    safety.arm()
    ok, _ = motors.drive_direction("FORWARD", 70)
    assert ok is True
    assert safety.state == SafetyState.DRIVING

    # Trigger Emergency Stop
    safety.trigger_emergency_stop("Test E-Stop button")
    assert safety.state == SafetyState.EMERGENCY_STOP
    assert safety.estop_latched is True
    assert motors.commanded_pwm == {1: 0, 2: 0, 3: 0, 4: 0}

    # Cannot arm or drive while E-Stop is latched
    arm_ok, _ = safety.arm()
    assert arm_ok is False
    drv_ok, _ = motors.drive_direction("FORWARD", 70)
    assert drv_ok is False

    # Reset E-Stop -> transitions to IDLE (requires manual re-arm)
    rst_ok, _ = safety.reset_emergency_stop()
    assert rst_ok is True
    assert safety.state == SafetyState.IDLE

    # Still cannot drive until explicitly re-armed
    drv_ok2, _ = motors.drive_direction("FORWARD", 70)
    assert drv_ok2 is False

    safety.arm()
    assert safety.state == SafetyState.ARMED


def test_servo_startup_gate_and_angle_limits(tmp_path: Path) -> None:
    cfg = ConfigManager(tmp_path / "robot_config.yaml")
    diag = DiagnosticsManager(log_file=tmp_path / "diag.csv")
    safety = SafetyController(cfg, diag)
    imu = IMUController(cfg, diag)
    ser = SerialController(cfg, diag, safety, imu)
    ser.set_simulation_mode(True)
    servo = ServoController(cfg, diag, safety, ser)

    safety.arm()
    # Servo must start disabled and reject movement until enabled
    ok, msg = servo.set_angle(90)
    assert ok is False and "disabled" in msg.lower()

    servo.set_enabled(True)
    servo.update_limits(20, 160, inverted=False)

    # Out-of-bounds angle rejected
    ok_low, _ = servo.set_angle(5)
    assert ok_low is False
    ok_high, _ = servo.set_angle(175)
    assert ok_high is False

    # Valid angle succeeds
    ok_valid, _ = servo.set_angle(90)
    assert ok_valid is True
    assert servo.get_status()["commanded_angle"] == 90


def test_camera_graceful_unavailable_handling(tmp_path: Path) -> None:
    cfg = ConfigManager(tmp_path / "robot_config.yaml")
    diag = DiagnosticsManager(log_file=tmp_path / "diag.csv")
    cam = CameraController(cfg, diag)

    # Requesting an invalid/unconnected USB index or unavailable Picamera2 must not crash
    ok, msg = cam.start(source="usb_99", width=640, height=480)
    # Even if usb_1 fails, camera controller stays alive and does not fake a frame
    if not ok:
        assert cam.get_latest_jpeg() is None
        assert msg != ""

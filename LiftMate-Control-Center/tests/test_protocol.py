"""Automated tests for LiftMate serial protocol framing and parser."""
from __future__ import annotations

from backend.app.serial_controller import format_command, parse_serial_frame


def test_parse_standard_responses() -> None:
    assert parse_serial_frame("#,READY,#")["type"] == "READY"
    assert parse_serial_frame("#,OK,#")["type"] == "ACK"
    assert parse_serial_frame("#,PONG,#")["type"] == "PONG"
    assert parse_serial_frame("#,TIMEOUT,#")["type"] == "TIMEOUT"

    err = parse_serial_frame("#,ERR,BAD_SPEED,#")
    assert err["type"] == "ERROR"
    assert err["reason"] == "BAD_SPEED"


def test_parse_imu_never_confused_with_ack() -> None:
    imu = parse_serial_frame("#,359.94,2.94,-0.94,#")
    assert imu["type"] == "IMU"
    assert imu["type"] != "ACK"
    assert abs(imu["heading"] - 359.94) < 1e-3
    assert abs(imu["pitch"] - 2.94) < 1e-3
    assert abs(imu["roll"] - (-0.94)) < 1e-3


def test_parse_malformed_frames() -> None:
    assert parse_serial_frame("READY")["type"] == "MALFORMED"
    assert parse_serial_frame("#,OK")["type"] == "MALFORMED"
    assert parse_serial_frame("#,,#")["type"] == "MALFORMED"
    assert parse_serial_frame("#,12.3,,4.5,#")["type"] == "MALFORMED"
    assert parse_serial_frame("#,abc,def,ghi,#")["type"] == "MALFORMED"
    assert parse_serial_frame("#,999.0,0.0,0.0,#")["type"] == "MALFORMED"


def test_format_commands() -> None:
    assert format_command("STOP") == "#,STOP,#\n"
    assert format_command("DRIVE", 100, -100, 50, -50) == "#,DRIVE,100,-100,50,-50,#\n"
    assert format_command("MOTOR", 1, 100) == "#,MOTOR,1,100,#\n"
    assert format_command("JACK", "UPPER", 120) == "#,JACK,UPPER,120,#\n"
    assert format_command("SERVO", 90) == "#,SERVO,90,#\n"

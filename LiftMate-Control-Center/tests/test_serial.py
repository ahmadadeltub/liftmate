"""Automated tests for SerialController simulation, watchdog, and reconnection."""
from __future__ import annotations

import time
from pathlib import Path

from backend.app.config import ConfigManager
from backend.app.diagnostics import DiagnosticsManager
from backend.app.imu_controller import IMUController
from backend.app.safety_controller import SafetyController, SafetyState
from backend.app.serial_controller import SerialController


def _build_stack(tmp_path: Path):
    cfg = ConfigManager(tmp_path / "robot_config.yaml")
    diag = DiagnosticsManager(log_file=tmp_path / "diag.csv")
    safety = SafetyController(cfg, diag)
    imu = IMUController(cfg, diag)
    ser = SerialController(cfg, diag, safety, imu)
    return cfg, diag, safety, imu, ser


def test_simulation_connect_ping_and_commands(tmp_path: Path) -> None:
    _, _, safety, imu, ser = _build_stack(tmp_path)
    ok, _ = ser.set_simulation_mode(True)
    assert ok is True
    assert ser.connected is True
    # Connection must enter IDLE (disarmed), never auto-armed
    assert safety.state == SafetyState.IDLE

    ping_ok, latency = ser.ping()
    assert ping_ok is True
    assert latency >= 0.0

    # Wait briefly for simulated IMU frame and verify it is marked simulated=True
    time.sleep(0.15)
    imu_status = imu.get_status()
    assert imu_status["connected"] is True
    assert imu_status["simulated"] is True


def test_reconnection_requires_manual_rearm(tmp_path: Path) -> None:
    _, _, safety, _, ser = _build_stack(tmp_path)
    ser.set_simulation_mode(True)
    ok, _ = safety.arm()
    assert ok is True
    assert safety.state == SafetyState.ARMED

    # Simulate communication loss / disconnect
    ser.disconnect("USB cable unplugged")
    assert safety.state == SafetyState.DISCONNECTED

    # Reconnect: must return to IDLE (disarmed), NOT ARMED
    ser.connect("SIMULATOR")
    assert safety.state == SafetyState.IDLE


def test_simulated_wheel_watchdog_timeout(tmp_path: Path) -> None:
    _, _, safety, _, ser = _build_stack(tmp_path)
    ser.set_simulation_mode(True)
    ser.sim_state["wheel_wdt_ms"] = 150
    safety.arm()

    ok, resp = ser.send_command("DRIVE", 60, 60, 60, 60)
    assert ok is True and resp == "OK"
    assert ser.sim_state["wheels"] == [60, 60, 60, 60]

    # Wait for watchdog expiration
    time.sleep(0.32)
    assert ser.sim_state["wheels"] == [0, 0, 0, 0]

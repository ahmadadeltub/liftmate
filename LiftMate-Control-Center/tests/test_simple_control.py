from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from backend.app.main import app, system


@pytest.fixture()
def client() -> TestClient:
    prev_sim = system.serial.simulation_mode
    with TestClient(app) as tc:
        system.ai_vision.set_enabled(False)
        system.ai_vision.set_mode("DETECTION")
        system.safety.reset_emergency_stop()
        system.safety.clear_fault()
        system.serial.disconnect()
        system.serial.simulation_mode = False
        yield tc
        system.safety.reset_emergency_stop()
        system.safety.clear_fault()
        system.serial.disconnect()
        system.serial.simulation_mode = prev_sim
        system.ai_vision.set_enabled(False)
        system.ai_vision.set_mode("DETECTION")


def test_disconnected_blocks_wheels_actuators_and_servo(client: TestClient) -> None:
    ai_res = client.get("/api/ai/status")
    assert ai_res.status_code == 200
    ai_data = ai_res.json()
    assert ai_data["serial_connected"] is False
    assert ai_data["safety_state"] == "DISCONNECTED"

    for payload in (
        {"mode": "WHEELS", "vy": 120, "vx": 0, "omega": 0},
        {"mode": "ACTUATORS", "upper_pwm": 150, "lower_pwm": 0},
        {"mode": "SERVO", "servo_angle": 45},
    ):
        res = client.post("/api/simple/control", json=payload)
        assert res.status_code == 200
        data = res.json()
        assert data["ok"] is False
        assert data["serial_connected"] is False
        assert "DISCONNECTED" in data["message"]


def test_simulator_wheels_actuators_servo_and_mode_switching(client: TestClient) -> None:
    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True

    ai_res = client.get("/api/ai/status")
    assert ai_res.status_code == 200
    assert ai_res.json()["serial_connected"] is True
    assert ai_res.json()["serial_port"].startswith("SIMULATOR")

    # Drive forward directly from mobile simple control (auto-arms if IDLE)
    w_res = client.post("/api/simple/control", json={"mode": "WHEELS", "vy": 140, "vx": 0, "omega": 0})
    assert w_res.status_code == 200
    assert w_res.json()["ok"] is True
    assert w_res.json()["safety_state"] == "DRIVING"

    # Switch straight to actuators from DRIVING without getting stuck in safety state
    a_res = client.post("/api/simple/control", json={"mode": "ACTUATORS", "upper_pwm": 180, "lower_pwm": 0})
    assert a_res.status_code == 200
    assert a_res.json()["ok"] is True
    assert a_res.json()["safety_state"] == "ACTUATING_UPPER"

    # Switch straight back to wheels from ACTUATING_UPPER
    w2_res = client.post("/api/simple/control", json={"mode": "WHEELS", "vy": 100, "vx": 50, "omega": 0})
    assert w2_res.status_code == 200
    assert w2_res.json()["ok"] is True
    assert w2_res.json()["safety_state"] == "DRIVING"

    # Servo control works while connected
    s_res = client.post("/api/simple/control", json={"mode": "SERVO", "servo_angle": 60})
    assert s_res.status_code == 200
    assert s_res.json()["ok"] is True

    # Zero wheel velocity stops wheels cleanly
    z_res = client.post("/api/simple/control", json={"mode": "WHEELS", "vy": 0, "vx": 0, "omega": 0})
    assert z_res.status_code == 200
    assert z_res.json()["ok"] is True


def test_estop_blocks_motion_and_reset_all_recovers(client: TestClient) -> None:
    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True
    system.safety.arm()

    stop_res = client.post("/api/simple/control", json={"mode": "STOP_ALL"})
    assert stop_res.status_code == 200
    assert stop_res.json()["ok"] is True

    estop_res = client.post("/api/safety/estop", json={"reason": "Test E-STOP"})
    assert estop_res.status_code == 200
    assert estop_res.json()["ok"] is True
    assert system.safety.estop_latched is True

    for payload in (
        {"mode": "WHEELS", "vy": 120, "vx": 0, "omega": 0},
        {"mode": "ACTUATORS", "upper_pwm": 150, "lower_pwm": 0},
        {"mode": "SERVO", "servo_angle": 45},
    ):
        res = client.post("/api/simple/control", json=payload)
        assert res.status_code == 200
        data = res.json()
        assert data["ok"] is False
        assert "EMERGENCY_STOP" in data["message"]

    reset_res = client.post("/api/simple/control", json={"mode": "RESET_ALL", "servo_angle": 6})
    assert reset_res.status_code == 200
    assert reset_res.json()["ok"] is True
    assert system.safety.estop_latched is False

    drive_res = client.post("/api/simple/control", json={"mode": "WHEELS", "vy": 90, "vx": 0, "omega": 0})
    assert drive_res.status_code == 200
    assert drive_res.json()["ok"] is True


def test_system_shutdown_requires_confirm_and_halts_robot(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True
    system.safety.arm()

    # Unconfirmed call must be rejected cleanly
    unconf = client.post("/api/system/shutdown", json={"confirm": False})
    assert unconf.status_code == 200
    assert unconf.json()["ok"] is False
    assert unconf.json()["poweroff_scheduled"] is False
    assert system.serial.connected is True

    # Non-Pi host skips OS poweroff while cleanly stopping/latching/disconnecting controller
    non_pi = client.post("/api/system/shutdown", json={"confirm": True, "cut_usb_power": True})
    assert non_pi.status_code == 200
    non_pi_data = non_pi.json()
    assert non_pi_data["ok"] is True
    assert non_pi_data["poweroff_scheduled"] is False
    assert system.safety.estop_latched is True
    assert system.serial.connected is False

    # Simulated Raspberry Pi host schedules detached poweroff command cleanly
    spawned_cmds = []

    class DummyProc:
        pid = 9999

    def fake_popen(cmd, **kwargs):
        spawned_cmds.append(cmd)
        return DummyProc()

    monkeypatch.setattr(system, "is_raspberry_pi_host", lambda: True)
    monkeypatch.setattr("backend.app.main.subprocess.Popen", fake_popen)

    ok2, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok2 is True
    system.safety.reset_emergency_stop()
    system.safety.arm()

    pi_res = client.post("/api/system/shutdown", json={"confirm": True, "cut_usb_power": True})
    assert pi_res.status_code == 200
    pi_data = pi_res.json()
    assert pi_data["ok"] is True
    assert pi_data["poweroff_scheduled"] is True
    assert len(spawned_cmds) == 1
    assert system.serial.connected is False

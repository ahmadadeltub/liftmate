from __future__ import annotations

import time
from typing import Any, Dict

import pytest
from fastapi.testclient import TestClient

from backend.app.main import app, system


@pytest.fixture()
def client() -> TestClient:
    prev_sim = system.serial.simulation_mode
    with TestClient(app) as tc:
        system.autonomous.set_enabled(False)
        system.safety.reset_emergency_stop()
        system.safety.clear_fault()
        system.serial.sim_state["estop"] = False
        system.serial.disconnect()
        system.serial.simulation_mode = False
        system.camera.streaming = False
        system.camera.connected = False
        system.camera.frame_count = 0
        system.camera.last_frame_ts = 0.0
        yield tc
        system.autonomous.set_enabled(False)
        system.safety.reset_emergency_stop()
        system.safety.clear_fault()
        system.serial.sim_state["estop"] = False
        system.serial.disconnect()
        system.serial.simulation_mode = prev_sim
        system.camera.streaming = False
        system.camera.connected = False
        system.camera.frame_count = 0
        system.camera.last_frame_ts = 0.0


def _fake_ai_status(
    left: Dict[str, Any],
    center: Dict[str, Any],
    right: Dict[str, Any],
    vision_status: str = "CLEAR_PATH",
    model_ready: bool = True,
) -> Dict[str, Any]:
    return {
        "enabled": True,
        "mode": "DETECTION",
        "model_ready": model_ready,
        "vision_status": vision_status,
        "regions": {
            "LEFT": left,
            "CENTER": center,
            "RIGHT": right,
        },
        "warnings": [],
        "detections": [],
    }


def _enable_ready_camera() -> None:
    system.camera.streaming = True
    system.camera.connected = True
    system.camera.frame_count = 5
    system.camera.last_frame_ts = time.time()


def test_autonomous_rejected_when_disconnected_or_no_camera(client: TestClient) -> None:
    res = client.post("/api/autonomous/mode", json={"enabled": True, "speed_pwm": 90})
    assert res.status_code == 200
    data = res.json()
    assert data["ok"] is False
    assert "disconnected" in data["message"].lower()
    assert data["autonomous"]["enabled"] is False

    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True

    res_no_cam = client.post("/api/autonomous/mode", json={"enabled": True, "speed_pwm": 90})
    assert res_no_cam.status_code == 200
    assert res_no_cam.json()["ok"] is False
    assert "camera" in res_no_cam.json()["message"].lower()

    _enable_ready_camera()
    client.post("/api/safety/estop", json={"reason": "Test E-STOP"})
    res_estop = client.post("/api/autonomous/mode", json={"enabled": True, "speed_pwm": 90})
    assert res_estop.status_code == 200
    assert res_estop.json()["ok"] is False
    assert "emergency" in res_estop.json()["message"].lower()


def test_autonomous_obstacle_avoidance_and_uncertainty_stop(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True
    _enable_ready_camera()

    ai_payload = _fake_ai_status(
        left={"status": "CLEAR", "risk": "LOW", "blocked": False},
        center={"status": "CLEAR", "risk": "LOW", "blocked": False},
        right={"status": "CLEAR", "risk": "LOW", "blocked": False},
        vision_status="CLEAR_PATH",
        model_ready=True,
    )
    monkeypatch.setattr(system.ai_vision, "get_status", lambda: ai_payload)

    res = client.post("/api/autonomous/mode", json={"enabled": True, "speed_pwm": 90})
    assert res.status_code == 200
    assert res.json()["ok"] is True

    # Step 1: clear path moves forward
    st = system.autonomous.step_once()
    assert st["enabled"] is True
    assert st["action"] == "FORWARD"
    assert st["vector"]["vy"] == 1.0
    assert any(v != 0 for v in st["channel_pwms"].values())

    # Step 2: obstacle dead ahead, LEFT clear & RIGHT blocked -> brakes first, then avoids left
    ai_payload.update(
        _fake_ai_status(
            left={"status": "CLEAR", "risk": "LOW", "blocked": False},
            center={"status": "BLOCKED", "risk": "HIGH", "blocked": True},
            right={"status": "BLOCKED", "risk": "HIGH", "blocked": True},
            vision_status="OBSTACLE_BLOCKED",
        )
    )
    st_brake = system.autonomous.step_once()
    assert st_brake["action"] == "BRAKING"
    assert all(v == 0 for v in st_brake["channel_pwms"].values())

    # Advance past brake hold window
    system.autonomous._brake_until = time.time() - 0.05
    st_left = system.autonomous.step_once()
    assert st_left["action"] == "AVOID_LEFT"
    assert st_left["vector"]["omega"] < 0

    # Real Pi status string format ("CLEAR CAMERA REGION") works just like "CLEAR"
    ai_payload.update(
        _fake_ai_status(
            left={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
            center={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
            right={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
            vision_status="CLEAR CAMERA REGION",
        )
    )
    st_pi = system.autonomous.step_once()
    assert st_pi["action"] == "FORWARD"

    # Step 3: uncertainty -> immediate halt
    ai_payload.update(
        _fake_ai_status(
            left={"status": "UNKNOWN / UNRELIABLE DETECTION", "risk": "UNKNOWN", "blocked": False},
            center={"status": "UNKNOWN / UNRELIABLE DETECTION", "risk": "UNKNOWN", "blocked": False},
            right={"status": "UNKNOWN / UNRELIABLE DETECTION", "risk": "UNKNOWN", "blocked": False},
            vision_status="CAMERA VISION UNAVAILABLE",
        )
    )
    st_uncertain = system.autonomous.step_once()
    assert st_uncertain["action"] == "PAUSED_UNCERTAIN"
    assert all(v == 0 for v in st_uncertain["channel_pwms"].values())


def test_manual_control_overrides_autonomous_mode(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True
    _enable_ready_camera()
    monkeypatch.setattr(
        system.ai_vision,
        "get_status",
        lambda: _fake_ai_status(
            left={"status": "CLEAR", "risk": "LOW", "blocked": False},
            center={"status": "CLEAR", "risk": "LOW", "blocked": False},
            right={"status": "CLEAR", "risk": "LOW", "blocked": False},
        ),
    )

    res = client.post("/api/autonomous/mode", json={"enabled": True, "speed_pwm": 95})
    assert res.status_code == 200
    assert res.json()["ok"] is True

    # Sending manual wheel control must immediately disengage autonomous mode
    man = client.post("/api/simple/control", json={"mode": "WHEELS", "vy": 100, "vx": 0, "omega": 0})
    assert man.status_code == 200
    assert man.json()["ok"] is True

    auto_after = client.get("/api/autonomous/status").json()["autonomous"]
    assert auto_after["enabled"] is False
    assert auto_after["mode"] == "MANUAL"


def test_mobile_map_page_served(client: TestClient) -> None:
    res = client.get("/mobile-map")
    assert res.status_code == 200
    assert "liftmateOfflineMap" in res.text
    assert "setAppTheme" in res.text


def test_autonomous_corridor_threading_and_wall_retreat(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ok, _ = system.serial.connect(port="SIMULATOR", baud_rate=115200)
    assert ok is True
    _enable_ready_camera()

    ai_payload = _fake_ai_status(
        left={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
        center={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
        right={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
        vision_status="CLEAR CAMERA REGION",
    )
    ai_payload.update({
        "corridor_scores": [0.95, 0.96, 0.94, 0.72, 0.65, 0.55, 0.25, 0.15, 0.12],
        "best_heading": -0.75,
        "open_space_ratio": 0.58,
        "wall_detected": True,
    })
    monkeypatch.setattr(system.ai_vision, "get_status", lambda: ai_payload)

    res = client.post("/api/autonomous/mode", json={"enabled": True, "speed_pwm": 90})
    assert res.status_code == 200 and res.json()["ok"] is True

    # 1. Proactive corridor threading slides smoothly toward wider open left gap
    st_steer = system.autonomous.step_once()
    assert st_steer["action"] == "STEER_OPEN_LEFT"
    assert st_steer["vector"]["vy"] > 0.8
    assert st_steer["vector"]["vx"] < 0
    assert st_steer["vector"]["omega"] < 0

    # 2. Flush wall dead-end -> brakes, then reverses safely to gain turn radius, then scans
    ai_payload.update(
        _fake_ai_status(
            left={"status": "POTENTIAL OBSTACLE ON LEFT", "risk": "HIGH", "blocked": True},
            center={"status": "POTENTIAL OBSTACLE AHEAD", "risk": "HIGH", "blocked": True},
            right={"status": "POTENTIAL OBSTACLE ON RIGHT", "risk": "HIGH", "blocked": True},
            vision_status="POTENTIAL OBSTACLE – STOP",
        )
    )
    ai_payload.update({
        "corridor_scores": [0.08] * 9,
        "best_heading": 0.0,
        "open_space_ratio": 0.08,
        "wall_detected": True,
    })
    st_brake = system.autonomous.step_once()
    assert st_brake["action"] == "BRAKING"

    system.autonomous._brake_until = time.time() - 0.05
    st_retreat = system.autonomous.step_once()
    assert st_retreat["action"] == "RETREAT_BACK"
    assert st_retreat["vector"]["vy"] < 0

    system.autonomous._retreat_until = time.time() - 0.05
    st_scan = system.autonomous.step_once()
    assert st_scan["action"] == "SCAN_ROTATE"
    assert abs(st_scan["vector"]["omega"]) > 0.5

    # 3. Right corridor opens -> immediately stops scanning and maneuvers right with lateral assist
    ai_payload.update(
        _fake_ai_status(
            left={"status": "POTENTIAL OBSTACLE ON LEFT", "risk": "HIGH", "blocked": True},
            center={"status": "POTENTIAL OBSTACLE AHEAD", "risk": "HIGH", "blocked": True},
            right={"status": "CLEAR CAMERA REGION", "risk": "LOW", "blocked": False},
            vision_status="POTENTIAL OBSTACLE AHEAD",
        )
    )
    ai_payload.update({
        "corridor_scores": [0.10, 0.10, 0.10, 0.15, 0.12, 0.18, 0.85, 0.92, 0.90],
        "best_heading": 0.75,
        "open_space_ratio": 0.36,
        "wall_detected": True,
    })
    st_avoid_r = system.autonomous.step_once()
    assert st_avoid_r["action"] == "AVOID_RIGHT"
    assert st_avoid_r["vector"]["omega"] > 0
    assert st_avoid_r["vector"]["vx"] > 0

"""Unit and Integration Tests for LiftMate Offline AI Vision and Obstacle Avoidance."""
from __future__ import annotations

import numpy as np
from fastapi.testclient import TestClient

from backend.app.main import app, system


def test_ai_offline_models_loaded() -> None:
    """Verifies offline MobileNet-SSD and YOLOv4-tiny models load locally without internet."""
    st = system.ai_vision.get_status()
    assert st["model_ready"] is True
    assert "MobileNet-SSD" in st["model_name"]


def test_ai_enable_disable_and_modes() -> None:
    """Verifies AI ON/OFF, Mode 1 (DETECTION), Mode 2 (SAFETY_ASSIST), and Mode 3 (AI_NAVIGATION locked)."""
    client = TestClient(app)

    # Disable AI
    r_off = client.post("/api/ai/config", json={"enabled": False})
    assert r_off.status_code == 200
    assert r_off.json()["ai_vision"]["enabled"] is False
    assert r_off.json()["ai_vision"]["vision_status"] == "AI OFF"

    # Enable Mode 1: DETECTION
    r_det = client.post("/api/ai/config", json={"enabled": True, "mode": "DETECTION"})
    assert r_det.status_code == 200
    assert r_det.json()["ok"] is True
    assert r_det.json()["ai_vision"]["enabled"] is True
    assert r_det.json()["ai_vision"]["mode"] == "DETECTION"

    # Enable Mode 2: SAFETY_ASSIST
    r_saf = client.post("/api/ai/config", json={"enabled": True, "mode": "SAFETY_ASSIST"})
    assert r_saf.status_code == 200
    assert r_saf.json()["ok"] is True
    assert r_saf.json()["ai_vision"]["mode"] == "SAFETY_ASSIST"

    # Attempt Mode 3: AI_NAVIGATION (must remain locked until physical validation)
    r_nav = client.post("/api/ai/config", json={"enabled": True, "mode": "AI_NAVIGATION"})
    assert r_nav.status_code == 200
    assert r_nav.json()["ok"] is False
    assert r_nav.json()["ai_vision"]["ai_navigation_locked"] is True
    assert r_nav.json()["ai_vision"]["mode"] == "SAFETY_ASSIST"

    # Restore Mode 1
    client.post("/api/ai/config", json={"enabled": True, "mode": "DETECTION"})


def test_car_counting_regions_and_safety_assist_stop(monkeypatch) -> None:
    """Verifies car counting, LEFT/CENTER/RIGHT regions, and Safety Assist stop trigger."""
    # Mock _run_mobilenet_ssd to return a close CAR in CENTER and a TRUCK on RIGHT
    fake_dets = [
        {
            "category": "CAR",
            "raw_class": "car",
            "confidence": 94.0,
            "bbox": [0.35, 0.30, 0.65, 0.88],
            "px_bbox": [112, 72, 208, 211],
        },
        {
            "category": "TRUCK",
            "raw_class": "truck",
            "confidence": 91.0,
            "bbox": [0.70, 0.25, 0.95, 0.65],
            "px_bbox": [224, 60, 304, 156],
        },
    ]
    monkeypatch.setattr(system.ai_vision, "_run_mobilenet_ssd", lambda frame: (fake_dets, False))
    monkeypatch.setattr(system.ai_vision, "_run_yolov4_tiny", lambda frame: ([], False))

    system.ai_vision.set_mode("SAFETY_ASSIST")
    frame = np.full((240, 320, 3), 120, dtype=np.uint8)
    st = system.ai_vision.process_frame(frame, run_yolo_supplement=False)

    assert st["car_count"] == 1
    assert st["vehicle_count"] == 2
    assert st["car_in_forward_region"] is True
    assert st["regions"]["CENTER"]["blocked"] is True
    assert st["regions"]["CENTER"]["risk"] == "HIGH"
    assert st["regions"]["RIGHT"]["blocked"] is True
    assert st["regions"]["LEFT"]["blocked"] is False
    assert st["active_warning"] == "POTENTIAL OBSTACLE – STOP"
    assert st["safety_assist_stops_count"] >= 1

    # Verify forward drive is blocked by Safety Assist while CENTER is HIGH risk
    client = TestClient(app)
    r_drive = client.post("/api/simple/control", json={"mode": "WHEELS", "vy": 120, "vx": 0, "omega": 0})
    assert r_drive.status_code == 200
    assert r_drive.json()["ok"] is False
    assert "POTENTIAL OBSTACLE" in r_drive.json()["message"]

    # Restore DETECTION mode
    system.ai_vision.set_mode("DETECTION")


def test_camera_unavailable_and_empty_frame_handling() -> None:
    """Verifies empty or disconnected camera frames are handled safely without crashing."""
    st = system.ai_vision.process_frame(np.array([]), run_yolo_supplement=False)
    assert st["vision_status"] == "CAMERA VISION UNAVAILABLE"
    assert st["active_warning"] == "CAMERA VISION UNAVAILABLE"

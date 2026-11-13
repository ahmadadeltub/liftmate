"""Configuration manager for LiftMate Control Center."""
from __future__ import annotations

import copy
import threading
from pathlib import Path
from typing import Any, Dict

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "robot_config.yaml"
LOGS_DIR = PROJECT_ROOT / "logs"
CAPTURES_DIR = PROJECT_ROOT / "logs" / "captures"
DATA_DIR = PROJECT_ROOT / "data"

DEFAULT_CONFIG: Dict[str, Any] = {
    "system": {
        "project_name": "LiftMate",
        "subtitle": "Advanced Robotic Control & Diagnostic Center",
        "simulation_mode": False,
        "host": "127.0.0.1",
        "port": 8000,
        "allow_lan": False,
    },
    "serial": {
        "port": "auto",
        "baud_rate": 115200,
        "timeout_s": 0.15,
        "heartbeat_interval_s": 1.0,
    },
    "pins": {
        "wheel_motors": {
            "1": {"pwm": 4, "in1": 26, "in2": 27, "note": "Position unconfirmed in original sketch"},
            "2": {"pwm": 5, "in1": 28, "in2": 29, "note": "Labeled right rear in original sketch"},
            "3": {"pwm": 6, "in1": 44, "in2": 45, "note": "Labeled left front in original sketch"},
            "4": {"pwm": 7, "in1": 46, "in2": 47, "note": "Labeled right front in original sketch"},
        },
        "lower_actuator_group": {"pwm": 2, "in1": 22, "in2": 23},
        "upper_actuator_group": {"pwm": 3, "in1": 24, "in2": 25},
        "servo_pin": 8,
        "bno055_i2c": {"sda": 20, "scl": 21, "address": "0x29"},
    },
    "wheel_calibration": {
        "calibrated": False,
        "positions": {
            "1": "rear_left",
            "2": "rear_right",
            "3": "front_left",
            "4": "front_right",
        },
        "inverted": {
            "1": False,
            "2": False,
            "3": False,
            "4": False,
        },
        "default_test_pwm": 50,
        "default_drive_pwm": 75,
        "max_pwm_limit": 200,
        "jog_duration_ms": 350,
        "ramp_step_pwm": 25,
    },
    "actuators": {
        "driver_electrical_verified": False,
        "allow_simultaneous_groups": False,
        "require_imu_for_lifting": False,
        "upper": {
            "name": "UPPER JACK – 100mm",
            "stroke_mm": 100,
            "rated_force_n": 50,
            "actuator_count": 4,
            "voltage_v": 12,
            "inverted": False,
            "default_pwm": 120,
            "max_pwm": 255,
            "jog_pulse_ms": 300,
            "max_continuous_run_ms": 10000,
            "position_feedback_available": False,
        },
        "lower": {
            "name": "LOWER BASE – 30mm",
            "stroke_mm": 30,
            "rated_force_n": 0,
            "actuator_count": 4,
            "voltage_v": 12,
            "inverted": False,
            "default_pwm": 120,
            "max_pwm": 255,
            "jog_pulse_ms": 250,
            "max_continuous_run_ms": 5000,
            "position_feedback_available": False,
        },
    },
    "servo": {
        "pin": 8,
        "enabled": False,
        "initial_center_deg": 90,
        "safe_min_deg": 10,
        "safe_max_deg": 170,
        "hard_min_deg": 0,
        "hard_max_deg": 180,
        "inverted": False,
    },
    "imu": {
        "i2c_address": "0x29",
        "tilt_warning_deg": 8.0,
        "tilt_critical_deg": 15.0,
        "stale_timeout_ms": 600,
    },
    "camera": {
        "preferred_source": "auto",
        "width": 640,
        "height": 480,
        "target_fps": 20,
        "usb_index": 0,
        "remote_pi_host": "192.168.18.36",
        "remote_pi_port": 8081,
        "remote_pi_user": "pi",
    },
    "safety": {
        "wheel_watchdog_ms": 450,
        "actuator_watchdog_ms": 600,
        "require_manual_arm": True,
        "driver_profile": "L298N_OR_PWM_2DIR",
    },
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = copy.deepcopy(v)
    return result


class ConfigManager:
    """Thread-safe configuration loader and saver."""

    def __init__(self, config_path: Path | None = None) -> None:
        self.config_path = config_path or DEFAULT_CONFIG_PATH
        self._lock = threading.RLock()
        self._data: Dict[str, Any] = copy.deepcopy(DEFAULT_CONFIG)
        LOGS_DIR.mkdir(parents=True, exist_ok=True)
        CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
        self.load()

    def load(self) -> Dict[str, Any]:
        with self._lock:
            if self.config_path.exists():
                try:
                    with self.config_path.open("r", encoding="utf-8") as f:
                        loaded = yaml.safe_load(f) or {}
                    self._data = _deep_merge(DEFAULT_CONFIG, loaded)
                except Exception:
                    self._data = copy.deepcopy(DEFAULT_CONFIG)
            else:
                self.save()
            return copy.deepcopy(self._data)

    def save(self) -> None:
        with self._lock:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            with self.config_path.open("w", encoding="utf-8") as f:
                yaml.safe_dump(self._data, f, sort_keys=False)

    def get_all(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._data)

    def get_section(self, section: str) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._data.get(section, {}))

    def update_section(self, section: str, values: Dict[str, Any], persist: bool = True) -> Dict[str, Any]:
        with self._lock:
            current = self._data.get(section, {})
            if isinstance(current, dict):
                self._data[section] = _deep_merge(current, values)
            else:
                self._data[section] = copy.deepcopy(values)
            if persist:
                self.save()
            return copy.deepcopy(self._data[section])

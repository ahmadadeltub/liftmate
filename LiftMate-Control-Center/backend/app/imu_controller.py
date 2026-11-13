"""BNO055 IMU Orientation Telemetry Controller for LiftMate."""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List

from .config import ConfigManager
from .diagnostics import DiagnosticsManager


class IMUController:
    """Tracks real-time BNO055 orientation telemetry, staleness, and tilt thresholds."""

    def __init__(self, config: ConfigManager, diagnostics: DiagnosticsManager) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics

        self.heading: float | None = None
        self.pitch: float | None = None
        self.roll: float | None = None
        self.last_update_ts: float = 0.0
        self.sample_count: int = 0
        self.simulated: bool = False
        self.sensor_error: str = ""
        self._history: Deque[Dict[str, Any]] = deque(maxlen=120)
        self._sim_phase: float = 0.0

    def update_from_frame(
        self,
        heading: float,
        pitch: float,
        roll: float,
        simulated: bool = False,
    ) -> None:
        now = time.time()
        with self._lock:
            self.heading = round(float(heading) % 360.0, 2)
            self.pitch = round(float(pitch), 2)
            self.roll = round(float(roll), 2)
            self.last_update_ts = now
            self.sample_count += 1
            self.simulated = simulated
            self.sensor_error = ""
            self._history.append({
                "ts": round(now, 2),
                "heading": self.heading,
                "pitch": self.pitch,
                "roll": self.roll,
                "simulated": simulated,
            })

    def report_error(self, reason: str) -> None:
        with self._lock:
            self.sensor_error = reason
        self.diagnostics.log_event("WARNING", "IMU", "IMU_SENSOR_ERROR", reason)

    def mark_disconnected(self, reason: str = "IMU disconnected") -> None:
        with self._lock:
            self.heading = None
            self.pitch = None
            self.roll = None
            self.last_update_ts = 0.0
            if reason:
                self.sensor_error = reason

    def step_simulation(self, active_motion: str = "IDLE") -> Dict[str, Any]:
        """Generates clearly-labeled simulated orientation values in simulation mode."""
        self._sim_phase += 0.12
        h = (180.0 + 8.0 * math.sin(self._sim_phase * 0.25)) % 360.0
        amp = 2.5 if active_motion != "IDLE" else 0.4
        p = amp * math.sin(self._sim_phase)
        r = amp * math.cos(self._sim_phase * 0.8)
        self.update_from_frame(h, p, r, simulated=True)
        return self.get_status()

    def update_thresholds(self, warning_deg: float, critical_deg: float) -> Dict[str, Any]:
        if warning_deg <= 0 or critical_deg <= warning_deg:
            raise ValueError("Critical threshold must be greater than warning threshold (> 0).")
        self.config.update_section("imu", {
            "tilt_warning_deg": round(float(warning_deg), 2),
            "tilt_critical_deg": round(float(critical_deg), 2),
        })
        self.diagnostics.log_event(
            "INFO",
            "IMU",
            "THRESHOLDS_UPDATED",
            f"warn={warning_deg} deg, crit={critical_deg} deg",
        )
        return self.get_status()

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            imu_cfg = self.config.get_section("imu")
            stale_ms = float(imu_cfg.get("stale_timeout_ms", 600))
            warn_deg = float(imu_cfg.get("tilt_warning_deg", 8.0))
            crit_deg = float(imu_cfg.get("tilt_critical_deg", 15.0))

            now = time.time()
            age_ms = (now - self.last_update_ts) * 1000.0 if self.last_update_ts > 0 else None
            connected = (
                age_ms is not None
                and age_ms <= stale_ms
                and self.heading is not None
                and self.pitch is not None
                and self.roll is not None
            )

            # Never assume missing IMU data means zero tilt!
            if connected and self.pitch is not None and self.roll is not None:
                max_tilt = round(max(abs(self.pitch), abs(self.roll)), 2)
                tilt_warning = max_tilt >= warn_deg
                tilt_critical = max_tilt >= crit_deg
            else:
                max_tilt = None
                tilt_warning = False
                tilt_critical = False

            return {
                "connected": connected,
                "simulated": self.simulated if connected else False,
                "i2c_address": imu_cfg.get("i2c_address", "0x29"),
                "heading": self.heading if connected else None,
                "pitch": self.pitch if connected else None,
                "roll": self.roll if connected else None,
                "max_tilt_deg": max_tilt,
                "tilt_warning": tilt_warning,
                "tilt_critical": tilt_critical,
                "tilt_warning_deg": warn_deg,
                "tilt_critical_deg": crit_deg,
                "last_update_age_ms": round(age_ms, 1) if age_ms is not None else None,
                "sample_count": self.sample_count,
                "sensor_error": self.sensor_error if not connected else "",
                "calibration_state": (
                    "SIMULATED (NDOF)"
                    if (connected and self.simulated)
                    else ("NDOF Active (0x29)" if connected else "Unavailable (Sensor Offline)")
                ),
                "history": list(self._history)[-40:],
            }

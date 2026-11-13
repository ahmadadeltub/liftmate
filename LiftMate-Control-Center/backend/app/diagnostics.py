"""Diagnostics and structured CSV event logging for LiftMate Control Center."""
from __future__ import annotations

import csv
import io
import platform
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List

from .config import LOGS_DIR


class DiagnosticsManager:
    """Thread-safe diagnostics tracker and CSV logger."""

    def __init__(self, max_events: int = 500, log_file: Path | None = None) -> None:
        self._lock = threading.RLock()
        self._events: Deque[Dict[str, Any]] = deque(maxlen=max_events)
        LOGS_DIR.mkdir(parents=True, exist_ok=True)
        self.csv_path = log_file or (LOGS_DIR / "liftmate_diagnostics.csv")
        self._ensure_csv_header()

        self.last_command: str = "NONE"
        self.last_command_ts: float = 0.0
        self.last_response: str = "NONE"
        self.last_response_ts: float = 0.0
        self.latency_ms: float = 0.0
        self.error_count: int = 0
        self.timeout_count: int = 0
        self.stop_event_count: int = 0
        self.last_fault: str = "None"
        self.start_time: float = time.time()

    def _ensure_csv_header(self) -> None:
        if not self.csv_path.exists():
            with self.csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp_iso", "level", "subsystem", "event", "details"])

    def log_event(self, level: str, subsystem: str, event: str, details: str = "") -> Dict[str, Any]:
        now = datetime.now(timezone.utc).astimezone()
        ts_iso = now.isoformat(timespec="milliseconds")
        entry = {
            "timestamp": ts_iso,
            "epoch": round(time.time(), 3),
            "level": level.upper(),
            "subsystem": subsystem.upper(),
            "event": event,
            "details": details,
        }
        with self._lock:
            self._events.appendleft(entry)
            if level.upper() in ("ERROR", "FAULT", "CRITICAL"):
                self.error_count += 1
                self.last_fault = f"[{subsystem}] {event}: {details}".strip(": ")
            if "STOP" in event.upper():
                self.stop_event_count += 1
            if "TIMEOUT" in event.upper():
                self.timeout_count += 1
            try:
                with self.csv_path.open("a", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow([ts_iso, entry["level"], entry["subsystem"], event, details])
            except Exception:
                pass
        return entry

    def record_command(self, cmd: str) -> None:
        with self._lock:
            self.last_command = cmd.strip()
            self.last_command_ts = time.time()

    def record_response(self, resp: str, latency_ms: float | None = None) -> None:
        with self._lock:
            self.last_response = resp.strip()
            self.last_response_ts = time.time()
            if latency_ms is not None:
                self.latency_ms = round(latency_ms, 2)
            elif self.last_command_ts > 0:
                dt = (self.last_response_ts - self.last_command_ts) * 1000.0
                if 0 <= dt < 5000:
                    self.latency_ms = round(dt, 2)

    def get_events(self, limit: int = 100) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._events)[:limit]

    def clear_fault_counters(self) -> None:
        with self._lock:
            self.last_fault = "None"
        self.log_event("INFO", "DIAGNOSTICS", "FAULT_HISTORY_ACK", "Operator cleared active fault banner")

    def export_csv_string(self) -> str:
        with self._lock:
            output = io.StringIO()
            writer = csv.writer(output)
            writer.writerow(["timestamp_iso", "level", "subsystem", "event", "details"])
            for item in reversed(self._events):
                writer.writerow([
                    item["timestamp"],
                    item["level"],
                    item["subsystem"],
                    item["event"],
                    item["details"],
                ])
            return output.getvalue()

    def get_summary(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "last_command": self.last_command,
                "last_response": self.last_response,
                "latency_ms": self.latency_ms,
                "error_count": self.error_count,
                "timeout_count": self.timeout_count,
                "stop_event_count": self.stop_event_count,
                "last_fault": self.last_fault,
                "uptime_s": round(time.time() - self.start_time, 1),
                "host_platform": f"{platform.system()} {platform.release()} ({platform.machine()})",
            }

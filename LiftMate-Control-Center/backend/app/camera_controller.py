"""Camera Controller supporting Picamera2 (CSI), Remote Pi Camera, and OpenCV (USB)."""
from __future__ import annotations

import http.client
import io
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import CAPTURES_DIR, ConfigManager
from .diagnostics import DiagnosticsManager

try:
    from picamera2 import Picamera2  # type: ignore

    HAS_PICAMERA2 = True
except Exception:
    Picamera2 = None  # type: ignore
    HAS_PICAMERA2 = False


class CameraController:
    """Non-blocking camera manager for Picamera2, Remote Pi CSI agent, and OpenCV USB cameras.

    Never fakes a live preview: if no real camera is streaming, returns clear error diagnostics.
    """

    def __init__(self, config: ConfigManager, diagnostics: DiagnosticsManager) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics

        cam_cfg = self.config.get_section("camera")
        self.source: str = str(cam_cfg.get("preferred_source", "auto"))
        self.active_backend: str = "NONE"
        self.width: int = int(cam_cfg.get("width", 320))
        self.height: int = int(cam_cfg.get("height", 240))
        self.target_fps: int = int(cam_cfg.get("target_fps", 10))
        self.usb_index: int = int(cam_cfg.get("usb_index", 0))

        self.streaming: bool = False
        self.connected: bool = False
        self.measured_fps: float = 0.0
        self.frame_count: int = 0
        self.error_message: str = ""
        self.last_frame_jpeg: Optional[bytes] = None
        self.last_frame_ts: float = 0.0
        self.saved_photos: List[str] = []

        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        self._picam: Any = None
        self._cv_cap: Optional[cv2.VideoCapture] = None

    def detect_sources(self) -> List[Dict[str, Any]]:
        """Probes available camera backends without blocking the control loop."""
        sources: List[Dict[str, Any]] = [
            {
                "id": "auto",
                "name": "Auto-Detect (Picamera2 -> Remote Pi -> USB OpenCV)",
                "available": True,
            },
            {
                "id": "picamera2",
                "name": "Raspberry Pi Camera (Local Picamera2 CSI)",
                "available": HAS_PICAMERA2,
            },
            {
                "id": "remote_pi",
                "name": f"Raspberry Pi 3 Camera v2.1 ({self.config.get_section('camera').get('remote_pi_host', '192.168.18.36')})",
                "available": True,
            },
            {
                "id": "usb_0",
                "name": "USB / System Camera #0 (OpenCV)",
                "available": True,
            },
            {
                "id": "usb_1",
                "name": "USB / System Camera #1 (OpenCV)",
                "available": True,
            },
        ]
        return sources

    def start(
        self,
        source: Optional[str] = None,
        width: Optional[int] = None,
        height: Optional[int] = None,
    ) -> Tuple[bool, str]:
        with self._lock:
            req_source = source or self.source
            if (
                self.streaming
                and self.connected
                and self.last_frame_jpeg is not None
                and req_source in ("auto", self.source)
            ):
                return True, f"Already streaming {self.active_backend} ({self.width}x{self.height})"

            if self.streaming:
                self.stop()

            if source:
                self.source = source
            if width and height:
                self.width = int(width)
                self.height = int(height)

            self.config.update_section("camera", {
                "preferred_source": self.source,
                "width": self.width,
                "height": self.height,
            })

            self.error_message = ""
            self.last_frame_jpeg = None
            self.measured_fps = 0.0
            self._stop_event.clear()

            chosen = self.source
            if chosen == "auto":
                if HAS_PICAMERA2 and self._try_init_picamera2():
                    chosen = "picamera2"
                elif self._check_remote_pi_available():
                    chosen = "remote_pi"
                else:
                    chosen = "usb_0"

            if chosen == "picamera2":
                if self._picam is None and not self._try_init_picamera2():
                    msg = self.error_message or "Picamera2 CSI camera unavailable on this host."
                    self.diagnostics.log_event("WARNING", "CAMERA", "PICAMERA2_UNAVAILABLE", msg)
                    return False, msg
                self.active_backend = "Picamera2 (CSI)"
                self.streaming = True
                self.connected = True
                self._worker_thread = threading.Thread(target=self._picamera2_loop, daemon=True)
                self._worker_thread.start()
                self.diagnostics.log_event("INFO", "CAMERA", "CAMERA_STARTED", self.active_backend)
                return True, f"Started {self.active_backend} ({self.width}x{self.height})"

            if chosen == "remote_pi":
                ok, msg = self._check_remote_pi_status()
                self.active_backend = "Remote Pi 3 Camera v2.1 (Picamera2 Stream)"
                self.streaming = True
                self.connected = ok
                if not ok:
                    self.error_message = msg
                self._worker_thread = threading.Thread(target=self._remote_pi_loop, daemon=True)
                self._worker_thread.start()
                self.diagnostics.log_event("INFO", "CAMERA", "CAMERA_STARTED", self.active_backend)
                return True, f"Started {self.active_backend}"

            # OpenCV USB / Local Camera
            idx = 1 if chosen == "usb_1" else 0
            self.usb_index = idx
            ok, msg = self._try_init_opencv(idx)
            if not ok:
                self.error_message = msg
                self.connected = False
                self.streaming = False
                self.diagnostics.log_event("WARNING", "CAMERA", "OPENCV_CAMERA_UNAVAILABLE", msg)
                return False, msg

            self.active_backend = f"OpenCV VideoCapture(#{idx})"
            self.streaming = True
            self.connected = True
            self._worker_thread = threading.Thread(target=self._opencv_loop, daemon=True)
            self._worker_thread.start()
            self.diagnostics.log_event("INFO", "CAMERA", "CAMERA_STARTED", self.active_backend)
            return True, f"Started {self.active_backend} ({self.width}x{self.height})"

    def stop(self) -> Tuple[bool, str]:
        with self._lock:
            self._stop_event.set()
            self.streaming = False
            self.connected = False
            self.measured_fps = 0.0

        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.5)

        with self._lock:
            self._cleanup_resources()
            self.active_backend = "NONE"
            self.diagnostics.log_event("INFO", "CAMERA", "CAMERA_STOPPED", "Operator stopped camera")
            return True, "Camera stopped."

    def _cleanup_resources(self) -> None:
        if self._picam is not None:
            try:
                self._picam.stop()
            except Exception:
                pass
            try:
                self._picam.close()
            except Exception:
                pass
            self._picam = None

        if self._cv_cap is not None:
            try:
                self._cv_cap.release()
            except Exception:
                pass
            self._cv_cap = None

    def _try_init_picamera2(self) -> bool:
        if not HAS_PICAMERA2 or Picamera2 is None:
            self.error_message = "Picamera2 library is not installed on this machine."
            return False
        try:
            cams = Picamera2.global_camera_info()
            if not cams:
                self.error_message = "No Raspberry Pi CSI camera detected by libcamera/Picamera2."
                return False
            picam = Picamera2()
            cfg = picam.create_preview_configuration(main={"size": (self.width, self.height)})
            picam.configure(cfg)
            picam.start()
            self._picam = picam
            return True
        except Exception as exc:
            self.error_message = f"Picamera2 initialization error: {exc}"
            self._cleanup_resources()
            return False

    def _check_remote_pi_available(self) -> bool:
        ok, _ = self._check_remote_pi_status()
        return ok

    def _resolve_pi_camera_host(self) -> Tuple[str, int]:
        cam_cfg = self.config.get_section("camera")
        cfg_host = str(cam_cfg.get("remote_pi_host", "192.168.18.36"))
        port = int(cam_cfg.get("remote_pi_port", 8081))
        candidates = ["127.0.0.1"] if cfg_host == "127.0.0.1" else ["127.0.0.1", cfg_host]
        import json
        for h in candidates:
            try:
                with urllib.request.urlopen(f"http://{h}:{port}/status", timeout=0.8) as resp:
                    data = json.loads(resp.read().decode("utf-8"))
                    if data.get("streaming") and data.get("has_frame"):
                        return h, port
            except Exception:
                continue
        return cfg_host, port

    def _check_remote_pi_status(self) -> Tuple[bool, str]:
        cam_cfg = self.config.get_section("camera")
        cfg_host = str(cam_cfg.get("remote_pi_host", "192.168.18.36"))
        port = int(cam_cfg.get("remote_pi_port", 8081))
        candidates = ["127.0.0.1"] if cfg_host == "127.0.0.1" else ["127.0.0.1", cfg_host]
        last_err = ""
        import json
        for host in candidates:
            url = f"http://{host}:{port}/status"
            try:
                with urllib.request.urlopen(url, timeout=1.0) as resp:
                    data = json.loads(resp.read().decode("utf-8"))
                    if data.get("streaming") and data.get("has_frame"):
                        return True, f"Pi Camera streaming ({host}:{port})"
                    err = data.get("error") or "Pi camera agent running, waiting for USB frame"
                    last_err = f"Raspberry Pi ({host}): {err}"
            except Exception as exc:
                last_err = f"Pi camera service at {host}:{port} unreachable ({exc})."
        return False, last_err

    def _try_init_opencv(self, index: int) -> Tuple[bool, str]:
        try:
            cap = cv2.VideoCapture(index)
            if not cap or not cap.isOpened():
                if cap:
                    cap.release()
                return False, f"OpenCV camera #{index} could not be opened or is not connected."
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                return False, f"OpenCV camera #{index} opened but returned no video frames."
            ok_enc, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok_enc:
                self.last_frame_jpeg = buf.tobytes()
                self.last_frame_ts = time.time()
                self.frame_count += 1
            self._cv_cap = cap
            return True, "OK"
        except Exception as exc:
            return False, f"OpenCV camera #{index} error: {exc}"

    def _picamera2_loop(self) -> None:
        fps_counter = 0
        fps_t0 = time.time()
        interval = 1.0 / max(1, min(self.target_fps, 30))

        while not self._stop_event.is_set() and self._picam is not None:
            loop_t0 = time.time()
            try:
                buf = io.BytesIO()
                self._picam.capture_file(buf, format="jpeg")
                jpeg_bytes = buf.getvalue()
                if jpeg_bytes:
                    with self._lock:
                        self.last_frame_jpeg = jpeg_bytes
                        self.last_frame_ts = time.time()
                        self.frame_count += 1
                        self.connected = True
                    fps_counter += 1
            except Exception as exc:
                with self._lock:
                    self.error_message = f"Picamera2 capture timeout/error: {exc}"
                    self.connected = False
                    self.streaming = False
                self.diagnostics.log_event("ERROR", "CAMERA", "PICAMERA2_RUNTIME_ERROR", str(exc))
                break

            now = time.time()
            if now - fps_t0 >= 1.0:
                with self._lock:
                    self.measured_fps = round(fps_counter / (now - fps_t0), 1)
                fps_counter = 0
                fps_t0 = now

            elapsed = time.time() - loop_t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

        with self._lock:
            self._cleanup_resources()

    @staticmethod
    def _sanitize_jpeg(raw_bytes: bytes) -> Optional[bytes]:
        """Extracts the single cleanest JPEG frame and re-encodes via OpenCV to prevent browser flicker."""
        if not raw_bytes or len(raw_bytes) < 500:
            return None
        end = raw_bytes.rfind(b"\xff\xd9")
        if end == -1:
            return None
        start = raw_bytes.rfind(b"\xff\xd8\xff", 0, end)
        if start == -1:
            start = raw_bytes.rfind(b"\xff\xd8", 0, end)
        if start == -1:
            return None
        clean_slice = raw_bytes[start : end + 2]
        if len(clean_slice) < 500:
            return None
        try:
            arr = np.frombuffer(clean_slice, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None or img.size == 0:
                return None
            if img.shape[1] > 320 or img.shape[0] > 240:
                img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
            ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            if not ok:
                return None
            return enc.tobytes()
        except Exception:
            return None

    def _remote_pi_loop(self) -> None:
        host, port = self._resolve_pi_camera_host()
        fps_counter = 0
        fps_t0 = time.time()
        interval = 1.0 / max(1, min(self.target_fps, 25))
        consecutive_failures = 0
        conn: Optional[http.client.HTTPConnection] = None

        while not self._stop_event.is_set():
            loop_t0 = time.time()
            try:
                if conn is None:
                    if consecutive_failures > 0 and consecutive_failures % 3 == 0:
                        host, port = self._resolve_pi_camera_host()
                    conn = http.client.HTTPConnection(host, port, timeout=1.5)
                conn.request("GET", "/frame.jpg", headers={"Connection": "keep-alive"})
                resp = conn.getresponse()
                raw_bytes = resp.read()
                if resp.status == 200 and raw_bytes:
                    clean_jpg = self._sanitize_jpeg(raw_bytes)
                    if clean_jpg:
                        with self._lock:
                            self.last_frame_jpeg = clean_jpg
                            self.last_frame_ts = time.time()
                            self.frame_count += 1
                            self.connected = True
                            self.streaming = True
                            self.error_message = ""
                        fps_counter += 1
                        consecutive_failures = 0
                else:
                    consecutive_failures += 1
            except Exception as exc:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
                    conn = None
                consecutive_failures += 1
                if consecutive_failures >= 10:
                    with self._lock:
                        self.error_message = f"Pi camera waiting/reconnecting ({host}:{port}): {exc}"
                        self.connected = False
                    time.sleep(1.0)
                    continue

            now = time.time()
            if now - fps_t0 >= 1.0:
                with self._lock:
                    self.measured_fps = round(fps_counter / (now - fps_t0), 1)
                fps_counter = 0
                fps_t0 = now

            elapsed = time.time() - loop_t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def _opencv_loop(self) -> None:
        fps_counter = 0
        fps_t0 = time.time()
        interval = 1.0 / max(1, min(self.target_fps, 30))
        failures = 0

        while not self._stop_event.is_set() and self._cv_cap is not None:
            loop_t0 = time.time()
            try:
                ok, frame = self._cv_cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= 8:
                        with self._lock:
                            self.error_message = "Camera disconnected during streaming."
                            self.connected = False
                            self.streaming = False
                        self.diagnostics.log_event("ERROR", "CAMERA", "CAMERA_DISCONNECTED", self.error_message)
                        break
                    time.sleep(0.05)
                    continue

                failures = 0
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))

                ok_enc, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok_enc:
                    with self._lock:
                        self.last_frame_jpeg = buf.tobytes()
                        self.last_frame_ts = time.time()
                        self.frame_count += 1
                        self.connected = True
                    fps_counter += 1
            except Exception as exc:
                with self._lock:
                    self.error_message = f"OpenCV capture error: {exc}"
                    self.connected = False
                    self.streaming = False
                self.diagnostics.log_event("ERROR", "CAMERA", "OPENCV_RUNTIME_ERROR", str(exc))
                break

            now = time.time()
            if now - fps_t0 >= 1.0:
                with self._lock:
                    self.measured_fps = round(fps_counter / (now - fps_t0), 1)
                fps_counter = 0
                fps_t0 = now

            elapsed = time.time() - loop_t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

        with self._lock:
            self._cleanup_resources()

    def get_latest_jpeg(self) -> Optional[bytes]:
        with self._lock:
            if not self.streaming or not self.connected:
                return None
            return self.last_frame_jpeg

    def capture_photo(self) -> Tuple[bool, str, Optional[str]]:
        """Saves the current real camera frame to disk in logs/captures/."""
        with self._lock:
            if not self.streaming or not self.connected or not self.last_frame_jpeg:
                return False, "Cannot capture photo: Camera is not actively streaming.", None
            jpeg_data = self.last_frame_jpeg

        CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
        fname = f"liftmate_capture_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
        fpath: Path = CAPTURES_DIR / fname
        try:
            fpath.write_bytes(jpeg_data)
            with self._lock:
                self.saved_photos.insert(0, fname)
                self.saved_photos = self.saved_photos[:25]
            self.diagnostics.log_event("INFO", "CAMERA", "PHOTO_CAPTURED", str(fpath))
            return True, f"Saved snapshot {fname}", fname
        except Exception as exc:
            return False, f"Failed to save photo: {exc}", None

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "streaming": self.streaming,
                "connected": self.connected,
                "source": self.source,
                "active_backend": self.active_backend,
                "width": self.width,
                "height": self.height,
                "target_fps": self.target_fps,
                "measured_fps": self.measured_fps if self.streaming else 0.0,
                "frame_count": self.frame_count,
                "error_message": self.error_message,
                "picamera2_installed": HAS_PICAMERA2,
                "saved_photos": list(self.saved_photos),
                "available_sources": self.detect_sources(),
            }

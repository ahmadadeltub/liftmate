#!/usr/bin/env python3
"""High-Speed V4L2 USB Camera MJPEG Streaming Agent for Raspberry Pi (Port 8081)."""
from __future__ import annotations

import glob
import json
import os
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def find_usb_video_device() -> str | None:
    """Finds the first USB V4L2 capture device (typically /dev/video0)."""
    for dev in sorted(glob.glob("/dev/video*")):
        base = os.path.basename(dev)
        try:
            num = int(base.replace("video", ""))
            if num >= 10:
                continue
        except ValueError:
            continue
        return dev
    return None


class FastThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self) -> None:
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        super().server_bind()


class PiCameraState:
    def __init__(self) -> None:
        self.lock = threading.Condition()
        self.streaming = False
        self.last_jpeg: bytes | None = None
        self.last_frame_ts: float = 0.0
        self.device_name: str = "/dev/video0"
        self.error: str = "Initializing USB Camera..."
        self.frame_count = 0

    def run_capture_loop(self) -> None:
        while True:
            proc = None
            try:
                dev = find_usb_video_device()
                if not dev:
                    with self.lock:
                        self.streaming = False
                        self.error = "No USB camera (/dev/video0) detected on Raspberry Pi."
                    time.sleep(2.0)
                    continue

                self.device_name = dev
                cmd = [
                    "v4l2-ctl",
                    "--device", dev,
                    "--set-fmt-video=width=320,height=240,pixelformat=MJPG",
                    "--set-parm=15",
                    "--stream-mmap=3",
                    "--stream-to=-",
                ]
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0,
                )

                buf = bytearray()
                assert proc.stdout is not None
                while proc.poll() is None:
                    chunk = proc.stdout.read(16384)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    while True:
                        end = buf.find(b"\xff\xd9")
                        if end == -1:
                            if len(buf) > 262144:
                                last_soi = buf.rfind(b"\xff\xd8\xff")
                                if last_soi > 0:
                                    del buf[:last_soi]
                                else:
                                    buf.clear()
                            break

                        # Always pair EOI (\xff\xd9) with the most recent SOI (\xff\xd8\xff) before it
                        # so dropped USB packets never concatenate multiple frames together.
                        start = buf.rfind(b"\xff\xd8\xff", 0, end)
                        if start == -1:
                            del buf[: end + 2]
                            continue

                        jpg = bytes(buf[start : end + 2])
                        del buf[: end + 2]

                        if 800 < len(jpg) < 65000 and jpg.find(b"\xff\xd8\xff", 2) == -1:
                            with self.lock:
                                self.last_jpeg = jpg
                                self.last_frame_ts = time.time()
                                self.streaming = True
                                self.error = ""
                                self.frame_count += 1
                                self.lock.notify_all()
            except Exception as exc:
                with self.lock:
                    self.streaming = False
                    self.error = f"USB camera capture error: {exc}"
            finally:
                if proc is not None:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            time.sleep(1.0)


STATE = PiCameraState()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args) -> None:
        return

    def setup(self) -> None:
        super().setup()
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass

    def do_GET(self) -> None:
        if self.path.startswith("/status"):
            with STATE.lock:
                fresh = (time.time() - STATE.last_frame_ts) < 3.0 if STATE.last_frame_ts > 0 else False
                payload = {
                    "streaming": STATE.streaming and fresh,
                    "has_frame": STATE.last_jpeg is not None and fresh,
                    "device": STATE.device_name,
                    "frame_count": STATE.frame_count,
                    "error": STATE.error,
                }
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/frame.jpg"):
            with STATE.lock:
                jpg = STATE.last_jpeg
            if not jpg:
                self.send_response(503)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
            return

        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            last_sent = -1
            try:
                while True:
                    with STATE.lock:
                        if STATE.frame_count == last_sent:
                            STATE.lock.wait(timeout=0.5)
                        jpg = STATE.last_jpeg
                        cnt = STATE.frame_count
                    if jpg and cnt != last_sent:
                        last_sent = cnt
                        part = (
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpg)).encode("ascii")
                            + b"\r\n\r\n"
                            + jpg
                            + b"\r\n"
                        )
                        self.wfile.write(part)
                        self.wfile.flush()
            except Exception:
                return

        self.send_response(404)
        self.send_header("Content-Length", "0")
        self.end_headers()


if __name__ == "__main__":
    t = threading.Thread(target=STATE.run_capture_loop, daemon=True)
    t.start()
    server = FastThreadingHTTPServer(("0.0.0.0", 8081), Handler)
    print("LiftMate USB Camera Agent listening on 0.0.0.0:8081")
    server.serve_forever()

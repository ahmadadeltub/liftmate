"""Thread-safe Serial Protocol Parser and Controller for LiftMate Arduino Mega 2560."""
from __future__ import annotations

import glob
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports

from .config import ConfigManager
from .diagnostics import DiagnosticsManager
from .imu_controller import IMUController
from .safety_controller import SafetyController


def parse_serial_frame(raw_line: str) -> Dict[str, Any]:
    """Parses a framed message from the Arduino Mega and classifies its type.

    Distinguishes:
      - READY:   #,READY,#
      - ACK:     #,OK,#
      - PONG:    #,PONG,#
      - TIMEOUT: #,TIMEOUT,#
      - ERROR:   #,ERR,<REASON>,#
      - STATUS:  #,STAT,<estop>,<imu>,<w1>,<w2>,<w3>,<w4>,<up>,<low>,<s_att>,<s_ang>,#
      - IMU:     #,<heading>,<pitch>,<roll>,#
      - MALFORMED: any invalid frame

    Never confuses an IMU telemetry frame with a command acknowledgement.
    """
    line = raw_line.strip()
    if len(line) < 5 or not line.startswith("#,") or not line.endswith(",#"):
        return {"type": "MALFORMED", "raw": line, "reason": "BAD_FRAME_DELIMITERS"}

    inner = line[2:-2]
    if not inner:
        return {"type": "MALFORMED", "raw": line, "reason": "EMPTY_PAYLOAD"}

    fields = inner.split(",")
    if any(f == "" for f in fields):
        return {"type": "MALFORMED", "raw": line, "reason": "EMPTY_FIELD"}

    head = fields[0]

    if head == "READY" and len(fields) == 1:
        return {"type": "READY", "raw": line}
    if head == "OK" and len(fields) == 1:
        return {"type": "ACK", "raw": line}
    if head == "PONG" and len(fields) == 1:
        return {"type": "PONG", "raw": line}
    if head == "TIMEOUT" and len(fields) == 1:
        return {"type": "TIMEOUT", "raw": line}
    if head == "ERR" and len(fields) >= 2:
        return {"type": "ERROR", "reason": ",".join(fields[1:]), "raw": line}
    if head == "STAT" and len(fields) == 11:
        try:
            return {
                "type": "STATUS",
                "estop": bool(int(fields[1])),
                "imu_ok": bool(int(fields[2])),
                "wheels": [int(fields[3]), int(fields[4]), int(fields[5]), int(fields[6])],
                "upper_jack": int(fields[7]),
                "lower_jack": int(fields[8]),
                "servo_attached": bool(int(fields[9])),
                "servo_angle": int(fields[10]),
                "raw": line,
            }
        except ValueError:
            return {"type": "MALFORMED", "raw": line, "reason": "BAD_STATUS_FIELDS"}

    # Check if it is a 3-number IMU Euler telemetry frame: #,heading,pitch,roll,#
    if len(fields) == 3:
        try:
            heading = float(fields[0])
            pitch = float(fields[1])
            roll = float(fields[2])
            if not (-360.0 <= heading <= 720.0 and -180.0 <= pitch <= 180.0 and -180.0 <= roll <= 180.0):
                return {"type": "MALFORMED", "raw": line, "reason": "IMU_OUT_OF_RANGE"}
            return {
                "type": "IMU",
                "heading": heading,
                "pitch": pitch,
                "roll": roll,
                "raw": line,
            }
        except ValueError:
            return {"type": "MALFORMED", "raw": line, "reason": "NON_NUMERIC_3FIELD"}

    return {"type": "MALFORMED", "raw": line, "reason": "UNKNOWN_FRAME_FORMAT"}


def format_command(command: str, *args: Any) -> str:
    """Formats an outgoing command frame with newline."""
    parts = ["#", command.strip().upper()]
    for a in args:
        parts.append(str(a))
    parts.append("#")
    return ",".join(parts) + "\n"


class SerialController:
    """Single coordinated thread-safe serial service for communicating with Arduino Mega."""

    def __init__(
        self,
        config: ConfigManager,
        diagnostics: DiagnosticsManager,
        safety: SafetyController,
        imu: IMUController,
    ) -> None:
        self.config = config
        self.diagnostics = diagnostics
        self.safety = safety
        self.imu = imu

        self._port_lock = threading.RLock()
        self._cmd_lock = threading.Lock()
        self._reply_cond = threading.Condition()

        self._serial: Optional[serial.Serial] = None
        self.port_name: str = ""
        self.baud_rate: int = int(self.config.get_section("serial").get("baud_rate", 115200))
        self.connected: bool = False
        self.arduino_ready: bool = False
        self.simulation_mode: bool = bool(self.config.get_section("system").get("simulation_mode", False))

        self._running: bool = False
        self._reader_thread: Optional[threading.Thread] = None
        self._auto_reconnect_enabled: bool = False
        self._auto_reconnect_thread: Optional[threading.Thread] = None
        self._pending_reply: Optional[Dict[str, Any]] = None
        self._timeout_callbacks: List[Callable[[], None]] = []

        # Simulated hardware state when in simulation_mode
        self.sim_state: Dict[str, Any] = {
            "wheels": [0, 0, 0, 0],
            "upper_jack": 0,
            "lower_jack": 0,
            "servo_angle": 90,
            "servo_attached": False,
            "estop": False,
            "wheel_last_cmd_ts": 0.0,
            "upper_last_cmd_ts": 0.0,
            "lower_last_cmd_ts": 0.0,
            "upper_start_ts": 0.0,
            "lower_start_ts": 0.0,
            "wheel_wdt_ms": 450,
            "jack_wdt_ms": 600,
        }

    def register_timeout_callback(self, cb: Callable[[], None]) -> None:
        self._timeout_callbacks.append(cb)

    def start_auto_reconnect(self) -> None:
        """Starts background USB hot-plug / auto-reconnect watcher for headless Raspberry Pi operation."""
        if self._auto_reconnect_thread and self._auto_reconnect_thread.is_alive():
            self._auto_reconnect_enabled = True
            return
        self._auto_reconnect_enabled = True
        self._auto_reconnect_thread = threading.Thread(target=self._auto_reconnect_loop, daemon=True)
        self._auto_reconnect_thread.start()

    def stop_auto_reconnect(self) -> None:
        self._auto_reconnect_enabled = False

    def _auto_reconnect_loop(self) -> None:
        while self._auto_reconnect_enabled:
            try:
                if not self.connected and not self.simulation_mode:
                    ports = self.discover_ports()
                    preferred = [p for p in ports if p.get("preferred") == "true"]
                    if preferred:
                        self.connect(preferred[0]["device"])
            except Exception:
                pass
            time.sleep(1.5)

    @staticmethod
    def discover_ports() -> List[Dict[str, str]]:
        """Lists candidate serial ports, prioritizing USB-Serial / Arduino Mega devices."""
        ports_info: List[Dict[str, str]] = []
        seen = set()

        for p in serial.tools.list_ports.comports():
            dev = p.device
            if any(ignore in dev for ignore in ("Bluetooth", "JBL", "wlan-debug", "debug-console", "ttyS", "ttyAMA", "printk")):
                continue
            seen.add(dev)
            is_usb = any(k in dev.lower() for k in ("usbserial", "usbmodem", "ttyacm", "ttyusb"))
            if not is_usb:
                continue
            ports_info.append({
                "device": dev,
                "description": p.description or "Serial Port",
                "hwid": p.hwid or "",
                "preferred": "true",
            })

        # Also scan standard device globs in case list_ports omits any
        for pattern in ("/dev/cu.usbserial*", "/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
            for path in sorted(glob.glob(pattern)):
                if path not in seen:
                    seen.add(path)
                    ports_info.append({
                        "device": path,
                        "description": "USB Serial Device",
                        "hwid": "USB",
                        "preferred": "true",
                    })

        ports_info.sort(key=lambda x: (x["preferred"] != "true", x["device"]))
        return ports_info

    def set_simulation_mode(self, enabled: bool) -> Tuple[bool, str]:
        if self.connected:
            self.disconnect()
        self.simulation_mode = bool(enabled)
        self.config.update_section("system", {"simulation_mode": self.simulation_mode})
        self.diagnostics.log_event(
            "INFO",
            "SERIAL",
            "SIMULATION_MODE_CHANGED",
            f"simulation_mode={self.simulation_mode}",
        )
        if self.simulation_mode:
            return self.connect("SIMULATOR")
        return True, "Simulation mode disabled. Ready to connect to physical Arduino Mega."

    def connect(self, port: Optional[str] = None, baud_rate: Optional[int] = None) -> Tuple[bool, str]:
        with self._port_lock:
            if self.connected:
                self.disconnect()

            if baud_rate:
                self.baud_rate = int(baud_rate)

            if self.simulation_mode or port == "SIMULATOR":
                self.simulation_mode = True
                self.port_name = "SIMULATOR (Virtual Mega 2560)"
                self.connected = True
                self.arduino_ready = True
                self._running = True
                self.safety.on_serial_connected()
                self.diagnostics.log_event("INFO", "SERIAL", "SIMULATOR_CONNECTED", self.port_name)
                self._reader_thread = threading.Thread(target=self._sim_loop, daemon=True)
                self._reader_thread.start()
                return True, f"Connected to {self.port_name}"

            target_port = port
            if not target_port or target_port == "auto":
                discovered = self.discover_ports()
                if not discovered:
                    return False, "No Arduino USB serial port detected. Check USB cable or enable Simulation Mode."
                target_port = discovered[0]["device"]

            try:
                ser = serial.Serial(
                    port=target_port,
                    baudrate=self.baud_rate,
                    timeout=0.08,
                    write_timeout=0.5,
                )
                self._serial = ser
                self.port_name = target_port
                self.connected = True
                self.arduino_ready = False
                self._running = True
            except Exception as exc:
                self._serial = None
                self.connected = False
                self.safety.on_serial_disconnected(f"Failed to open {target_port}: {exc}")
                self.diagnostics.log_event("ERROR", "SERIAL", "CONNECT_FAILED", f"{target_port}: {exc}")
                return False, f"Failed to open {target_port}: {exc}"

            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

            # Wait up to 3.4s for Arduino Mega DTR reset & #,READY,# or verify via #,PING,#
            t0 = time.time()
            while time.time() - t0 < 3.4:
                if self.arduino_ready:
                    break
                if time.time() - t0 > 2.1:
                    ok, _ = self.ping()
                    if ok:
                        self.arduino_ready = True
                        break
                time.sleep(0.1)

            # Immediately ensure all motors/actuators are stopped upon connection
            self.send_command("STOP")
            if self.config.get_section("actuators").get("allow_simultaneous_groups", True):
                self.send_command("DUAL_JACK", 1)
            self.safety.on_serial_connected()
            self.diagnostics.log_event(
                "INFO",
                "SERIAL",
                "CONNECTED",
                f"port={self.port_name}, baud={self.baud_rate}, ready={self.arduino_ready}",
            )
            return True, f"Connected to {self.port_name} @ {self.baud_rate} baud"

    def disconnect(self, reason: str = "Operator disconnected serial") -> Tuple[bool, str]:
        with self._port_lock:
            if self.connected and not self.simulation_mode and self._serial:
                try:
                    self._serial.write(b"#,STOP,#\n")
                    self._serial.flush()
                except Exception:
                    pass
            self._running = False
            ser = self._serial
            self._serial = None
            self.connected = False
            self.arduino_ready = False
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            self.imu.mark_disconnected("Serial disconnected")
            self.safety.on_serial_disconnected(reason)
            self.diagnostics.log_event("INFO", "SERIAL", "DISCONNECTED", reason)
            return True, "Disconnected."

    def _handle_incoming_line(self, line: str, simulated: bool = False) -> None:
        parsed = parse_serial_frame(line)
        ftype = parsed["type"]

        if ftype == "IMU":
            self.arduino_ready = True
            self.imu.update_from_frame(
                parsed["heading"],
                parsed["pitch"],
                parsed["roll"],
                simulated=simulated,
            )
            return

        if ftype == "READY":
            self.arduino_ready = True
            self.diagnostics.record_response(line)
            self.diagnostics.log_event("INFO", "SERIAL", "ARDUINO_READY", line)
            return

        if ftype == "TIMEOUT":
            self.diagnostics.record_response(line)
            self.diagnostics.log_event("WARNING", "SERIAL", "FIRMWARE_WATCHDOG_TIMEOUT", line)
            self.safety.notify_motion_stopped("Firmware watchdog timeout")
            for cb in self._timeout_callbacks:
                try:
                    cb()
                except Exception:
                    pass
            return

        if ftype == "STATUS":
            self.diagnostics.record_response(line)
            return

        if ftype in ("ACK", "PONG", "ERROR"):
            with self._reply_cond:
                self._pending_reply = parsed
                self._reply_cond.notify_all()
            if ftype == "ERROR":
                reason = parsed.get("reason", "UNKNOWN")
                if "IMU" in reason:
                    self.imu.report_error(reason)
                self.diagnostics.log_event("ERROR", "SERIAL", "FIRMWARE_ERR", reason)
            return

        if ftype == "MALFORMED":
            self.diagnostics.log_event("WARNING", "SERIAL", "MALFORMED_FRAME", f"{parsed.get('reason')}: {line}")

    def _reader_loop(self) -> None:
        buf = ""
        while self._running:
            ser = self._serial
            if ser is None or not ser.is_open:
                break
            try:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buf += chunk.decode("utf-8", errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip("\r ")
                        if line:
                            self._handle_incoming_line(line, simulated=False)
            except Exception as exc:
                if self._running:
                    self.diagnostics.log_event("ERROR", "SERIAL", "SERIAL_LOST", str(exc))
                    self.disconnect(f"Serial read failure: {exc}")
                break

    def _sim_loop(self) -> None:
        """Background loop for simulation mode: emits simulated IMU frames and enforces watchdogs."""
        while self._running and self.simulation_mode:
            now = time.time()
            # Check simulated wheel watchdog
            if any(w != 0 for w in self.sim_state["wheels"]):
                elapsed_ms = (now - self.sim_state["wheel_last_cmd_ts"]) * 1000.0
                if elapsed_ms >= self.sim_state["wheel_wdt_ms"]:
                    self.sim_state["wheels"] = [0, 0, 0, 0]
                    self._handle_incoming_line("#,TIMEOUT,#", simulated=True)

            # Check simulated upper/lower jack watchdog & max continuous limits
            if self.sim_state["upper_jack"] != 0:
                cmd_elapsed = (now - self.sim_state["upper_last_cmd_ts"]) * 1000.0
                run_elapsed = (now - self.sim_state["upper_start_ts"]) * 1000.0
                if run_elapsed >= 10000:
                    self.sim_state["upper_jack"] = 0
                    self._handle_incoming_line("#,ERR,ACTUATOR_MAX_RUN_LIMIT,#", simulated=True)
                    self._handle_incoming_line("#,TIMEOUT,#", simulated=True)
                elif cmd_elapsed >= self.sim_state["jack_wdt_ms"]:
                    self.sim_state["upper_jack"] = 0
                    self._handle_incoming_line("#,TIMEOUT,#", simulated=True)

            if self.sim_state["lower_jack"] != 0:
                cmd_elapsed = (now - self.sim_state["lower_last_cmd_ts"]) * 1000.0
                run_elapsed = (now - self.sim_state["lower_start_ts"]) * 1000.0
                if run_elapsed >= 5000:
                    self.sim_state["lower_jack"] = 0
                    self._handle_incoming_line("#,ERR,ACTUATOR_MAX_RUN_LIMIT,#", simulated=True)
                    self._handle_incoming_line("#,TIMEOUT,#", simulated=True)
                elif cmd_elapsed >= self.sim_state["jack_wdt_ms"]:
                    self.sim_state["lower_jack"] = 0
                    self._handle_incoming_line("#,TIMEOUT,#", simulated=True)

            active_mode = "IDLE"
            if any(w != 0 for w in self.sim_state["wheels"]):
                active_mode = "DRIVING"
            elif self.sim_state["upper_jack"] != 0 or self.sim_state["lower_jack"] != 0:
                active_mode = "ACTUATING"

            self.imu.step_simulation(active_mode)
            time.sleep(0.1)

    def _simulate_command(self, frame: str) -> Tuple[bool, str]:
        """Simulates Arduino Mega command execution and response framing."""
        line = frame.strip()
        t0 = time.time()
        self.diagnostics.record_command(line)

        if not (line.startswith("#,") and line.endswith(",#")):
            resp = "#,ERR,BAD_FRAME,#"
            self.diagnostics.record_response(resp, latency_ms=1.2)
            return False, "BAD_FRAME"

        parts = line[2:-2].split(",")
        cmd = parts[0] if parts else ""
        now = time.time()

        if cmd == "PING" and len(parts) == 1:
            resp = "#,PONG,#"
            self.diagnostics.record_response(resp, latency_ms=1.1)
            return True, "PONG"

        if cmd == "STOP" and len(parts) == 1:
            self.sim_state["wheels"] = [0, 0, 0, 0]
            self.sim_state["upper_jack"] = 0
            self.sim_state["lower_jack"] = 0
            self.diagnostics.record_response("#,OK,#", latency_ms=1.2)
            return True, "OK"

        if cmd == "ESTOP" and len(parts) == 1:
            self.sim_state["wheels"] = [0, 0, 0, 0]
            self.sim_state["upper_jack"] = 0
            self.sim_state["lower_jack"] = 0
            self.sim_state["estop"] = True
            self.diagnostics.record_response("#,OK,#", latency_ms=1.2)
            return True, "OK"

        if cmd == "CLEAR_ESTOP" and len(parts) == 1:
            self.sim_state["estop"] = False
            self.diagnostics.record_response("#,OK,#", latency_ms=1.2)
            return True, "OK"

        if cmd == "IMU" and len(parts) == 1:
            self.imu.step_simulation("IDLE")
            self.diagnostics.record_response("#,IMU_SAMPLE,#", latency_ms=1.5)
            return True, "OK"

        if self.sim_state["estop"]:
            self.diagnostics.record_response("#,ERR,ESTOP_ACTIVE,#", latency_ms=1.2)
            return False, "ESTOP_ACTIVE"

        try:
            if cmd == "DRIVE" and len(parts) == 5:
                vals = [int(x) for x in parts[1:5]]
                if any(v < -255 or v > 255 for v in vals):
                    return False, "BAD_SPEED"
                self.sim_state["upper_jack"] = 0
                self.sim_state["lower_jack"] = 0
                self.sim_state["wheels"] = vals
                self.sim_state["wheel_last_cmd_ts"] = now
            elif cmd == "MOTOR" and len(parts) == 3:
                idx = int(parts[1])
                spd = int(parts[2])
                if not (1 <= idx <= 4 and -255 <= spd <= 255):
                    return False, "BAD_MOTOR_OR_SPEED"
                self.sim_state["upper_jack"] = 0
                self.sim_state["lower_jack"] = 0
                self.sim_state["wheels"][idx - 1] = spd
                self.sim_state["wheel_last_cmd_ts"] = now
            elif cmd == "JACK" and len(parts) == 3:
                grp = parts[1]
                spd = int(parts[2])
                if grp not in ("UPPER", "LOWER") or not (-255 <= spd <= 255):
                    return False, "BAD_JACK_OR_SPEED"
                self.sim_state["wheels"] = [0, 0, 0, 0]
                if grp == "UPPER":
                    if self.sim_state["upper_jack"] == 0 and spd != 0:
                        self.sim_state["upper_start_ts"] = now
                    self.sim_state["upper_jack"] = spd
                    self.sim_state["upper_last_cmd_ts"] = now
                else:
                    if self.sim_state["lower_jack"] == 0 and spd != 0:
                        self.sim_state["lower_start_ts"] = now
                    self.sim_state["lower_jack"] = spd
                    self.sim_state["lower_last_cmd_ts"] = now
            elif cmd == "ACT" and len(parts) == 3:
                u_spd = int(parts[1])
                l_spd = int(parts[2])
                if not (-255 <= u_spd <= 255 and -255 <= l_spd <= 255):
                    return False, "BAD_SPEED"
                self.sim_state["wheels"] = [0, 0, 0, 0]
                if self.sim_state["upper_jack"] == 0 and u_spd != 0:
                    self.sim_state["upper_start_ts"] = now
                if self.sim_state["lower_jack"] == 0 and l_spd != 0:
                    self.sim_state["lower_start_ts"] = now
                self.sim_state["upper_jack"] = u_spd
                self.sim_state["lower_jack"] = l_spd
                self.sim_state["upper_last_cmd_ts"] = now
                self.sim_state["lower_last_cmd_ts"] = now
            elif cmd == "SERVO" and len(parts) == 2:
                ang = int(parts[1])
                if not (0 <= ang <= 180):
                    return False, "BAD_ANGLE"
                self.sim_state["servo_attached"] = True
                self.sim_state["servo_angle"] = ang
            elif cmd in ("SERVO_LIMITS", "WDT", "SERVO_DETACH", "DUAL_JACK", "STATUS"):
                pass
            else:
                self.diagnostics.record_response("#,ERR,BAD_COMMAND_OR_ARGUMENTS,#", latency_ms=1.2)
                return False, "BAD_COMMAND_OR_ARGUMENTS"
        except ValueError:
            self.diagnostics.record_response("#,ERR,BAD_ARGUMENTS,#", latency_ms=1.2)
            return False, "BAD_ARGUMENTS"

        latency = max(0.8, (time.time() - t0) * 1000.0)
        self.diagnostics.record_response("#,OK,#", latency_ms=latency)
        return True, "OK"

    def send_raw_frame(self, raw_frame: str, timeout_s: float = 0.35) -> Tuple[bool, str]:
        """Sends a raw command string and waits for ACK/PONG/ERROR (ignoring IMU frames)."""
        frame = raw_frame if raw_frame.endswith("\n") else (raw_frame + "\n")
        with self._cmd_lock:
            if not self.connected:
                return False, "DISCONNECTED"

            if self.simulation_mode:
                return self._simulate_command(frame)

            ser = self._serial
            if ser is None or not ser.is_open:
                self.disconnect("Serial port closed unexpectedly")
                return False, "DISCONNECTED"

            with self._reply_cond:
                self._pending_reply = None

            t0 = time.time()
            self.diagnostics.record_command(frame.strip())
            try:
                ser.write(frame.encode("ascii", errors="ignore"))
                ser.flush()
            except Exception as exc:
                self.diagnostics.log_event("ERROR", "SERIAL", "WRITE_FAILED", str(exc))
                self.disconnect(f"Serial write failure: {exc}")
                return False, f"WRITE_ERROR: {exc}"

            # IMU command returns an IMU telemetry frame, not #,OK,#
            if frame.strip() == "#,IMU,#":
                time.sleep(0.05)
                return True, "IMU_REQUESTED"

            with self._reply_cond:
                self._reply_cond.wait(timeout=timeout_s)
                reply = self._pending_reply

            latency = (time.time() - t0) * 1000.0
            if reply is None:
                self.diagnostics.log_event("WARNING", "SERIAL", "COMMAND_ACK_TIMEOUT", frame.strip())
                return False, "ACK_TIMEOUT"

            self.diagnostics.record_response(reply.get("raw", ""), latency_ms=latency)
            if reply["type"] in ("ACK", "PONG"):
                self.arduino_ready = True
                return True, reply["type"]
            if reply["type"] == "ERROR":
                reason = reply.get("reason", "UNKNOWN_ERROR")
                # If running original Robot_jack_code.ino, extended commands like ESTOP/WDT return BAD_COMMAND_OR_ARGUMENTS
                return False, reason
            return False, "UNEXPECTED_REPLY"

    def send_command(self, command: str, *args: Any, timeout_s: float = 0.35) -> Tuple[bool, str]:
        frame = format_command(command, *args)
        return self.send_raw_frame(frame, timeout_s=timeout_s)

    def ping(self) -> Tuple[bool, float]:
        t0 = time.time()
        ok, resp = self.send_command("PING", timeout_s=0.4)
        dt_ms = round((time.time() - t0) * 1000.0, 2)
        return (ok and resp in ("PONG", "OK")), dt_ms

    def get_status(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "simulation_mode": self.simulation_mode,
            "port": self.port_name or "DISCONNECTED",
            "baud_rate": self.baud_rate,
            "arduino_ready": self.arduino_ready,
            "available_ports": self.discover_ports(),
        }

"""FastAPI Application and WebSocket Telemetry Server for LiftMate Control Center."""
from __future__ import annotations

import asyncio
import os
import platform
import subprocess
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse, PlainTextResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .actuator_controller import ActuatorController
from .ai_vision_controller import AIVisionController
from .autonomous_controller import AutonomousController
from .camera_controller import CameraController
from .config import CAPTURES_DIR, PROJECT_ROOT, ConfigManager
from .diagnostics import DiagnosticsManager
from .imu_controller import IMUController
from .motor_controller import MotorController
from .navigation_controller import NavigationController
from .safety_controller import SafetyController
from .serial_controller import SerialController
from .servo_controller import ServoController

FRONTEND_DIR = PROJECT_ROOT / "frontend"


class ConnectRequest(BaseModel):
    port: Optional[str] = "auto"
    baud_rate: Optional[int] = 115200


class SimulationRequest(BaseModel):
    enabled: bool


class RawCommandRequest(BaseModel):
    command: str


class EstopRequest(BaseModel):
    reason: Optional[str] = "Operator pressed Emergency Stop"


class SingleMotorRequest(BaseModel):
    motor_id: int
    pwm: int = 50
    direction: str = "FORWARD"
    duration_ms: Optional[int] = None


class DriveRequest(BaseModel):
    direction: Optional[str] = None
    speed_pwm: int = 75
    vy: Optional[float] = None
    vx: Optional[float] = None
    omega: Optional[float] = None


class WheelCalibrationRequest(BaseModel):
    positions: Dict[str, str]
    inverted: Dict[str, bool]
    max_pwm_limit: Optional[int] = None


class MotorTestResultRequest(BaseModel):
    motor_id: int
    result: str


class ActuatorCommandRequest(BaseModel):
    group: str
    action: str
    pwm: int = 120
    duration_ms: Optional[int] = None


class ActuatorSettingsRequest(BaseModel):
    driver_electrical_verified: Optional[bool] = None
    allow_simultaneous_groups: Optional[bool] = None
    require_imu_for_lifting: Optional[bool] = None
    upper_inverted: Optional[bool] = None
    lower_inverted: Optional[bool] = None


class ServoEnableRequest(BaseModel):
    enabled: bool


class ServoAngleRequest(BaseModel):
    angle_deg: int


class ServoLimitsRequest(BaseModel):
    safe_min_deg: int
    safe_max_deg: int
    inverted: bool = False


class ImuThresholdsRequest(BaseModel):
    warning_deg: float
    critical_deg: float


class CameraStartRequest(BaseModel):
    source: Optional[str] = "auto"
    width: Optional[int] = 640
    height: Optional[int] = 480


class SimpleControlRequest(BaseModel):
    mode: str
    vy: int = 0
    vx: int = 0
    omega: int = 0
    upper_pwm: int = 0
    lower_pwm: int = 0
    servo_angle: int = 68


class AutonomousModeRequest(BaseModel):
    enabled: bool
    speed_pwm: int = 90


class NavModeRequest(BaseModel):
    mode: str


class NavSaveRequest(BaseModel):
    name: Optional[str] = None


class NavOriginRequest(BaseModel):
    lat: float
    lon: float


class NavMobileGpsRequest(BaseModel):
    lat: float
    lon: float
    accuracy: Optional[float] = None
    heading: Optional[float] = None
    source: str = "mobile"


class NavRepairPinRequest(BaseModel):
    lat: Optional[float] = None
    lon: Optional[float] = None
    label: Optional[str] = None


class NavDemoStepRequest(BaseModel):
    dx_m: float = 0.0
    dy_m: float = 0.8
    dheading_deg: float = 15.0


class AIConfigRequest(BaseModel):
    enabled: Optional[bool] = None
    mode: Optional[str] = None


class SystemShutdownRequest(BaseModel):
    confirm: bool = False
    cut_usb_power: bool = True
    reason: Optional[str] = "Operator initiated full robot shutdown"


class LiftMateSystem:
    """Container holding all coordinated hardware and safety controllers."""

    def __init__(self, config_path: Optional[Path] = None) -> None:
        self.config = ConfigManager(config_path)
        self.diagnostics = DiagnosticsManager()
        self.safety = SafetyController(self.config, self.diagnostics)
        self.imu = IMUController(self.config, self.diagnostics)
        self.serial = SerialController(self.config, self.diagnostics, self.safety, self.imu)
        self.motors = MotorController(self.config, self.diagnostics, self.safety, self.serial)
        self.actuators = ActuatorController(
            self.config, self.diagnostics, self.safety, self.serial, self.imu
        )
        self.servo = ServoController(self.config, self.diagnostics, self.safety, self.serial)
        self.camera = CameraController(self.config, self.diagnostics)
        self.navigation = NavigationController(self.config, self.diagnostics, self.imu)
        self.ai_vision = AIVisionController(
            self.config,
            self.diagnostics,
            self.safety,
            self.camera,
            stop_wheels_callback=self._ai_safety_stop_wheels,
        )
        self.autonomous = AutonomousController(
            self.config,
            self.diagnostics,
            self.safety,
            self.serial,
            self.motors,
            self.actuators,
            self.camera,
            self.ai_vision,
            self.navigation,
        )

    def _ai_safety_stop_wheels(self, reason: str = "AI Safety Assist Stop") -> None:
        self.motors.stop_all_wheels()
        self.navigation.notify_wheel_command(0, 0, 0)
        if self.serial.connected:
            self.serial.send_command("DRIVE", 0, 0, 0, 0)
            self.serial.send_command("STOP")
        self.safety.notify_motion_stopped(reason)

    def stop_all_outputs(self, reason: str = "STOP ALL") -> Dict[str, Any]:
        self.autonomous.manual_override(reason)
        self.motors.stop_all_wheels()
        self.actuators.stop_all_actuators()
        self.navigation.notify_wheel_command(0, 0, 0)
        if self.serial.connected:
            self.serial.send_command("DRIVE", 0, 0, 0, 0)
            self.serial.send_command("ACT", 0, 0)
            self.serial.send_command("SERVO_DETACH")
            self.serial.send_command("STOP")
        self.servo.hardware_attached = False
        self.safety.notify_motion_stopped(reason)
        self.diagnostics.log_event("WARNING", "SAFETY", "STOP_ALL_EXECUTED", reason)
        return self.get_full_state()

    def get_full_state(self) -> Dict[str, Any]:
        return {
            "safety": self.safety.get_status(),
            "serial": self.serial.get_status(),
            "motors": self.motors.get_status(),
            "actuators": self.actuators.get_status(),
            "servo": self.servo.get_status(),
            "imu": self.imu.get_status(),
            "camera": self.camera.get_status(),
            "ai_vision": self.ai_vision.get_status(),
            "autonomous": self.autonomous.get_status(),
            "navigation": self.navigation.get_status(),
            "diagnostics": self.diagnostics.get_summary(),
            "pins": self.config.get_section("pins"),
        }

    def shutdown(self) -> None:
        try:
            self.autonomous.stop()
        except Exception:
            pass
        try:
            self.ai_vision.stop_worker()
        except Exception:
            pass
        try:
            self.serial.stop_auto_reconnect()
        except Exception:
            pass
        try:
            self.stop_all_outputs("Application shutdown")
        except Exception:
            pass
        try:
            self.camera.stop()
        except Exception:
            pass
        try:
            self.serial.disconnect("Application shutdown")
        except Exception:
            pass

    @staticmethod
    def is_raspberry_pi_host() -> bool:
        """Returns True only when running on a physical Raspberry Pi Linux host."""
        if os.environ.get("LIFTMATE_FORCE_POWEROFF") == "1":
            return True
        if platform.system() != "Linux":
            return False
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            try:
                model_text = model_path.read_text(encoding="utf-8", errors="ignore").lower()
                return "raspberry pi" in model_text
            except Exception:
                return False
        return False

    def initiate_poweroff(
        self,
        cut_usb_power: bool = True,
        reason: str = "Operator initiated full robot shutdown",
    ) -> Dict[str, Any]:
        """Halts all actuators/motors, locks Arduino Mega in safe E-STOP, and schedules Pi OS poweroff."""
        self.diagnostics.log_event("WARNING", "SAFETY", "SYSTEM_SHUTDOWN_INITIATED", reason)
        try:
            self.autonomous.manual_override(reason)
            self.autonomous.stop()
        except Exception:
            pass
        try:
            self.serial.stop_auto_reconnect()
        except Exception:
            pass
        try:
            self.stop_all_outputs(reason)
        except Exception:
            pass
        if self.serial.connected:
            try:
                self.serial.send_command("ESTOP")
                time.sleep(0.08)
            except Exception:
                pass
        try:
            self.safety.trigger_emergency_stop(reason)
        except Exception:
            pass
        try:
            self.camera.stop()
        except Exception:
            pass
        try:
            self.serial.disconnect(reason)
        except Exception:
            pass

        if not self.is_raspberry_pi_host():
            return {
                "ok": True,
                "poweroff_scheduled": False,
                "usb_power_cut_requested": bool(cut_usb_power),
                "host_platform": platform.system(),
                "message": (
                    "All motors, actuators, servo & Mega outputs locked safely. "
                    "OS poweroff skipped on non-Raspberry Pi host."
                ),
            }

        helper_path = Path("/usr/local/bin/liftmate-poweroff.sh")
        usb_arg = "1" if cut_usb_power else "0"
        is_root = hasattr(os, "geteuid") and os.geteuid() == 0

        if helper_path.exists():
            cmd = [str(helper_path), usb_arg] if is_root else ["sudo", "-n", str(helper_path), usb_arg]
        else:
            power_cmd = ["systemctl", "poweroff"]
            inline_sh = (
                "sleep 1.2 && sync && systemctl poweroff || shutdown -h now"
            )
            cmd = ["bash", "-c", inline_sh] if is_root else ["sudo", "-n", "bash", "-c", inline_sh]
            if is_root is False and not Path("/usr/bin/sudo").exists():
                cmd = power_cmd

        try:
            subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                close_fds=True,
            )
            return {
                "ok": True,
                "poweroff_scheduled": True,
                "usb_power_cut_requested": bool(cut_usb_power),
                "host_platform": platform.system(),
                "message": (
                    "All motors & Mega outputs locked in E-STOP. "
                    "Shutting down Raspberry Pi OS cleanly in 1s — switch off battery once green LED stops."
                ),
            }
        except Exception as exc:
            self.diagnostics.log_event("ERROR", "SAFETY", "SYSTEM_SHUTDOWN_FAILED", str(exc))
            return {
                "ok": False,
                "poweroff_scheduled": False,
                "usb_power_cut_requested": bool(cut_usb_power),
                "host_platform": platform.system(),
                "message": f"Robot halted safely, but OS poweroff failed: {exc}",
            }


system = LiftMateSystem()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: ensure all outputs are OFF; auto-connect serial if present (remains IDLE/disarmed!)
    ports = system.serial.discover_ports()
    if system.serial.simulation_mode:
        system.serial.connect("SIMULATOR")
    elif ports:
        system.serial.connect(ports[0]["device"])
    system.serial.start_auto_reconnect()
    try:
        system.camera.start("remote_pi")
    except Exception:
        pass
    yield
    system.shutdown()


app = FastAPI(
    title="LiftMate Robotic Control & Diagnostic Center",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

if (FRONTEND_DIR / "css").exists():
    app.mount("/css", StaticFiles(directory=str(FRONTEND_DIR / "css")), name="css")
if (FRONTEND_DIR / "js").exists():
    app.mount("/js", StaticFiles(directory=str(FRONTEND_DIR / "js")), name="js")
if (FRONTEND_DIR / "vendor").exists():
    app.mount("/vendor", StaticFiles(directory=str(FRONTEND_DIR / "vendor")), name="vendor")
CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/captures", StaticFiles(directory=str(CAPTURES_DIR)), name="captures")


@app.get("/", response_class=HTMLResponse)
@app.get("/simple", response_class=HTMLResponse)
async def serve_simple_dashboard() -> Response:
    simple_file = FRONTEND_DIR / "simple.html"
    if simple_file.exists():
        return FileResponse(simple_file)
    index_file = FRONTEND_DIR / "index.html"
    return FileResponse(index_file)


@app.get("/mobile-map", response_class=HTMLResponse)
async def serve_mobile_map() -> Response:
    map_file = FRONTEND_DIR / "mobile_map.html"
    if map_file.exists():
        return FileResponse(map_file)
    raise HTTPException(status_code=404, detail="Mobile map asset not found.")


@app.get("/diagnostics", response_class=HTMLResponse)
async def serve_full_dashboard() -> Response:
    index_file = FRONTEND_DIR / "index.html"
    if not index_file.exists():
        return HTMLResponse("<h1>LiftMate Frontend Not Found</h1>", status_code=404)
    return FileResponse(index_file)


def _compact_simple_response(ok: bool, message: str) -> Dict[str, Any]:
    """Lightweight response for high-rate mobile control without serializing full map/IMU history."""
    return {
        "ok": ok,
        "message": message,
        "serial_connected": system.serial.connected,
        "serial_port": system.serial.port_name or "DISCONNECTED",
        "safety_state": system.safety.state.value,
    }


@app.post("/api/simple/control")
def simple_slider_control(req: SimpleControlRequest) -> Dict[str, Any]:
    import time as _time

    # Prioritize responsive motor control over AI inference
    system.ai_vision.notify_motor_activity()

    # Auto-arm if connected and in IDLE so sliders work immediately
    if system.safety.connected and not system.safety.estop_latched and not system.safety.get_status()["armed"]:
        system.safety.arm()

    mode = req.mode.upper()

    if mode in ("STOP_ALL", "RESET_ALL", "WHEELS", "ACTUATORS"):
        system.autonomous.manual_override(f"Operator manual command: {mode}")

    if mode == "STOP_ALL":
        state = system.stop_all_outputs("Simple controller STOP ALL")
        return {"ok": True, "message": "All motors and actuators stopped.", "state": state}

    if mode == "RESET_ALL":
        if system.serial.connected:
            system.serial.send_command("CLEAR_ESTOP")
            system.serial.send_command("STOP")
            system.serial.send_command("DUAL_JACK", 1)
            system.serial.send_command("SERVO_LIMITS", 6, 130)
        system.diagnostics.clear_fault_counters()
        system.safety.reset_emergency_stop()
        system.safety.clear_fault()
        if system.safety.connected:
            system.safety.arm()
        system.motors.stop_all_wheels()
        system.actuators.stop_all_actuators()
        system.actuators.update_settings(
            driver_electrical_verified=True,
            allow_simultaneous_groups=True,
        )
        system.servo.set_enabled(True)
        system.servo.update_limits(6, 130, False)
        normal_servo = max(6, min(130, int(req.servo_angle if req.servo_angle else 6)))
        system.servo.set_angle(normal_servo)
        return {
            "ok": True,
            "message": f"All robot parts reset to normal (Servo {normal_servo}°).",
            "state": system.get_full_state(),
        }

    if mode == "WHEELS":
        vy = max(-255, min(255, int(req.vy)))
        vx = max(-255, min(255, int(req.vx)))
        omega = max(-255, min(255, int(req.omega)))
        if vy == 0 and vx == 0 and omega == 0:
            system.navigation.notify_wheel_command(0, 0, 0)
            ok, msg = system.motors.stop_all_wheels()
            return _compact_simple_response(ok, msg)

        # AI SAFETY ASSIST: Block forward drive if high-risk obstacle is directly ahead in CENTER region
        if (
            system.ai_vision.enabled
            and system.ai_vision.mode == "SAFETY_ASSIST"
            and vy > 0
            and system.ai_vision.regions.get("CENTER", {}).get("risk") == "HIGH"
        ):
            system._ai_safety_stop_wheels("AI Safety Assist blocked forward motion: POTENTIAL OBSTACLE – STOP")
            return _compact_simple_response(
                False,
                "AI Safety Assist: POTENTIAL OBSTACLE – STOP (forward path blocked)",
            )

        if system.safety.estop_latched or system.safety.state.value == "EMERGENCY_STOP":
            return _compact_simple_response(False, "Blocked: EMERGENCY_STOP is active. Tap Reset to resume.")

        if not system.serial.connected:
            return _compact_simple_response(False, "DISCONNECTED: Plug Arduino Mega USB into Raspberry Pi")

        if system.safety.state.value in ("ACTUATING_UPPER", "ACTUATING_LOWER"):
            system.safety.notify_motion_stopped("Switching from actuators to wheels")

        # Compute omnidirectional Mecanum speeds directly from slider values (-255..255)
        raw_corners = {
            "front_left": vy + vx + omega,
            "front_right": vy - vx - omega,
            "rear_left": vy - vx + omega,
            "rear_right": vy + vx - omega,
        }
        peak = max(abs(v) for v in raw_corners.values())
        scale = 255.0 / peak if peak > 255 else 1.0

        cal = system.config.get_section("wheel_calibration")
        pos_map = cal.get("positions", {"1": "rear_left", "2": "rear_right", "3": "front_left", "4": "front_right"})
        inv_map = cal.get("inverted", {})

        ch_pwms = {1: 0, 2: 0, 3: 0, 4: 0}
        for idx in (1, 2, 3, 4):
            corner = pos_map.get(str(idx), "front_left")
            val = int(round(raw_corners.get(corner, 0) * scale))
            if bool(inv_map.get(str(idx), False)):
                val = -val
            ch_pwms[idx] = max(-255, min(255, val))

        ok, msg = system.serial.send_command("DRIVE", ch_pwms[1], ch_pwms[2], ch_pwms[3], ch_pwms[4])
        if ok:
            system.motors.commanded_pwm = ch_pwms
            system.motors.last_command_ts = _time.time()
            system.safety.notify_driving_started()
            system.navigation.notify_wheel_command(vx, vy, omega)
        return _compact_simple_response(ok, msg)

    if mode == "ACTUATORS":
        u_pwm = max(-255, min(255, int(req.upper_pwm)))
        l_pwm = max(-255, min(255, int(req.lower_pwm)))
        if u_pwm == 0 and l_pwm == 0:
            ok, msg = system.actuators.stop_all_actuators()
            return _compact_simple_response(ok, msg)

        if system.safety.estop_latched or system.safety.state.value == "EMERGENCY_STOP":
            return _compact_simple_response(False, "Blocked: EMERGENCY_STOP is active. Tap Reset to resume.")

        if not system.serial.connected:
            return _compact_simple_response(False, "DISCONNECTED: Plug Arduino Mega USB into Raspberry Pi")

        if system.safety.state.value == "DRIVING":
            system.safety.notify_motion_stopped("Switching from wheels to actuators")

        # Ensure driver verification and dual-jack support when using sliders
        act_cfg = system.config.get_section("actuators")
        if not act_cfg.get("driver_electrical_verified", False) or not act_cfg.get("allow_simultaneous_groups", False):
            system.actuators.update_settings(
                driver_electrical_verified=True,
                allow_simultaneous_groups=True,
            )

        now = _time.time()
        ok, msg = system.serial.send_command("ACT", u_pwm, l_pwm)
        if not ok and "DUAL_JACK" in str(msg):
            system.serial.send_command("DUAL_JACK", 1)
            ok, msg = system.serial.send_command("ACT", u_pwm, l_pwm)
        if ok:
            system.actuators.upper_pwm = u_pwm
            system.actuators.lower_pwm = l_pwm
            system.actuators.upper_direction = "EXTEND" if u_pwm > 0 else ("RETRACT" if u_pwm < 0 else "STOPPED")
            system.actuators.lower_direction = "EXTEND" if l_pwm > 0 else ("RETRACT" if l_pwm < 0 else "STOPPED")
            system.actuators.upper_last_cmd_ts = now
            system.actuators.lower_last_cmd_ts = now
            if system.actuators.upper_start_ts == 0 and u_pwm != 0:
                system.actuators.upper_start_ts = now
            if system.actuators.lower_start_ts == 0 and l_pwm != 0:
                system.actuators.lower_start_ts = now
            system.safety.notify_actuating_started("UPPER" if u_pwm != 0 else "LOWER")
        return _compact_simple_response(ok, msg)

    if mode == "SERVO":
        if system.safety.estop_latched or system.safety.state.value == "EMERGENCY_STOP":
            return _compact_simple_response(False, "Blocked: EMERGENCY_STOP is active. Tap Reset to resume.")
        if not system.serial.connected:
            return _compact_simple_response(False, "DISCONNECTED: Plug Arduino Mega USB into Raspberry Pi")

        angle = max(6, min(130, int(req.servo_angle)))
        if not system.servo.enabled:
            system.servo.set_enabled(True)
        srv_cfg = system.config.get_section("servo")
        if srv_cfg.get("safe_min_deg", 6) != 6 or srv_cfg.get("safe_max_deg", 130) != 130:
            system.servo.update_limits(6, 130, bool(srv_cfg.get("inverted", False)))
        ok, msg = system.servo.set_angle(angle)
        return _compact_simple_response(ok, msg)

    return _compact_simple_response(False, f"Unknown mode {mode}")


@app.get("/api/state")
async def get_state() -> Dict[str, Any]:
    return system.get_full_state()


# ==================== SERIAL ENDPOINTS ====================

@app.post("/api/serial/connect")
async def serial_connect(req: ConnectRequest) -> Dict[str, Any]:
    ok, msg = system.serial.connect(port=req.port, baud_rate=req.baud_rate)
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/serial/disconnect")
async def serial_disconnect() -> Dict[str, Any]:
    system.stop_all_outputs("Serial disconnect requested")
    ok, msg = system.serial.disconnect("Operator disconnected")
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/serial/ping")
async def serial_ping() -> Dict[str, Any]:
    ok, latency_ms = system.serial.ping()
    return {
        "ok": ok,
        "latency_ms": latency_ms,
        "message": f"PONG received in {latency_ms} ms" if ok else "PING failed or timed out",
        "state": system.get_full_state(),
    }


@app.post("/api/serial/simulation")
async def serial_simulation(req: SimulationRequest) -> Dict[str, Any]:
    system.stop_all_outputs("Switching simulation mode")
    ok, msg = system.serial.set_simulation_mode(req.enabled)
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/serial/raw")
async def serial_raw(req: RawCommandRequest) -> Dict[str, Any]:
    ok, resp = system.serial.send_raw_frame(req.command)
    return {"ok": ok, "response": resp, "state": system.get_full_state()}


# ==================== SAFETY ENDPOINTS ====================

@app.post("/api/safety/arm")
async def safety_arm() -> Dict[str, Any]:
    ok, msg = system.safety.arm()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/safety/disarm")
async def safety_disarm() -> Dict[str, Any]:
    system.stop_all_outputs("Operator disarmed system")
    ok, msg = system.safety.disarm()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/safety/stop_all")
async def safety_stop_all() -> Dict[str, Any]:
    state = system.stop_all_outputs("Operator pressed STOP ALL")
    return {"ok": True, "message": "All wheels and actuators stopped immediately.", "state": state}


@app.post("/api/safety/estop")
async def safety_estop(req: EstopRequest) -> Dict[str, Any]:
    system.motors.stop_all_wheels()
    system.actuators.stop_all_actuators()
    if system.serial.connected:
        system.serial.send_command("ESTOP")
        system.serial.send_command("STOP")
    system.safety.trigger_emergency_stop(req.reason or "Emergency Stop Activated")
    return {
        "ok": True,
        "message": "EMERGENCY STOP LATCHED. All outputs disabled.",
        "state": system.get_full_state(),
    }


@app.post("/api/safety/reset_estop")
async def safety_reset_estop() -> Dict[str, Any]:
    if system.serial.connected:
        system.serial.send_command("CLEAR_ESTOP")
        system.serial.send_command("STOP")
    ok, msg = system.safety.reset_emergency_stop()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/safety/clear_fault")
async def safety_clear_fault() -> Dict[str, Any]:
    system.diagnostics.clear_fault_counters()
    ok, msg = system.safety.clear_fault()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/system/shutdown")
async def system_shutdown(req: SystemShutdownRequest) -> Dict[str, Any]:
    """Safely halts robot hardware, locks Mega in E-STOP, and shuts down Raspberry Pi OS."""
    if not req.confirm:
        return {
            "ok": False,
            "poweroff_scheduled": False,
            "message": "Shutdown requires explicit confirmation (confirm=true).",
        }
    return system.initiate_poweroff(
        cut_usb_power=req.cut_usb_power,
        reason=req.reason or "Operator initiated full robot shutdown",
    )


# ==================== WHEEL MOTOR & MECANUM ENDPOINTS ====================

@app.post("/api/motors/single")
async def motor_single(req: SingleMotorRequest) -> Dict[str, Any]:
    ok, msg = system.motors.command_single_motor(
        motor_id=req.motor_id,
        pwm=req.pwm,
        direction=req.direction,
        duration_ms=req.duration_ms,
    )
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/motors/stop")
async def motor_stop() -> Dict[str, Any]:
    ok, msg = system.motors.stop_all_wheels()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/motors/drive")
async def motor_drive(req: DriveRequest) -> Dict[str, Any]:
    if req.direction:
        ok, msg = system.motors.drive_direction(req.direction, req.speed_pwm)
    elif req.vy is not None and req.vx is not None and req.omega is not None:
        ok, msg = system.motors.drive_vector(req.vy, req.vx, req.omega, req.speed_pwm)
    else:
        ok, msg = system.motors.stop_all_wheels()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/motors/calibration")
async def motor_calibration(req: WheelCalibrationRequest) -> Dict[str, Any]:
    ok, msg = system.motors.update_calibration(
        positions=req.positions,
        inverted=req.inverted,
        max_pwm_limit=req.max_pwm_limit,
    )
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/motors/test_result")
async def motor_test_result(req: MotorTestResultRequest) -> Dict[str, Any]:
    system.motors.record_test_result(req.motor_id, req.result)
    return {"ok": True, "state": system.get_full_state()}


# ==================== LINEAR ACTUATOR ENDPOINTS ====================

@app.post("/api/actuators/command")
async def actuator_command(req: ActuatorCommandRequest) -> Dict[str, Any]:
    ok, msg = system.actuators.command_group(
        group=req.group,
        action=req.action,
        pwm=req.pwm,
        duration_ms=req.duration_ms,
    )
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/actuators/stop")
async def actuator_stop() -> Dict[str, Any]:
    ok, msg = system.actuators.stop_all_actuators()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/actuators/settings")
async def actuator_settings(req: ActuatorSettingsRequest) -> Dict[str, Any]:
    system.actuators.update_settings(
        driver_electrical_verified=req.driver_electrical_verified,
        allow_simultaneous_groups=req.allow_simultaneous_groups,
        require_imu_for_lifting=req.require_imu_for_lifting,
        upper_inverted=req.upper_inverted,
        lower_inverted=req.lower_inverted,
    )
    return {"ok": True, "message": "Actuator safety settings updated.", "state": system.get_full_state()}


# ==================== SERVO ENDPOINTS ====================

@app.post("/api/servo/enable")
async def servo_enable(req: ServoEnableRequest) -> Dict[str, Any]:
    ok, msg = system.servo.set_enabled(req.enabled)
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/servo/angle")
async def servo_angle(req: ServoAngleRequest) -> Dict[str, Any]:
    ok, msg = system.servo.set_angle(req.angle_deg)
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/servo/center")
async def servo_center() -> Dict[str, Any]:
    ok, msg = system.servo.center()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/servo/limits")
async def servo_limits(req: ServoLimitsRequest) -> Dict[str, Any]:
    ok, msg = system.servo.update_limits(req.safe_min_deg, req.safe_max_deg, req.inverted)
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


# ==================== IMU ENDPOINTS ====================

@app.post("/api/imu/thresholds")
async def imu_thresholds(req: ImuThresholdsRequest) -> Dict[str, Any]:
    try:
        system.imu.update_thresholds(req.warning_deg, req.critical_deg)
        return {"ok": True, "message": "IMU tilt thresholds updated.", "state": system.get_full_state()}
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))


@app.post("/api/imu/request_sample")
async def imu_request_sample() -> Dict[str, Any]:
    ok, msg = system.serial.send_command("IMU")
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


# ==================== CAMERA ENDPOINTS ====================

@app.post("/api/camera/start")
async def camera_start(req: CameraStartRequest) -> Dict[str, Any]:
    ok, msg = system.camera.start(source=req.source, width=req.width, height=req.height)
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/camera/stop")
async def camera_stop() -> Dict[str, Any]:
    ok, msg = system.camera.stop()
    return {"ok": ok, "message": msg, "state": system.get_full_state()}


@app.post("/api/camera/capture")
async def camera_capture() -> Dict[str, Any]:
    ok, msg, filename = system.camera.capture_photo()
    return {
        "ok": ok,
        "message": msg,
        "filename": filename,
        "url": f"/captures/{filename}" if filename else None,
        "state": system.get_full_state(),
    }


@app.get("/api/camera/frame.jpg")
async def camera_single_frame() -> Response:
    frame = system.camera.get_latest_jpeg()
    if not frame:
        return Response(status_code=204)
    return Response(
        content=frame,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0"},
    )


@app.get("/api/ai/status")
async def ai_vision_status() -> Dict[str, Any]:
    return {
        "ok": True,
        "ai_vision": system.ai_vision.get_status(),
        "autonomous": system.autonomous.get_status(),
        "serial_connected": system.serial.connected,
        "serial_port": system.serial.port_name or "DISCONNECTED",
        "safety_state": system.safety.state.value,
    }


@app.get("/api/autonomous/status")
async def autonomous_status() -> Dict[str, Any]:
    return {
        "ok": True,
        "autonomous": system.autonomous.get_status(),
        "ai_vision": system.ai_vision.get_status(),
        "serial_connected": system.serial.connected,
        "safety_state": system.safety.state.value,
    }


@app.post("/api/autonomous/mode")
async def autonomous_set_mode(req: AutonomousModeRequest) -> Dict[str, Any]:
    ok, msg, auto_state = system.autonomous.set_enabled(req.enabled, speed_pwm=req.speed_pwm)
    return {
        "ok": ok,
        "message": msg,
        "autonomous": auto_state,
        "ai_vision": system.ai_vision.get_status(),
    }


@app.api_route("/api/app/liftmate.apk", methods=["GET", "HEAD"])
async def download_android_apk() -> FileResponse:
    apk_path = PROJECT_ROOT / "android" / "LiftMate-Control.apk"
    if not apk_path.exists():
        raise HTTPException(status_code=404, detail="Android APK has not been built yet.")
    return FileResponse(
        apk_path,
        media_type="application/vnd.android.package-archive",
        filename="LiftMate-Control.apk",
    )


@app.post("/api/ai/config")
async def ai_vision_config(req: AIConfigRequest) -> Dict[str, Any]:
    msg = "AI configuration updated."
    ok = True
    if req.enabled is not None:
        system.ai_vision.set_enabled(req.enabled)
        msg = "AI Detection enabled." if req.enabled else "AI Detection disabled."
    if req.mode is not None:
        ok, msg, _ = system.ai_vision.set_mode(req.mode)
    return {
        "ok": ok,
        "message": msg,
        "ai_vision": system.ai_vision.get_status(),
    }


@app.get("/api/camera/stream.mjpg")
async def camera_mjpeg_stream() -> StreamingResponse:
    async def frame_generator():
        last_sent = -1
        while system.camera.streaming and system.camera.connected:
            frame = system.camera.get_latest_jpeg()
            cnt = system.camera.frame_count
            if frame and cnt != last_sent:
                last_sent = cnt
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode("ascii") + b"\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
            await asyncio.sleep(0.03)

    return StreamingResponse(
        frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0"},
    )


# ==================== OFFLINE MAP & NAVIGATION ENDPOINTS ====================

@app.get("/api/map/tiles/{mode}/{z}/{x}/{y}.png")
async def get_offline_map_tile(mode: str, z: int, x: int, y: int) -> Response:
    z_clamped = max(0, min(22, int(z)))
    tile_data = await asyncio.to_thread(
        system.navigation.tiles.get_tile, mode.lower(), z_clamped, int(x), int(y)
    )
    media = "image/jpeg" if tile_data[:2] == b"\xff\xd8" else "image/png"
    return Response(
        content=tile_data,
        media_type=media,
        headers={"Cache-Control": "public, max-age=3600"},
    )


@app.get("/api/navigation/status")
async def navigation_status() -> Dict[str, Any]:
    return {"ok": True, "navigation": system.navigation.get_status()}


@app.post("/api/navigation/start")
async def navigation_start() -> Dict[str, Any]:
    nav = system.navigation.start_tracking()
    return {"ok": True, "message": "Route tracking started.", "navigation": nav}


@app.post("/api/navigation/stop")
async def navigation_stop() -> Dict[str, Any]:
    nav = system.navigation.stop_tracking()
    return {"ok": True, "message": "Route tracking paused.", "navigation": nav}


@app.post("/api/navigation/reset")
async def navigation_reset() -> Dict[str, Any]:
    nav = system.navigation.reset_route()
    return {"ok": True, "message": "Route & relative pose reset (X=0, Y=0, Heading=0).", "navigation": nav}


@app.post("/api/navigation/save")
async def navigation_save(req: NavSaveRequest) -> Dict[str, Any]:
    saved = system.navigation.save_current_route(req.name)
    return {"ok": True, "saved": saved, "navigation": system.navigation.get_status()}


@app.get("/api/navigation/routes")
async def navigation_list_routes() -> Dict[str, Any]:
    return {"ok": True, "routes": system.navigation.route_db.list_routes()}


@app.post("/api/navigation/mode")
async def navigation_set_mode(req: NavModeRequest) -> Dict[str, Any]:
    nav = system.navigation.set_mode(req.mode)
    return {"ok": True, "navigation": nav}


@app.post("/api/navigation/calibrate_imu")
async def navigation_calibrate_imu() -> Dict[str, Any]:
    system.navigation.mpu.calibrate_stationary_gyro()
    return {"ok": True, "message": "Stationary gyroscope calibrated & heading zeroed.", "navigation": system.navigation.get_status()}


@app.post("/api/navigation/set_origin")
async def navigation_set_origin(req: NavOriginRequest) -> Dict[str, Any]:
    nav = system.navigation.set_origin(req.lat, req.lon)
    return {"ok": True, "navigation": nav}


@app.post("/api/navigation/mobile_gps")
async def navigation_mobile_gps(req: NavMobileGpsRequest) -> Dict[str, Any]:
    nav = system.navigation.update_mobile_gps(
        lat=req.lat,
        lon=req.lon,
        accuracy_m=req.accuracy,
        heading_deg=req.heading,
        source=req.source,
    )
    return {"ok": True, "navigation": nav}


@app.post("/api/navigation/demo_step")
async def navigation_demo_step(req: NavDemoStepRequest) -> Dict[str, Any]:
    nav = system.navigation.simulate_demo_step(req.dx_m, req.dy_m, req.dheading_deg)
    return {"ok": True, "navigation": nav}


@app.post("/api/navigation/repair_pins")
async def navigation_add_repair_pin(req: NavRepairPinRequest) -> Dict[str, Any]:
    pin = system.navigation.add_repair_pin(lat=req.lat, lon=req.lon, label=req.label)
    return {"ok": True, "pin": pin, "navigation": system.navigation.get_status()}


@app.post("/api/navigation/repair_pins/repaired_latest")
async def navigation_mark_latest_repaired() -> Dict[str, Any]:
    pin = system.navigation.mark_repair_pin_repaired(None)
    return {"ok": True, "pin": pin, "navigation": system.navigation.get_status()}


@app.post("/api/navigation/repair_pins/{pin_id}/repaired")
async def navigation_mark_pin_repaired(pin_id: int) -> Dict[str, Any]:
    pin = system.navigation.mark_repair_pin_repaired(pin_id)
    return {"ok": True, "pin": pin, "navigation": system.navigation.get_status()}


@app.put("/api/navigation/repair_pins/{pin_id}/move")
async def navigation_move_repair_pin(pin_id: int, req: NavOriginRequest) -> Dict[str, Any]:
    nav = system.navigation.update_repair_pin_location(pin_id, req.lat, req.lon)
    return {"ok": True, "navigation": nav}


@app.delete("/api/navigation/repair_pins/{pin_id}")
async def navigation_delete_repair_pin(pin_id: int) -> Dict[str, Any]:
    nav = system.navigation.delete_repair_pin(pin_id)
    return {"ok": True, "navigation": nav}


@app.post("/api/navigation/repair_pins/clear")
async def navigation_clear_repair_pins() -> Dict[str, Any]:
    nav = system.navigation.clear_repair_pins()
    return {"ok": True, "navigation": nav}


# ==================== DIAGNOSTICS ENDPOINTS ====================

@app.get("/api/diagnostics/events")
async def diagnostics_events(limit: int = 100) -> Dict[str, Any]:
    return {
        "summary": system.diagnostics.get_summary(),
        "events": system.diagnostics.get_events(limit=limit),
    }


@app.get("/api/diagnostics/export.csv")
async def diagnostics_export_csv() -> PlainTextResponse:
    csv_text = system.diagnostics.export_csv_string()
    return PlainTextResponse(
        content=csv_text,
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=liftmate_diagnostics.csv"},
    )


# ==================== WEBSOCKET TELEMETRY & REAL-TIME CONTROL ====================

@app.websocket("/ws/telemetry")
async def websocket_telemetry(ws: WebSocket) -> None:
    await ws.accept()
    try:
        while True:
            # Broadcast full state every 100ms (10 Hz)
            await ws.send_json({"type": "STATE_UPDATE", "data": system.get_full_state()})
            try:
                msg = await asyncio.wait_for(ws.receive_json(), timeout=0.1)
                action = msg.get("action")
                if action == "STOP_ALL":
                    system.stop_all_outputs("WebSocket STOP_ALL received")
                elif action == "DRIVE_HOLD":
                    system.motors.drive_direction(
                        msg.get("direction", "STOP"),
                        int(msg.get("speed_pwm", 75)),
                    )
                elif action == "ACTUATOR_HOLD":
                    system.actuators.command_group(
                        group=msg.get("group", "UPPER"),
                        action=msg.get("cmd", "STOP"),
                        pwm=int(msg.get("pwm", 120)),
                    )
                elif action == "MOBILE_GPS":
                    system.navigation.update_mobile_gps(
                        lat=float(msg.get("lat", 25.185685042934363)),
                        lon=float(msg.get("lon", 51.57190161939388)),
                        accuracy_m=msg.get("accuracy"),
                        heading_deg=msg.get("heading"),
                        source=str(msg.get("source", "mobile")),
                    )
            except asyncio.TimeoutError:
                pass
    except WebSocketDisconnect:
        # Safety requirement: stop all motors/actuators immediately if browser tab disconnects!
        system.stop_all_outputs("Browser WebSocket client disconnected")
    except Exception:
        system.stop_all_outputs("WebSocket error/disconnect")

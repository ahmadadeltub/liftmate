# LiftMate — Advanced Robotic Control & Diagnostic Center

**LiftMate: A Mobile Robotic System for Recovering Disabled Vehicles and Clearing Accident Lanes to Reduce Traffic Congestion**

> **IMPORTANT SAFETY NOTICE**: This software is designed exclusively for **unloaded bench testing and elevated-wheel diagnostics** of the LiftMate research prototype. Do **not** use this application for autonomous vehicle lifting or roadside recovery operations. A software `STOP ALL` button is **not** a substitute for a hardwired physical Emergency Stop switch that removes 12V motor and actuator power.

---

## 1. Project Architecture

```text
LiftMate-Control-Center/
|-- firmware/
|   |-- RobotJackSafe/
|   |   |-- RobotJackSafe.ino               # Improved safety-hardened Arduino Mega 2560 firmware
|   |-- Robot_jack_code_backup/
|       |-- Robot_jack_code.ino             # Preserved backup of original firmware
|-- backend/
|   |-- app/
|   |   |-- main.py                         # FastAPI REST + 10Hz WebSocket Telemetry Server
|   |   |-- config.py                       # YAML configuration manager
|   |   |-- serial_controller.py            # Thread-safe serial service & strict frame parser
|   |   |-- motor_controller.py             # 4-wheel testing, calibration wizard, Mecanum mixing
|   |   |-- actuator_controller.py          # Upper (100mm) & Lower (30mm) grouped actuator control
|   |   |-- servo_controller.py             # Lazy-enabled pin 8 servo controller with safe limits
|   |   |-- camera_controller.py            # Picamera2 (CSI), Remote Pi 3, and OpenCV (USB) camera manager
|   |   |-- imu_controller.py               # BNO055 (0x29) orientation & tilt threshold monitor
|   |   |-- safety_controller.py            # Software safety state machine & mutual-exclusion interlocks
|   |   |-- diagnostics.py                  # Event ring buffer & CSV logger
|   |-- pi_camera_agent.py                  # Lightweight Picamera2 stream agent for Raspberry Pi
|-- frontend/
|   |-- index.html                          # 10-section responsive dark-navy control center
|   |-- css/styles.css                      # Styling & responsive layout
|   |-- js/app.js                           # WebSocket telemetry, press-and-hold engine, IMU canvas graph
|-- tests/
|   |-- test_protocol.py                    # Framing & IMU-vs-ACK separation tests
|   |-- test_serial.py                      # Simulator, watchdog, and reconnect re-arm tests
|   |-- test_motors.py                      # Mecanum kinematics, PWM clamping, calibration tests
|   |-- test_actuators.py                   # Actuator interlock & driver verification tests
|   |-- test_safety.py                      # State machine, latched E-Stop, servo & camera tests
|-- config/
|   |-- robot_config.yaml                   # Persistent calibration & hardware configuration
|-- logs/                                   # Diagnostic CSV logs & captured camera snapshots
|-- requirements.txt
|-- install.sh
|-- run.sh
|-- README.md
|-- WIRING.md
|-- TESTING_GUIDE.md
```

---

## 2. Quick Start (Local Laptop or Raspberry Pi 5 / Raspberry Pi 3)

### Option A: Automated Install & Run
```bash
cd LiftMate-Control-Center
chmod +x install.sh run.sh
./install.sh
./run.sh
```
Open **`http://127.0.0.1:8000`** in your browser.

### Option B: Run in Hardware Simulation Mode (No Physical Arduino Required)
```bash
./run.sh --sim
```
You can also toggle between **Hardware Mode** and **Simulation Mode** live inside the web dashboard.

### Option C: Enable LAN Access (When Accessing Raspberry Pi Remotely)
By default, the service binds strictly to `127.0.0.1:8000`. To allow LAN access after verifying local network security:
```bash
./run.sh --lan --port 8000
```

---

## 3. Raspberry Pi 5 / Raspberry Pi OS Setup Instructions

1. **System Packages & Picamera2**:
   `Picamera2` and `libcamera` are distributed via the Raspberry Pi OS `apt` repository. Always create the Python virtual environment with `--system-site-packages` (handled automatically by `./install.sh`):
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-venv python3-pip python3-picamera2 python3-opencv rpicam-apps i2c-tools
   python3 -m venv --system-site-packages .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```
2. **Serial Device Permissions**:
   Grant your user access to `/dev/ttyACM0` / `/dev/ttyUSB0`:
   ```bash
   sudo usermod -a -G dialout,video,i2c $USER
   ```
3. **Detect Connected Arduino Mega 2560**:
   - On Raspberry Pi: `ls -l /dev/ttyACM* /dev/ttyUSB*`
   - On macOS: `ls -l /dev/cu.usbserial* /dev/cu.usbmodem*`
4. **Compile & Upload Improved Firmware (`RobotJackSafe.ino`)**:
   ```bash
   arduino-cli core install arduino:avr
   arduino-cli lib install "Adafruit BNO055" "Adafruit Unified Sensor" "Adafruit BusIO" "Servo"
   arduino-cli compile --fqbn arduino:avr:mega firmware/RobotJackSafe/RobotJackSafe.ino
   arduino-cli upload -p /dev/cu.usbserial-11330 --fqbn arduino:avr:mega firmware/RobotJackSafe/RobotJackSafe.ino
   ```
5. **Raspberry Pi Camera v2.1 (IMX219) Setup**:
   - Ensure `/boot/firmware/config.txt` includes `camera_auto_detect=1` and `dtoverlay=imx219`.
   - Verify detection with `rpicam-hello --list-cameras`.

---

## 4. Running Automated Tests

```bash
PYTHONPATH=. pytest -v tests/
```

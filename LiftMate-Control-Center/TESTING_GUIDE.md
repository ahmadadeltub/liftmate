# LiftMate — Guided 14-Step Physical Testing & Calibration Guide

> **PRE-TEST MANDATE**: All tests must be performed with **all four mecanum wheels elevated safely off the table/floor** on rigid blocks or a test stand, and with **both linear actuator groups unloaded**.

---

## Step-by-Step Verification Sequence

### Step 1: Verify Motor Power Isolation
- Confirm the robot chassis is securely supported on blocks with all four wheels spinning freely in the air.
- Confirm the physical 12V motor power switch / E-Stop is within immediate reach.

### Step 2: Connect Arduino Mega 2560
- Connect the Arduino Mega 2560 to the host (Raspberry Pi or laptop) via USB cable.
- Start the LiftMate Control Center (`./run.sh`) and open `http://127.0.0.1:8000`.

### Step 3: Verify Serial Communication
- In **1. System Overview** or **9. Diagnostics & Logs**, confirm the serial port is connected at `115200` baud and `READY Handshake` shows `YES (#,READY,#)`.
- Click **PING ARDUINO** and verify `#,PONG,#` is received with low latency (`< 20 ms`).

### Step 4: Verify Emergency Stop & Arming Interlocks
- While the state is **IDLE (Disarmed)**, attempt to jog a wheel motor — confirm the software blocks the command.
- Click **ARM SYSTEM** (state transitions to **ARMED**).
- Click **E-STOP LATCH** (state transitions to **EMERGENCY_STOP**). Verify all motion is blocked until **RESET E-STOP** is clicked, and confirm the system returns to **IDLE** (requiring manual **ARM SYSTEM** again).

### Step 5: Test One Wheel at a Time (Wheels Elevated)
- Navigate to **2. Wheel Testing** and click **ARM SYSTEM**.
- Keep the test speed slider at the default **`50 PWM`** and Timed Jog Pulse at **`350 ms`**.
- Click **JOG FWD** and **JOG REV** on **Motor 1**, **Motor 2**, **Motor 3**, and **Motor 4** one at a time.
- Verify each motor stops automatically after `350 ms`.

### Step 6: Calibrate Physical Wheel Order & Polarity
- In the **Interactive Wheel Mapping & Polarity Calibration Wizard**:
  - Assign each Motor (`1–4`) to the physical wheel corner (`Front Left`, `Front Right`, `Rear Left`, `Rear Right`) that spun during Step 5.
  - Check **Invert Motor Polarity** for any channel where `JOG FWD` rotated the wheel backward.
  - Click **SAVE CALIBRATION TO CONFIG** (persists to `config/robot_config.yaml`).

### Step 7: Test Mecanum Movements at Low Speed
- Navigate to **3. Movement Control**.
- Set **Movement Speed** to `60–75 PWM`.
- Press and hold each directional button (`FORWARD`, `BACKWARD`, `STRAFE L`, `STRAFE R`, `ROTATE LEFT`, `ROTATE RIGHT`, and the 4 diagonals).
- Verify that releasing the button immediately sends `STOP` (`0 PWM` on all 4 wheels).

### Step 8: Verify Actuator Power & Driver Rating
- Review **10. Safety & Electrical** and `WIRING.md`.
- Once verified, check **"Confirm Motor Driver & 12V Supply Rated for 4 Parallel Actuators"** in **4. Upper Actuators**.

### Step 9: Test Unloaded Lower Base Actuator Group (`LOWER BASE – 30mm`)
- Navigate to **5. Lower Actuators**.
- Click **PULSE EXTEND** (`250 ms`) and **PULSE RETRACT** (`250 ms`), then test **HOLD TO EXTEND** and **HOLD TO RETRACT** briefly, confirming immediate stop on button release.

### Step 10: Test Unloaded Upper Jack Actuator Group (`UPPER JACK – 100mm`)
- Navigate to **4. Upper Actuators**.
- Click **PULSE EXTEND** (`300 ms`) and **PULSE RETRACT** (`300 ms`), then test **HOLD TO EXTEND** and **HOLD TO RETRACT** briefly, confirming immediate stop on release and verifying that Upper and Lower groups cannot run simultaneously by default.

### Step 11: Verify Servo Movement (`Pin 8`)
- Navigate to **7. Servo Controller**.
- Confirm the servo has not moved during startup.
- Check **Enable Servo Control Gate**, click **CENTER (90°)**, and test gentle angle adjustments within the configured safe travel window (`10°–170°`).

### Step 12: Verify Camera Preview
- Navigate to **6. Camera Monitor**.
- Select camera source (`Auto-Detect`, `Picamera2`, `Remote Raspberry Pi 3`, or `USB Webcam`) and click **START CAMERA**.
- Verify live FPS indicator and click **CAPTURE PHOTO** to save a timestamped JPEG in `logs/captures/`.

### Step 13: Verify BNO055 IMU Telemetry
- Navigate to **8. IMU Monitoring**.
- Verify live `Heading`, `Pitch`, and `Roll` values from I2C address `0x29` (`#,heading,pitch,roll,#` at 10 Hz) and inspect the live Pitch/Roll time-series graph.

### Step 14: Test Disconnection & Watchdog Behavior
- While holding a wheel movement button, click outside the browser window (blur focus) and verify immediate automatic stop.
- Verify that firmware and software watchdogs (`450 ms` wheel, `600 ms` actuator hold) halt outputs automatically if command refresh ceases.

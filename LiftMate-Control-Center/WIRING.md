# LiftMate — Hardware Pinout, Motor Driver & Electrical Verification Guide

## 1. Preserved Arduino Mega 2560 Pin Configuration

All pin assignments from `Robot_jack_code.ino` are preserved as the default configuration:

| Subsystem | Channel | PWM Pin | Direction `IN1` (`inA`) | Direction `IN2` (`inB`) | Notes |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **Wheel Motor 1** | `wheels[0]` | **Pin 4** | **Pin 26** | **Pin 27** | Physical corner calibrated via Wheel Mapping Wizard |
| **Wheel Motor 2** | `wheels[1]` | **Pin 5** | **Pin 28** | **Pin 29** | Labeled `right rear` in original sketch |
| **Wheel Motor 3** | `wheels[2]` | **Pin 6** | **Pin 44** | **Pin 45** | Labeled `left front` in original sketch |
| **Wheel Motor 4** | `wheels[3]` | **Pin 7** | **Pin 46** | **Pin 47** | Labeled `right front` in original sketch |
| **Lower Actuator Group** | `lowerJack` | **Pin 2** | **Pin 22** | **Pin 23** | 4 × 12V, 30mm stroke actuators wired as **ONE group** |
| **Upper Actuator Group** | `upperJack` | **Pin 3** | **Pin 24** | **Pin 25** | 4 × 12V, 50N, 100mm stroke actuators wired as **ONE group** |
| **Servo Motor** | `robotServo` | **Pin 8** | — | — | Lazy-attached on first `#,SERVO,<angle>,#` command |
| **BNO055 IMU** | Hardware I2C | **0x29** | **SDA = Pin 20** | **SCL = Pin 21** | Requires `ADR` pin pulled HIGH (3.3V/5V) for address `0x29` |

---

## 2. Analysis of `setMotor()` Driver Interface

In `Robot_jack_code.ino` and `RobotJackSafe.ino`, `setMotor()` uses **1 PWM Enable pin + 2 Digital Direction pins (`inA`, `inB`)** per motor channel:

```cpp
analogWrite(motor.pwm, 0);
digitalWrite(motor.inA, speed > 0 ? HIGH : LOW);
digitalWrite(motor.inB, speed < 0 ? HIGH : LOW);
analogWrite(motor.pwm, speed < 0 ? -speed : speed);
```

### Truth Table

| Commanded Speed | `PWM` Pin | `inA` (`IN1`) | `inB` (`IN2`) | Bridge Output State |
| :--- | :---: | :---: | :---: | :--- |
| `speed == 0` | `0` (LOW) | `LOW` | `LOW` | Bridge Disabled (High-Z Coast) |
| `speed > 0` (`+1..+255`) | `speed` | `HIGH` | `LOW` | Forward / Extend Polarity |
| `speed < 0` (`-1..-255`) | `abs(speed)` | `LOW` | `HIGH` | Reverse / Retract Polarity |

### Driver Compatibility Profile

1. **L298N Dual H-Bridge / TB6612FNG / VNH2SP30 (Direct Match)**:
   - Connect Arduino `PWM` pin → Driver `ENA` / `ENB` (remove the 5V jumper on `ENA`/`ENB`).
   - Connect Arduino `inA` → Driver `IN1` (`IN3`).
   - Connect Arduino `inB` → Driver `IN2` (`IN4`).
2. **BTS7960 (IBT-2 43A H-Bridge) — IMPORTANT WIRING DIFFERENCE**:
   - Standard BTS7960 modules have **two PWM inputs (`RPWM`, `LPWM`)** and **two Enable inputs (`R_EN`, `L_EN`)**.
   - **Do NOT assume BTS7960 and L298N have identical wiring.**
   - To use a BTS7960 with the existing 1-PWM + 2-Direction pin mapping without changing Arduino pins:
     - Tie `R_EN` and `L_EN` together and connect them to the Arduino `PWM` pin (`Pin 2` for Lower Jack, `Pin 3` for Upper Jack).
     - Connect `RPWM` to `inA` (`Pin 22` / `Pin 24`) and `LPWM` to `inB` (`Pin 23` / `Pin 25`).
     - Verify with a multimeter before connecting all four actuators.

---

## 3. Electrical & Parallel Actuator Current Verification Checklist

- [ ] **Parallel Actuator Current Draw**: Four 12V linear actuators connected in parallel draw **4× the single-actuator current** (typically `4 × 1.5A–3A = 6A–12A` peak/inrush). A single L298N channel (rated at `2A` continuous) **cannot safely drive 4 parallel actuators under load** without overheating or thermal shutdown. Use a high-current driver (such as a properly wired BTS7960 / Cytron MD10C / VNH5019) for each 4-actuator group.
- [ ] **Common Ground**: Arduino Mega `GND`, Raspberry Pi `GND` (via USB), and all 12V Motor Driver logic `GND` terminals **must share a solid common ground**.
- [ ] **Power Supply & Fuse Isolation**: Use an inline blade fuse (`15A–20A` depending on wire gauge and driver rating) on the main 12V bus.
- [ ] **Mandatory Physical Emergency-Stop Switch**: Wire a physical mushroom-head NC (Normally Closed) Emergency Stop switch directly in series with the **+12V motor/actuator power bus**. Keep the Arduino Mega and Raspberry Pi powered via 5V USB so diagnostics and IMU telemetry remain active when 12V motor power is cut.
- [ ] **5V Logic & Servo Power**: Do **not** power high-torque servos directly from the Arduino Mega's onboard 5V linear regulator if powered from 12V `VIN`; use a dedicated 5V/6V BEC with common `GND`.

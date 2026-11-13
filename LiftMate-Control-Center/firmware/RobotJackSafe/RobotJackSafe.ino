/*
 * LiftMate - RobotJackSafe.ino
 * Production-Grade Safety Firmware for Arduino Mega 2560
 *
 * Hardware Mapping (Preserved from Robot_jack_code.ino):
 *   Wheel Motor 1 : PWM = 4, IN1 = 26, IN2 = 27
 *   Wheel Motor 2 : PWM = 5, IN1 = 28, IN2 = 29
 *   Wheel Motor 3 : PWM = 6, IN1 = 44, IN2 = 45
 *   Wheel Motor 4 : PWM = 7, IN1 = 46, IN2 = 47
 *   Lower Jack    : PWM = 2, IN1 = 22, IN2 = 23 (4x 12V 30mm actuators grouped)
 *   Upper Jack    : PWM = 3, IN1 = 24, IN2 = 25 (4x 12V 50N 100mm actuators grouped)
 *   Servo         : Pin 8 (Lazy-attached on first SERVO command)
 *   BNO055 IMU    : I2C 0x29 (SDA = 20, SCL = 21)
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// Set true when the Raspberry Pi is connected to Serial3 (RX3=15, TX3=14).
constexpr bool USE_SERIAL3 = false;
constexpr unsigned long BAUD = 115200;
constexpr unsigned long SAMPLE_INTERVAL_MS = 100;
constexpr unsigned long IMU_RETRY_INTERVAL_MS = 3000;

// Default Watchdog & Absolute Motion Limits (ms)
constexpr unsigned long DEFAULT_WHEEL_TIMEOUT_MS = 450;
constexpr unsigned long DEFAULT_JACK_CMD_TIMEOUT_MS = 600;
constexpr unsigned long UPPER_JACK_MAX_RUN_MS = 10000;
constexpr unsigned long LOWER_JACK_MAX_RUN_MS = 5000;

constexpr uint8_t SERVO_PIN = 8;
constexpr int HARD_SERVO_MIN_ANGLE = 0;
constexpr int HARD_SERVO_MAX_ANGLE = 180;
constexpr size_t RX_CAPACITY = 96;

struct MotorChannel {
  uint8_t pwm;
  uint8_t inA;
  uint8_t inB;
  int speed;
  unsigned long lastCommandMs;
  unsigned long motionStartMs;
  bool limitTripped;
};

// Wheel order in DRIVE is motor1, motor2, motor3, motor4.
MotorChannel wheels[] = {
  {4, 26, 27, 0, 0, 0, false}, // Motor 1: physical position calibrated in software
  {5, 28, 29, 0, 0, 0, false}, // Motor 2
  {6, 44, 45, 0, 0, 0, false}, // Motor 3
  {7, 46, 47, 0, 0, 0, false}  // Motor 4
};
MotorChannel lowerJack = {2, 22, 23, 0, 0, 0, false};
MotorChannel upperJack = {3, 24, 25, 0, 0, 0, false};

Adafruit_BNO055 bno(55, 0x29, &Wire);
Servo robotServo;

char rxBuffer[RX_CAPACITY];
size_t rxLength = 0;
bool discardLine = false;

unsigned long lastSampleMs = 0;
unsigned long lastImuRetryMs = 0;
unsigned long wheelTimeoutMs = DEFAULT_WHEEL_TIMEOUT_MS;
unsigned long jackCmdTimeoutMs = DEFAULT_JACK_CMD_TIMEOUT_MS;

int servoMinAngle = HARD_SERVO_MIN_ANGLE;
int servoMaxAngle = HARD_SERVO_MAX_ANGLE;
int currentServoAngle = 90;

bool imuDetected = false;
bool estopLatched = false;
bool prohibitSimultaneousJacks = true;

HardwareSerial &controlPort() {
  if (USE_SERIAL3) return Serial3;
  return Serial;
}

void replyError(const __FlashStringHelper *reason) {
  controlPort().print(F("#,ERR,"));
  controlPort().print(reason);
  controlPort().println(F(",#"));
}

void setMotor(MotorChannel &motor, int speed) {
  const unsigned long now = millis();
  if (speed == 0) {
    analogWrite(motor.pwm, 0);
    digitalWrite(motor.inA, LOW);
    digitalWrite(motor.inB, LOW);
    motor.speed = 0;
    motor.lastCommandMs = now;
    motor.motionStartMs = 0;
    motor.limitTripped = false;
    return;
  }

  // If transitioning from stop or reversing direction, reset continuous run timer
  if (motor.speed == 0 || (motor.speed > 0 && speed < 0) || (motor.speed < 0 && speed > 0)) {
    // Brief dead-time disable before changing bridge polarity
    analogWrite(motor.pwm, 0);
    digitalWrite(motor.inA, LOW);
    digitalWrite(motor.inB, LOW);
    motor.motionStartMs = now;
    motor.limitTripped = false;
  }

  digitalWrite(motor.inA, speed > 0 ? HIGH : LOW);
  digitalWrite(motor.inB, speed < 0 ? HIGH : LOW);
  analogWrite(motor.pwm, speed < 0 ? -speed : speed);
  motor.speed = speed;
  motor.lastCommandMs = now;
}

void initMotor(MotorChannel &motor) {
  pinMode(motor.pwm, OUTPUT);
  pinMode(motor.inA, OUTPUT);
  pinMode(motor.inB, OUTPUT);
  setMotor(motor, 0);
}

void stopWheels() {
  for (uint8_t i = 0; i < 4; ++i) {
    setMotor(wheels[i], 0);
  }
}

void stopJacks() {
  setMotor(upperJack, 0);
  setMotor(lowerJack, 0);
}

void stopAll() {
  stopWheels();
  stopJacks();
}

bool anyMotorActive() {
  for (uint8_t i = 0; i < 4; ++i) {
    if (wheels[i].speed != 0) return true;
  }
  return (upperJack.speed != 0) || (lowerJack.speed != 0);
}

bool checkCommandTimeout(MotorChannel &motor, unsigned long now, unsigned long timeoutMs) {
  if (motor.speed != 0 && (unsigned long)(now - motor.lastCommandMs) >= timeoutMs) {
    setMotor(motor, 0);
    return true;
  }
  return false;
}

bool checkContinuousLimit(MotorChannel &motor, unsigned long now, unsigned long maxRunMs) {
  if (motor.speed != 0 && (unsigned long)(now - motor.motionStartMs) >= maxRunMs) {
    setMotor(motor, 0);
    motor.limitTripped = true;
    return true;
  }
  return false;
}

void checkMotionTimeouts() {
  const unsigned long now = millis();
  bool timedOut = false;
  bool maxRunTripped = false;

  for (uint8_t i = 0; i < 4; ++i) {
    if (checkCommandTimeout(wheels[i], now, wheelTimeoutMs)) {
      timedOut = true;
    }
  }

  if (checkCommandTimeout(upperJack, now, jackCmdTimeoutMs)) {
    timedOut = true;
  }
  if (checkCommandTimeout(lowerJack, now, jackCmdTimeoutMs)) {
    timedOut = true;
  }

  if (checkContinuousLimit(upperJack, now, UPPER_JACK_MAX_RUN_MS)) {
    maxRunTripped = true;
    timedOut = true;
  }
  if (checkContinuousLimit(lowerJack, now, LOWER_JACK_MAX_RUN_MS)) {
    maxRunTripped = true;
    timedOut = true;
  }

  if (maxRunTripped) {
    replyError(F("ACTUATOR_MAX_RUN_LIMIT"));
  }
  if (timedOut) {
    controlPort().println(F("#,TIMEOUT,#"));
  }
}

void configureI2C() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true); // microseconds; reset TWI on timeout
}

float eulerDegrees(uint8_t low, uint8_t high) {
  return int16_t(uint16_t(low) | (uint16_t(high) << 8)) / 16.0f;
}

bool sendIMU() {
  if (!imuDetected) return false;
  if (controlPort().availableForWrite() < 40) return false;

  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(0x29);
  Wire.write(uint8_t(0x1A)); // Euler heading, roll, pitch (six bytes)
  if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) {
    imuDetected = false;
    return false;
  }
  const uint8_t received = Wire.requestFrom(uint8_t(0x29), uint8_t(6));
  if (received != 6 || Wire.getWireTimeoutFlag()) {
    imuDetected = false;
    return false;
  }
  uint8_t data[6];
  for (uint8_t i = 0; i < 6; ++i) {
    data[i] = uint8_t(Wire.read());
  }
  controlPort().print(F("#,"));
  controlPort().print(eulerDegrees(data[0], data[1]), 2);
  controlPort().print(',');
  controlPort().print(eulerDegrees(data[4], data[5]), 2);
  controlPort().print(',');
  controlPort().print(eulerDegrees(data[2], data[3]), 2);
  controlPort().println(F(",#"));
  return true;
}

void sendStatusFrame() {
  controlPort().print(F("#,STAT,"));
  controlPort().print(estopLatched ? 1 : 0);
  controlPort().print(',');
  controlPort().print(imuDetected ? 1 : 0);
  for (uint8_t i = 0; i < 4; ++i) {
    controlPort().print(',');
    controlPort().print(wheels[i].speed);
  }
  controlPort().print(',');
  controlPort().print(upperJack.speed);
  controlPort().print(',');
  controlPort().print(lowerJack.speed);
  controlPort().print(',');
  controlPort().print(robotServo.attached() ? 1 : 0);
  controlPort().print(',');
  controlPort().print(currentServoAngle);
  controlPort().println(F(",#"));
}

bool parseNumber(const char *text, int minimum, int maximum, int &value) {
  const char *digit = text;
  if (*digit == '-' || *digit == '+') ++digit;
  if (*digit == '\0') return false;
  long result = 0;
  while (*digit) {
    if (*digit < '0' || *digit > '9') return false;
    result = result * 10 + (*digit++ - '0');
    if (result > 32767) return false;
  }
  if (*text == '-') result = -result;
  if (result < minimum || result > maximum) return false;
  value = int(result);
  return true;
}

void handleCommand(char *line) {
  const size_t length = strlen(line);
  if (length < 5 || line[0] != '#' || line[1] != ',' ||
      line[length - 2] != ',' || line[length - 1] != '#') {
    replyError(F("BAD_FRAME"));
    return;
  }
  line[length - 2] = '\0';

  char *fields[6];
  uint8_t count = 1;
  fields[0] = line + 2;
  for (char *p = fields[0]; *p; ++p) {
    if (*p == ',') {
      if (count == 6) {
        replyError(F("BAD_ARGUMENTS"));
        return;
      }
      *p = '\0';
      fields[count++] = p + 1;
    }
  }

  // Always allow STOP, ESTOP, CLEAR_ESTOP, PING, IMU, STATUS even during E-STOP
  if (strcmp(fields[0], "STOP") == 0 && count == 1) {
    stopAll();
    controlPort().println(F("#,OK,#"));
    return;
  }
  if (strcmp(fields[0], "ESTOP") == 0 && count == 1) {
    stopAll();
    estopLatched = true;
    controlPort().println(F("#,OK,#"));
    return;
  }
  if (strcmp(fields[0], "CLEAR_ESTOP") == 0 && count == 1) {
    stopAll();
    estopLatched = false;
    upperJack.limitTripped = false;
    lowerJack.limitTripped = false;
    controlPort().println(F("#,OK,#"));
    return;
  }
  if (strcmp(fields[0], "PING") == 0 && count == 1) {
    controlPort().println(F("#,PONG,#"));
    return;
  }
  if (strcmp(fields[0], "IMU") == 0 && count == 1) {
    if (!sendIMU()) {
      replyError(F("IMU_OFFLINE"));
    }
    return;
  }
  if (strcmp(fields[0], "STATUS") == 0 && count == 1) {
    sendStatusFrame();
    controlPort().println(F("#,OK,#"));
    return;
  }

  // Block all motion and configuration commands if E-STOP is latched
  if (estopLatched) {
    stopAll();
    replyError(F("ESTOP_ACTIVE"));
    return;
  }

  int values[4];
  if (strcmp(fields[0], "DRIVE") == 0 && count == 5) {
    for (uint8_t i = 0; i < 4; ++i) {
      if (!parseNumber(fields[i + 1], -255, 255, values[i])) {
        replyError(F("BAD_SPEED"));
        return;
      }
    }
    stopJacks();
    for (uint8_t i = 0; i < 4; ++i) {
      setMotor(wheels[i], values[i]);
    }
  } else if (strcmp(fields[0], "MOTOR") == 0 && count == 3) {
    if (!parseNumber(fields[1], 1, 4, values[0]) ||
        !parseNumber(fields[2], -255, 255, values[1])) {
      replyError(F("BAD_MOTOR_OR_SPEED"));
      return;
    }
    stopJacks();
    setMotor(wheels[values[0] - 1], values[1]);
  } else if (strcmp(fields[0], "ACT") == 0 && count == 3) {
    if (!parseNumber(fields[1], -255, 255, values[0]) ||
        !parseNumber(fields[2], -255, 255, values[1])) {
      replyError(F("BAD_SPEED"));
      return;
    }
    if (prohibitSimultaneousJacks && values[0] != 0 && values[1] != 0) {
      stopAll();
      replyError(F("DUAL_JACK_LOCKED"));
      return;
    }
    stopWheels();
    setMotor(upperJack, values[0]);
    setMotor(lowerJack, values[1]);
  } else if (strcmp(fields[0], "JACK") == 0 && count == 3) {
    const bool upper = strcmp(fields[1], "UPPER") == 0;
    const bool lower = strcmp(fields[1], "LOWER") == 0;
    if ((!upper && !lower) || !parseNumber(fields[2], -255, 255, values[0])) {
      replyError(F("BAD_JACK_OR_SPEED"));
      return;
    }
    stopWheels();
    if (prohibitSimultaneousJacks && values[0] != 0) {
      if (upper) setMotor(lowerJack, 0);
      if (lower) setMotor(upperJack, 0);
    }
    setMotor(upper ? upperJack : lowerJack, values[0]);
  } else if (strcmp(fields[0], "SERVO") == 0 && count == 2) {
    if (!parseNumber(fields[1], servoMinAngle, servoMaxAngle, values[0])) {
      replyError(F("BAD_ANGLE"));
      return;
    }
    if (!robotServo.attached()) {
      robotServo.write(values[0]);
      robotServo.attach(SERVO_PIN);
    }
    robotServo.write(values[0]);
    currentServoAngle = values[0];
  } else if (strcmp(fields[0], "SERVO_DETACH") == 0 && count == 1) {
    if (robotServo.attached()) {
      robotServo.detach();
    }
  } else if (strcmp(fields[0], "SERVO_LIMITS") == 0 && count == 3) {
    if (!parseNumber(fields[1], HARD_SERVO_MIN_ANGLE, HARD_SERVO_MAX_ANGLE, values[0]) ||
        !parseNumber(fields[2], HARD_SERVO_MIN_ANGLE, HARD_SERVO_MAX_ANGLE, values[1]) ||
        values[0] >= values[1]) {
      replyError(F("BAD_LIMITS"));
      return;
    }
    servoMinAngle = values[0];
    servoMaxAngle = values[1];
  } else if (strcmp(fields[0], "WDT") == 0 && count == 3) {
    if (!parseNumber(fields[1], 100, 5000, values[0]) ||
        !parseNumber(fields[2], 200, 10000, values[1])) {
      replyError(F("BAD_WDT"));
      return;
    }
    wheelTimeoutMs = (unsigned long)values[0];
    jackCmdTimeoutMs = (unsigned long)values[1];
  } else if (strcmp(fields[0], "DUAL_JACK") == 0 && count == 2) {
    if (!parseNumber(fields[1], 0, 1, values[0])) {
      replyError(F("BAD_ARGUMENTS"));
      return;
    }
    prohibitSimultaneousJacks = (values[0] == 0);
  } else {
    replyError(F("BAD_COMMAND_OR_ARGUMENTS"));
    return;
  }
  controlPort().println(F("#,OK,#"));
}

void readCommands() {
  for (uint8_t budget = 0; budget < 64 && controlPort().available(); ++budget) {
    const char c = char(controlPort().read());
    if (c == '\n') {
      if (discardLine) {
        replyError(F("BAD_LINE"));
      } else if (rxLength != 0) {
        rxBuffer[rxLength] = '\0';
        handleCommand(rxBuffer);
      }
      rxLength = 0;
      discardLine = false;
    } else if (c == '\r') {
      // Accept CRLF as well as LF.
    } else if (!discardLine) {
      if (c < 32 || c > 126 || rxLength >= RX_CAPACITY - 1) {
        discardLine = true;
      } else {
        rxBuffer[rxLength++] = c;
      }
    }
  }
}

void setup() {
  // Ensure all motor and actuator bridges are strictly OFF before any peripheral init
  for (uint8_t i = 0; i < 4; ++i) {
    initMotor(wheels[i]);
  }
  initMotor(upperJack);
  initMotor(lowerJack);

  controlPort().begin(BAUD);
  configureI2C();

  // Non-blocking IMU initialization: if BNO055 fails, report error but do NOT hang!
  imuDetected = bno.begin(OPERATION_MODE_NDOF);
  if (!imuDetected) {
    replyError(F("IMU_NOT_DETECTED"));
  } else {
    delay(200);
  }
  lastSampleMs = millis();
  lastImuRetryMs = millis();
  controlPort().println(F("#,READY,#"));
}

void loop() {
  checkMotionTimeouts();
  readCommands();
  checkMotionTimeouts();

  const unsigned long now = millis();
  if ((unsigned long)(now - lastSampleMs) >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    if (imuDetected) {
      sendIMU();
    } else if (!anyMotorActive() && (unsigned long)(now - lastImuRetryMs) >= IMU_RETRY_INTERVAL_MS) {
      lastImuRetryMs = now;
      configureI2C();
      if (bno.begin(OPERATION_MODE_NDOF)) {
        imuDetected = true;
      }
    }
  }
}

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
constexpr unsigned long UPPER_JACK_TIMEOUT_MS = 10000;
constexpr unsigned long LOWER_JACK_TIMEOUT_MS = 5000;
constexpr uint8_t SERVO_PIN = 8;
constexpr int SERVO_MIN_ANGLE = 0;
constexpr int SERVO_MAX_ANGLE = 180;
constexpr size_t RX_CAPACITY = 96;

struct MotorChannel {
  uint8_t pwm;
  uint8_t inA;
  uint8_t inB;
  int speed;
  unsigned long lastCommandMs;
};

// Wheel order in DRIVE is motor1, motor2, motor3, motor4.
MotorChannel wheels[] = {
  {4, 26, 27, 0, 0}, // Motor 1: position not yet confirmed
  {5, 28, 29, 0, 0}, // Motor 2: right rear
  {6, 44, 45, 0, 0}, // Motor 3: left front
  {7, 46, 47, 0, 0}  // Motor 4: right front
};
MotorChannel lowerJack = {2, 22, 23, 0, 0};
MotorChannel upperJack = {3, 24, 25, 0, 0};

Adafruit_BNO055 bno(55, 0x29, &Wire);
Servo robotServo;
char rxBuffer[RX_CAPACITY];
size_t rxLength = 0;
bool discardLine = false;
unsigned long lastSampleMs = 0;

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
  // Disable output before changing direction. Zero disables the bridge.
  analogWrite(motor.pwm, 0);
  digitalWrite(motor.inA, speed > 0 ? HIGH : LOW);
  digitalWrite(motor.inB, speed < 0 ? HIGH : LOW);
  analogWrite(motor.pwm, speed < 0 ? -speed : speed);
  motor.speed = speed;
  motor.lastCommandMs = millis();
}

void initMotor(MotorChannel &motor) {
  pinMode(motor.pwm, OUTPUT);
  pinMode(motor.inA, OUTPUT);
  pinMode(motor.inB, OUTPUT);
  setMotor(motor, 0);
}

void stopWheels() {
  for (uint8_t i = 0; i < 4; ++i) setMotor(wheels[i], 0);
}

void stopJacks() {
  setMotor(upperJack, 0);
  setMotor(lowerJack, 0);
}

void stopAll() {
  stopWheels();
  stopJacks();
}

bool checkTimeout(MotorChannel &motor, unsigned long now, unsigned long timeoutMs) {
  if (motor.speed != 0 && now - motor.lastCommandMs >= timeoutMs) {
    setMotor(motor, 0);
    return true;
  }
  return false;
}

void checkMotionTimeouts() {
  const unsigned long now = millis();
  bool stopped = false;
  if (checkTimeout(upperJack, now, UPPER_JACK_TIMEOUT_MS)) stopped = true;
  if (checkTimeout(lowerJack, now, LOWER_JACK_TIMEOUT_MS)) stopped = true;
  if (stopped) controlPort().println(F("#,TIMEOUT,#"));
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
  // Drop telemetry rather than wait for space in the serial transmit buffer.
  // A complete Euler frame fits within 40 bytes, including CRLF.
  if (controlPort().availableForWrite() < 40) return false;
  // No calibration wait, retry loop or fault latch. Failed samples are skipped.
  Wire.clearWireTimeoutFlag();
  // Check the transfer itself: the library's getVector does not expose read
  // failure, which can otherwise appear as a valid all-zero orientation.
  Wire.beginTransmission(0x29);
  Wire.write(uint8_t(0x1A)); // Euler heading, roll, pitch (six bytes)
  if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) {
    return false;
  }
  const uint8_t received = Wire.requestFrom(uint8_t(0x29), uint8_t(6));
  if (received != 6 || Wire.getWireTimeoutFlag()) {
    return false;
  }
  uint8_t data[6];
  for (uint8_t i = 0; i < 6; ++i) data[i] = uint8_t(Wire.read());
  controlPort().print(F("#,"));
  controlPort().print(eulerDegrees(data[0], data[1]), 2);
  controlPort().print(',');
  controlPort().print(eulerDegrees(data[4], data[5]), 2);
  controlPort().print(',');
  controlPort().print(eulerDegrees(data[2], data[3]), 2);
  controlPort().println(F(",#"));
  return true;
}

bool parseNumber(const char *text, int minimum, int maximum, int &value) {
  // Strict integers only: no empty fields, spaces, decimals or trailing junk.
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
  // Split without skipping empty fields, so malformed commands are rejected.
  char *fields[5];
  uint8_t count = 1;
  fields[0] = line + 2;
  for (char *p = fields[0]; *p; ++p) {
    if (*p == ',') {
      if (count == 5) {
        replyError(F("BAD_ARGUMENTS"));
        return;
      }
      *p = '\0';
      fields[count++] = p + 1;
    }
  }

  int values[4];
  if (strcmp(fields[0], "STOP") == 0 && count == 1) {
    stopAll(); // Servo keeps its commanded position.
  } else if (strcmp(fields[0], "DRIVE") == 0 && count == 5) {
    for (uint8_t i = 0; i < 4; ++i) {
      if (!parseNumber(fields[i + 1], -255, 255, values[i])) {
        replyError(F("BAD_SPEED"));
        return;
      }
    }
    stopJacks();
    for (uint8_t i = 0; i < 4; ++i) setMotor(wheels[i], values[i]);
  } else if (strcmp(fields[0], "MOTOR") == 0 && count == 3) {
    if (!parseNumber(fields[1], 1, 4, values[0]) ||
        !parseNumber(fields[2], -255, 255, values[1])) {
      replyError(F("BAD_MOTOR_OR_SPEED"));
      return;
    }
    stopJacks(); // Change only the selected wheel; preserve the other wheels.
    setMotor(wheels[values[0] - 1], values[1]);
  } else if (strcmp(fields[0], "ACT") == 0 && count == 3) {
    // ACT order is upper, lower.
    if (!parseNumber(fields[1], -255, 255, values[0]) ||
        !parseNumber(fields[2], -255, 255, values[1])) {
      replyError(F("BAD_SPEED"));
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
    stopWheels(); // Preserve the other jack's speed and existing timer.
    setMotor(upper ? upperJack : lowerJack, values[0]);
  } else if (strcmp(fields[0], "SERVO") == 0 && count == 2) {
    if (!parseNumber(fields[1], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, values[0])) {
      replyError(F("BAD_ANGLE"));
      return;
    }
    // Attach on the first command to avoid moving the servo during startup.
    if (!robotServo.attached()) {
      robotServo.write(values[0]);
      robotServo.attach(SERVO_PIN);
    }
    robotServo.write(values[0]);
  } else if (strcmp(fields[0], "IMU") == 0 && count == 1) {
    sendIMU();
    // A failed sample is silent, including an explicitly requested sample.
    return;
  } else if (strcmp(fields[0], "PING") == 0 && count == 1) {
    controlPort().println(F("#,PONG,#"));
    return;
  } else {
    replyError(F("BAD_COMMAND_OR_ARGUMENTS"));
    return;
  }
  controlPort().println(F("#,OK,#"));
}

void readCommands() {
  // Limit work per loop so incoming traffic cannot starve motion timeouts.
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
  // All bridges disabled before IMU initialization.
  for (uint8_t i = 0; i < 4; ++i) initMotor(wheels[i]);
  initMotor(upperJack);
  initMotor(lowerJack);
  controlPort().begin(BAUD);
  // AVR Wire otherwise permits unbounded waits on a stuck I2C bus.
  // Requires a current Arduino AVR Boards package.
  configureI2C();
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    replyError(F("IMU_NOT_DETECTED"));
    while (true) delay(1000);
  }
  delay(1000);
  controlPort().println(F("#,READY,#"));
}

void loop() {
  checkMotionTimeouts();
  readCommands();
  checkMotionTimeouts();
  const unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    sendIMU();
  }
}

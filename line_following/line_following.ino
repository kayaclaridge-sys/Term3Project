#include <Wire.h>
#include <Motoron.h>
#include <kvstore_global_api.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ctype.h>
#include <string.h>

// -------------------- WiFi UDP kill switch --------------------
// Replace these with the WiFi details used during the trial run.
const char WIFI_SSID[] = "YOUR_WIFI_SSID";
const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

const unsigned int UDP_KILL_PORT = 4210;
const char STOP_SEQUENCE[] = "Stop";
const char GO_SEQUENCE[] = "Go";

// Connect your top red LED to this pin, or leave LED_BUILTIN for bench testing.
const int STOP_LED_PIN = LED_BUILTIN;
const int STOP_LED_ON = HIGH;
const int STOP_LED_OFF = LOW;

const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
const unsigned long STOP_LED_BLINK_MS = 250;

WiFiUDP killUdp;
char udpPacketBuffer[64];
unsigned long lastWifiAttemptMs = 0;
unsigned long lastStopLedToggleMs = 0;
bool udpServerStarted = false;
bool stopLedState = false;

// -------------------- IR sensor array --------------------
const int SensorCount = 9;
const int sensorPins[SensorCount] = {2, 3, 4, 5, 26, 27, 8, 9, 10};
const int sensorCtrlPin = 12;
const unsigned int sensorTimeout = 2500;

uint16_t minValues[SensorCount];
uint16_t maxValues[SensorCount];
uint16_t lastPosition = 4000;

// -------------------- Motoron setup --------------------
// Hardcoded addresses on Wire1, matching the working tank control sketch.
MotoronI2C mc1; // Shield 0x10 — LEFT side  (M1 = front-left, M2 = rear-left)
MotoronI2C mc2; // Shield 0x11 — RIGHT side (M1 = front-right, M2 = rear-right)

// Channel layout
const uint8_t FRONT_LEFT_MOTOR  = 1;
const uint8_t REAR_LEFT_MOTOR   = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR  = 2;

// Per the working sketch, the LEFT side is wired such that NEGATIVE = forward.
// The RIGHT side is normal (POSITIVE = forward).
const int LEFT_MOTOR_SIGN  = -1;
const int RIGHT_MOTOR_SIGN =  1;

// Motoron speed range is roughly -800 to 800.
const int baseSpeed   = 600;
const int maxSpeed    = 800;
const int searchSpeed = 500;

// Line-following gains.
const float Kp = 0.115;
const float Kd = 0.85;

const int linePresentMin = 250;
const int intersectionSensorThreshold = 700;
const int intersectionSensorCount = 6;
const unsigned long lostLineStopMs = 900;

int lastError = 0;
unsigned long lastLineSeenMs = 0;
bool robotEnabled = true;

bool wifiCredentialsConfigured() {
  return strlen(WIFI_SSID) > 0 && strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0;
}

bool equalsIgnoreCase(const char *left, const char *right) {
  while (*left != '\0' && *right != '\0') {
    if (tolower((unsigned char)*left) != tolower((unsigned char)*right)) {
      return false;
    }
    left++;
    right++;
  }

  return *left == '\0' && *right == '\0';
}

void trimPacket(char *message) {
  char *start = message;

  while (*start != '\0' && isspace((unsigned char)*start)) {
    start++;
  }

  if (start != message) {
    memmove(message, start, strlen(start) + 1);
  }

  char *end = message + strlen(message);
  while (end > message && isspace((unsigned char)*(end - 1))) {
    end--;
  }

  *end = '\0';
}

void setupStopLed() {
  pinMode(STOP_LED_PIN, OUTPUT);
  digitalWrite(STOP_LED_PIN, STOP_LED_OFF);
}

void updateStopLed() {
  if (robotEnabled) {
    if (stopLedState) {
      stopLedState = false;
      digitalWrite(STOP_LED_PIN, STOP_LED_OFF);
    }
    return;
  }

  if (millis() - lastStopLedToggleMs >= STOP_LED_BLINK_MS) {
    lastStopLedToggleMs = millis();
    stopLedState = !stopLedState;
    digitalWrite(STOP_LED_PIN, stopLedState ? STOP_LED_ON : STOP_LED_OFF);
  }
}

void sendUdpReply(const char *message) {
  killUdp.beginPacket(killUdp.remoteIP(), killUdp.remotePort());
  killUdp.print(message);
  killUdp.endPacket();
}

void connectToWifiIfNeeded() {
  if (!wifiCredentialsConfigured()) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!udpServerStarted) {
      killUdp.begin(UDP_KILL_PORT);
      udpServerStarted = true;
      Serial.print("UDP kill switch listening on ");
      Serial.print(WiFi.localIP());
      Serial.print(":");
      Serial.println(UDP_KILL_PORT);
    }
    return;
  }

  udpServerStarted = false;

  if (lastWifiAttemptMs != 0 && millis() - lastWifiAttemptMs < WIFI_RETRY_INTERVAL_MS) {
    return;
  }

  lastWifiAttemptMs = millis();
  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(WIFI_SSID);

  int status;
  if (strlen(WIFI_PASSWORD) == 0) {
    status = WiFi.begin(WIFI_SSID);
  } else {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  if (status != WL_CONNECTED) {
    Serial.println("WiFi not connected yet. Will retry.");
  }
}

void setupWifiKillSwitch() {
  setupStopLed();

  if (!wifiCredentialsConfigured()) {
    Serial.println("WiFi kill switch not configured. Set WIFI_SSID and WIFI_PASSWORD.");
    return;
  }

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not detected.");
    return;
  }

  connectToWifiIfNeeded();
}

void handleWifiKillSwitch() {
  connectToWifiIfNeeded();

  if (WiFi.status() != WL_CONNECTED || !udpServerStarted) {
    return;
  }

  int packetSize = killUdp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  int length = killUdp.read(udpPacketBuffer, sizeof(udpPacketBuffer) - 1);
  if (length < 0) {
    length = 0;
  }
  udpPacketBuffer[length] = '\0';
  trimPacket(udpPacketBuffer);

  Serial.print("UDP command from ");
  Serial.print(killUdp.remoteIP());
  Serial.print(":");
  Serial.print(killUdp.remotePort());
  Serial.print(" -> ");
  Serial.println(udpPacketBuffer);

  if (equalsIgnoreCase(udpPacketBuffer, STOP_SEQUENCE)) {
    robotEnabled = false;
    stopDrive();
    sendUdpReply("STOPPED");
    Serial.println("WiFi kill switch activated. Robot stopped.");
  } else if (equalsIgnoreCase(udpPacketBuffer, GO_SEQUENCE)) {
    robotEnabled = true;
    lastLineSeenMs = millis();
    sendUdpReply("ENABLED");
    Serial.println("Robot enabled by UDP command.");
  } else {
    sendUdpReply("IGNORED");
  }
}

// -------------------- Front encoder setup --------------------
const bool ENABLE_ENCODERS = false;

const int frontLeftEncoderA = 22;
const int frontLeftEncoderB = 23;
const int frontRightEncoderA = 24;
const int frontRightEncoderB = 25;

const int FRONT_LEFT_ENCODER_SIGN  = 1;
const int FRONT_RIGHT_ENCODER_SIGN = 1;

volatile long frontLeftTicks = 0;
volatile long frontRightTicks = 0;

void readFrontLeftEncoder() {
  if (digitalRead(frontLeftEncoderB) == HIGH) {
    frontLeftTicks += FRONT_LEFT_ENCODER_SIGN;
  } else {
    frontLeftTicks -= FRONT_LEFT_ENCODER_SIGN;
  }
}

void readFrontRightEncoder() {
  if (digitalRead(frontRightEncoderB) == HIGH) {
    frontRightTicks += FRONT_RIGHT_ENCODER_SIGN;
  } else {
    frontRightTicks -= FRONT_RIGHT_ENCODER_SIGN;
  }
}

// -------------------- Sensor calibration storage --------------------
unsigned int readSensorPrivate(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);

  unsigned long start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > sensorTimeout) return sensorTimeout;
  }

  return micros() - start;
}

void saveCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    char key[20];
    sprintf(key, "/kv/min%d", i);
    kv_set(key, &minValues[i], sizeof(uint16_t), 0);
    sprintf(key, "/kv/max%d", i);
    kv_set(key, &maxValues[i], sizeof(uint16_t), 0);
  }
  Serial.println("Calibration saved.");
}

bool loadCalibration() {
  size_t actualSize;
  for (int i = 0; i < SensorCount; i++) {
    char key[20];
    sprintf(key, "/kv/min%d", i);
    if (kv_get(key, &minValues[i], sizeof(uint16_t), &actualSize) != 0) {
      return false;
    }
    sprintf(key, "/kv/max%d", i);
    if (kv_get(key, &maxValues[i], sizeof(uint16_t), &actualSize) != 0) {
      return false;
    }
  }
  Serial.println("Calibration loaded.");
  return true;
}

int calibratedSensorValue(unsigned int rawValue, int sensorIndex) {
  long range = (long)maxValues[sensorIndex] - (long)minValues[sensorIndex];
  if (range < 2) return 0;
  long calibratedValue = ((long)rawValue - (long)minValues[sensorIndex]) * 1000L / range;
  return constrain((int)calibratedValue, 0, 1000);
}

void runCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = sensorTimeout;
    maxValues[i] = 0;
  }

  Serial.println("--- CALIBRATION STARTING (10 SECONDS) ---");
  Serial.println("Move the sensor bar over both black tape and pale floor.");

  for (int j = 0; j < 400; j++) {
    for (int i = 0; i < SensorCount; i++) {
      unsigned int value = readSensorPrivate(sensorPins[i]);
      if (value < minValues[i]) minValues[i] = value;
      if (value > maxValues[i]) maxValues[i] = value;
    }
    if (j % 40 == 0) Serial.println("Still calibrating...");
    delay(25);
  }

  saveCalibration();
  Serial.println("--- CALIBRATION COMPLETE ---");
}

int readLinePosition(int calibratedValues[SensorCount], long &sumOut, int &blackCountOut) {
  long avg = 0;
  long sum = 0;
  int blackCount = 0;

  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawValue = readSensorPrivate(sensorPins[i]);
    int calibratedValue = calibratedSensorValue(rawValue, i);
    calibratedValues[i] = calibratedValue;

    avg += (long)calibratedValue * (i * 1000);
    sum += calibratedValue;

    if (calibratedValue > intersectionSensorThreshold) {
      blackCount++;
    }
  }

  sumOut = sum;
  blackCountOut = blackCount;

  if (sum > linePresentMin) {
    lastPosition = avg / sum;
    lastLineSeenMs = millis();
  } else if (lastPosition < ((SensorCount - 1) * 1000 / 2)) {
    lastPosition = 0;
  } else {
    lastPosition = (SensorCount - 1) * 1000;
  }

  return lastPosition;
}

// -------------------- Motor helpers --------------------
void setupMotoron(MotoronI2C *controller, const char *name) {
  controller->reinitialize();
  delay(20);
  controller->disableCrc();
  controller->clearResetFlag();
  controller->clearMotorFaultUnconditional();
  controller->disableCommandTimeout();

  for (uint8_t channel = 1; channel <= 3; channel++) {
    controller->setMaxAcceleration(channel, 200);
    controller->setMaxDeceleration(channel, 300);
  }

  Serial.print(name);
  Serial.println(" Motoron ready.");
}

// "speed" is in robot-forward terms (positive = forward).
// LEFT_MOTOR_SIGN flips the left side so the wiring matches the working sketch.
void setLeftSideSpeed(int speed) {
  int command = constrain(speed, -maxSpeed, maxSpeed) * LEFT_MOTOR_SIGN;
  mc1.setSpeed(FRONT_LEFT_MOTOR, command);
  mc1.setSpeed(REAR_LEFT_MOTOR, command);
}

void setRightSideSpeed(int speed) {
  int command = constrain(speed, -maxSpeed, maxSpeed) * RIGHT_MOTOR_SIGN;
  mc2.setSpeed(FRONT_RIGHT_MOTOR, command);
  mc2.setSpeed(REAR_RIGHT_MOTOR, command);
}

void setDriveSpeeds(int leftSpeed, int rightSpeed) {
  setLeftSideSpeed(leftSpeed);
  setRightSideSpeed(rightSpeed);
}

void stopDrive() {
  setDriveSpeeds(0, 0);
}

void printDebug(int position, long sum, int blackCount, int leftSpeed, int rightSpeed) {
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();

  long leftTicks;
  long rightTicks;
  noInterrupts();
  leftTicks = frontLeftTicks;
  rightTicks = frontRightTicks;
  interrupts();

  Serial.print("pos=");
  Serial.print(position);
  Serial.print(" sum=");
  Serial.print(sum);
  Serial.print(" black=");
  Serial.print(blackCount);
  Serial.print(" L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.print(rightSpeed);
  Serial.print(" encL=");
  Serial.print(leftTicks);
  Serial.print(" encR=");
  Serial.println(rightTicks);
}

void handleSerialCommands() {
  if (!Serial.available()) return;

  char command = Serial.read();

  if (command == 'c') {
    stopDrive();
    runCalibration();
  } else if (command == 's') {
    robotEnabled = false;
    stopDrive();
    Serial.println("Robot stopped. Send 'g' to go.");
  } else if (command == 'g') {
    robotEnabled = true;
    lastLineSeenMs = millis();
    Serial.println("Robot enabled.");
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000) {}

  Serial.println("Line follower starting...");
  setupWifiKillSwitch();

  Serial.println("Setting up IR control pin...");
  pinMode(sensorCtrlPin, OUTPUT);
  digitalWrite(sensorCtrlPin, HIGH);

  if (ENABLE_ENCODERS) {
    Serial.println("Setting up encoder pins and interrupts...");
    pinMode(frontLeftEncoderA, INPUT_PULLUP);
    pinMode(frontLeftEncoderB, INPUT_PULLUP);
    pinMode(frontRightEncoderA, INPUT_PULLUP);
    pinMode(frontRightEncoderB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(frontLeftEncoderA), readFrontLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(frontRightEncoderA), readFrontRightEncoder, CHANGE);
  } else {
    Serial.println("Encoders disabled for this test.");
  }

  // -------------------- Motoron init (matches working sketch) --------------------
  Serial.println("Initializing Motorons on Wire1...");
  Wire1.begin();
  delay(100);

  // Shield 1 — 0x10 (default address)
  mc1.setBus(&Wire1);
  setupMotoron(&mc1, "Left (0x10)");

  // Shield 2 — 0x11
  mc2.setBus(&Wire1);
  mc2.setAddress(17); // 17 decimal = 0x11
  setupMotoron(&mc2, "Right (0x11)");

  stopDrive();

  Serial.println("Loading IR calibration...");
  if (!loadCalibration()) {
    Serial.println("No saved calibration found.");
    runCalibration();
  } else {
    Serial.println("Send 'c' within 3 seconds to recalibrate.");
    unsigned long start = millis();
    while (millis() - start < 3000) {
      if (Serial.available() && Serial.read() == 'c') {
        runCalibration();
        break;
      }
    }
  }

  lastLineSeenMs = millis();
  Serial.println("Ready. Send 's' to stop, 'g' to go, 'c' to calibrate. UDP 'Stop' also stops.");
}

void loop() {
  handleSerialCommands();
  handleWifiKillSwitch();
  updateStopLed();

  if (!robotEnabled) {
    stopDrive();
    updateStopLed();
    delay(20);
    return;
  }

  int values[SensorCount];
  long sum;
  int blackCount;
  int position = readLinePosition(values, sum, blackCount);

  const int centerPosition = (SensorCount - 1) * 1000 / 2;
  int error = position - centerPosition;
  int derivative = error - lastError;
  lastError = error;

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  if (sum <= linePresentMin) {
    if (millis() - lastLineSeenMs > lostLineStopMs) {
      leftSpeed = 0;
      rightSpeed = 0;
    } else if (error < 0) {
      leftSpeed = -searchSpeed;
      rightSpeed = searchSpeed;
    } else {
      leftSpeed = searchSpeed;
      rightSpeed = -searchSpeed;
    }
  } else if (blackCount >= intersectionSensorCount) {
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
  } else {
    int turn = (int)(Kp * error + Kd * derivative);
    leftSpeed = constrain(baseSpeed + turn, -maxSpeed, maxSpeed);
    rightSpeed = constrain(baseSpeed - turn, -maxSpeed, maxSpeed);
  }

  setDriveSpeeds(leftSpeed, rightSpeed);
  printDebug(position, sum, blackCount, leftSpeed, rightSpeed);
  updateStopLed();

  delay(10);
}

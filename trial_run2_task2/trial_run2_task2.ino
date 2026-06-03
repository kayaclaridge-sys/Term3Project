#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include "secrets.h"

// =====================================================
// Sensor + motor integration test
// QTR/IR line sensor + Motoron drive, no encoders.
// =====================================================

// -------------------- QTR 9-channel RC line sensor --------------------
const uint8_t QTR_SENSOR_COUNT = 9;
const uint8_t QTR_EMITTER_PIN = 31;

// OUT1~OUT9 -> D22~D30, matching full_v1.

const uint8_t qtrPins[QTR_SENSOR_COUNT] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

const uint16_t QTR_TIMEOUT_US = 8000;
const unsigned long QTR_CALIBRATION_MS = 5000;

uint16_t qtrRaw[QTR_SENSOR_COUNT];
uint16_t qtrCalMin[QTR_SENSOR_COUNT];
uint16_t qtrCalMax[QTR_SENSOR_COUNT];
uint16_t qtrCalibrated[QTR_SENSOR_COUNT];

const int CENTER_POSITION = (QTR_SENSOR_COUNT - 1) * 1000 / 2;
uint16_t lastPosition = CENTER_POSITION;
unsigned long lastLineSeenMs = 0;

const long LINE_PRESENT_MIN = 250;
const int BLACK_SENSOR_THRESHOLD = 600;
const unsigned long LOST_LINE_STOP_MS = 650;

// -------------------- Motoron motor controllers --------------------
MotoronI2C leftController;   // 0x10: left side
MotoronI2C rightController;  // 0x11: right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

// These signs match the corrected trial run test: positive speed = forward.
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;

const int BASE_SPEED = 200;
const int CURVE_BASE_SPEED = 200;
const int MAX_SPEED = 600;
const int MAX_TURN = 600;
const int SEARCH_SPEED = 170;

const int TURN_SPEED = 450;
const int TURN_BOOST_SPEED = 550;
const int PRE_TURN_SPEED = 550;
const int BRIDGE_SPEED = 550;

const float KP = 0.12;
const float KD = 0.80;

int lastError = 0;

// -------------------- Buttons and LEDs --------------------
const int START_BUTTON_PIN = 32;
const int STOP_BUTTON_PIN = 53;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

// -------------------- MiniMessenger airlock request --------------------
MiniMessenger messenger;

const char *BOARD_ID = "Robot1";
const char *SERVER_BOARD_ID = "server";
const char *TASK2_AIRLOCK_ID = "A";

const unsigned long REGISTER_SEND_MS = 10000;
const unsigned long AIRLOCK_REQUEST_RETRY_MS = 1200;
const byte TEAM_STATUS_BYTES = 6;

unsigned long lastRegisterSendMs = 0;
unsigned long lastAirlockSendAttemptMs = 0;
String pendingAirlockUid = "";
String lastAirlockRequestedUid = "";

// -------------------- RFID pause marker --------------------
const byte RFID_I2C_ADDRESS = 0x28;
const int RFID_RESET_PIN = -1;
const unsigned long RFID_POLL_MS = 80;
const unsigned long RFID_PAUSE_MS = 250;
const unsigned long RFID_REPEAT_PAUSE_GUARD_MS = 1500;

MFRC522_I2C rfid(RFID_I2C_ADDRESS, RFID_RESET_PIN, &Wire1);

// -------------------- Junction detection tuning --------------------
const uint8_t SPECIAL_DETECT_FRAMES = 3;
const uint8_t RIGHT_ANGLE_DETECT_FRAMES = 3;
const unsigned long PRE_TURN_MS = 115;
const unsigned long PIVOT_KICK_MS = 0;
const unsigned long TURN_BOOST_MS = 500;
const unsigned long TURN_MIN_MS = 300;
const unsigned long TURN_MAX_MS = 6000;
const unsigned long BRIDGE_MIN_MS = 160;
const unsigned long BRIDGE_MAX_MS = 900;
const unsigned long RECOVER_MAX_MS = 900;

struct LineSnapshot {
  int values[QTR_SENSOR_COUNT];
  bool black[QTR_SENSOR_COUNT];
  long sum;
  int position;
  int blackCount;
  int leftCount;
  int midCount;
  int rightCount;
  long leftSum;
  long midSum;
  long rightSum;
  bool linePresent;
  bool centerPresent;
  bool leftWing;
  bool rightWing;
};

enum RobotState {
  STATE_IDLE,
  STATE_FOLLOW_LINE,
  STATE_RFID_PAUSE,
  STATE_PRE_TURN,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_BRIDGE_HOLLOW,
  STATE_RECOVER_LINE
};

enum TurnDirection {
  TURN_NONE,
  TURN_LEFT,
  TURN_RIGHT
};

enum TJunctionDecision {
  T_DECISION_STOP,
  T_DECISION_LEFT,
  T_DECISION_RIGHT
};

RobotState robotState = STATE_IDLE;
TurnDirection pendingTurn = TURN_NONE;
TJunctionDecision nextTJunctionDecision = T_DECISION_STOP;
bool lineOnlyMode = false;
bool task2Mode = false;
bool task2FirstTJunctionHandled = false;
bool rfidReady = false;

bool monitorSensors = false;
unsigned long stateStartedMs = 0;
unsigned long lastMonitorPrintMs = 0;
unsigned long lastRfidPollMs = 0;
unsigned long lastRfidPauseMs = 0;
bool lineLostLatched = false;
bool turnReleaseArmed = false;

String latestRfidUid = "";
String lastPausedRfidUid = "";

uint8_t leftAngleFrames = 0;
uint8_t rightAngleFrames = 0;
uint8_t tJunctionFrames = 0;
uint8_t hollowCrossFrames = 0;

void stopDrive();
void stopRobot();
void printHelp();
void printSnapshot(const LineSnapshot &snapshot);
void enterState(RobotState newState);
void pollRFID();
void sendRegisterIfNeeded();
void sendPendingAirlockRequestIfNeeded();
void queueAirlockRequest(const String &uid);
void onMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length);

// =====================================================
// QTR reading and calibration
// =====================================================

void readQTRRC() {
  digitalWrite(QTR_EMITTER_PIN, HIGH);
  delayMicroseconds(300);

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    pinMode(qtrPins[i], OUTPUT);
    digitalWrite(qtrPins[i], HIGH);
  }

  delayMicroseconds(20);

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    pinMode(qtrPins[i], INPUT);
    qtrRaw[i] = QTR_TIMEOUT_US;
  }

  uint32_t startTime = micros();

  while ((uint32_t)(micros() - startTime) < QTR_TIMEOUT_US) {
    uint16_t elapsed = micros() - startTime;
    bool allDone = true;

    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
      if (qtrRaw[i] == QTR_TIMEOUT_US) {
        if (digitalRead(qtrPins[i]) == LOW) {
          qtrRaw[i] = elapsed;
        } else {
          allDone = false;
        }
      }
    }

    if (allDone) {
      break;
    }
  }
}

void resetQTRCalibration() {
  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    qtrCalMin[i] = QTR_TIMEOUT_US;
    qtrCalMax[i] = 0;
  }
}

void calibrateQTROnce() {
  readQTRRC();

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    if (qtrRaw[i] < qtrCalMin[i]) {
      qtrCalMin[i] = qtrRaw[i];
    }

    if (qtrRaw[i] > qtrCalMax[i]) {
      qtrCalMax[i] = qtrRaw[i];
    }
  }
}

void readQTRCalibrated() {
  readQTRRC();

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    uint16_t range = qtrCalMax[i] - qtrCalMin[i];

    if (range < 20) {
      qtrCalibrated[i] = 0;
      continue;
    }

    int32_t value = ((int32_t)qtrRaw[i] - qtrCalMin[i]) * 1000 / range;

    if (value < 0) {
      value = 0;
    }

    if (value > 1000) {
      value = 1000;
    }

    qtrCalibrated[i] = value;
  }
}

void runQTRCalibration() {
  stopDrive();
  resetQTRCalibration();

  Serial.println("QTR calibration: 5 seconds.");
  Serial.println("Move all 9 sensors across both pale floor and black tape.");

  unsigned long startMs = millis();

  while (millis() - startMs < QTR_CALIBRATION_MS) {
    calibrateQTROnce();

    bool blinkOn = ((millis() / 200) % 2) == 0;
    digitalWrite(LED_RED_PIN, blinkOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, blinkOn ? LOW : HIGH);

    delay(20);
  }

  Serial.println("QTR calibration done.");
}

LineSnapshot readLineSnapshot() {
  LineSnapshot snapshot;
  long weightedSum = 0;
  long sum = 0;

  snapshot.blackCount = 0;
  snapshot.leftCount = 0;
  snapshot.midCount = 0;
  snapshot.rightCount = 0;
  snapshot.leftSum = 0;
  snapshot.midSum = 0;
  snapshot.rightSum = 0;

  readQTRCalibrated();

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    int value = qtrCalibrated[i];
    bool isBlack = value > BLACK_SENSOR_THRESHOLD;

    snapshot.values[i] = value;
    snapshot.black[i] = isBlack;

    weightedSum += (long)value * (i * 1000);
    sum += value;

    if (isBlack) {
      snapshot.blackCount++;
    }

    if (i <= 2) {
      snapshot.leftSum += value;
      if (isBlack) {
        snapshot.leftCount++;
      }
    } else if (i <= 5) {
      snapshot.midSum += value;
      if (isBlack) {
        snapshot.midCount++;
      }
    } else {
      snapshot.rightSum += value;
      if (isBlack) {
        snapshot.rightCount++;
      }
    }
  }

  snapshot.sum = sum;
  snapshot.linePresent = sum > LINE_PRESENT_MIN;
  snapshot.centerPresent = snapshot.black[3] || snapshot.black[4] || snapshot.black[5];
  snapshot.leftWing = snapshot.leftCount >= 2 || snapshot.leftSum > 1300;
  snapshot.rightWing = snapshot.rightCount >= 2 || snapshot.rightSum > 1300;

  if (snapshot.linePresent) {
    snapshot.position = weightedSum / sum;
    lastPosition = snapshot.position;
    lastLineSeenMs = millis();
    lineLostLatched = false;
  } else if (lastPosition < CENTER_POSITION) {
    snapshot.position = 0;
  } else {
    snapshot.position = (QTR_SENSOR_COUNT - 1) * 1000;
  }

  return snapshot;
}

// =====================================================
// Motor helpers
// =====================================================

void setupMotoron(MotoronI2C *controller, const char *name) {
  controller->reinitialize();
  delay(20);
  controller->disableCrc();
  controller->clearResetFlag();
  controller->clearMotorFaultUnconditional();
  controller->disableCommandTimeout();

  for (uint8_t channel = 1; channel <= 3; channel++) {
    controller->setMaxAcceleration(channel, 800);
    controller->setMaxDeceleration(channel, 800);
  }

  Serial.print(name);
  Serial.println(" Motoron ready.");
}

void setLeftSideSpeed(int speed) {
  int command = constrain(speed, -MAX_SPEED, MAX_SPEED) * LEFT_MOTOR_SIGN;
  leftController.setSpeed(FRONT_LEFT_MOTOR, command);
  leftController.setSpeed(REAR_LEFT_MOTOR, command);
}

void setRightSideSpeed(int speed) {
  int command = constrain(speed, -MAX_SPEED, MAX_SPEED) * RIGHT_MOTOR_SIGN;
  rightController.setSpeed(FRONT_RIGHT_MOTOR, command);
  rightController.setSpeed(REAR_RIGHT_MOTOR, command);
}

void setDriveSpeeds(int leftSpeed, int rightSpeed) {
  setLeftSideSpeed(leftSpeed);
  setRightSideSpeed(rightSpeed);
}

void setLeftSideSpeedNow(int speed) {
  int command = constrain(speed, -MAX_SPEED, MAX_SPEED) * LEFT_MOTOR_SIGN;
  leftController.setSpeedNow(FRONT_LEFT_MOTOR, command);
  leftController.setSpeedNow(REAR_LEFT_MOTOR, command);
}

void setRightSideSpeedNow(int speed) {
  int command = constrain(speed, -MAX_SPEED, MAX_SPEED) * RIGHT_MOTOR_SIGN;
  rightController.setSpeedNow(FRONT_RIGHT_MOTOR, command);
  rightController.setSpeedNow(REAR_RIGHT_MOTOR, command);
}

void setDriveSpeedsNow(int leftSpeed, int rightSpeed) {
  setLeftSideSpeedNow(leftSpeed);
  setRightSideSpeedNow(rightSpeed);
}

void stopDrive() {
  setDriveSpeeds(0, 0);
}

// =====================================================
// MiniMessenger helpers
// =====================================================

String payloadToString(const uint8_t *payload, size_t length) {
  String text = "";

  for (size_t i = 0; i < length; i++) {
    text += (char)payload[i];
  }

  text.trim();
  return text;
}

bool payloadLooksLikeTextMessage(const uint8_t *payload, size_t length) {
  return length >= 5 &&
         payload[0] == 't' &&
         payload[1] == 'y' &&
         payload[2] == 'p' &&
         payload[3] == 'e' &&
         payload[4] == '=';
}

String getMessageField(const String &messageText, const char *key) {
  String prefix = String(key) + "=";
  int start = messageText.indexOf(prefix);

  if (start < 0) {
    return "";
  }

  start += prefix.length();
  int end = messageText.indexOf(' ', start);

  if (end < 0) {
    end = messageText.length();
  }

  return messageText.substring(start, end);
}

bool messageTypeIs(const String &messageText, const char *expectedType) {
  return getMessageField(messageText, "type") == expectedType;
}

bool textIsTrue(String text) {
  text.trim();
  text.toLowerCase();
  return text == "true" || text == "1" || text == "yes";
}

bool sendServerText(const char *messageText) {
  if (!messenger.isConnected()) {
    Serial.print("MiniMessenger disconnected, cannot send: ");
    Serial.println(messageText);
    return false;
  }

  if (messenger.sendToBoard(SERVER_BOARD_ID, messageText)) {
    Serial.print("Sent to server: ");
    Serial.println(messageText);
    return true;
  }

  Serial.print("Server send failed: ");
  Serial.println(messageText);
  return false;
}

void sendRegisterIfNeeded() {
  if (!messenger.isConnected()) {
    return;
  }

  if (lastRegisterSendMs != 0 && millis() - lastRegisterSendMs < REGISTER_SEND_MS) {
    return;
  }

  lastRegisterSendMs = millis();

  char message[96];
  snprintf(message,
           sizeof(message),
           "type=register team_id=%s board_id=%s",
           GROUP_ID,
           BOARD_ID);

  sendServerText(message);
}

bool sendOpenAirlockRequest(const String &uid) {
  if (uid.length() == 0) {
    Serial.println("Airlock request skipped: no RFID UID.");
    return false;
  }

  char message[160];
  snprintf(message,
           sizeof(message),
           "type=openAirlock team_id=%s airlock=%s tag_id=%s board_id=%s",
           GROUP_ID,
           TASK2_AIRLOCK_ID,
           uid.c_str(),
           BOARD_ID);

  return sendServerText(message);
}

void queueAirlockRequest(const String &uid) {
  if (uid.length() == 0) {
    return;
  }

  pendingAirlockUid = uid;
  lastAirlockSendAttemptMs = 0;

  Serial.print("Queued airlock request for RFID: ");
  Serial.println(uid);
  sendPendingAirlockRequestIfNeeded();
}

void sendPendingAirlockRequestIfNeeded() {
  if (pendingAirlockUid.length() == 0) {
    return;
  }

  if (lastAirlockSendAttemptMs != 0 &&
      millis() - lastAirlockSendAttemptMs < AIRLOCK_REQUEST_RETRY_MS) {
    return;
  }

  lastAirlockSendAttemptMs = millis();

  if (sendOpenAirlockRequest(pendingAirlockUid)) {
    lastAirlockRequestedUid = pendingAirlockUid;
    pendingAirlockUid = "";
  }
}

void handleOpenAirlockAck(const String &messageText) {
  String success = getMessageField(messageText, "success");
  String accepted = getMessageField(messageText, "accepted");
  String airlock = getMessageField(messageText, "airlock");
  String reason = getMessageField(messageText, "reason");

  Serial.print("Airlock ack | airlock=");
  Serial.print(airlock.length() ? airlock : TASK2_AIRLOCK_ID);
  Serial.print(" success=");
  if (success.length() > 0) {
    Serial.print(success);
  } else if (accepted.length() > 0) {
    Serial.print(accepted);
  } else {
    Serial.print("unknown");
  }

  if (reason.length() > 0) {
    Serial.print(" reason=");
    Serial.print(reason);
  }

  Serial.println();
}

void handleTeamStatusPayload(const uint8_t *payload) {
  Serial.print("Team status | queueExit=");
  Serial.print(payload[0]);
  Serial.print(" airlockB=");
  Serial.print(payload[1]);
  Serial.print(" queueEnter=");
  Serial.print(payload[2]);
  Serial.print(" airlockA=");
  Serial.print(payload[3]);
  Serial.print(" emergency=");
  Serial.print(payload[4]);
  Serial.print(" reEntry=");
  Serial.println(payload[5]);

  if (payload[4] == 1) {
    stopRobot();
    Serial.println("Task 2 stopped by team emergency status.");
  }
}

void handleServerMessage(const String &messageText) {
  if (messageTypeIs(messageText, "openAirlockAck") ||
      messageTypeIs(messageText, "openAirlockReply")) {
    handleOpenAirlockAck(messageText);
    return;
  }

  if (messageTypeIs(messageText, "heartbeat")) {
    String enable = getMessageField(messageText, "enable");
    if (enable.length() > 0 && !textIsTrue(enable)) {
      stopRobot();
      Serial.println("Task 2 stopped by server heartbeat.");
    }
    return;
  }

  if (messageTypeIs(messageText, "emergency") || messageTypeIs(messageText, "disable")) {
    stopRobot();
    Serial.println("Task 2 stopped by server message.");
    return;
  }

  if (messageTypeIs(messageText, "enable")) {
    Serial.println("Server enable received. Task 2 still starts from D32 or serial g.");
  }
}

void onMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length) {
  if (length == TEAM_STATUS_BYTES && !payloadLooksLikeTextMessage(payload, length)) {
    Serial.print("Binary message from Board ");
    Serial.print(metadata.fromBoardId);
    Serial.print(" length=");
    Serial.println(length);
    handleTeamStatusPayload(payload);
    return;
  }

  if (!payloadLooksLikeTextMessage(payload, length)) {
    Serial.print("Binary message ignored from Board ");
    Serial.print(metadata.fromBoardId);
    Serial.print(" length=");
    Serial.println(length);
    return;
  }

  String messageText = payloadToString(payload, length);

  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(messageText);

  if (messageText.length() > 0) {
    handleServerMessage(messageText);
  }
}

// =====================================================
// Detection and state machine
// =====================================================

const char *stateName(RobotState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_FOLLOW_LINE:
      return "FOLLOW_LINE";
    case STATE_RFID_PAUSE:
      return "RFID_PAUSE";
    case STATE_PRE_TURN:
      return "PRE_TURN";
    case STATE_TURN_LEFT:
      return "TURN_LEFT";
    case STATE_TURN_RIGHT:
      return "TURN_RIGHT";
    case STATE_BRIDGE_HOLLOW:
      return "BRIDGE_HOLLOW";
    case STATE_RECOVER_LINE:
      return "RECOVER_LINE";
  }

  return "?";
}

const char *tDecisionName(TJunctionDecision decision) {
  switch (decision) {
    case T_DECISION_STOP:
      return "STOP";
    case T_DECISION_LEFT:
      return "LEFT";
    case T_DECISION_RIGHT:
      return "RIGHT";
  }

  return "?";
}

void enterState(RobotState newState) {
  if (robotState != newState) {
    Serial.print("State: ");
    Serial.print(stateName(robotState));
    Serial.print(" -> ");
    Serial.println(stateName(newState));
  }

  robotState = newState;
  stateStartedMs = millis();

  if (newState == STATE_TURN_LEFT || newState == STATE_TURN_RIGHT) {
    turnReleaseArmed = false;
  }
}

void resetDetectionCounters() {
  leftAngleFrames = 0;
  rightAngleFrames = 0;
  tJunctionFrames = 0;
  hollowCrossFrames = 0;
}

void incrementOrReset(uint8_t &counter, bool condition) {
  if (condition) {
    if (counter < 255) {
      counter++;
    }
  } else {
    counter = 0;
  }
}

bool isI2cDevicePresent(byte address) {
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

String normalizeUid(String uid) {
  uid.trim();
  uid.toUpperCase();
  uid.replace(":", "");
  uid.replace("-", "");
  uid.replace(" ", "");
  return uid;
}

String readRfidUid() {
  String uid = "";

  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      uid += "0";
    }

    uid += String(rfid.uid.uidByte[i], HEX);
  }

  return normalizeUid(uid);
}

void startRfidPause(const String &uid) {
  stopDrive();
  resetDetectionCounters();

  Serial.print("RFID pause: ");
  Serial.println(uid);

  enterState(STATE_RFID_PAUSE);
}

void pollRFID() {
  if (!rfidReady || millis() - lastRfidPollMs < RFID_POLL_MS) {
    return;
  }

  lastRfidPollMs = millis();

  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println("RFID card detected, but UID could not be read.");
    return;
  }

  String uid = readRfidUid();
  latestRfidUid = uid;

  Serial.print("RFID detected: ");
  Serial.println(uid);

  unsigned long nowMs = millis();
  bool sameUidTooSoon = uid == lastPausedRfidUid && nowMs - lastRfidPauseMs < RFID_REPEAT_PAUSE_GUARD_MS;

  if (task2Mode && robotState == STATE_FOLLOW_LINE && !lineOnlyMode && !sameUidTooSoon) {
    lastPausedRfidUid = uid;
    lastRfidPauseMs = nowMs;
    queueAirlockRequest(uid);
    startRfidPause(uid);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

bool isLeftRightAngleBend(const LineSnapshot &snapshot, bool tJunctionCandidate, bool hollowCrossCandidate) {
  return !tJunctionCandidate &&
         !hollowCrossCandidate &&
         snapshot.leftCount == 3 &&
         snapshot.rightCount <= 1 &&
         snapshot.midCount >= 1 &&
         snapshot.position < 4300 &&
         snapshot.blackCount >= 5 &&
         snapshot.blackCount < QTR_SENSOR_COUNT;
}

bool isRightRightAngleBend(const LineSnapshot &snapshot, bool tJunctionCandidate, bool hollowCrossCandidate) {
  return !tJunctionCandidate &&
         !hollowCrossCandidate &&
         snapshot.rightCount == 3 &&
         snapshot.leftCount <= 1 &&
         snapshot.midCount >= 1 &&
         snapshot.position > 3700 &&
         snapshot.blackCount >= 5 &&
         snapshot.blackCount < QTR_SENSOR_COUNT;
}

void updateDetectionCounters(const LineSnapshot &snapshot) {
  bool hollowCrossCandidate =
    snapshot.leftWing &&
    snapshot.rightWing &&
    snapshot.midCount <= 1 &&
    snapshot.blackCount >= 2;

  bool leftAngleCandidate = isLeftRightAngleBend(snapshot, false, hollowCrossCandidate);
  bool rightAngleCandidate = isRightRightAngleBend(snapshot, false, hollowCrossCandidate);

  bool tJunctionCandidate =
    !hollowCrossCandidate &&
    !leftAngleCandidate &&
    !rightAngleCandidate &&
    snapshot.blackCount == QTR_SENSOR_COUNT;

  incrementOrReset(hollowCrossFrames, hollowCrossCandidate);
  incrementOrReset(tJunctionFrames, tJunctionCandidate);
  incrementOrReset(leftAngleFrames, leftAngleCandidate);
  incrementOrReset(rightAngleFrames, rightAngleCandidate);
}

void startRobot() {
  lineOnlyMode = false;
  task2Mode = true;
  task2FirstTJunctionHandled = false;
  lastPausedRfidUid = "";
  lastRfidPauseMs = 0;
  resetDetectionCounters();
  lastError = 0;
  lineLostLatched = false;
  lastLineSeenMs = millis();
  enterState(STATE_FOLLOW_LINE);
  Serial.println("Trial Run 2 Task 2 started. First T junction will turn left.");
}

void startLineOnlyRobot() {
  lineOnlyMode = true;
  task2Mode = false;
  task2FirstTJunctionHandled = false;
  lastPausedRfidUid = "";
  lastRfidPauseMs = 0;
  resetDetectionCounters();
  lastError = 0;
  lineLostLatched = false;
  lastLineSeenMs = millis();
  enterState(STATE_FOLLOW_LINE);
  Serial.println("Robot started in line-only mode.");
}

void startTask2Robot() {
  startRobot();
}

void stopRobot() {
  stopDrive();
  enterState(STATE_IDLE);
  Serial.println("Robot stopped.");
}

void handleTurnDecision(const char *sourceName) {
  Serial.print(sourceName);
  Serial.print(" detected. Decision: ");

  if (task2Mode && !task2FirstTJunctionHandled) {
    task2FirstTJunctionHandled = true;
    Serial.println("TASK2_FORCE_LEFT");
    pendingTurn = TURN_LEFT;
    enterState(STATE_PRE_TURN);
    return;
  }

  Serial.println(tDecisionName(nextTJunctionDecision));

  if (nextTJunctionDecision == T_DECISION_LEFT) {
    pendingTurn = TURN_LEFT;
    enterState(STATE_PRE_TURN);
  } else if (nextTJunctionDecision == T_DECISION_RIGHT) {
    pendingTurn = TURN_RIGHT;
    enterState(STATE_PRE_TURN);
  } else {
    stopRobot();
  }
}

void followLine(const LineSnapshot &snapshot) {
  if (!lineOnlyMode) {
    updateDetectionCounters(snapshot);

    if (hollowCrossFrames >= SPECIAL_DETECT_FRAMES) {
      resetDetectionCounters();
      Serial.println("Hollow cross / center gap detected.");
      enterState(STATE_BRIDGE_HOLLOW);
      return;
    }

    if (leftAngleFrames >= RIGHT_ANGLE_DETECT_FRAMES) {
      resetDetectionCounters();
      Serial.println("Left right-angle bend detected.");
      pendingTurn = TURN_LEFT;
      enterState(STATE_PRE_TURN);
      return;
    }

    if (rightAngleFrames >= RIGHT_ANGLE_DETECT_FRAMES) {
      resetDetectionCounters();
      Serial.println("Right right-angle bend detected.");
      pendingTurn = TURN_RIGHT;
      enterState(STATE_PRE_TURN);
      return;
    }

    if (tJunctionFrames >= SPECIAL_DETECT_FRAMES) {
      resetDetectionCounters();
      handleTurnDecision("T junction");
      return;
    }
  } else if (leftAngleFrames != 0 || rightAngleFrames != 0 || tJunctionFrames != 0 || hollowCrossFrames != 0) {
    resetDetectionCounters();
  }

  int error = snapshot.position - CENTER_POSITION;
  int derivative = error - lastError;
  lastError = error;

  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (!snapshot.linePresent) {
    if (millis() - lastLineSeenMs > LOST_LINE_STOP_MS) {
      if (!lineLostLatched) {
        Serial.println("Line lost. Recovering.");
        lineLostLatched = true;
      }

      enterState(STATE_RECOVER_LINE);
      return;
    }

    if (error < 0) {
      leftSpeed = -SEARCH_SPEED;
      rightSpeed = SEARCH_SPEED;
    } else {
      leftSpeed = SEARCH_SPEED;
      rightSpeed = -SEARCH_SPEED;
    }
  } else {
    int base = abs(error) > 2200 ? CURVE_BASE_SPEED : BASE_SPEED;
    int turn = (int)(KP * error + KD * derivative);
    turn = constrain(turn, -MAX_TURN, MAX_TURN);

    leftSpeed = constrain(base + turn, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(base - turn, -MAX_SPEED, MAX_SPEED);
  }

  setDriveSpeeds(leftSpeed, rightSpeed);
}

bool centerLineFound(const LineSnapshot &snapshot) {
  return snapshot.centerPresent &&
         snapshot.position > 2600 &&
         snapshot.position < 5400;
}

void updateStateMachine() {
  LineSnapshot snapshot = readLineSnapshot();
  unsigned long elapsed = millis() - stateStartedMs;

  if (monitorSensors && millis() - lastMonitorPrintMs >= 180) {
    lastMonitorPrintMs = millis();
    printSnapshot(snapshot);
  }

  if (robotState == STATE_IDLE) {
    stopDrive();
    return;
  }

  if (robotState == STATE_FOLLOW_LINE) {
    followLine(snapshot);
  } else if (robotState == STATE_RFID_PAUSE) {
    stopDrive();

    if (elapsed >= RFID_PAUSE_MS) {
      if (snapshot.linePresent) {
        lastError = snapshot.position - CENTER_POSITION;
        lastLineSeenMs = millis();
      }

      enterState(STATE_FOLLOW_LINE);
    }
  } else if (robotState == STATE_PRE_TURN) {
    setDriveSpeeds(PRE_TURN_SPEED, PRE_TURN_SPEED);

    if (elapsed >= PRE_TURN_MS) {
      if (pendingTurn == TURN_LEFT) {
        enterState(STATE_TURN_LEFT);
      } else if (pendingTurn == TURN_RIGHT) {
        enterState(STATE_TURN_RIGHT);
      } else {
        enterState(STATE_FOLLOW_LINE);
      }
    }
  } else if (robotState == STATE_TURN_LEFT) {
    // Pivot turn only. Direction is calibrated for the current chassis wiring.
    int pivotSpeed = elapsed < PIVOT_KICK_MS + TURN_BOOST_MS ? TURN_BOOST_SPEED : TURN_SPEED;

    if (elapsed < PIVOT_KICK_MS) {
      setDriveSpeedsNow(TURN_BOOST_SPEED, -TURN_BOOST_SPEED);
    } else {
      setDriveSpeedsNow(-pivotSpeed, pivotSpeed);
    }

    if (elapsed > PIVOT_KICK_MS + TURN_MIN_MS && (!snapshot.centerPresent || snapshot.position < 2600 || snapshot.position > 5400)) {
      turnReleaseArmed = true;
    }

    if (elapsed > PIVOT_KICK_MS + TURN_MIN_MS && turnReleaseArmed && centerLineFound(snapshot)) {
      pendingTurn = TURN_NONE;
      lastError = 0;
      enterState(STATE_FOLLOW_LINE);
    } else if (elapsed > TURN_MAX_MS) {
      Serial.println("Left turn safety timeout. Recovering line.");
      enterState(STATE_RECOVER_LINE);
    }
  } else if (robotState == STATE_TURN_RIGHT) {
    // Pivot turn only. Direction is calibrated for the current chassis wiring.
    int pivotSpeed = elapsed < PIVOT_KICK_MS + TURN_BOOST_MS ? TURN_BOOST_SPEED : TURN_SPEED;

    if (elapsed < PIVOT_KICK_MS) {
      setDriveSpeedsNow(-TURN_BOOST_SPEED, TURN_BOOST_SPEED);
    } else {
      setDriveSpeedsNow(pivotSpeed, -pivotSpeed);
    }

    if (elapsed > PIVOT_KICK_MS + TURN_MIN_MS && (!snapshot.centerPresent || snapshot.position < 2600 || snapshot.position > 5400)) {
      turnReleaseArmed = true;
    }

    if (elapsed > PIVOT_KICK_MS + TURN_MIN_MS && turnReleaseArmed && centerLineFound(snapshot)) {
      pendingTurn = TURN_NONE;
      lastError = 0;
      enterState(STATE_FOLLOW_LINE);
    } else if (elapsed > TURN_MAX_MS) {
      Serial.println("Right turn safety timeout. Recovering line.");
      enterState(STATE_RECOVER_LINE);
    }
  } else if (robotState == STATE_BRIDGE_HOLLOW) {
    int holdTurn = constrain((int)(KP * lastError), -220, 220);
    setDriveSpeeds(BRIDGE_SPEED + holdTurn, BRIDGE_SPEED - holdTurn);

    if (elapsed > BRIDGE_MIN_MS && centerLineFound(snapshot)) {
      lastError = snapshot.position - CENTER_POSITION;
      enterState(STATE_FOLLOW_LINE);
    } else if (elapsed > BRIDGE_MAX_MS) {
      Serial.println("Hollow bridge timed out. Recovering line.");
      enterState(STATE_RECOVER_LINE);
    }
  } else if (robotState == STATE_RECOVER_LINE) {
    if (snapshot.linePresent && centerLineFound(snapshot)) {
      lastError = snapshot.position - CENTER_POSITION;
      enterState(STATE_FOLLOW_LINE);
      return;
    }

    if (lastPosition < CENTER_POSITION) {
      setDriveSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
    } else {
      setDriveSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
    }

    if (elapsed > RECOVER_MAX_MS) {
      Serial.println("Recover timed out. Stopping.");
      stopRobot();
    }
  }
}

// =====================================================
// Controls and debug output
// =====================================================

void updateStatusLED() {
  bool running = robotState != STATE_IDLE;
  digitalWrite(LED_RED_PIN, running ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, running ? HIGH : LOW);
}

void handleButtons() {
  static bool previousStartDown = false;
  static bool previousStopDown = false;

  bool startDown = digitalRead(START_BUTTON_PIN) == LOW;
  bool stopDown = digitalRead(STOP_BUTTON_PIN) == LOW;

  if (startDown && !previousStartDown) {
    startRobot();
  }

  if (stopDown && !previousStopDown) {
    stopRobot();
  }

  previousStartDown = startDown;
  previousStopDown = stopDown;
}

void printSnapshot(const LineSnapshot &snapshot) {
  Serial.print("state=");
  Serial.print(stateName(robotState));
  Serial.print(" mode=");
  if (lineOnlyMode) {
    Serial.print("LINE_ONLY");
  } else if (task2Mode) {
    Serial.print("TASK2");
  } else {
    Serial.print("FULL");
  }
  Serial.print(" pattern=");

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    Serial.print(snapshot.black[i] ? '1' : '0');
  }

  Serial.print(" values=");

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    Serial.print(snapshot.values[i]);

    if (i < QTR_SENSOR_COUNT - 1) {
      Serial.print(',');
    }
  }

  Serial.print(" pos=");
  Serial.print(snapshot.position);
  Serial.print(" sum=");
  Serial.print(snapshot.sum);
  Serial.print(" L/M/R=");
  Serial.print(snapshot.leftCount);
  Serial.print('/');
  Serial.print(snapshot.midCount);
  Serial.print('/');
  Serial.print(snapshot.rightCount);
  Serial.print(" T=");
  Serial.print(tJunctionFrames);
  Serial.print(" H=");
  Serial.print(hollowCrossFrames);
  Serial.print(" LA=");
  Serial.print(leftAngleFrames);
  Serial.print(" RA=");
  Serial.print(rightAngleFrames);
  Serial.print(" RFID=");
  if (!rfidReady) {
    Serial.println("NOT_FOUND");
  } else if (latestRfidUid.length() == 0) {
    Serial.println("READY");
  } else {
    Serial.println(latestRfidUid);
  }
}

void printCurrentSnapshot() {
  LineSnapshot snapshot = readLineSnapshot();
  printSnapshot(snapshot);
}

void printHelp() {
  Serial.println();
  Serial.println("--- Trial Run 2 Task 2 ---");
  Serial.println("g = start Task 2: first T junction turns left");
  Serial.println("2 = start Task 2, same as g");
  Serial.println("o = start line-only following, no junction state machine");
  Serial.println("s = stop");
  Serial.println("c = recalibrate QTR");
  Serial.println("p = print one sensor snapshot");
  Serial.println("m = toggle live sensor monitor");
  Serial.println("l/r/x = debug only: decision for any extra T junction");
  Serial.println("RFID read during Task 2 pauses briefly, then resumes line following");
  Serial.println("RFID read during Task 2 also sends openAirlock to the server");
  Serial.println("Right-angle bends auto-turn: left bend -> left, right bend -> right");
  Serial.println("a = force left right-angle turn state");
  Serial.println("d = force right right-angle turn state");
  Serial.println("h/? = help");
  Serial.print("Current T junction decision: ");
  Serial.println(tDecisionName(nextTJunctionDecision));
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char command = Serial.read();

    if (command == '\n' || command == '\r' || command == ' ') {
      continue;
    }

    if (command == 'g' || command == 'G') {
      startRobot();
    } else if (command == '2') {
      startTask2Robot();
    } else if (command == 'o' || command == 'O') {
      startLineOnlyRobot();
    } else if (command == 's' || command == 'S') {
      stopRobot();
    } else if (command == 'c' || command == 'C') {
      stopRobot();
      runQTRCalibration();
      Serial.println("Ready after calibration.");
    } else if (command == 'p' || command == 'P') {
      printCurrentSnapshot();
    } else if (command == 'm' || command == 'M') {
      monitorSensors = !monitorSensors;
      Serial.print("Live monitor: ");
      Serial.println(monitorSensors ? "ON" : "OFF");
    } else if (command == 'l' || command == 'L') {
      nextTJunctionDecision = T_DECISION_LEFT;
      Serial.println("Next T junction decision: LEFT");
    } else if (command == 'r' || command == 'R') {
      nextTJunctionDecision = T_DECISION_RIGHT;
      Serial.println("Next T junction decision: RIGHT");
    } else if (command == 'x' || command == 'X') {
      nextTJunctionDecision = T_DECISION_STOP;
      Serial.println("Next T junction decision: STOP");
    } else if (command == 'a' || command == 'A') {
      lineOnlyMode = false;
      pendingTurn = TURN_LEFT;
      enterState(STATE_PRE_TURN);
    } else if (command == 'd' || command == 'D') {
      lineOnlyMode = false;
      pendingTurn = TURN_RIGHT;
      enterState(STATE_PRE_TURN);
    } else if (command == 'h' || command == 'H' || command == '?') {
      printHelp();
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
      printHelp();
    }
  }
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();

  while (!Serial && millis() - startWait < 3000) {
  }

  Serial.println("Trial Run 2 Task 2 starting...");
  Serial.println("Encoders are not used in this test.");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  pinMode(QTR_EMITTER_PIN, OUTPUT);
  digitalWrite(QTR_EMITTER_PIN, HIGH);

  Serial.println("Initializing Motorons on Wire1...");
  Wire1.begin();
  Wire1.setClock(400000);
  delay(100);

  leftController.setBus(&Wire1);
  setupMotoron(&leftController, "Left 0x10");

  rightController.setBus(&Wire1);
  rightController.setAddress(17);
  setupMotoron(&rightController, "Right 0x11");

  if (isI2cDevicePresent(RFID_I2C_ADDRESS)) {
    rfidReady = true;
    Serial.println("RFID module found at I2C address 0x28.");
  } else {
    rfidReady = false;
    Serial.println("RFID module not found at I2C address 0x28.");
    Serial.println("Check WS1850S wiring on Wire1: SDA1/SCL1, 5V, GND.");
  }

  rfid.PCD_Init();

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);

  stopDrive();
  enterState(STATE_IDLE);
  runQTRCalibration();

  lastLineSeenMs = millis();
  updateStatusLED();
  printHelp();
  Serial.println("Place robot on the line. Send g or press D32 to start Task 2.");
}

void loop() {
  messenger.loop();
  sendRegisterIfNeeded();
  sendPendingAirlockRequestIfNeeded();
  handleSerialCommands();
  handleButtons();
  pollRFID();
  updateStateMachine();
  updateStatusLED();
  delay(10);
}

#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Servo.h>

// =====================================================
// Trial Run 2 - Task 3 RFID navigation
// QTR line tracking + RFID map routing + Motoron drive, no encoders.
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
const int BLACK_SENSOR_THRESHOLD = 550;
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

const int TURN_SPEED = 550;
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

// -------------------- Seed / plant dropper --------------------
const byte SEED_SERVO_PIN = 47;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_RANGE_DEGREES = 300;
const int GATE_CLOSED_DEGREES = 20;
const int GATE_OPEN_DEGREES = 90;
const int INITIAL_SEED_COUNT = 5;
const int MAX_SEED_COUNT = 5;
const int MANUAL_SEED_STEP_DEGREES = 60;
const bool MANUAL_CLOCKWISE_IS_INCREASING = true;
const bool TASK3_AUTO_DROP_CLOCKWISE = true;

const unsigned long GATE_OPEN_MS = 450;
const unsigned long GATE_CLOSE_SETTLE_MS = 250;
const int MOVE_STEP_DEGREES = 2;
const unsigned long MOVE_STEP_MS = 8;

enum DropperState {
  DROPPER_IDLE,
  DROPPER_OPENING,
  DROPPER_OPEN,
  DROPPER_CLOSING,
  DROPPER_WAIT_AFTER_CLOSE,
  DROPPER_MANUAL_OPENING,
  DROPPER_MANUAL_CLOSING
};

Servo seedServo;
DropperState dropperState = DROPPER_IDLE;
unsigned long dropperWaitStartedMs = 0;
unsigned long lastDropperMoveStepMs = 0;
int currentDropperAngle = GATE_CLOSED_DEGREES;
int targetDropperAngle = GATE_CLOSED_DEGREES;
int seedsRemaining = INITIAL_SEED_COUNT;

// -------------------- RFID navigation map --------------------
const byte RFID_I2C_ADDRESS = 0x28;
const int RFID_RESET_PIN = -1;
const unsigned long RFID_POLL_MS = 80;

MFRC522_I2C rfid(RFID_I2C_ADDRESS, RFID_RESET_PIN, &Wire1);

const uint8_t MAP_ROWS = 9;
const uint8_t MAP_COLS = 9;
const uint8_t MAX_ROUTE_POINTS = MAP_ROWS * MAP_COLS;

const char *const RFID_MAP[MAP_ROWS][MAP_COLS] = {
  {"C3DFAA41", "671BAB41", "855AAB41", "70CBAA41", "6E54A641", "7447AB41", "1B0AAB41", "418BAB41", "43DB2CDD"},
  {"2802AB41", "7074AB41", "DF54A941", "03CCAA41", "1D65AA41", "F6B6A941", "A42DAB41", "F164AB41", "A335126A"},
  {"4A12AB41", "685EAB41", "E7F7AA41", "9C01AB41", "E238A941", "54C4AA41", "8CE5AA41", "4E4DAB41", "28E3AA41"},
  {"BD47AB41", "F94FAB41", "CE9CAA41", "060DAB41", "6666AA41", "B3DA2ADD", "D3DDAA41", "0D46AB41", "A142AB41"},
  {"5663AB41", "0077AB41", "48CBAA41", "F85EAB41", "4FC0AA41", "AE55AA41", "41AB4141", "FCD6AA41", "D157AB41"},
  {"9259AB41", "3D84AB41", "70D7AA41", "B811AB41", "3ACEAA41", "6C5FAB41", "F459AB41", "47FAAA41", "773DAB41"},
  {"7451AB41", "B493AB41", "6D19AB41", "8A45AB41", "9312AB41", "AC5CAB41", "E840AB41", "F052AB41", "10C7AA41"},
  {"7C88AB41", "2A60AB41", "E74BA941", "C47CAB41", "BCCFAA41", "07F6AA41", "3385AB41", "573DAB41", "F642AB41"},
  {"F07EAB41", "528AAB41", "375CAB41", "8145A941", "76F0AA41", "F63BAB41", "9017AB41", "390DAB41", "1F27AB41"}
};

const char *const TASK3_START_UID = "855AAB41";
const char *const TASK3_EXIT_UID = "1B0AAB41";
const uint8_t TASK3_NAV_ROWS = 4;
const uint8_t TASK3_TARGET_SEEDS = 5;
const int PLANT_FORWARD_SPEED = BASE_SPEED;
const unsigned long PLANT_FORWARD_2CM_MS = 180;

// -------------------- Turn and recovery tuning --------------------
const unsigned long PRE_TURN_MS = 150;
const unsigned long RFID_PAUSE_MS = 250;
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
  STATE_PLANT_RFID_PAUSE,
  STATE_PLANT_FORWARD,
  STATE_PLANT_DROP,
  STATE_WAIT_FOR_SEED_LOAD,
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

enum NavigationMode {
  NAV_MODE_TASK3_PLANTING,
  NAV_MODE_TASK3_FIXED,
  NAV_MODE_AUTO_ROUTE
};

enum Heading {
  HEADING_NORTH,
  HEADING_EAST,
  HEADING_SOUTH,
  HEADING_WEST
};

enum RouteAction {
  ROUTE_ACTION_IGNORE,
  ROUTE_ACTION_STRAIGHT,
  ROUTE_ACTION_LEFT,
  ROUTE_ACTION_RIGHT,
  ROUTE_ACTION_STOP,
  ROUTE_ACTION_UNSUPPORTED
};

const RouteAction TASK3_FIXED_ACTIONS[] = {
  ROUTE_ACTION_STRAIGHT,
  ROUTE_ACTION_RIGHT,
  ROUTE_ACTION_LEFT,
  ROUTE_ACTION_STRAIGHT
};
const uint8_t TASK3_FIXED_ACTION_COUNT = sizeof(TASK3_FIXED_ACTIONS) / sizeof(TASK3_FIXED_ACTIONS[0]);

struct GridPoint {
  int8_t row;
  int8_t col;
};

RobotState robotState = STATE_IDLE;
TurnDirection pendingTurn = TURN_NONE;
NavigationMode selectedNavMode = NAV_MODE_TASK3_PLANTING;
Heading currentHeading = HEADING_EAST;
Heading configuredStartHeading = HEADING_EAST;
RouteAction pendingPausedAction = ROUTE_ACTION_IGNORE;
bool lineOnlyMode = false;
bool rfidReady = false;
bool routeActive = false;

GridPoint activeRoute[MAX_ROUTE_POINTS];
uint8_t activeRouteLength = 0;
uint8_t activeRouteIndex = 0;
uint8_t fixedActionIndex = 0;

bool task3Planted[MAP_ROWS][MAP_COLS];
uint8_t task3SeedsPlanted = 0;
bool task3ReturningToExit = false;
bool task3MissionComplete = false;
bool task3CurrentPointValid = false;
bool task3PendingPlantValid = false;
GridPoint task3CurrentPoint = {-1, -1};
GridPoint task3PendingPlantPoint = {-1, -1};
GridPoint task3RouteGoal = {-1, -1};

String autoStartUid = "";
String autoGoalUid = "";
String latestRfidUid = "";
String lastActionUid = "";
String serialLine = "";

bool monitorSensors = false;
unsigned long stateStartedMs = 0;
unsigned long lastMonitorPrintMs = 0;
unsigned long lastRfidPollMs = 0;
bool lineLostLatched = false;
bool turnReleaseArmed = false;

uint8_t leftAngleFrames = 0;
uint8_t rightAngleFrames = 0;
uint8_t hollowCrossFrames = 0;

void stopDrive();
void stopRobot();
void printHelp();
void printSnapshot(const LineSnapshot &snapshot);
void printActiveRoute();
void enterState(RobotState newState);
void performRouteAction(RouteAction action);
void executeRouteAction(RouteAction action);
void setupSeedDropper();
void updateSeedDropper();
bool isSeedDropperBusy();
bool handlePlantCommand(const String &lower);
void printTask3PlantingStatus();
void resetTask3PlantingMemory();
bool runTask3ServoDropAtPendingPoint();
void completeTask3PlantingDrop();

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
// Seed / plant dropper
// =====================================================

int dropperAngleToPulse(int angle) {
  angle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  long pulse = SERVO_MIN_US +
               (long)(SERVO_MAX_US - SERVO_MIN_US) * angle / SERVO_RANGE_DEGREES;
  return (int)pulse;
}

void setDropperAngle(int angle) {
  currentDropperAngle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  seedServo.writeMicroseconds(dropperAngleToPulse(currentDropperAngle));
}

void startDropperMove(int angle) {
  targetDropperAngle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  lastDropperMoveStepMs = millis();
}

bool updateDropperMove() {
  if (currentDropperAngle == targetDropperAngle) {
    return true;
  }

  unsigned long nowMs = millis();
  if (nowMs - lastDropperMoveStepMs < MOVE_STEP_MS) {
    return false;
  }

  lastDropperMoveStepMs = nowMs;

  if (targetDropperAngle > currentDropperAngle) {
    setDropperAngle(min(currentDropperAngle + MOVE_STEP_DEGREES, targetDropperAngle));
  } else {
    setDropperAngle(max(currentDropperAngle - MOVE_STEP_DEGREES, targetDropperAngle));
  }

  return currentDropperAngle == targetDropperAngle;
}

bool moveDropperToAngleBlocking(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, SERVO_RANGE_DEGREES);

  if (targetAngle == currentDropperAngle) {
    Serial.print("Plant servo already at ");
    Serial.print(currentDropperAngle);
    Serial.println(" degrees.");
    targetDropperAngle = currentDropperAngle;
    return false;
  }

  if (targetAngle > currentDropperAngle) {
    for (int angle = currentDropperAngle; angle <= targetAngle; angle += MOVE_STEP_DEGREES) {
      setDropperAngle(angle);
      delay(MOVE_STEP_MS);
    }
  } else {
    for (int angle = currentDropperAngle; angle >= targetAngle; angle -= MOVE_STEP_DEGREES) {
      setDropperAngle(angle);
      delay(MOVE_STEP_MS);
    }
  }

  setDropperAngle(targetAngle);
  targetDropperAngle = targetAngle;

  Serial.print("Plant servo angle: ");
  Serial.print(currentDropperAngle);
  Serial.println(" degrees.");
  delay(100);
  return true;
}

bool rotateDropperClockwise60Blocking() {
  int targetAngle = MANUAL_CLOCKWISE_IS_INCREASING
                    ? currentDropperAngle + MANUAL_SEED_STEP_DEGREES
                    : currentDropperAngle - MANUAL_SEED_STEP_DEGREES;
  return moveDropperToAngleBlocking(targetAngle);
}

bool rotateDropperCounterClockwise60Blocking() {
  int targetAngle = MANUAL_CLOCKWISE_IS_INCREASING
                    ? currentDropperAngle - MANUAL_SEED_STEP_DEGREES
                    : currentDropperAngle + MANUAL_SEED_STEP_DEGREES;
  return moveDropperToAngleBlocking(targetAngle);
}

bool resetDropperToZeroBlocking() {
  return moveDropperToAngleBlocking(0);
}

bool rotateTask3AutoDropStepBlocking() {
  if (TASK3_AUTO_DROP_CLOCKWISE) {
    return rotateDropperClockwise60Blocking();
  }

  return rotateDropperCounterClockwise60Blocking();
}

bool isSeedDropperBusy() {
  return dropperState != DROPPER_IDLE;
}

const char *dropperStateName() {
  switch (dropperState) {
    case DROPPER_IDLE:
      return "IDLE";
    case DROPPER_OPENING:
      return "OPENING";
    case DROPPER_OPEN:
      return "OPEN";
    case DROPPER_CLOSING:
      return "CLOSING";
    case DROPPER_WAIT_AFTER_CLOSE:
      return "WAIT_AFTER_CLOSE";
    case DROPPER_MANUAL_OPENING:
      return "MANUAL_OPENING";
    case DROPPER_MANUAL_CLOSING:
      return "MANUAL_CLOSING";
  }

  return "?";
}

void setupSeedDropper() {
  seedServo.attach(SEED_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setDropperAngle(0);
  targetDropperAngle = 0;
}

bool startSeedDrop() {
  if (isSeedDropperBusy()) {
    Serial.println("Plant drop ignored: dropper is busy.");
    return false;
  }

  if (seedsRemaining <= 0) {
    Serial.println("Plant drop ignored: no seeds remaining.");
    return false;
  }

  startDropperMove(GATE_OPEN_DEGREES);
  dropperState = DROPPER_OPENING;
  Serial.println("Plant seed drop started.");
  return true;
}

void updateSeedDropper() {
  if (dropperState == DROPPER_IDLE) {
    return;
  }

  unsigned long elapsedMs = millis() - dropperWaitStartedMs;

  if (dropperState == DROPPER_OPENING && updateDropperMove()) {
    dropperState = DROPPER_OPEN;
    dropperWaitStartedMs = millis();
  } else if (dropperState == DROPPER_OPEN && elapsedMs >= GATE_OPEN_MS) {
    startDropperMove(GATE_CLOSED_DEGREES);
    dropperState = DROPPER_CLOSING;
  } else if (dropperState == DROPPER_CLOSING && updateDropperMove()) {
    dropperState = DROPPER_WAIT_AFTER_CLOSE;
    dropperWaitStartedMs = millis();
  } else if (dropperState == DROPPER_WAIT_AFTER_CLOSE && elapsedMs >= GATE_CLOSE_SETTLE_MS) {
    seedsRemaining--;
    dropperState = DROPPER_IDLE;

    Serial.print("Plant seed dropped. Remaining: ");
    Serial.println(seedsRemaining);
  } else if (dropperState == DROPPER_MANUAL_OPENING && updateDropperMove()) {
    dropperState = DROPPER_IDLE;
    Serial.println("Plant gate manual open complete.");
  } else if (dropperState == DROPPER_MANUAL_CLOSING && updateDropperMove()) {
    dropperState = DROPPER_IDLE;
    Serial.println("Plant gate manual close complete.");
  }
}

void printSeedDropperStatus() {
  Serial.print("Plant seeds=");
  Serial.print(seedsRemaining);
  Serial.print("/");
  Serial.print(MAX_SEED_COUNT);
  Serial.print(" state=");
  Serial.print(dropperStateName());
  Serial.print(" angle=");
  Serial.print(currentDropperAngle);
  Serial.print(" target=");
  Serial.println(targetDropperAngle);
}

bool requireIdleForPlantCommand() {
  if (robotState != STATE_IDLE) {
    Serial.println("Stop Task 3 before manual plant/dropper control.");
    return false;
  }

  return true;
}

bool handlePlantCommand(const String &lower) {
  if (lower == "plant status") {
    printSeedDropperStatus();
    return true;
  }

  bool isLoadCommand = lower == "plant load" || lower == "load" || lower == "seed load";
  bool isManualDropCommand = lower == "plant drop" || lower == "drop" || lower == "seed drop";

  if (!isLoadCommand && !isManualDropCommand && !lower.startsWith("plant ")) {
    return false;
  }

  if (isLoadCommand) {
    bool task3WaitingForSeed = selectedNavMode == NAV_MODE_TASK3_PLANTING &&
                               robotState == STATE_WAIT_FOR_SEED_LOAD;

    if (!task3WaitingForSeed && !requireIdleForPlantCommand()) {
      return true;
    }

    if (isSeedDropperBusy()) {
      Serial.println("Plant load ignored: dropper is busy.");
    } else if (seedsRemaining >= MAX_SEED_COUNT) {
      Serial.println("Plant seed count already full.");
    } else if (rotateDropperClockwise60Blocking()) {
      seedsRemaining++;
      Serial.print("Plant seed loaded. Remaining: ");
      Serial.println(seedsRemaining);
    } else {
      Serial.println("Plant load did not move the servo; seed count unchanged.");
    }

    if (task3WaitingForSeed && seedsRemaining > 0 && task3PendingPlantValid) {
      Serial.println("Task3 manual seed loaded. Resuming 60-degree seed drop.");
      runTask3ServoDropAtPendingPoint();
    }

    return true;
  }

  if (isManualDropCommand) {
    if (!requireIdleForPlantCommand()) {
      return true;
    }

    if (isSeedDropperBusy()) {
      Serial.println("Plant manual drop ignored: dropper is busy.");
    } else if (seedsRemaining <= 0) {
      Serial.println("Plant manual drop ignored: no seeds remaining.");
    } else if (rotateDropperCounterClockwise60Blocking()) {
      seedsRemaining--;
      Serial.print("Plant seed manually dropped. Remaining: ");
      Serial.println(seedsRemaining);
    } else {
      Serial.println("Plant manual drop did not move the servo; seed count unchanged.");
    }

    return true;
  }

  if (lower == "plant reset") {
    if (!requireIdleForPlantCommand()) {
      return true;
    }

    if (isSeedDropperBusy()) {
      Serial.println("Plant reset ignored: dropper is busy.");
    } else {
      seedsRemaining = INITIAL_SEED_COUNT;
      setDropperAngle(0);
      targetDropperAngle = 0;
      Serial.println("Plant seed count reset and servo zeroed.");
    }
    return true;
  }

  if (lower == "plant zero" || lower == "plant servo reset" || lower == "plant reset servo") {
    if (!requireIdleForPlantCommand()) {
      return true;
    }

    if (isSeedDropperBusy()) {
      Serial.println("Plant servo reset ignored: dropper is busy.");
    } else {
      resetDropperToZeroBlocking();
    }
    return true;
  }

  if (lower == "plant open") {
    if (!requireIdleForPlantCommand()) {
      return true;
    }

    if (isSeedDropperBusy()) {
      Serial.println("Plant open ignored: dropper is busy.");
    } else {
      startDropperMove(GATE_OPEN_DEGREES);
      dropperState = DROPPER_MANUAL_OPENING;
      Serial.println("Plant gate manual open started.");
    }
    return true;
  }

  if (lower == "plant close") {
    if (!requireIdleForPlantCommand()) {
      return true;
    }

    if (dropperState != DROPPER_IDLE && dropperState != DROPPER_MANUAL_OPENING) {
      Serial.println("Plant close ignored: automatic drop is active.");
    } else {
      startDropperMove(GATE_CLOSED_DEGREES);
      dropperState = DROPPER_MANUAL_CLOSING;
      Serial.println("Plant gate manual close started.");
    }
    return true;
  }

  if (lower == "plant auto drop") {
    if (!requireIdleForPlantCommand()) {
      return true;
    }

    stopDrive();
    startSeedDrop();
    return true;
  }

  Serial.println("Unknown plant command. Use load/drop/plant auto drop/plant reset/plant zero/plant open/plant close/plant status.");
  return true;
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
    case STATE_PLANT_RFID_PAUSE:
      return "PLANT_RFID_PAUSE";
    case STATE_PLANT_FORWARD:
      return "PLANT_FORWARD";
    case STATE_PLANT_DROP:
      return "PLANT_DROP";
    case STATE_WAIT_FOR_SEED_LOAD:
      return "WAIT_FOR_SEED_LOAD";
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

const char *navigationModeName(NavigationMode mode) {
  switch (mode) {
    case NAV_MODE_TASK3_PLANTING:
      return "TASK3_PLANTING";
    case NAV_MODE_TASK3_FIXED:
      return "TASK3_FIXED";
    case NAV_MODE_AUTO_ROUTE:
      return "AUTO_ROUTE";
  }

  return "?";
}

const char *headingName(Heading heading) {
  switch (heading) {
    case HEADING_NORTH:
      return "NORTH";
    case HEADING_EAST:
      return "EAST";
    case HEADING_SOUTH:
      return "SOUTH";
    case HEADING_WEST:
      return "WEST";
  }

  return "?";
}

const char *routeActionName(RouteAction action) {
  switch (action) {
    case ROUTE_ACTION_IGNORE:
      return "IGNORE";
    case ROUTE_ACTION_STRAIGHT:
      return "STRAIGHT";
    case ROUTE_ACTION_LEFT:
      return "LEFT";
    case ROUTE_ACTION_RIGHT:
      return "RIGHT";
    case ROUTE_ACTION_STOP:
      return "STOP";
    case ROUTE_ACTION_UNSUPPORTED:
      return "UNSUPPORTED";
  }

  return "?";
}

String normalizeUid(String uid) {
  uid.trim();
  uid.toUpperCase();
  uid.replace(":", "");
  uid.replace("-", "");
  uid.replace(" ", "");
  return uid;
}

bool samePoint(GridPoint a, GridPoint b) {
  return a.row == b.row && a.col == b.col;
}

bool findUidInMap(const String &uid, GridPoint &point) {
  String normalized = normalizeUid(uid);

  for (uint8_t row = 0; row < MAP_ROWS; row++) {
    for (uint8_t col = 0; col < MAP_COLS; col++) {
      if (normalized == RFID_MAP[row][col]) {
        point.row = row;
        point.col = col;
        return true;
      }
    }
  }

  return false;
}

String uidAtPoint(GridPoint point) {
  if (point.row < 0 || point.row >= MAP_ROWS || point.col < 0 || point.col >= MAP_COLS) {
    return "";
  }

  return String(RFID_MAP[point.row][point.col]);
}

int pointIndex(GridPoint point) {
  return point.row * MAP_COLS + point.col;
}

GridPoint indexToPoint(int index) {
  GridPoint point;
  point.row = index / MAP_COLS;
  point.col = index % MAP_COLS;
  return point;
}

bool computeSegment(GridPoint start, GridPoint goal, GridPoint *segment, uint8_t &segmentLength) {
  bool visited[MAX_ROUTE_POINTS];
  int8_t parent[MAX_ROUTE_POINTS];
  uint8_t queue[MAX_ROUTE_POINTS];
  uint8_t head = 0;
  uint8_t tail = 0;

  for (uint8_t i = 0; i < MAX_ROUTE_POINTS; i++) {
    visited[i] = false;
    parent[i] = -1;
  }

  int startIndex = pointIndex(start);
  int goalIndex = pointIndex(goal);
  visited[startIndex] = true;
  queue[tail++] = startIndex;

  const int8_t rowDelta[4] = {-1, 0, 1, 0};
  const int8_t colDelta[4] = {0, 1, 0, -1};

  while (head < tail) {
    int current = queue[head++];

    if (current == goalIndex) {
      break;
    }

    GridPoint currentPoint = indexToPoint(current);

    for (uint8_t i = 0; i < 4; i++) {
      GridPoint nextPoint;
      nextPoint.row = currentPoint.row + rowDelta[i];
      nextPoint.col = currentPoint.col + colDelta[i];

      if (nextPoint.row < 0 || nextPoint.row >= MAP_ROWS || nextPoint.col < 0 || nextPoint.col >= MAP_COLS) {
        continue;
      }

      int nextIndex = pointIndex(nextPoint);
      if (visited[nextIndex]) {
        continue;
      }

      visited[nextIndex] = true;
      parent[nextIndex] = current;
      queue[tail++] = nextIndex;
    }
  }

  if (!visited[goalIndex]) {
    segmentLength = 0;
    return false;
  }

  uint8_t reverseLength = 0;
  int current = goalIndex;

  while (current >= 0 && reverseLength < MAX_ROUTE_POINTS) {
    segment[reverseLength++] = indexToPoint(current);
    if (current == startIndex) {
      break;
    }
    current = parent[current];
  }

  segmentLength = reverseLength;
  for (uint8_t i = 0; i < segmentLength / 2; i++) {
    GridPoint temp = segment[i];
    segment[i] = segment[segmentLength - 1 - i];
    segment[segmentLength - 1 - i] = temp;
  }

  return true;
}

bool buildRouteFromPoints(GridPoint *waypoints, uint8_t waypointCount) {
  if (waypointCount < 2) {
    return false;
  }

  activeRouteLength = 0;
  activeRouteIndex = 0;

  for (uint8_t i = 0; i < waypointCount - 1; i++) {
    GridPoint segment[MAX_ROUTE_POINTS];
    uint8_t segmentLength = 0;

    if (!computeSegment(waypoints[i], waypoints[i + 1], segment, segmentLength)) {
      activeRouteLength = 0;
      return false;
    }

    uint8_t startAt = activeRouteLength == 0 ? 0 : 1;
    for (uint8_t j = startAt; j < segmentLength; j++) {
      if (activeRouteLength >= MAX_ROUTE_POINTS) {
        return false;
      }
      activeRoute[activeRouteLength++] = segment[j];
    }
  }

  return activeRouteLength >= 2;
}

bool buildRouteFromUids(const char *const *uids, uint8_t uidCount) {
  GridPoint waypoints[MAX_ROUTE_POINTS];

  if (uidCount > MAX_ROUTE_POINTS) {
    return false;
  }

  for (uint8_t i = 0; i < uidCount; i++) {
    if (!findUidInMap(String(uids[i]), waypoints[i])) {
      Serial.print("Route UID not found in map: ");
      Serial.println(uids[i]);
      return false;
    }
  }

  return buildRouteFromPoints(waypoints, uidCount);
}

bool buildAutoRoute() {
  GridPoint waypoints[2];

  if (!findUidInMap(autoStartUid, waypoints[0])) {
    Serial.print("Auto start UID not found: ");
    Serial.println(autoStartUid);
    return false;
  }

  if (!findUidInMap(autoGoalUid, waypoints[1])) {
    Serial.print("Auto goal UID not found: ");
    Serial.println(autoGoalUid);
    return false;
  }

  return buildRouteFromPoints(waypoints, 2);
}

Heading headingBetween(GridPoint from, GridPoint to) {
  if (to.row < from.row) {
    return HEADING_NORTH;
  }
  if (to.row > from.row) {
    return HEADING_SOUTH;
  }
  if (to.col > from.col) {
    return HEADING_EAST;
  }
  return HEADING_WEST;
}

RouteAction actionFromHeading(Heading from, Heading to) {
  int delta = ((int)to - (int)from + 4) % 4;

  if (delta == 0) {
    return ROUTE_ACTION_STRAIGHT;
  }
  if (delta == 1) {
    return ROUTE_ACTION_RIGHT;
  }
  if (delta == 3) {
    return ROUTE_ACTION_LEFT;
  }

  return ROUTE_ACTION_UNSUPPORTED;
}

Heading leftOfHeading(Heading heading) {
  return (Heading)(((int)heading + 3) % 4);
}

Heading rightOfHeading(Heading heading) {
  return (Heading)(((int)heading + 1) % 4);
}

void headingDelta(Heading heading, int8_t &rowDelta, int8_t &colDelta) {
  rowDelta = 0;
  colDelta = 0;

  if (heading == HEADING_NORTH) {
    rowDelta = -1;
  } else if (heading == HEADING_EAST) {
    colDelta = 1;
  } else if (heading == HEADING_SOUTH) {
    rowDelta = 1;
  } else if (heading == HEADING_WEST) {
    colDelta = -1;
  }
}

bool isValidGridPoint(GridPoint point) {
  return point.row >= 0 && point.row < MAP_ROWS && point.col >= 0 && point.col < MAP_COLS;
}

bool gridPointUidEquals(GridPoint point, const char *uid) {
  if (!isValidGridPoint(point)) {
    return false;
  }

  return String(RFID_MAP[point.row][point.col]) == uid;
}

bool isTask3StartPoint(GridPoint point) {
  return gridPointUidEquals(point, TASK3_START_UID);
}

bool isTask3ExitPoint(GridPoint point) {
  return gridPointUidEquals(point, TASK3_EXIT_UID);
}

bool isTask3Endpoint(GridPoint point) {
  return isTask3StartPoint(point) || isTask3ExitPoint(point);
}

bool isTask3PlantableCell(GridPoint point) {
  return isValidGridPoint(point) &&
         point.row < TASK3_NAV_ROWS &&
         !isTask3Endpoint(point);
}

bool isTask3CellPlanted(GridPoint point) {
  if (!isValidGridPoint(point)) {
    return false;
  }

  return task3Planted[point.row][point.col];
}

void setInvalidPoint(GridPoint &point) {
  point.row = -1;
  point.col = -1;
}

void resetTask3PlantingMemory() {
  for (uint8_t row = 0; row < MAP_ROWS; row++) {
    for (uint8_t col = 0; col < MAP_COLS; col++) {
      task3Planted[row][col] = false;
    }
  }

  task3SeedsPlanted = 0;
  task3ReturningToExit = false;
  task3MissionComplete = false;
  task3CurrentPointValid = false;
  task3PendingPlantValid = false;
  setInvalidPoint(task3CurrentPoint);
  setInvalidPoint(task3PendingPlantPoint);
  setInvalidPoint(task3RouteGoal);
  activeRouteLength = 0;
  activeRouteIndex = 0;

  GridPoint startPoint;
  if (findUidInMap(String(TASK3_START_UID), startPoint)) {
    task3CurrentPoint = startPoint;
    task3CurrentPointValid = true;
  }

  Serial.println("Task 3 planting memory reset.");
}

void printGridPoint(const char *label, GridPoint point) {
  Serial.print(label);

  if (!isValidGridPoint(point)) {
    Serial.println("(unknown)");
    return;
  }

  Serial.print("r");
  Serial.print(point.row + 1);
  Serial.print(" c");
  Serial.print(point.col + 1);
  Serial.print(" ");
  Serial.println(uidAtPoint(point));
}

void printTask3PlantingStatus() {
  Serial.print("Task3 planted=");
  Serial.print(task3SeedsPlanted);
  Serial.print("/");
  Serial.print(TASK3_TARGET_SEEDS);
  Serial.print(" seedsRemaining=");
  Serial.print(seedsRemaining);
  Serial.print(" returning=");
  Serial.print(task3ReturningToExit ? "yes" : "no");
  Serial.print(" complete=");
  Serial.println(task3MissionComplete ? "yes" : "no");

  printGridPoint("Current: ", task3CurrentPoint);
  printGridPoint("Route goal: ", task3RouteGoal);
}

bool pointAllowedForTask3Route(GridPoint point, GridPoint start, GridPoint goal, bool avoidPlanted) {
  if (!isValidGridPoint(point) || point.row >= TASK3_NAV_ROWS) {
    return false;
  }

  if (samePoint(point, start) || samePoint(point, goal)) {
    return true;
  }

  if (avoidPlanted && isTask3CellPlanted(point)) {
    return false;
  }

  return true;
}

bool computeTask3Segment(GridPoint start, GridPoint goal, GridPoint *segment, uint8_t &segmentLength, bool avoidPlanted) {
  if (!pointAllowedForTask3Route(start, start, goal, false) ||
      !pointAllowedForTask3Route(goal, start, goal, false)) {
    segmentLength = 0;
    return false;
  }

  bool visited[MAX_ROUTE_POINTS];
  int8_t parent[MAX_ROUTE_POINTS];
  uint8_t queue[MAX_ROUTE_POINTS];
  uint8_t head = 0;
  uint8_t tail = 0;

  for (uint8_t i = 0; i < MAX_ROUTE_POINTS; i++) {
    visited[i] = false;
    parent[i] = -1;
  }

  int startIndex = pointIndex(start);
  int goalIndex = pointIndex(goal);
  visited[startIndex] = true;
  queue[tail++] = startIndex;

  const int8_t rowDelta[4] = {-1, 0, 1, 0};
  const int8_t colDelta[4] = {0, 1, 0, -1};

  while (head < tail) {
    int current = queue[head++];

    if (current == goalIndex) {
      break;
    }

    GridPoint currentPoint = indexToPoint(current);

    for (uint8_t i = 0; i < 4; i++) {
      GridPoint nextPoint;
      nextPoint.row = currentPoint.row + rowDelta[i];
      nextPoint.col = currentPoint.col + colDelta[i];

      if (!pointAllowedForTask3Route(nextPoint, start, goal, avoidPlanted)) {
        continue;
      }

      int nextIndex = pointIndex(nextPoint);
      if (visited[nextIndex]) {
        continue;
      }

      visited[nextIndex] = true;
      parent[nextIndex] = current;
      queue[tail++] = nextIndex;
    }
  }

  if (!visited[goalIndex]) {
    segmentLength = 0;
    return false;
  }

  uint8_t reverseLength = 0;
  int current = goalIndex;

  while (current >= 0 && reverseLength < MAX_ROUTE_POINTS) {
    segment[reverseLength++] = indexToPoint(current);
    if (current == startIndex) {
      break;
    }
    current = parent[current];
  }

  segmentLength = reverseLength;
  for (uint8_t i = 0; i < segmentLength / 2; i++) {
    GridPoint temp = segment[i];
    segment[i] = segment[segmentLength - 1 - i];
    segment[segmentLength - 1 - i] = temp;
  }

  return true;
}

RouteAction firstActionForRoute(GridPoint *route, uint8_t routeLength) {
  if (routeLength < 2) {
    return ROUTE_ACTION_STOP;
  }

  Heading nextHeading = headingBetween(route[0], route[1]);
  return actionFromHeading(currentHeading, nextHeading);
}

int routeActionPenalty(RouteAction action) {
  if (action == ROUTE_ACTION_STRAIGHT) {
    return 0;
  }

  if (action == ROUTE_ACTION_LEFT || action == ROUTE_ACTION_RIGHT) {
    return 1;
  }

  return 200;
}

bool routeReturnsToCurrent(GridPoint *route, uint8_t routeLength) {
  for (uint8_t i = 1; i < routeLength; i++) {
    if (samePoint(route[i], task3CurrentPoint)) {
      return true;
    }
  }

  return false;
}

void copyRouteToActive(GridPoint *route, uint8_t routeLength) {
  activeRouteLength = routeLength;
  activeRouteIndex = 0;

  for (uint8_t i = 0; i < routeLength && i < MAX_ROUTE_POINTS; i++) {
    activeRoute[i] = route[i];
  }
}

bool buildTask3RouteToPoint(GridPoint goal, GridPoint *route, uint8_t &routeLength, bool avoidPlanted) {
  routeLength = 0;

  if (!task3CurrentPointValid) {
    Serial.println("Task3 route failed: current RFID point is unknown.");
    return false;
  }

  GridPoint direct[MAX_ROUTE_POINTS];
  uint8_t directLength = 0;
  bool hasDirect = computeTask3Segment(task3CurrentPoint, goal, direct, directLength, avoidPlanted);

  if (hasDirect && directLength < 2) {
    for (uint8_t i = 0; i < directLength; i++) {
      route[i] = direct[i];
    }
    routeLength = directLength;
    return true;
  }

  if (hasDirect && firstActionForRoute(direct, directLength) != ROUTE_ACTION_UNSUPPORTED) {
    for (uint8_t i = 0; i < directLength; i++) {
      route[i] = direct[i];
    }
    routeLength = directLength;
    return true;
  }

  Heading options[3] = {
    currentHeading,
    leftOfHeading(currentHeading),
    rightOfHeading(currentHeading)
  };

  bool foundAlternative = false;
  int bestScore = 32767;
  GridPoint bestRoute[MAX_ROUTE_POINTS];
  uint8_t bestLength = 0;

  for (uint8_t option = 0; option < 3; option++) {
    Heading firstHeading = options[option];
    RouteAction firstAction = actionFromHeading(currentHeading, firstHeading);

    if (firstAction == ROUTE_ACTION_UNSUPPORTED) {
      continue;
    }

    int8_t rowStep = 0;
    int8_t colStep = 0;
    headingDelta(firstHeading, rowStep, colStep);

    GridPoint neighbor;
    neighbor.row = task3CurrentPoint.row + rowStep;
    neighbor.col = task3CurrentPoint.col + colStep;

    if (!pointAllowedForTask3Route(neighbor, task3CurrentPoint, goal, avoidPlanted)) {
      continue;
    }

    GridPoint tail[MAX_ROUTE_POINTS];
    uint8_t tailLength = 0;
    if (!computeTask3Segment(neighbor, goal, tail, tailLength, avoidPlanted)) {
      continue;
    }

    if (tailLength + 1 > MAX_ROUTE_POINTS) {
      continue;
    }

    GridPoint candidateRoute[MAX_ROUTE_POINTS];
    uint8_t candidateLength = tailLength + 1;
    candidateRoute[0] = task3CurrentPoint;

    for (uint8_t i = 0; i < tailLength; i++) {
      candidateRoute[i + 1] = tail[i];
    }

    if (routeReturnsToCurrent(candidateRoute, candidateLength)) {
      continue;
    }

    int score = ((int)candidateLength - 1) * 10 + routeActionPenalty(firstAction);
    if (!foundAlternative || score < bestScore) {
      foundAlternative = true;
      bestScore = score;
      bestLength = candidateLength;

      for (uint8_t i = 0; i < candidateLength; i++) {
        bestRoute[i] = candidateRoute[i];
      }
    }
  }

  if (foundAlternative) {
    for (uint8_t i = 0; i < bestLength; i++) {
      route[i] = bestRoute[i];
    }
    routeLength = bestLength;
    return true;
  }

  if (hasDirect) {
    Serial.println("Task3 route warning: first move would be a U-turn.");
    for (uint8_t i = 0; i < directLength; i++) {
      route[i] = direct[i];
    }
    routeLength = directLength;
    return true;
  }

  return false;
}

bool setTask3ActiveRoute(GridPoint goal, bool avoidPlanted) {
  GridPoint route[MAX_ROUTE_POINTS];
  uint8_t routeLength = 0;

  if (!buildTask3RouteToPoint(goal, route, routeLength, avoidPlanted)) {
    if (avoidPlanted && buildTask3RouteToPoint(goal, route, routeLength, false)) {
      Serial.println("Task3 route fallback: using route through planted cells.");
    } else {
      activeRouteLength = 0;
      activeRouteIndex = 0;
      return false;
    }
  }

  copyRouteToActive(route, routeLength);
  task3RouteGoal = goal;
  return true;
}

bool planTask3RouteToExit() {
  GridPoint exitPoint;

  if (!findUidInMap(String(TASK3_EXIT_UID), exitPoint)) {
    Serial.println("Task3 exit UID is not in the RFID map.");
    return false;
  }

  task3ReturningToExit = true;

  if (!setTask3ActiveRoute(exitPoint, true)) {
    Serial.println("Task3 cannot plan a route back to the exit.");
    return false;
  }

  Serial.println("Task3 return route planned to exit.");
  printActiveRoute();
  return true;
}

bool planTask3RouteToNearestPlant() {
  bool foundTarget = false;
  int bestScore = 32767;
  GridPoint bestTarget = {-1, -1};
  GridPoint bestRoute[MAX_ROUTE_POINTS];
  uint8_t bestLength = 0;

  for (uint8_t row = 0; row < TASK3_NAV_ROWS; row++) {
    for (uint8_t col = 0; col < MAP_COLS; col++) {
      GridPoint candidate;
      candidate.row = row;
      candidate.col = col;

      if (!isTask3PlantableCell(candidate) || isTask3CellPlanted(candidate)) {
        continue;
      }

      GridPoint route[MAX_ROUTE_POINTS];
      uint8_t routeLength = 0;
      if (!buildTask3RouteToPoint(candidate, route, routeLength, true)) {
        continue;
      }

      RouteAction firstAction = firstActionForRoute(route, routeLength);
      int score = ((int)routeLength - 1) * 10 + routeActionPenalty(firstAction);

      if (!foundTarget || score < bestScore) {
        foundTarget = true;
        bestScore = score;
        bestTarget = candidate;
        bestLength = routeLength;

        for (uint8_t i = 0; i < routeLength; i++) {
          bestRoute[i] = route[i];
        }
      }
    }
  }

  if (!foundTarget) {
    Serial.println("Task3 has no reachable unplanted cells. Returning to exit.");
    return planTask3RouteToExit();
  }

  task3ReturningToExit = false;
  task3RouteGoal = bestTarget;
  copyRouteToActive(bestRoute, bestLength);

  Serial.print("Task3 next planting target: ");
  Serial.println(uidAtPoint(bestTarget));
  printActiveRoute();
  return true;
}

bool prepareTask3PlantingMission() {
  resetTask3PlantingMemory();

  if (!task3CurrentPointValid) {
    Serial.println("Task3 start UID is not in the RFID map.");
    return false;
  }

  if (seedsRemaining < TASK3_TARGET_SEEDS) {
    Serial.println("Warning: fewer than 5 seeds are recorded as loaded.");
  }

  Serial.print("Task3 start UID: ");
  Serial.println(TASK3_START_UID);
  Serial.print("Task3 exit UID: ");
  Serial.println(TASK3_EXIT_UID);
  Serial.println("Task3 plants only in RFID map rows 1-4.");

  return planTask3RouteToNearestPlant();
}

void completeTask3MissionAtExit() {
  task3MissionComplete = true;
  routeActive = false;
  stopDrive();
  enterState(STATE_IDLE);
  Serial.println("Task3 complete: returned to exit RFID.");
}

RouteAction nextTask3RouteAction() {
  if (activeRouteLength < 2) {
    return ROUTE_ACTION_STOP;
  }

  Heading nextHeading = headingBetween(activeRoute[0], activeRoute[1]);
  RouteAction action = actionFromHeading(currentHeading, nextHeading);

  if (action != ROUTE_ACTION_UNSUPPORTED) {
    currentHeading = nextHeading;
    activeRouteIndex = 1;
  }

  return action;
}

bool continueTask3NavigationFromCurrent(bool pauseBeforeAction) {
  if (!task3CurrentPointValid) {
    Serial.println("Task3 cannot continue: current RFID point is unknown.");
    stopRobot();
    return false;
  }

  if (task3SeedsPlanted >= TASK3_TARGET_SEEDS || task3ReturningToExit) {
    if (isTask3ExitPoint(task3CurrentPoint)) {
      completeTask3MissionAtExit();
      return true;
    }

    if (!planTask3RouteToExit()) {
      stopRobot();
      return false;
    }
  } else if (!planTask3RouteToNearestPlant()) {
    stopRobot();
    return false;
  }

  if (activeRouteLength < 2) {
    if (task3ReturningToExit && isTask3ExitPoint(task3CurrentPoint)) {
      completeTask3MissionAtExit();
      return true;
    }

    enterState(STATE_FOLLOW_LINE);
    return true;
  }

  RouteAction action = nextTask3RouteAction();

  if (pauseBeforeAction) {
    executeRouteAction(action);
  } else {
    performRouteAction(action);
  }

  return true;
}

void startTask3PlantAtCurrentPoint() {
  task3PendingPlantPoint = task3CurrentPoint;
  task3PendingPlantValid = true;

  stopDrive();
  Serial.print("Task3 planting check accepted at ");
  Serial.println(uidAtPoint(task3PendingPlantPoint));
  enterState(STATE_PLANT_RFID_PAUSE);
}

bool runTask3ServoDropAtPendingPoint() {
  if (!task3PendingPlantValid || !isValidGridPoint(task3PendingPlantPoint)) {
    Serial.println("Task3 60-degree drop ignored: no pending planting point.");
    return false;
  }

  if (isSeedDropperBusy()) {
    Serial.println("Task3 60-degree drop ignored: dropper is busy.");
    return false;
  }

  if (seedsRemaining <= 0) {
    Serial.println("Task3 60-degree drop ignored: no seeds remaining.");
    return false;
  }

  if (!rotateTask3AutoDropStepBlocking()) {
    Serial.println("Task3 60-degree drop failed: servo did not move.");
    return false;
  }

  seedsRemaining--;
  Serial.print("Task3 60-degree seed dropped. Remaining: ");
  Serial.println(seedsRemaining);

  completeTask3PlantingDrop();
  return true;
}

void completeTask3PlantingDrop() {
  if (task3PendingPlantValid && isValidGridPoint(task3PendingPlantPoint)) {
    task3Planted[task3PendingPlantPoint.row][task3PendingPlantPoint.col] = true;
    task3SeedsPlanted++;

    Serial.print("Task3 planted ");
    Serial.print(task3SeedsPlanted);
    Serial.print("/");
    Serial.print(TASK3_TARGET_SEEDS);
    Serial.print(" at ");
    Serial.println(uidAtPoint(task3PendingPlantPoint));
  }

  task3PendingPlantValid = false;
  setInvalidPoint(task3PendingPlantPoint);

  if (task3SeedsPlanted >= TASK3_TARGET_SEEDS) {
    task3ReturningToExit = true;
    Serial.println("Task3 planted 5 seeds. Returning to exit.");
  }

  continueTask3NavigationFromCurrent(false);
}

void printActiveRoute() {
  Serial.print("Route points: ");
  Serial.println(activeRouteLength);

  for (uint8_t i = 0; i < activeRouteLength; i++) {
    Serial.print(i);
    Serial.print(": r");
    Serial.print(activeRoute[i].row + 1);
    Serial.print(" c");
    Serial.print(activeRoute[i].col + 1);
    Serial.print(" ");
    Serial.println(uidAtPoint(activeRoute[i]));
  }
}

void resetQTRSensor() {
  stopRobot();
  resetQTRCalibration();

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    qtrRaw[i] = QTR_TIMEOUT_US;
    qtrCalibrated[i] = 0;
  }

  lastPosition = CENTER_POSITION;
  lastLineSeenMs = millis();
  lastError = 0;
  lineLostLatched = false;
  resetDetectionCounters();

  Serial.println("QTR sensor reset. Starting calibration.");
  runQTRCalibration();
  lastLineSeenMs = millis();
  Serial.println("QTR reset and calibration done.");
}

void printFixedActions() {
  Serial.println("Fixed Task 3 action sequence on any new RFID:");

  for (uint8_t i = 0; i < TASK3_FIXED_ACTION_COUNT; i++) {
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(routeActionName(TASK3_FIXED_ACTIONS[i]));
  }

  Serial.println("Each RFID pauses briefly, then consumes one action. The next RFID after the sequence stops.");
}

bool prepareSelectedRoute() {
  activeRouteIndex = 0;
  currentHeading = configuredStartHeading;

  selectedNavMode = NAV_MODE_TASK3_PLANTING;
  bool routeBuilt = prepareTask3PlantingMission();

  if (!routeBuilt) {
    Serial.println("Route build failed.");
    routeActive = false;
    return false;
  }

  routeActive = true;

  Serial.print("Navigation mode: ");
  Serial.println(navigationModeName(selectedNavMode));
  Serial.print("Start heading: ");
  Serial.println(headingName(currentHeading));
  printTask3PlantingStatus();
  printActiveRoute();
  return true;
}

void performRouteAction(RouteAction action) {
  Serial.print("Route action: ");
  Serial.println(routeActionName(action));

  if (action == ROUTE_ACTION_STRAIGHT || action == ROUTE_ACTION_IGNORE) {
    enterState(STATE_FOLLOW_LINE);
    return;
  }

  if (action == ROUTE_ACTION_LEFT) {
    pendingTurn = TURN_LEFT;
    enterState(STATE_PRE_TURN);
    return;
  }

  if (action == ROUTE_ACTION_RIGHT) {
    pendingTurn = TURN_RIGHT;
    enterState(STATE_PRE_TURN);
    return;
  }

  if (action == ROUTE_ACTION_UNSUPPORTED) {
    Serial.println("Route requires a U-turn here. Stopping for safety.");
    stopRobot();
    return;
  }

  stopRobot();
}

void executeRouteAction(RouteAction action) {
  pendingPausedAction = action;
  stopDrive();
  Serial.print("RFID pause before action: ");
  Serial.println(routeActionName(action));
  enterState(STATE_RFID_PAUSE);
}

void handleFixedTask3Rfid(const String &uid) {
  if (fixedActionIndex < TASK3_FIXED_ACTION_COUNT) {
    RouteAction action = TASK3_FIXED_ACTIONS[fixedActionIndex];

    Serial.print("Fixed action ");
    Serial.print(fixedActionIndex + 1);
    Serial.print("/");
    Serial.print(TASK3_FIXED_ACTION_COUNT);
    Serial.print(" at ");
    Serial.print(uid);
    Serial.print(": ");
    Serial.println(routeActionName(action));

    fixedActionIndex++;
    executeRouteAction(action);

    if (fixedActionIndex >= TASK3_FIXED_ACTION_COUNT) {
      Serial.println("Fixed action sequence complete. Next new RFID will stop.");
    }

    return;
  }

  routeActive = false;
  Serial.println("Fixed Task 3 sequence complete. Stopping at this RFID.");
  executeRouteAction(ROUTE_ACTION_STOP);
}

void handleTask3PlantingRfid(const String &uid) {
  GridPoint point;
  if (!findUidInMap(uid, point)) {
    Serial.print("Task3 RFID not found in map: ");
    Serial.println(uid);
    return;
  }

  task3CurrentPoint = point;
  task3CurrentPointValid = true;

  Serial.print("Task3 RFID r");
  Serial.print(point.row + 1);
  Serial.print(" c");
  Serial.print(point.col + 1);
  Serial.print(" ");
  Serial.println(uid);

  if (task3MissionComplete) {
    Serial.println("Task3 mission already complete. Ignoring RFID.");
    return;
  }

  if (task3SeedsPlanted >= TASK3_TARGET_SEEDS) {
    task3ReturningToExit = true;
  }

  if (task3ReturningToExit) {
    continueTask3NavigationFromCurrent(true);
    return;
  }

  if (isTask3PlantableCell(point)) {
    if (!isTask3CellPlanted(point)) {
      startTask3PlantAtCurrentPoint();
      return;
    }

    Serial.println("Task3 cell already planted. Replanning around it.");
  } else if (point.row >= TASK3_NAV_ROWS) {
    Serial.println("Task3 RFID is outside rows 1-4, navigation only.");
  } else {
    Serial.println("Task3 start/exit RFID, navigation only.");
  }

  continueTask3NavigationFromCurrent(true);
}

void handleRouteRfid(const String &uid) {
  if (!routeActive || robotState != STATE_FOLLOW_LINE || lineOnlyMode) {
    return;
  }

  handleTask3PlantingRfid(uid);
}

bool isI2cDevicePresent(byte address) {
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
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

  bool navigationCanUseUid = routeActive && robotState == STATE_FOLLOW_LINE && !lineOnlyMode;
  bool repeatedAction = uid == lastActionUid;
  if (navigationCanUseUid && !repeatedAction) {
    lastActionUid = uid;
    handleRouteRfid(uid);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void startRobot() {
  lineOnlyMode = false;
  if (!prepareSelectedRoute()) {
    stopRobot();
    return;
  }

  resetDetectionCounters();
  lastError = 0;
  lineLostLatched = false;
  lastActionUid = "";
  lastLineSeenMs = millis();
  enterState(STATE_FOLLOW_LINE);
  Serial.println("Robot started in RFID navigation mode.");
}

void startLineOnlyRobot() {
  lineOnlyMode = true;
  routeActive = false;
  resetDetectionCounters();
  lastError = 0;
  lineLostLatched = false;
  lastLineSeenMs = millis();
  enterState(STATE_FOLLOW_LINE);
  Serial.println("Robot started in line-only mode.");
}

void stopRobot() {
  routeActive = false;
  stopDrive();
  enterState(STATE_IDLE);
  Serial.println("Robot stopped.");
}

void followLine(const LineSnapshot &snapshot) {
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
      performRouteAction(pendingPausedAction);
    }
  } else if (robotState == STATE_PLANT_RFID_PAUSE) {
    stopDrive();

    if (elapsed >= RFID_PAUSE_MS) {
      enterState(STATE_PLANT_FORWARD);
    }
  } else if (robotState == STATE_PLANT_FORWARD) {
    setDriveSpeeds(PLANT_FORWARD_SPEED, PLANT_FORWARD_SPEED);

    if (elapsed >= PLANT_FORWARD_2CM_MS) {
      stopDrive();

      if (seedsRemaining <= 0) {
        Serial.println("Task3 out of seeds. Load one seed, then send: plant load");
        enterState(STATE_WAIT_FOR_SEED_LOAD);
      } else if (runTask3ServoDropAtPendingPoint()) {
        // runTask3ServoDropAtPendingPoint marks the cell and continues navigation.
      } else {
        Serial.println("Task3 60-degree seed drop could not run. Waiting for manual seed load.");
        enterState(STATE_WAIT_FOR_SEED_LOAD);
      }
    }
  } else if (robotState == STATE_PLANT_DROP) {
    stopDrive();

    if (!isSeedDropperBusy()) {
      completeTask3PlantingDrop();
    }
  } else if (robotState == STATE_WAIT_FOR_SEED_LOAD) {
    stopDrive();
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
    // Pivot turn only. Direction is swapped to match the current chassis wiring.
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
      Serial.println("Left turn safety timeout. Recovering line.");
      enterState(STATE_RECOVER_LINE);
    }
  } else if (robotState == STATE_TURN_RIGHT) {
    // Pivot turn only. Direction is swapped to match the current chassis wiring.
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
  } else {
    Serial.print(navigationModeName(selectedNavMode));
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
  Serial.print(" H=");
  Serial.print(hollowCrossFrames);
  Serial.print(" LA=");
  Serial.print(leftAngleFrames);
  Serial.print(" RA=");
  Serial.print(rightAngleFrames);
  Serial.print(" RFID=");
  if (latestRfidUid.length() > 0) {
    Serial.print(latestRfidUid);
  } else {
    Serial.print("none");
  }
  Serial.print(" route=");
  Serial.print(activeRouteIndex);
  Serial.print("/");
  Serial.print(activeRouteLength);
  Serial.print(" task3=");
  Serial.print(task3SeedsPlanted);
  Serial.print("/");
  Serial.print(TASK3_TARGET_SEEDS);
  Serial.print(" fixed=");
  Serial.print(fixedActionIndex);
  Serial.print("/");
  Serial.println(TASK3_FIXED_ACTION_COUNT);
}

void printCurrentSnapshot() {
  LineSnapshot snapshot = readLineSnapshot();
  printSnapshot(snapshot);
}

void printHelp() {
  Serial.println();
  Serial.println("--- Trial Run 2 Task 3 RFID navigation ---");
  Serial.println("g = start automatic RFID map planting Task 3");
  Serial.println("o = start line-only following, RFID actions disabled");
  Serial.println("s = stop");
  Serial.println("c = reset and recalibrate QTR");
  Serial.println("p = print one sensor snapshot");
  Serial.println("m = toggle live sensor monitor");
  Serial.println("route = print Task 3 planting status and active route");
  Serial.println("rfid = print latest RFID");
  Serial.println("task3 status/reset = print or reset RFID planting memory");
  Serial.println("heading north/east/south/west = set starting heading if robot is placed differently");
  Serial.println("load / plant load = full_v1 clockwise 60 deg load, also resumes Task3 when waiting");
  Serial.println("drop / plant drop = full_v1 counter-clockwise 60 deg manual drop");
  Serial.println("plant auto drop = run automatic gate open/close drop test");
  Serial.println("plant reset/zero/open/close/status = seed dropper controls");
  Serial.println("a/d = debug force left/right turn state");
  Serial.println("h/?/help = help");
  Serial.println("qtr / qtr reset / reset qtr = reset and recalibrate QTR");
  Serial.print("Current mode: ");
  Serial.println(navigationModeName(selectedNavMode));
  Serial.print("Start heading: ");
  Serial.println(headingName(configuredStartHeading));
  printTask3PlantingStatus();
  Serial.println();
}

bool setHeadingFromText(String text) {
  text.trim();
  text.toUpperCase();

  if (text == "N" || text == "NORTH") {
    configuredStartHeading = HEADING_NORTH;
  } else if (text == "E" || text == "EAST") {
    configuredStartHeading = HEADING_EAST;
  } else if (text == "S" || text == "SOUTH") {
    configuredStartHeading = HEADING_SOUTH;
  } else if (text == "W" || text == "WEST") {
    configuredStartHeading = HEADING_WEST;
  } else {
    return false;
  }

  Serial.print("Start heading set to ");
  Serial.println(headingName(configuredStartHeading));
  return true;
}

void processSerialCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  String lower = command;
  lower.toLowerCase();

  if (handlePlantCommand(lower)) {
    return;
  }

  if (command.length() == 1) {
    char key = command.charAt(0);

    if (key == '1') {
      selectedNavMode = NAV_MODE_TASK3_PLANTING;
      Serial.println("Task3 automatic planting is the only navigation mode.");
    } else if (key == '2') {
      Serial.println("User-selected route mode is disabled. Send g to start automatic Task3.");
    } else if (key == 'g' || key == 'G') {
      startRobot();
    } else if (key == 'o' || key == 'O') {
      startLineOnlyRobot();
    } else if (key == 's' || key == 'S') {
      stopRobot();
    } else if (key == 'c' || key == 'C') {
      resetQTRSensor();
      Serial.println("Ready after QTR reset.");
    } else if (key == 'p' || key == 'P') {
      printCurrentSnapshot();
    } else if (key == 'm' || key == 'M') {
      monitorSensors = !monitorSensors;
      Serial.print("Live monitor: ");
      Serial.println(monitorSensors ? "ON" : "OFF");
    } else if (key == 'a' || key == 'A') {
      lineOnlyMode = false;
      pendingTurn = TURN_LEFT;
      enterState(STATE_PRE_TURN);
    } else if (key == 'd' || key == 'D') {
      lineOnlyMode = false;
      pendingTurn = TURN_RIGHT;
      enterState(STATE_PRE_TURN);
    } else if (key == 'h' || key == 'H' || key == '?') {
      printHelp();
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
      printHelp();
    }

    return;
  }

  if (lower.startsWith("heading ")) {
    if (!setHeadingFromText(command.substring(8))) {
      Serial.println("Unknown heading. Use north/east/south/west.");
    }
  } else if (lower == "mode planting" || lower == "mode task3") {
    selectedNavMode = NAV_MODE_TASK3_PLANTING;
    Serial.println("Task3 automatic planting is the only navigation mode.");
  } else if (lower == "mode fixed") {
    Serial.println("Fixed route mode is disabled. Send g to start automatic Task3.");
  } else if (lower == "mode auto") {
    Serial.println("User-selected auto route mode is disabled. Send g to start automatic Task3.");
  } else if (lower.startsWith("start ") || lower.startsWith("goal ")) {
    Serial.println("User-selected start/goal is disabled. Task3 uses the fixed map start and exit.");
  } else if (lower == "route") {
    printTask3PlantingStatus();
    printActiveRoute();
  } else if (lower == "rfid") {
    Serial.print("Latest RFID: ");
    if (latestRfidUid.length() > 0) {
      Serial.println(latestRfidUid);
    } else {
      Serial.println("(none)");
    }
  } else if (lower == "help") {
    printHelp();
  } else if (lower == "task3 status") {
    printTask3PlantingStatus();
  } else if (lower == "task3 reset") {
    if (robotState != STATE_IDLE) {
      Serial.println("Stop Task 3 before resetting planting memory.");
    } else {
      resetTask3PlantingMemory();
      printTask3PlantingStatus();
    }
  } else if (lower == "qtr" || lower == "qtr reset" || lower == "reset qtr") {
    resetQTRSensor();
    Serial.println("Ready after QTR reset.");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    printHelp();
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char input = Serial.read();

    if (input == '\r' || input == '\n') {
      if (serialLine.length() > 0) {
        processSerialCommand(serialLine);
      }
      serialLine = "";
    } else if (serialLine.length() == 0 &&
               (input == 'g' || input == 'G' ||
                input == 'o' || input == 'O' ||
                input == 's' || input == 'S' ||
                input == 'c' || input == 'C' ||
                input == 'p' || input == 'P' ||
                input == 'm' || input == 'M' ||
                input == 'a' || input == 'A' ||
                input == 'd' || input == 'D' ||
                input == '1' || input == '2' ||
                input == '?')) {
      String immediateCommand = "";
      immediateCommand += input;
      processSerialCommand(immediateCommand);
    } else {
      serialLine += input;
      if (serialLine.length() > 64) {
        serialLine = "";
        Serial.println("Command too long; cleared.");
      }
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

  Serial.println("Trial Run 2 Task 3 RFID navigation starting...");
  Serial.println("Encoders are not used in this test.");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  setupSeedDropper();

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

  stopDrive();
  enterState(STATE_IDLE);
  runQTRCalibration();

  lastLineSeenMs = millis();
  updateStatusLED();
  printHelp();
  Serial.println("Send g or press D32 to start automatic Task3 planting.");
}

void loop() {
  handleSerialCommands();
  handleButtons();
  updateSeedDropper();
  pollRFID();
  updateStateMachine();
  updateStatusLED();
  delay(10);
}

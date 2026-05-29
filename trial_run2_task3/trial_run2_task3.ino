#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>

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

const int BASE_SPEED = 250;
const int CURVE_BASE_SPEED = 250;
const int MAX_SPEED = 650;
const int MAX_TURN = 650;
const int SEARCH_SPEED = 220;

const int TURN_SPEED = 550;
const int TURN_BOOST_SPEED = 550;
const int PRE_TURN_SPEED = 600;
const int BRIDGE_SPEED = 600;

const float KP = 0.12;
const float KD = 0.80;

int lastError = 0;

// -------------------- Buttons and LEDs --------------------
const int START_BUTTON_PIN = 32;
const int STOP_BUTTON_PIN = 33;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

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
NavigationMode selectedNavMode = NAV_MODE_TASK3_FIXED;
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
void printHelp();
void printSnapshot(const LineSnapshot &snapshot);
void enterState(RobotState newState);

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
  return mode == NAV_MODE_TASK3_FIXED ? "TASK3_FIXED" : "AUTO_ROUTE";
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
  bool routeBuilt = false;

  if (selectedNavMode == NAV_MODE_TASK3_FIXED) {
    activeRouteLength = 0;
    activeRouteIndex = 0;
    fixedActionIndex = 0;
    routeBuilt = true;
  } else {
    if (autoStartUid.length() == 0 && latestRfidUid.length() > 0) {
      autoStartUid = latestRfidUid;
      Serial.print("Auto start set from latest RFID: ");
      Serial.println(autoStartUid);
    }

    if (autoStartUid.length() == 0 || autoGoalUid.length() == 0) {
      Serial.println("Auto route needs start and goal UIDs. Use: start <uid> and goal <uid>");
      return false;
    }

    routeBuilt = buildAutoRoute();
  }

  if (!routeBuilt) {
    Serial.println("Route build failed.");
    routeActive = false;
    return false;
  }

  activeRouteIndex = 0;
  currentHeading = configuredStartHeading;
  routeActive = true;

  Serial.print("Navigation mode: ");
  Serial.println(navigationModeName(selectedNavMode));
  Serial.print("Start heading: ");
  Serial.println(headingName(currentHeading));
  if (selectedNavMode == NAV_MODE_TASK3_FIXED) {
    printFixedActions();
  } else {
    printActiveRoute();
  }
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

void handleRouteRfid(const String &uid) {
  if (!routeActive || robotState != STATE_FOLLOW_LINE || lineOnlyMode) {
    return;
  }

  if (selectedNavMode == NAV_MODE_TASK3_FIXED) {
    handleFixedTask3Rfid(uid);
    return;
  }

  GridPoint point;
  if (!findUidInMap(uid, point)) {
    Serial.print("RFID not found in map: ");
    Serial.println(uid);
    return;
  }

  int routeMatch = -1;
  for (uint8_t i = activeRouteIndex; i < activeRouteLength; i++) {
    if (samePoint(point, activeRoute[i])) {
      routeMatch = i;
      break;
    }
  }

  if (routeMatch < 0) {
    Serial.print("RFID is not on active route: ");
    Serial.println(uid);
    return;
  }

  activeRouteIndex = routeMatch;

  Serial.print("RFID route point ");
  Serial.print(activeRouteIndex);
  Serial.print("/");
  Serial.print(activeRouteLength - 1);
  Serial.print(": ");
  Serial.println(uid);

  if (activeRouteIndex >= activeRouteLength - 1) {
    routeActive = false;
    executeRouteAction(ROUTE_ACTION_STOP);
    return;
  }

  Heading nextHeading = headingBetween(activeRoute[activeRouteIndex], activeRoute[activeRouteIndex + 1]);
  RouteAction action = actionFromHeading(currentHeading, nextHeading);
  currentHeading = nextHeading;
  activeRouteIndex++;

  executeRouteAction(action);
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
  Serial.println("1 = select fixed Task 3 sequence: straight, straight, right, left, straight");
  Serial.println("2 = select user start/goal auto route");
  Serial.println("start <uid> = set auto-route start RFID");
  Serial.println("goal <uid> = set auto-route goal RFID");
  Serial.println("heading north/east/south/west = set starting heading");
  Serial.println("g = start selected navigation mode");
  Serial.println("o = start line-only following, RFID actions disabled");
  Serial.println("s = stop");
  Serial.println("c = reset and recalibrate QTR");
  Serial.println("p = print one sensor snapshot");
  Serial.println("m = toggle live sensor monitor");
  Serial.println("route = print active route or fixed action sequence");
  Serial.println("rfid = print latest RFID");
  Serial.println("a/d = debug force left/right turn state");
  Serial.println("h/?/help = help");
  Serial.println("qtr / qtr reset / reset qtr = reset and recalibrate QTR");
  Serial.print("Current mode: ");
  Serial.println(navigationModeName(selectedNavMode));
  Serial.print("Start heading: ");
  Serial.println(headingName(configuredStartHeading));
  Serial.print("Auto start: ");
  if (autoStartUid.length() > 0) {
    Serial.println(autoStartUid);
  } else {
    Serial.println("(not set)");
  }
  Serial.print("Auto goal: ");
  if (autoGoalUid.length() > 0) {
    Serial.println(autoGoalUid);
  } else {
    Serial.println("(not set)");
  }
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

  if (command.length() == 1) {
    char key = command.charAt(0);

    if (key == '1') {
      selectedNavMode = NAV_MODE_TASK3_FIXED;
      Serial.println("Selected mode: fixed Task 3 route.");
    } else if (key == '2') {
      selectedNavMode = NAV_MODE_AUTO_ROUTE;
      Serial.println("Selected mode: user start/goal auto route.");
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

  if (lower.startsWith("start ")) {
    autoStartUid = normalizeUid(command.substring(6));
    Serial.print("Auto start UID set to ");
    Serial.println(autoStartUid);
  } else if (lower.startsWith("goal ")) {
    autoGoalUid = normalizeUid(command.substring(5));
    Serial.print("Auto goal UID set to ");
    Serial.println(autoGoalUid);
  } else if (lower.startsWith("heading ")) {
    if (!setHeadingFromText(command.substring(8))) {
      Serial.println("Unknown heading. Use north/east/south/west.");
    }
  } else if (lower == "mode fixed") {
    selectedNavMode = NAV_MODE_TASK3_FIXED;
    Serial.println("Selected mode: fixed Task 3 route.");
  } else if (lower == "mode auto") {
    selectedNavMode = NAV_MODE_AUTO_ROUTE;
    Serial.println("Selected mode: user start/goal auto route.");
  } else if (lower == "route") {
    if (selectedNavMode == NAV_MODE_TASK3_FIXED) {
      printFixedActions();
    } else {
      printActiveRoute();
    }
  } else if (lower == "rfid") {
    Serial.print("Latest RFID: ");
    if (latestRfidUid.length() > 0) {
      Serial.println(latestRfidUid);
    } else {
      Serial.println("(none)");
    }
  } else if (lower == "help") {
    printHelp();
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
      processSerialCommand(serialLine);
      serialLine = "";
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
  Serial.println("Choose 1 or 2, set heading/start/goal if needed, then send g or press D32.");
}

void loop() {
  handleSerialCommands();
  handleButtons();
  pollRFID();
  updateStateMachine();
  updateStatusLED();
  delay(10);
}

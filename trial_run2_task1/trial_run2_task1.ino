#include <Wire.h>
#include <Motoron.h>

// =====================================================
// Trial Run 2 - Task 1 Standard Line Tracking
// Extracted from sensor_motor_integration_test.
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
const int BLACK_SENSOR_THRESHOLD = 550;
const unsigned long LOST_LINE_STOP_MS = 650;
const unsigned long RECOVER_MAX_MS = 900;

// -------------------- Motoron motor controllers --------------------
MotoronI2C leftController;   // 0x10: left side
MotoronI2C rightController;  // 0x11: right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

// These signs match the tested sensor_motor_integration wiring.
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;

const int BASE_SPEED = 480;
const int CURVE_BASE_SPEED = 320;
const int SHARP_CURVE_BASE_SPEED = 280;
const int MAX_SPEED = 650;
const int MAX_TURN = 650;
const int SEARCH_SPEED = 220;
const int CURVE_MIN_TURN = 390;
const int CURVE_EXTRA_TURN = 210;
const int CURVE_SEARCH_INNER_SPEED = -180;
const int CURVE_SEARCH_OUTER_SPEED = 440;

const int CURVE_SIDE_SUM_MARGIN = 350;
const int CURVE_POSITION_MARGIN = 350;
const uint8_t CURVE_DETECT_FRAMES = 2;

const float KP = 0.15;
const float KD = 0.9;
const float CURVE_KP = 0.24;
const float CURVE_KD = 1.05;

int lastError = 0;

// -------------------- Buttons and LEDs --------------------
const int START_BUTTON_PIN = 32;
const int STOP_BUTTON_PIN = 53;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

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
  STATE_RECOVER_LINE
};

enum TurnDirection {
  TURN_NONE,
  TURN_LEFT,
  TURN_RIGHT
};

RobotState robotState = STATE_IDLE;
TurnDirection activeCurve = TURN_NONE;
TurnDirection lastCurveDirection = TURN_NONE;

bool monitorSensors = false;
unsigned long stateStartedMs = 0;
unsigned long lastMonitorPrintMs = 0;
bool lineLostLatched = false;
uint8_t leftCurveFrames = 0;
uint8_t rightCurveFrames = 0;

void stopDrive();
void printHelp();
void printSnapshot(const LineSnapshot &snapshot);
void enterState(RobotState newState);
void incrementOrReset(uint8_t &counter, bool condition);
void resetCurveDetection();
TurnDirection updateCurveDetection(const LineSnapshot &snapshot);
void setCurveSearchSpeeds(TurnDirection direction);
void followLine(const LineSnapshot &snapshot);
bool centerLineFound(const LineSnapshot &snapshot);

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

void stopDrive() {
  setDriveSpeeds(0, 0);
}

// =====================================================
// Task 1 line-following state machine
// =====================================================

const char *stateName(RobotState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_FOLLOW_LINE:
      return "FOLLOW_LINE";
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

void resetCurveDetection() {
  activeCurve = TURN_NONE;
  lastCurveDirection = TURN_NONE;
  leftCurveFrames = 0;
  rightCurveFrames = 0;
}

TurnDirection updateCurveDetection(const LineSnapshot &snapshot) {
  bool leftCurveCandidate =
    snapshot.linePresent &&
    snapshot.leftWing &&
    snapshot.leftSum > snapshot.rightSum + CURVE_SIDE_SUM_MARGIN &&
    snapshot.position < CENTER_POSITION - CURVE_POSITION_MARGIN;

  bool rightCurveCandidate =
    snapshot.linePresent &&
    snapshot.rightWing &&
    snapshot.rightSum > snapshot.leftSum + CURVE_SIDE_SUM_MARGIN &&
    snapshot.position > CENTER_POSITION + CURVE_POSITION_MARGIN;

  if (leftCurveCandidate && rightCurveCandidate) {
    leftCurveCandidate = false;
    rightCurveCandidate = false;
  }

  incrementOrReset(leftCurveFrames, leftCurveCandidate);
  incrementOrReset(rightCurveFrames, rightCurveCandidate);

  if (leftCurveFrames >= CURVE_DETECT_FRAMES) {
    activeCurve = TURN_LEFT;
    lastCurveDirection = TURN_LEFT;
  } else if (rightCurveFrames >= CURVE_DETECT_FRAMES) {
    activeCurve = TURN_RIGHT;
    lastCurveDirection = TURN_RIGHT;
  } else if (activeCurve == TURN_LEFT) {
    bool keepCurve =
      snapshot.linePresent &&
      snapshot.leftSum > snapshot.rightSum + 180 &&
      snapshot.position < CENTER_POSITION + CURVE_POSITION_MARGIN;

    if (!keepCurve) {
      activeCurve = TURN_NONE;
    }
  } else if (activeCurve == TURN_RIGHT) {
    bool keepCurve =
      snapshot.linePresent &&
      snapshot.rightSum > snapshot.leftSum + 180 &&
      snapshot.position > CENTER_POSITION - CURVE_POSITION_MARGIN;

    if (!keepCurve) {
      activeCurve = TURN_NONE;
    }
  }

  return activeCurve;
}

void setCurveSearchSpeeds(TurnDirection direction) {
  if (direction == TURN_LEFT) {
    setDriveSpeeds(CURVE_SEARCH_INNER_SPEED, CURVE_SEARCH_OUTER_SPEED);
  } else if (direction == TURN_RIGHT) {
    setDriveSpeeds(CURVE_SEARCH_OUTER_SPEED, CURVE_SEARCH_INNER_SPEED);
  } else if (lastPosition < CENTER_POSITION) {
    setDriveSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
  } else {
    setDriveSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
  }
}

void startRobot() {
  resetCurveDetection();
  lastError = 0;
  lineLostLatched = false;
  lastLineSeenMs = millis();
  enterState(STATE_FOLLOW_LINE);
  Serial.println("Trial Run 2 Task 1 standard line tracking started.");
}

void stopRobot() {
  stopDrive();
  resetCurveDetection();
  enterState(STATE_IDLE);
  Serial.println("Robot stopped.");
}

void followLine(const LineSnapshot &snapshot) {
  TurnDirection curveDirection = updateCurveDetection(snapshot);
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

    TurnDirection searchDirection = lastCurveDirection;

    if (searchDirection == TURN_NONE) {
      searchDirection = error < 0 ? TURN_LEFT : TURN_RIGHT;
    }

    setCurveSearchSpeeds(searchDirection);
    return;
  } else {
    bool edgeCurve = abs(error) > 2200;

    if (curveDirection == TURN_NONE && edgeCurve) {
      curveDirection = error < 0 ? TURN_LEFT : TURN_RIGHT;
      lastCurveDirection = curveDirection;
    }

    int base = BASE_SPEED;
    float kp = KP;
    float kd = KD;

    if (curveDirection != TURN_NONE) {
      base = edgeCurve ? SHARP_CURVE_BASE_SPEED : CURVE_BASE_SPEED;
      kp = CURVE_KP;
      kd = CURVE_KD;
    }

    int turn = (int)(kp * error + kd * derivative);

    if (curveDirection == TURN_LEFT) {
      turn -= CURVE_EXTRA_TURN;

      if (turn > -CURVE_MIN_TURN) {
        turn = -CURVE_MIN_TURN;
      }
    } else if (curveDirection == TURN_RIGHT) {
      turn += CURVE_EXTRA_TURN;

      if (turn < CURVE_MIN_TURN) {
        turn = CURVE_MIN_TURN;
      }
    }

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
  } else if (robotState == STATE_RECOVER_LINE) {
    if (snapshot.linePresent && centerLineFound(snapshot)) {
      lastError = snapshot.position - CENTER_POSITION;
      enterState(STATE_FOLLOW_LINE);
      return;
    }

    TurnDirection searchDirection = lastCurveDirection;

    if (searchDirection == TURN_NONE) {
      searchDirection = lastPosition < CENTER_POSITION ? TURN_LEFT : TURN_RIGHT;
    }

    setCurveSearchSpeeds(searchDirection);

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
  Serial.print(" black=");
  Serial.print(snapshot.blackCount);
  Serial.print(" L/M/R=");
  Serial.print(snapshot.leftCount);
  Serial.print('/');
  Serial.print(snapshot.midCount);
  Serial.print('/');
  Serial.print(snapshot.rightCount);
  Serial.print(" sums=");
  Serial.print(snapshot.leftSum);
  Serial.print('/');
  Serial.print(snapshot.midSum);
  Serial.print('/');
  Serial.print(snapshot.rightSum);
  Serial.print(" wing=");
  Serial.print(snapshot.leftWing ? 'L' : '-');
  Serial.print(snapshot.rightWing ? 'R' : '-');
  Serial.print(" curve=");
  if (activeCurve == TURN_LEFT) {
    Serial.print('L');
  } else if (activeCurve == TURN_RIGHT) {
    Serial.print('R');
  } else {
    Serial.print('-');
  }
  Serial.print(" center=");
  Serial.println(snapshot.centerPresent ? "Y" : "N");
}

void printCurrentSnapshot() {
  LineSnapshot snapshot = readLineSnapshot();
  printSnapshot(snapshot);
}

void printHelp() {
  Serial.println();
  Serial.println("--- Trial Run 2 Task 1 Standard Line Tracking ---");
  Serial.println("g = start Task 1 line tracking");
  Serial.println("s = stop");
  Serial.println("c = recalibrate QTR");
  Serial.println("p = print one sensor snapshot");
  Serial.println("m = toggle live sensor monitor");
  Serial.println("Curve assist: stronger differential steering on closed bends");
  Serial.println("h/? = help");
  Serial.println("D32 starts, D53 kills/stops.");
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

  Serial.println("Trial Run 2 Task 1 standard line tracking starting...");
  Serial.println("Encoders are not used in this task.");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  pinMode(QTR_EMITTER_PIN, OUTPUT);
  digitalWrite(QTR_EMITTER_PIN, HIGH);

  Serial.println("Initializing Motorons on Wire1...");
  Wire1.begin();
  delay(100);

  leftController.setBus(&Wire1);
  setupMotoron(&leftController, "Left 0x10");

  rightController.setBus(&Wire1);
  rightController.setAddress(17);
  setupMotoron(&rightController, "Right 0x11");

  stopDrive();
  enterState(STATE_IDLE);
  runQTRCalibration();

  lastLineSeenMs = millis();
  updateStatusLED();
  printHelp();
  Serial.println("Place robot on the line. Send g or press D32 to start Task 1.");
}

void loop() {
  handleSerialCommands();
  handleButtons();
  updateStateMachine();
  updateStatusLED();
  delay(10);
}

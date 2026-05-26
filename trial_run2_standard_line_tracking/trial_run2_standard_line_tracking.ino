#include <Wire.h>
#include <Motoron.h>

// =====================================================
// Trial Run 2 - Standard Line Tracking
// Based on the QTR sensor calls used in full_v1/full_v1.ino.
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

// Calibrated values are 0 = pale floor, 1000 = black tape.
const long LINE_PRESENT_MIN = 250;
const int BLACK_SENSOR_THRESHOLD = 550;
const unsigned long LOST_LINE_STOP_MS = 900;

// -------------------- Motoron motor controllers --------------------
MotoronI2C leftController;   // 0x10: left side
MotoronI2C rightController;  // 0x11: right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

// Positive line-following speed should move the robot physically forward.
// If the robot drives backward, flip both signs together.
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;

const int BASE_SPEED = 580;
const int MAX_SPEED = 800;
const int SEARCH_SPEED = 460;
const int MAX_TURN = 620;

// Tune these first if the robot oscillates or cuts corners.
const float KP = 0.10;
const float KD = 0.65;

int lastError = 0;

// -------------------- Start/stop controls from full_v1 --------------------
const int START_BUTTON_PIN = 32;
const int STOP_BUTTON_PIN = 33;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

bool robotEnabled = false;
bool lineLostLatched = false;

void stopDrive();
void printQTRDebug();

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
  resetQTRCalibration();
  stopDrive();

  Serial.println("QTR calibration: 5 seconds.");
  Serial.println("Move the sensor bar over both pale floor and black tape.");

  unsigned long startMs = millis();

  while (millis() - startMs < QTR_CALIBRATION_MS) {
    calibrateQTROnce();

    bool blinkOn = ((millis() / 200) % 2) == 0;
    digitalWrite(LED_RED_PIN, blinkOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, blinkOn ? LOW : HIGH);

    delay(20);
  }

  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  Serial.println("QTR calibration done.");
}

int readLinePosition(long &sumOut, int &blackCountOut) {
  long weightedSum = 0;
  long sum = 0;
  int blackCount = 0;

  readQTRCalibrated();

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    int value = qtrCalibrated[i];

    weightedSum += (long)value * (i * 1000);
    sum += value;

    if (value > BLACK_SENSOR_THRESHOLD) {
      blackCount++;
    }
  }

  sumOut = sum;
  blackCountOut = blackCount;

  if (sum > LINE_PRESENT_MIN) {
    lastPosition = weightedSum / sum;
    lastLineSeenMs = millis();
    lineLostLatched = false;
  } else if (lastPosition < CENTER_POSITION) {
    lastPosition = 0;
  } else {
    lastPosition = (QTR_SENSOR_COUNT - 1) * 1000;
  }

  return lastPosition;
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
    controller->setMaxAcceleration(channel, 220);
    controller->setMaxDeceleration(channel, 320);
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
// User controls and debug
// =====================================================

void startRobot() {
  robotEnabled = true;
  lineLostLatched = false;
  lastError = 0;
  lastLineSeenMs = millis();
  Serial.println("Robot enabled.");
}

void stopRobot() {
  robotEnabled = false;
  stopDrive();
  Serial.println("Robot stopped.");
}

void updateStatusLED() {
  digitalWrite(LED_RED_PIN, robotEnabled ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, robotEnabled ? HIGH : LOW);
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

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  if (command == 'g' || command == 'G') {
    startRobot();
  } else if (command == 's' || command == 'S') {
    stopRobot();
  } else if (command == 'c' || command == 'C') {
    runQTRCalibration();
    Serial.println("Ready. Send g or press D32 to start.");
  } else if (command == 'p' || command == 'P') {
    printQTRDebug();
  } else if (command != '\n' && command != '\r') {
    Serial.println("Commands: g=start, s=stop, c=calibrate, p=print QTR.");
  }
}

void printQTRDebug() {
  long sum = 0;
  int blackCount = 0;
  int position = readLinePosition(sum, blackCount);

  Serial.print("QTR: ");

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    Serial.print(qtrCalibrated[i]);

    if (i < QTR_SENSOR_COUNT - 1) {
      Serial.print('\t');
    }
  }

  Serial.print(" | pos=");
  Serial.print(position);
  Serial.print(" sum=");
  Serial.print(sum);
  Serial.print(" black=");
  Serial.println(blackCount);
}

void printDriveDebug(int position, long sum, int blackCount, int leftSpeed, int rightSpeed) {
  static unsigned long lastPrintMs = 0;

  if (millis() - lastPrintMs < 200) {
    return;
  }

  lastPrintMs = millis();

  Serial.print("pos=");
  Serial.print(position);
  Serial.print(" sum=");
  Serial.print(sum);
  Serial.print(" black=");
  Serial.print(blackCount);
  Serial.print(" error=");
  Serial.print(position - CENTER_POSITION);
  Serial.print(" L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();

  while (!Serial && millis() - startWait < 3000) {
  }

  Serial.println("Trial Run 2 standard line tracking starting...");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  updateStatusLED();

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
  runQTRCalibration();

  lastLineSeenMs = millis();

  Serial.println("Ready.");
  Serial.println("Place robot on the start line, then send g or press D32.");
  Serial.println("Send s or press D33 to stop. Send c to recalibrate.");
}

void loop() {
  handleSerialCommands();
  handleButtons();
  updateStatusLED();

  if (!robotEnabled) {
    stopDrive();
    delay(20);
    return;
  }

  long sum = 0;
  int blackCount = 0;
  int position = readLinePosition(sum, blackCount);
  int error = position - CENTER_POSITION;
  int derivative = error - lastError;
  lastError = error;

  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (sum <= LINE_PRESENT_MIN) {
    if (millis() - lastLineSeenMs > LOST_LINE_STOP_MS) {
      leftSpeed = 0;
      rightSpeed = 0;

      if (!lineLostLatched) {
        lineLostLatched = true;
        Serial.println("Line lost for too long. Stopping for safety.");
      }
    } else if (error < 0) {
      leftSpeed = -SEARCH_SPEED;
      rightSpeed = SEARCH_SPEED;
    } else {
      leftSpeed = SEARCH_SPEED;
      rightSpeed = -SEARCH_SPEED;
    }
  } else {
    int turn = (int)(KP * error + KD * derivative);
    turn = constrain(turn, -MAX_TURN, MAX_TURN);

    leftSpeed = constrain(BASE_SPEED + turn, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(BASE_SPEED - turn, -MAX_SPEED, MAX_SPEED);
  }

  setDriveSpeeds(leftSpeed, rightSpeed);
  printDriveDebug(position, sum, blackCount, leftSpeed, rightSpeed);

  delay(10);
}

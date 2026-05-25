#include <Servo.h>

// Standalone test pin. Change this before merging with line_following because D9 is an IR sensor pin there.
// Power the servo from a separate 5-6V supply and share GND.
const byte SEED_SERVO_PIN = 9;
// DSR005 / DS-R005 300 degree servo values verified by bench test.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_RANGE_DEGREES = 300;
const int GATE_CLOSED_DEGREES = 20;
const int GATE_OPEN_DEGREES = 90;
const int INITIAL_SEED_COUNT = 5;

const unsigned long GATE_OPEN_MS = 450;
const unsigned long GATE_CLOSE_SETTLE_MS = 250;
const int MOVE_STEP_DEGREES = 2;
const unsigned long MOVE_STEP_MS = 8;

enum DropperState {
  DROPPER_IDLE,
  DROPPER_OPENING,
  DROPPER_OPEN,
  DROPPER_CLOSING,
  DROPPER_WAIT_AFTER_CLOSE
};

Servo seedServo;
DropperState dropperState = DROPPER_IDLE;
unsigned long waitStartedMs = 0;
unsigned long lastMoveStepMs = 0;
int currentAngle = GATE_CLOSED_DEGREES;
int targetAngle = GATE_CLOSED_DEGREES;
int seedsRemaining = INITIAL_SEED_COUNT;

int angleToPulse(int angle) {
  angle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  long pulse = SERVO_MIN_US + (long)(SERVO_MAX_US - SERVO_MIN_US) * angle / SERVO_RANGE_DEGREES;
  return (int)pulse;
}

void setServoAngle(int angle) {
  currentAngle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  seedServo.writeMicroseconds(angleToPulse(currentAngle));
}

void startServoMove(int angle) {
  targetAngle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  lastMoveStepMs = millis();
}

bool updateServoMove() {
  if (currentAngle == targetAngle) {
    return true;
  }

  unsigned long nowMs = millis();

  if (nowMs - lastMoveStepMs < MOVE_STEP_MS) {
    return false;
  }

  lastMoveStepMs = nowMs;

  if (targetAngle > currentAngle) {
    setServoAngle(min(currentAngle + MOVE_STEP_DEGREES, targetAngle));
  } else {
    setServoAngle(max(currentAngle - MOVE_STEP_DEGREES, targetAngle));
  }

  return currentAngle == targetAngle;
}

bool isSeedDropperBusy() {
  return dropperState != DROPPER_IDLE;
}

void setupSeedDropper() {
  seedServo.attach(SEED_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setServoAngle(GATE_CLOSED_DEGREES);
}

bool startSeedDrop() {
  if (isSeedDropperBusy() || seedsRemaining <= 0) {
    return false;
  }

  startServoMove(GATE_OPEN_DEGREES);
  dropperState = DROPPER_OPENING;
  Serial.println("Seed drop started.");
  return true;
}

void updateSeedDropper() {
  if (dropperState == DROPPER_IDLE) {
    return;
  }

  unsigned long elapsedMs = millis() - waitStartedMs;

  if (dropperState == DROPPER_OPENING && updateServoMove()) {
    dropperState = DROPPER_OPEN;
    waitStartedMs = millis();
  } else if (dropperState == DROPPER_OPEN && elapsedMs >= GATE_OPEN_MS) {
    startServoMove(GATE_CLOSED_DEGREES);
    dropperState = DROPPER_CLOSING;
  } else if (dropperState == DROPPER_CLOSING && updateServoMove()) {
    dropperState = DROPPER_WAIT_AFTER_CLOSE;
    waitStartedMs = millis();
  } else if (dropperState == DROPPER_WAIT_AFTER_CLOSE && elapsedMs >= GATE_CLOSE_SETTLE_MS) {
    seedsRemaining--;
    dropperState = DROPPER_IDLE;

    Serial.print("Seed dropped. Remaining: ");
    Serial.println(seedsRemaining);
  }
}

void resetSeedDropper() {
  if (isSeedDropperBusy()) {
    Serial.println("Wait for the current drop to finish before reset.");
    return;
  }

  seedsRemaining = INITIAL_SEED_COUNT;
  setServoAngle(GATE_CLOSED_DEGREES);
  targetAngle = GATE_CLOSED_DEGREES;
  Serial.println("Seed count reset.");
}

void printSeedDropperStatus() {
  Serial.print("Seeds remaining: ");
  Serial.print(seedsRemaining);
  Serial.print(" | Busy: ");
  Serial.print(isSeedDropperBusy() ? "yes" : "no");
  Serial.print(" | Angle: ");
  Serial.println(currentAngle);
}

void printHelp() {
  Serial.println("Commands: d=drop one seed, r=reset count, s=status, h=help");
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  if (command == '\n' || command == '\r' || command == ' ') {
    return;
  }

  if (command == 'd') {
    if (!startSeedDrop()) {
      Serial.println(seedsRemaining <= 0 ? "No seeds remaining." : "Dropper is busy.");
    }
  } else if (command == 'r') {
    resetSeedDropper();
  } else if (command == 's') {
    printSeedDropperStatus();
  } else if (command == 'h') {
    printHelp();
  } else {
    Serial.println("Unknown command. Send h for help.");
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long startWaitMs = millis();
  while (!Serial && millis() - startWaitMs < 3000) {}

  setupSeedDropper();

  Serial.println("Seed dropper test ready.");
  printHelp();
  printSeedDropperStatus();
}

void loop() {
  handleSerialCommands();
  updateSeedDropper();
}

#include <Servo.h>

// =====================================================
// Servo 舵机部分
// =====================================================

Servo beadServo;

const byte SERVO_PIN = 47;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_RANGE_DEG = 300;

const int STEP_ANGLE = 60;
int currentAngle = 0;

const bool CLOCKWISE_IS_INCREASING = true;

const int MOVE_STEP_DEG = 2;
const int MOVE_DELAY_MS = 8;

int angleToPulse(int angle) {
  angle = constrain(angle, 0, SERVO_RANGE_DEG);

  long pulse = SERVO_MIN_US +
               (long)(SERVO_MAX_US - SERVO_MIN_US) * angle / SERVO_RANGE_DEG;

  return (int)pulse;
}

void setServoAngle(int angle) {
  angle = constrain(angle, 0, SERVO_RANGE_DEG);
  beadServo.writeMicroseconds(angleToPulse(angle));
}

void moveToAngle(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, SERVO_RANGE_DEG);

  if (targetAngle == currentAngle) {
    Serial.print("Already at ");
    Serial.print(currentAngle);
    Serial.println(" degrees");
    return;
  }

  if (targetAngle > currentAngle) {
    for (int a = currentAngle; a <= targetAngle; a += MOVE_STEP_DEG) {
      setServoAngle(a);
      delay(MOVE_DELAY_MS);
    }
  } else {
    for (int a = currentAngle; a >= targetAngle; a -= MOVE_STEP_DEG) {
      setServoAngle(a);
      delay(MOVE_DELAY_MS);
    }
  }

  setServoAngle(targetAngle);
  currentAngle = targetAngle;

  Serial.print("Current angle: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");

  delay(100);
}

void rotateClockwise60() {
  int targetAngle;

  if (CLOCKWISE_IS_INCREASING) {
    targetAngle = currentAngle + STEP_ANGLE;
  } else {
    targetAngle = currentAngle - STEP_ANGLE;
  }

  moveToAngle(targetAngle);
}

void rotateCounterClockwise60() {
  int targetAngle;

  if (CLOCKWISE_IS_INCREASING) {
    targetAngle = currentAngle - STEP_ANGLE;
  } else {
    targetAngle = currentAngle + STEP_ANGLE;
  }

  moveToAngle(targetAngle);
}

void resetToZero() {
  moveToAngle(0);
}

// =====================================================
// Button + RGB LED 复活按钮部分
// =====================================================

const int buttonPin1 = 32;
const int buttonPin2 = 33;
const int ledRed = 34;
const int ledGreen = 35;

void updateReviveButtonLED() {
  int state1 = digitalRead(buttonPin1);
  int state2 = digitalRead(buttonPin2);

  if (state1 == LOW || state2 == LOW) {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, HIGH);
  } else {
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, LOW);
  }
}

// =====================================================
// HC-SR04 左右距离传感器部分
// =====================================================

const int trigLeft  = 41;
const int echoLeft  = 40;

const int trigRight = 39;
const int echoRight = 38;

const float SOUND_SPEED_CM_PER_US = 0.0343;

const float MIN_VALID_CM = 3.0;
const float MAX_VALID_CM = 200.0;

const int DIST_SAMPLES = 3;
const float MAX_SPREAD_CM = 20.0;

unsigned long timeoutFromDistance(float cm) {
  return (unsigned long)(cm * 2.0 / SOUND_SPEED_CM_PER_US) + 800;
}

float readOnceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long timeoutUs = timeoutFromDistance(MAX_VALID_CM);
  unsigned long duration = pulseIn(echoPin, HIGH, timeoutUs);

  if (duration == 0) {
    return -1.0;
  }

  float distance = duration * SOUND_SPEED_CM_PER_US / 2.0;

  if (distance < MIN_VALID_CM || distance > MAX_VALID_CM) {
    return -1.0;
  }

  return distance;
}

void sortFloatArray(float a[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (a[j] < a[i]) {
        float t = a[i];
        a[i] = a[j];
        a[j] = t;
      }
    }
  }
}

float readMedianCM(int trigPin, int echoPin) {
  float values[DIST_SAMPLES];
  int count = 0;

  for (int i = 0; i < DIST_SAMPLES; i++) {
    float d = readOnceCM(trigPin, echoPin);

    if (d > 0) {
      values[count] = d;
      count++;
    }

    delay(8);
  }

  if (count == 0) {
    return -1.0;
  }

  sortFloatArray(values, count);

  if (count >= 2) {
    float spread = values[count - 1] - values[0];

    if (spread > MAX_SPREAD_CM) {
      return -1.0;
    }
  }

  return values[count / 2];
}

void printDistanceSensors() {
  float left = readMedianCM(trigLeft, echoLeft);

  delay(25);

  float right = readMedianCM(trigRight, echoRight);

  Serial.print("Distance | L: ");

  if (left < 0) {
    Serial.print("---");
  } else {
    Serial.print(left, 1);
    Serial.print(" cm");
  }

  Serial.print("    R: ");

  if (right < 0) {
    Serial.print("---");
  } else {
    Serial.print(right, 1);
    Serial.print(" cm");
  }

  Serial.println();
}

// =====================================================
// QTR 9 路 RC 巡线传感器部分
// =====================================================

const uint8_t QTR_SENSOR_COUNT = 9;

const uint8_t QTR_EMITTER_PIN = 31;

// OUT1~OUT9 -> D22~D30
const uint8_t qtrPins[QTR_SENSOR_COUNT] = {
  22, 23, 24, 25, 26, 27, 28, 29, 30
};

uint16_t qtrRaw[QTR_SENSOR_COUNT];
uint16_t qtrCalMin[QTR_SENSOR_COUNT];
uint16_t qtrCalMax[QTR_SENSOR_COUNT];
uint16_t qtrCalibrated[QTR_SENSOR_COUNT];

bool qtrBlackDetected[QTR_SENSOR_COUNT];

const uint16_t QTR_TIMEOUT_US = 8000;

// 判断黑线阈值：0~1000
// 越小越灵敏，显示的 1 越多
// 越大越严格，显示的 1 越少
uint16_t BLACK_THRESHOLD = 250;

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

    if (allDone) break;
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
    if (qtrRaw[i] < qtrCalMin[i]) qtrCalMin[i] = qtrRaw[i];
    if (qtrRaw[i] > qtrCalMax[i]) qtrCalMax[i] = qtrRaw[i];
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

    if (value < 0) value = 0;
    if (value > 1000) value = 1000;

    qtrCalibrated[i] = value;
  }
}

void updateQTRBlackDetection() {
  readQTRCalibrated();

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    qtrBlackDetected[i] = qtrCalibrated[i] > BLACK_THRESHOLD;
  }
}

void printQTRSensors() {
  updateQTRBlackDetection();

  Serial.print("QTR Black: ");

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    Serial.print(qtrBlackDetected[i] ? "1" : "0");

    if (i < QTR_SENSOR_COUNT - 1) {
      Serial.print("  ");
    }
  }

  Serial.print("    Cal: ");

  for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
    Serial.print(qtrCalibrated[i]);

    if (i < QTR_SENSOR_COUNT - 1) {
      Serial.print('\t');
    }
  }

  Serial.println();
}

// =====================================================
// 串口显示模式控制
// =====================================================

enum DisplayMode {
  MODE_NONE,
  MODE_QTR,
  MODE_DISTANCE
};

DisplayMode displayMode = MODE_NONE;

unsigned long lastPrintTime = 0;

// 显示刷新间隔
const unsigned long QTR_PRINT_INTERVAL_MS = 80;
const unsigned long DIST_PRINT_INTERVAL_MS = 120;

void handleSerialCommand() {
  if (Serial.available() <= 0) {
    return;
  }

  char key = Serial.read();

  if (key == 'u' || key == 'U') {
    displayMode = MODE_QTR;
    Serial.println("Mode: QTR line sensors");
    Serial.println("Press i = distance sensors, s = stop display");
  }

  else if (key == 'i' || key == 'I') {
    displayMode = MODE_DISTANCE;
    Serial.println("Mode: HC-SR04 distance sensors");
    Serial.println("Press u = QTR line sensors, s = stop display");
  }

  else if (key == 's' || key == 'S') {
    displayMode = MODE_NONE;
    Serial.println("Mode: stopped display");
  }

  else if (key == 'a' || key == 'A') {
    Serial.println("Command: servo clockwise 60 degrees");
    rotateClockwise60();
  }

  else if (key == 'd' || key == 'D') {
    Serial.println("Command: servo counter-clockwise 60 degrees");
    rotateCounterClockwise60();
  }

  else if (key == 'x' || key == 'X') {
    Serial.println("Command: servo reset to 0 degrees");
    resetToZero();
  }

  else if (key == '\n' || key == '\r') {
    // ignore
  }

  else {
    Serial.print("Unknown command: ");
    Serial.println(key);
  }
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Servo
  beadServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  currentAngle = 0;
  setServoAngle(currentAngle);

  // Button + LED
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  // HC-SR04
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  digitalWrite(trigLeft, LOW);
  digitalWrite(trigRight, LOW);

  // QTR
  pinMode(QTR_EMITTER_PIN, OUTPUT);
  digitalWrite(QTR_EMITTER_PIN, HIGH);

  resetQTRCalibration();

  Serial.println("Integrated robot sensor debug ready.");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("u = show QTR line sensor readings");
  Serial.println("i = show HC-SR04 distance sensor readings");
  Serial.println("s = stop display");
  Serial.println("a = servo clockwise 60 degrees");
  Serial.println("d = servo counter-clockwise 60 degrees");
  Serial.println("x = servo reset to 0 degrees");
  Serial.println();

  Serial.println("QTR calibration: 5 seconds.");
  Serial.println("Move the QTR sensor over white background and black line.");
  Serial.println("Make sure every QTR sensor sees both white and black.");

  for (uint16_t i = 0; i < 250; i++) {
    calibrateQTROnce();
    updateReviveButtonLED();
    delay(20);
  }

  Serial.println("QTR calibration done.");
  Serial.println("Ready. Press u or i.");
}

void loop() {
  updateReviveButtonLED();
  handleSerialCommand();

  unsigned long now = millis();

  if (displayMode == MODE_QTR) {
    if (now - lastPrintTime >= QTR_PRINT_INTERVAL_MS) {
      lastPrintTime = now;
      printQTRSensors();
    }
  }

  else if (displayMode == MODE_DISTANCE) {
    if (now - lastPrintTime >= DIST_PRINT_INTERVAL_MS) {
      lastPrintTime = now;
      printDistanceSensors();
    }
  }
}

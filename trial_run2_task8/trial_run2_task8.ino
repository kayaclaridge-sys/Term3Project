#include <Wire.h>
#include <Motoron.h>
#include <math.h>
#include <string.h>

// =====================================================
// Trial Run 2 - Task 8 Touch-Based Robot Revival
// Arduino GIGA + Motoron + 4 encoders + MPU6050 + front HC-SR04
//
// Goal:
// - Start facing the stranded robot from roughly 2-3 RFID-tag spaces away.
// - Drive straight only; no turning or route planning.
// - Slow down smoothly as the front distance closes.
// - D32 first press starts the task; after release, D32 becomes revive contact.
// - Keep creeping until D32 or the optional D33 contact input is pressed.
// - Hold gentle contact briefly, then stop.
// =====================================================

// -------------------- Motoron motor controllers --------------------
MotoronI2C leftController;   // 0x10: left side
MotoronI2C rightController;  // 0x11: right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

// Positive speed means robot-forward. These signs match the trial_run2 sketches.
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;

const int MOTOR_MAX_SPEED = 550;
const uint16_t MOTOR_MAX_ACCEL = 260;
const uint16_t MOTOR_MAX_DECEL = 420;

// -------------------- Task 8 approach tuning --------------------
// Start slowing down when the target is inside this distance.
const float DECEL_START_CM = 85.0;
const float CREEP_START_CM = 30.0;
const float TOUCH_ZONE_CM = 12.0;

// Motor commands are intentionally conservative to avoid a hard hit.
const int CRUISE_COMMAND = 260;
const int START_BOOST_COMMAND = 320;
const int CREEP_COMMAND = 140;
const int TOUCH_COMMAND = 110;
const int NO_DISTANCE_COMMAND = 130;

const unsigned long START_BOOST_MS = 650;
const unsigned long CONTACT_DEBOUNCE_MS = 35;
const unsigned long START_CONTACT_IGNORE_MS = 500;
const unsigned long RUN_TIMEOUT_MS = 60000;

// Safety distance limit for a missed target. Tune after one measured straight run.
const long MAX_APPROACH_TICKS = 9500;

const float YAW_HOLD_KP = 6.0;
const float ENCODER_BALANCE_KP = 0.18;
const int MAX_STRAIGHT_CORRECTION = 150;
const int COMMAND_SLEW_STEP = 10;

int speedTrimCommand = 0;

// -------------------- Encoder wiring --------------------
// Right back RB: H1 -> D42, H2 -> D43
// Left back  LB: H1 -> D44, H2 -> D45
// Right front RF: H1 -> D48, H2 -> D49
// Left front  LF: H1 -> D50, H2 -> D51
const uint8_t RB_A = 42;
const uint8_t RB_B = 43;
const uint8_t LB_A = 44;
const uint8_t LB_B = 45;
const uint8_t RF_A = 48;
const uint8_t RF_B = 49;
const uint8_t LF_A = 50;
const uint8_t LF_B = 51;

// If one wheel count decreases while the robot moves forward, flip only that sign.
int RB_DIR = 1;
int LB_DIR = 1;
int RF_DIR = 1;
int LF_DIR = 1;

struct EncoderState {
  uint8_t pinA;
  uint8_t pinB;
  uint8_t lastState;
  int dir;
  long count;
  long lastSpeedCount;
  float speedTicksPerSec;
};

EncoderState rbEnc = {RB_A, RB_B, 0, 1, 0, 0, 0.0};
EncoderState lbEnc = {LB_A, LB_B, 0, 1, 0, 0, 0.0};
EncoderState rfEnc = {RF_A, RF_B, 0, 1, 0, 0, 0.0};
EncoderState lfEnc = {LF_A, LF_B, 0, 1, 0, 0, 0.0};

unsigned long lastEncoderSpeedMs = 0;

// -------------------- MPU6050 IMU on Wire D20/D21 --------------------
TwoWire &MPU_BUS = Wire;
uint8_t MPU_ADDR = 0x68;

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

const float GYRO_LSB_PER_DPS = 131.0;  // +/-250 dps
int YAW_SIGN = 1;

bool imuReady = false;
float gyroZBiasDps = 0.0;
float yawDeg = 0.0;
float gyroZDps = 0.0;
unsigned long lastImuMicros = 0;

// -------------------- Front HC-SR04 distance sensor --------------------
const int FRONT_TRIG_PIN = 37;
const int FRONT_ECHO_PIN = 36;

const float SOUND_SPEED_CM_PER_US = 0.0343;
const float ULTRASONIC_MIN_VALID_CM = 3.0;
const float ULTRASONIC_MAX_VALID_CM = 160.0;
const float DISTANCE_FILTER_ALPHA = 0.35;
const unsigned long FRONT_DISTANCE_POLL_MS = 40;
const unsigned long FRONT_DISTANCE_STALE_MS = 650;

float frontDistanceRawCm = -1.0;
float frontDistanceFilteredCm = -1.0;
unsigned long lastFrontDistancePollMs = 0;
unsigned long lastFrontDistanceSeenMs = 0;

// -------------------- Revive buttons and LEDs --------------------
// D32 is dual-role: first press starts Task 8, later presses are revive contact.
// D33 remains available as an auxiliary revive contact input.
// D53 is the dedicated kill/stop switch.
const int START_REVIVE_BUTTON_PIN = 32;
const int AUX_REVIVE_BUTTON_PIN = 33;
const int KILL_SWITCH_PIN = 53;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

bool reviveRawContact = false;
bool reviveContact = false;
bool startReviveButtonDown = false;
bool previousStartReviveButtonDown = false;
bool auxReviveButtonDown = false;
bool previousKillSwitchDown = false;
bool d32ReleasedAfterStart = false;
unsigned long reviveRawChangedMs = 0;

// -------------------- Task state --------------------
enum ReviveState {
  STATE_IDLE,
  STATE_APPROACH,
  STATE_DECEL,
  STATE_CREEP,
  STATE_DONE,
  STATE_FAULT
};

ReviveState reviveState = STATE_IDLE;

long runStartLeftTicks = 0;
long runStartRightTicks = 0;
float targetYawDeg = 0.0;
unsigned long missionStartMs = 0;
unsigned long stateStartedMs = 0;
unsigned long lastTelemetryMs = 0;

int currentLeftCommand = 0;
int currentRightCommand = 0;
bool monitorTelemetry = true;
char faultMessage[80] = "";

void startReviveRun();

// =====================================================
// Small helpers
// =====================================================

float wrap180(float degrees) {
  while (degrees > 180.0) {
    degrees -= 360.0;
  }

  while (degrees < -180.0) {
    degrees += 360.0;
  }

  return degrees;
}

float angleDiffDeg(float target, float current) {
  return wrap180(target - current);
}

int slewToward(int current, int target, int maxStep) {
  if (current < target) {
    return min(current + maxStep, target);
  }

  if (current > target) {
    return max(current - maxStep, target);
  }

  return current;
}

int interpolateCommand(float distanceCm,
                       float nearCm,
                       float farCm,
                       int nearCommand,
                       int farCommand) {
  float ratio = (distanceCm - nearCm) / (farCm - nearCm);
  ratio = constrain(ratio, 0.0, 1.0);
  return nearCommand + (int)((farCommand - nearCommand) * ratio);
}

bool isI2cDevicePresent(TwoWire &bus, byte address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

// =====================================================
// Encoder functions
// =====================================================

uint8_t readEncoderState(uint8_t pinA, uint8_t pinB) {
  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  return (a << 1) | b;
}

int8_t decodeEncoder(uint8_t oldState, uint8_t newState) {
  static const int8_t table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  return table[(oldState << 2) | newState];
}

void updateOneEncoder(EncoderState &encoder) {
  uint8_t nowState = readEncoderState(encoder.pinA, encoder.pinB);

  if (nowState != encoder.lastState) {
    int8_t step = decodeEncoder(encoder.lastState, nowState);
    encoder.count += step * encoder.dir;
    encoder.lastState = nowState;
  }
}

void updateAllEncoders() {
  updateOneEncoder(rbEnc);
  updateOneEncoder(lbEnc);
  updateOneEncoder(rfEnc);
  updateOneEncoder(lfEnc);
}

void updateEncoderSpeeds() {
  unsigned long now = millis();

  if (now - lastEncoderSpeedMs < 100) {
    return;
  }

  float dt = (now - lastEncoderSpeedMs) / 1000.0;
  lastEncoderSpeedMs = now;

  rbEnc.speedTicksPerSec = (rbEnc.count - rbEnc.lastSpeedCount) / dt;
  lbEnc.speedTicksPerSec = (lbEnc.count - lbEnc.lastSpeedCount) / dt;
  rfEnc.speedTicksPerSec = (rfEnc.count - rfEnc.lastSpeedCount) / dt;
  lfEnc.speedTicksPerSec = (lfEnc.count - lfEnc.lastSpeedCount) / dt;

  rbEnc.lastSpeedCount = rbEnc.count;
  lbEnc.lastSpeedCount = lbEnc.count;
  rfEnc.lastSpeedCount = rfEnc.count;
  lfEnc.lastSpeedCount = lfEnc.count;
}

void setupEncoders() {
  rbEnc.dir = RB_DIR;
  lbEnc.dir = LB_DIR;
  rfEnc.dir = RF_DIR;
  lfEnc.dir = LF_DIR;

  pinMode(RB_A, INPUT_PULLUP);
  pinMode(RB_B, INPUT_PULLUP);
  pinMode(LB_A, INPUT_PULLUP);
  pinMode(LB_B, INPUT_PULLUP);
  pinMode(RF_A, INPUT_PULLUP);
  pinMode(RF_B, INPUT_PULLUP);
  pinMode(LF_A, INPUT_PULLUP);
  pinMode(LF_B, INPUT_PULLUP);

  delay(50);

  rbEnc.lastState = readEncoderState(RB_A, RB_B);
  lbEnc.lastState = readEncoderState(LB_A, LB_B);
  rfEnc.lastState = readEncoderState(RF_A, RF_B);
  lfEnc.lastState = readEncoderState(LF_A, LF_B);

  lastEncoderSpeedMs = millis();
}

void resetEncoders() {
  rbEnc.count = 0;
  lbEnc.count = 0;
  rfEnc.count = 0;
  lfEnc.count = 0;

  rbEnc.lastSpeedCount = 0;
  lbEnc.lastSpeedCount = 0;
  rfEnc.lastSpeedCount = 0;
  lfEnc.lastSpeedCount = 0;

  rbEnc.lastState = readEncoderState(RB_A, RB_B);
  lbEnc.lastState = readEncoderState(LB_A, LB_B);
  rfEnc.lastState = readEncoderState(RF_A, RF_B);
  lfEnc.lastState = readEncoderState(LF_A, LF_B);

  Serial.println("Encoders reset.");
}

long leftTicks() {
  return (lfEnc.count + lbEnc.count) / 2;
}

long rightTicks() {
  return (rfEnc.count + rbEnc.count) / 2;
}

long forwardTicks() {
  return (leftTicks() + rightTicks()) / 2;
}

// =====================================================
// MPU6050 yaw functions
// =====================================================

void imuWriteRegister(uint8_t reg, uint8_t value) {
  MPU_BUS.beginTransmission(MPU_ADDR);
  MPU_BUS.write(reg);
  MPU_BUS.write(value);
  MPU_BUS.endTransmission();
}

uint8_t imuReadRegister(uint8_t reg) {
  MPU_BUS.beginTransmission(MPU_ADDR);
  MPU_BUS.write(reg);
  MPU_BUS.endTransmission(false);
  MPU_BUS.requestFrom(MPU_ADDR, (uint8_t)1);

  if (MPU_BUS.available()) {
    return MPU_BUS.read();
  }

  return 0xFF;
}

bool imuReadBytes(uint8_t reg, uint8_t count, uint8_t *buffer) {
  MPU_BUS.beginTransmission(MPU_ADDR);
  MPU_BUS.write(reg);

  if (MPU_BUS.endTransmission(false) != 0) {
    return false;
  }

  MPU_BUS.requestFrom(MPU_ADDR, count);

  uint8_t i = 0;
  while (MPU_BUS.available() && i < count) {
    buffer[i++] = MPU_BUS.read();
  }

  return i == count;
}

bool findMPU6050() {
  if (isI2cDevicePresent(MPU_BUS, 0x68)) {
    MPU_ADDR = 0x68;
    return true;
  }

  if (isI2cDevicePresent(MPU_BUS, 0x69)) {
    MPU_ADDR = 0x69;
    return true;
  }

  return false;
}

bool readGyroZ(float &gzOut) {
  uint8_t data[14];

  if (!imuReadBytes(ACCEL_XOUT_H, 14, data)) {
    return false;
  }

  int16_t rawGz = (data[12] << 8) | data[13];
  gzOut = rawGz / GYRO_LSB_PER_DPS;
  return true;
}

void setupMPU6050() {
  imuWriteRegister(PWR_MGMT_1, 0x00);
  delay(100);
  imuWriteRegister(SMPLRT_DIV, 0x07);    // 125 Hz
  imuWriteRegister(CONFIG, 0x06);        // DLPF
  imuWriteRegister(GYRO_CONFIG, 0x00);   // +/-250 dps
  imuWriteRegister(ACCEL_CONFIG, 0x00);  // +/-2g
}

void calibrateGyroZ() {
  Serial.println("IMU gyro Z calibration: keep the robot still.");

  const uint16_t samples = 350;
  float sum = 0.0;
  uint16_t goodSamples = 0;

  for (uint16_t i = 0; i < samples; i++) {
    float gz = 0.0;

    if (readGyroZ(gz)) {
      sum += gz;
      goodSamples++;
    }

    delay(5);
  }

  if (goodSamples > 0) {
    gyroZBiasDps = sum / goodSamples;
  }

  yawDeg = 0.0;
  lastImuMicros = micros();

  Serial.print("IMU gyro Z bias dps=");
  Serial.println(gyroZBiasDps, 4);
}

bool setupIMU() {
  MPU_BUS.begin();
  MPU_BUS.setClock(100000);

  if (!findMPU6050()) {
    Serial.println("WARNING: MPU6050 not found on Wire D20/D21. Yaw hold disabled.");
    return false;
  }

  uint8_t whoAmI = imuReadRegister(WHO_AM_I);

  Serial.print("MPU6050 found at 0x");
  Serial.print(MPU_ADDR, HEX);
  Serial.print(" WHO_AM_I=0x");
  Serial.println(whoAmI, HEX);

  setupMPU6050();
  calibrateGyroZ();
  return true;
}

void updateIMU() {
  if (!imuReady) {
    return;
  }

  float measuredGz = 0.0;

  if (!readGyroZ(measuredGz)) {
    return;
  }

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastImuMicros) / 1000000.0;
  lastImuMicros = nowMicros;

  gyroZDps = (measuredGz - gyroZBiasDps) * YAW_SIGN;
  yawDeg = wrap180(yawDeg + gyroZDps * dt);
}

void resetYaw() {
  yawDeg = 0.0;
  lastImuMicros = micros();
  Serial.println("Yaw reset to 0.");
}

// =====================================================
// Front distance functions
// =====================================================

unsigned long timeoutFromDistance(float cm) {
  return (unsigned long)(cm * 2.0 / SOUND_SPEED_CM_PER_US) + 800;
}

float readFrontOnceCm() {
  digitalWrite(FRONT_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(FRONT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG_PIN, LOW);

  unsigned long timeoutUs = timeoutFromDistance(ULTRASONIC_MAX_VALID_CM);
  unsigned long duration = pulseIn(FRONT_ECHO_PIN, HIGH, timeoutUs);

  if (duration == 0) {
    return -1.0;
  }

  float distance = duration * SOUND_SPEED_CM_PER_US / 2.0;

  if (distance < ULTRASONIC_MIN_VALID_CM || distance > ULTRASONIC_MAX_VALID_CM) {
    return -1.0;
  }

  return distance;
}

void setupFrontDistanceSensor() {
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  digitalWrite(FRONT_TRIG_PIN, LOW);
}

void updateFrontDistance() {
  unsigned long now = millis();

  if (now - lastFrontDistancePollMs < FRONT_DISTANCE_POLL_MS) {
    return;
  }

  lastFrontDistancePollMs = now;
  frontDistanceRawCm = readFrontOnceCm();

  if (frontDistanceRawCm > 0.0) {
    if (frontDistanceFilteredCm < 0.0) {
      frontDistanceFilteredCm = frontDistanceRawCm;
    } else {
      frontDistanceFilteredCm =
        DISTANCE_FILTER_ALPHA * frontDistanceRawCm +
        (1.0 - DISTANCE_FILTER_ALPHA) * frontDistanceFilteredCm;
    }

    lastFrontDistanceSeenMs = now;
  }
}

bool hasFreshFrontDistance() {
  return frontDistanceFilteredCm > 0.0 &&
         millis() - lastFrontDistanceSeenMs <= FRONT_DISTANCE_STALE_MS;
}

// =====================================================
// Motor helpers
// =====================================================

void printHex16(uint16_t value) {
  if (value < 0x1000) {
    Serial.print("0");
  }

  if (value < 0x0100) {
    Serial.print("0");
  }

  if (value < 0x0010) {
    Serial.print("0");
  }

  Serial.print(value, HEX);
}

void setupMotoron(MotoronI2C *controller, const char *name) {
  controller->reinitialize();
  delay(20);
  controller->disableCrc();
  controller->clearResetFlag();
  controller->clearMotorFaultUnconditional();
  controller->disableCommandTimeout();

  for (uint8_t channel = 1; channel <= 3; channel++) {
    controller->setMaxAcceleration(channel, MOTOR_MAX_ACCEL);
    controller->setMaxDeceleration(channel, MOTOR_MAX_DECEL);
  }

  uint16_t productId = 0;
  uint16_t firmwareVersion = 0;
  controller->getFirmwareVersion(&productId, &firmwareVersion);

  Serial.print(name);
  Serial.print(" Motoron ready product=0x");
  printHex16(productId);
  Serial.print(" firmware=0x");
  printHex16(firmwareVersion);
  Serial.print(" lastError=");
  Serial.println(controller->getLastError());
}

void setupMotorControllers() {
  Wire1.begin();
  delay(100);

  if (!isI2cDevicePresent(Wire1, 0x10)) {
    Serial.println("WARNING: Left Motoron not found at 0x10.");
  }

  if (!isI2cDevicePresent(Wire1, 0x11)) {
    Serial.println("WARNING: Right Motoron not found at 0x11.");
  }

  leftController.setBus(&Wire1);
  setupMotoron(&leftController, "Left 0x10");

  rightController.setBus(&Wire1);
  rightController.setAddress(17);  // 17 decimal = 0x11
  setupMotoron(&rightController, "Right 0x11");
}

void writeLeftSideSpeed(int speed) {
  int command = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED) * LEFT_MOTOR_SIGN;
  leftController.setSpeed(FRONT_LEFT_MOTOR, command);
  leftController.setSpeed(REAR_LEFT_MOTOR, command);
}

void writeRightSideSpeed(int speed) {
  int command = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED) * RIGHT_MOTOR_SIGN;
  rightController.setSpeed(FRONT_RIGHT_MOTOR, command);
  rightController.setSpeed(REAR_RIGHT_MOTOR, command);
}

void applyDriveSpeeds(int leftSpeed, int rightSpeed, bool immediate) {
  int targetLeft = constrain(leftSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  int targetRight = constrain(rightSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  if (immediate) {
    currentLeftCommand = targetLeft;
    currentRightCommand = targetRight;
  } else {
    currentLeftCommand = slewToward(currentLeftCommand, targetLeft, COMMAND_SLEW_STEP);
    currentRightCommand = slewToward(currentRightCommand, targetRight, COMMAND_SLEW_STEP);
  }

  writeLeftSideSpeed(currentLeftCommand);
  writeRightSideSpeed(currentRightCommand);
}

void stopDrive() {
  applyDriveSpeeds(0, 0, true);
}

void applyStraightDrive(int baseCommand) {
  int safeBase = constrain(baseCommand, 0, MOTOR_MAX_SPEED);

  long leftDelta = leftTicks() - runStartLeftTicks;
  long rightDelta = rightTicks() - runStartRightTicks;
  long balanceError = leftDelta - rightDelta;

  int yawCorrection = 0;
  if (imuReady) {
    float headingError = angleDiffDeg(targetYawDeg, yawDeg);
    yawCorrection = constrain((int)(headingError * YAW_HOLD_KP),
                              -MAX_STRAIGHT_CORRECTION,
                              MAX_STRAIGHT_CORRECTION);
  }

  int encoderCorrection = constrain((int)(balanceError * ENCODER_BALANCE_KP),
                                    -MAX_STRAIGHT_CORRECTION,
                                    MAX_STRAIGHT_CORRECTION);

  int leftCommand = safeBase + yawCorrection - encoderCorrection;
  int rightCommand = safeBase - yawCorrection + encoderCorrection;

  applyDriveSpeeds(leftCommand, rightCommand, false);
}

// =====================================================
// Revive contact and state machine
// =====================================================

const char *stateName(ReviveState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_APPROACH:
      return "APPROACH";
    case STATE_DECEL:
      return "DECEL";
    case STATE_CREEP:
      return "CREEP";
    case STATE_DONE:
      return "DONE";
    case STATE_FAULT:
      return "FAULT";
  }

  return "?";
}

void enterState(ReviveState newState) {
  if (reviveState != newState) {
    Serial.print("State: ");
    Serial.print(stateName(reviveState));
    Serial.print(" -> ");
    Serial.println(stateName(newState));
  }

  reviveState = newState;
  stateStartedMs = millis();
}

void updateReviveContact() {
  unsigned long now = millis();
  startReviveButtonDown = digitalRead(START_REVIVE_BUTTON_PIN) == LOW;
  auxReviveButtonDown = digitalRead(AUX_REVIVE_BUTTON_PIN) == LOW;

  bool running = reviveState == STATE_APPROACH ||
                 reviveState == STATE_DECEL ||
                 reviveState == STATE_CREEP;

  if (running && !d32ReleasedAfterStart && !startReviveButtonDown) {
    d32ReleasedAfterStart = true;
    Serial.println("D32 released after start; next D32 press can trigger revive contact.");
  }

  bool d32ContactArmed = running &&
                         d32ReleasedAfterStart &&
                         now - missionStartMs >= START_CONTACT_IGNORE_MS;
  bool raw = running && ((d32ContactArmed && startReviveButtonDown) || auxReviveButtonDown);

  if (raw != reviveRawContact) {
    reviveRawContact = raw;
    reviveRawChangedMs = now;
  }

  if (now - reviveRawChangedMs >= CONTACT_DEBOUNCE_MS) {
    reviveContact = reviveRawContact;
  }
}

void handleD32StartButton() {
  if (reviveState == STATE_IDLE &&
      startReviveButtonDown &&
      !previousStartReviveButtonDown) {
    startReviveRun();
  }

  previousStartReviveButtonDown = startReviveButtonDown;
}

int calculateApproachCommand() {
  int cruise = constrain(CRUISE_COMMAND + speedTrimCommand, CREEP_COMMAND, MOTOR_MAX_SPEED);
  int startBoost = constrain(START_BOOST_COMMAND + speedTrimCommand, cruise, MOTOR_MAX_SPEED);
  int creep = constrain(CREEP_COMMAND + speedTrimCommand / 2, TOUCH_COMMAND, cruise);
  int touch = constrain(TOUCH_COMMAND + speedTrimCommand / 3, 35, creep);

  bool startBoostActive = reviveState == STATE_APPROACH &&
                          millis() - missionStartMs < START_BOOST_MS &&
                          (!hasFreshFrontDistance() || frontDistanceFilteredCm > CREEP_START_CM);

  if (startBoostActive) {
    return startBoost;
  }

  if (!hasFreshFrontDistance()) {
    if (frontDistanceFilteredCm > 0.0 && frontDistanceFilteredCm <= TOUCH_ZONE_CM) {
      return touch;
    }

    if (frontDistanceFilteredCm > 0.0 && frontDistanceFilteredCm <= CREEP_START_CM) {
      return creep;
    }

    return constrain(NO_DISTANCE_COMMAND + speedTrimCommand / 2, touch, cruise);
  }

  float distance = frontDistanceFilteredCm;

  if (distance > DECEL_START_CM) {
    return cruise;
  }

  if (distance > CREEP_START_CM) {
    return interpolateCommand(distance, CREEP_START_CM, DECEL_START_CM, creep, cruise);
  }

  if (distance > TOUCH_ZONE_CM) {
    return interpolateCommand(distance, TOUCH_ZONE_CM, CREEP_START_CM, touch, creep);
  }

  return touch;
}

ReviveState stateForDistance() {
  if (!hasFreshFrontDistance()) {
    if (frontDistanceFilteredCm > 0.0 && frontDistanceFilteredCm <= CREEP_START_CM) {
      return STATE_CREEP;
    }

    return STATE_APPROACH;
  }

  if (frontDistanceFilteredCm <= CREEP_START_CM) {
    return STATE_CREEP;
  }

  if (frontDistanceFilteredCm <= DECEL_START_CM) {
    return STATE_DECEL;
  }

  return STATE_APPROACH;
}

void stopWithFault(const char *message) {
  stopDrive();
  strncpy(faultMessage, message, sizeof(faultMessage) - 1);
  faultMessage[sizeof(faultMessage) - 1] = '\0';
  enterState(STATE_FAULT);

  Serial.print("FAULT: ");
  Serial.println(faultMessage);
}

void startReviveRun() {
  resetEncoders();
  resetYaw();
  runStartLeftTicks = leftTicks();
  runStartRightTicks = rightTicks();
  targetYawDeg = yawDeg;
  missionStartMs = millis();
  d32ReleasedAfterStart = !startReviveButtonDown;
  reviveRawContact = false;
  reviveContact = false;
  reviveRawChangedMs = missionStartMs;
  faultMessage[0] = '\0';
  currentLeftCommand = 0;
  currentRightCommand = 0;

  enterState(STATE_APPROACH);
  Serial.println("Task 8 revive approach started.");

  if (!d32ReleasedAfterStart) {
    Serial.println("Release D32 once; the next D32 press will count as revive contact.");
  }
}

void stopReviveRun() {
  stopDrive();
  enterState(STATE_IDLE);
  Serial.println("Task 8 stopped.");
}

void finishReviveRun() {
  stopDrive();
  enterState(STATE_DONE);
  Serial.println("Task 8 complete: revive contact detected, robot stopped.");
}

void updateReviveStateMachine() {
  if (reviveState == STATE_IDLE || reviveState == STATE_DONE || reviveState == STATE_FAULT) {
    if (currentLeftCommand != 0 || currentRightCommand != 0) {
      stopDrive();
    }

    return;
  }

  unsigned long now = millis();

  if (now - missionStartMs > RUN_TIMEOUT_MS) {
    stopWithFault("Run timeout before revive contact.");
    return;
  }

  long runTicks = abs(forwardTicks() - ((runStartLeftTicks + runStartRightTicks) / 2));
  if (runTicks > MAX_APPROACH_TICKS && !reviveContact) {
    stopWithFault("Max straight approach ticks reached without contact.");
    return;
  }

  if (reviveContact) {
    finishReviveRun();
    return;
  }

  ReviveState desiredState = stateForDistance();
  if (desiredState != reviveState) {
    enterState(desiredState);
  }

  int approachCommand = calculateApproachCommand();
  applyStraightDrive(approachCommand);
}

// =====================================================
// Controls and telemetry
// =====================================================

void updateStatusLED() {
  bool contactButtonHeld = (d32ReleasedAfterStart && startReviveButtonDown) ||
                           auxReviveButtonDown;

  if (reviveState == STATE_DONE && contactButtonHeld) {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    return;
  }

  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
}

void printTelemetry() {
  Serial.print("state=");
  Serial.print(stateName(reviveState));
  Serial.print(" contact=");
  Serial.print(reviveContact ? "Y" : "N");
  Serial.print(" rawContact=");
  Serial.print(reviveRawContact ? "Y" : "N");
  Serial.print(" D32=");
  Serial.print(startReviveButtonDown ? "DOWN" : "UP");
  Serial.print(" released=");
  Serial.print(d32ReleasedAfterStart ? "Y" : "N");
  Serial.print(" D33=");
  Serial.print(auxReviveButtonDown ? "DOWN" : "UP");

  Serial.print(" distRaw=");
  if (frontDistanceRawCm < 0.0) {
    Serial.print("---");
  } else {
    Serial.print(frontDistanceRawCm, 1);
  }

  Serial.print(" distFilt=");
  if (!hasFreshFrontDistance()) {
    Serial.print("---");
  } else {
    Serial.print(frontDistanceFilteredCm, 1);
  }

  Serial.print(" cmdL=");
  Serial.print(currentLeftCommand);
  Serial.print(" cmdR=");
  Serial.print(currentRightCommand);
  Serial.print(" ticksL=");
  Serial.print(leftTicks());
  Serial.print(" ticksR=");
  Serial.print(rightTicks());
  Serial.print(" fwd=");
  Serial.print(forwardTicks());
  Serial.print(" yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" targetYaw=");
  Serial.print(targetYawDeg, 1);
  Serial.print(" imu=");
  Serial.print(imuReady ? "Y" : "N");
  Serial.print(" trim=");
  Serial.println(speedTrimCommand);
}

void printHelp() {
  Serial.println();
  Serial.println("--- Trial Run 2 Task 8 Touch-Based Robot Revival ---");
  Serial.println("D32 first press = start straight revive approach");
  Serial.println("D32 after release = revive contact");
  Serial.println("g = start from Serial Monitor");
  Serial.println("s/x = stop and return to idle");
  Serial.println("p = print telemetry");
  Serial.println("m = toggle live telemetry");
  Serial.println("z = reset encoders and yaw");
  Serial.println("+/- = adjust speed trim");
  Serial.println("D33 = optional auxiliary revive contact, pressed LOW");
  Serial.println("D53 = kill/stop switch, pressed LOW");
  Serial.println("The robot slows by front ultrasonic distance and stops only after contact.");
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char command = Serial.read();

    if (command == '\n' || command == '\r' || command == ' ') {
      continue;
    }

    if (command == 'g' || command == 'G') {
      startReviveRun();
    } else if (command == 's' || command == 'S' ||
               command == 'x' || command == 'X') {
      stopReviveRun();
    } else if (command == 'p' || command == 'P') {
      printTelemetry();
    } else if (command == 'm' || command == 'M') {
      monitorTelemetry = !monitorTelemetry;
      Serial.print("Live telemetry: ");
      Serial.println(monitorTelemetry ? "ON" : "OFF");
    } else if (command == 'z' || command == 'Z') {
      resetEncoders();
      resetYaw();
    } else if (command == '+' || command == '=') {
      speedTrimCommand = constrain(speedTrimCommand + 15, -90, 90);
      Serial.print("speedTrimCommand=");
      Serial.println(speedTrimCommand);
    } else if (command == '-' || command == '_') {
      speedTrimCommand = constrain(speedTrimCommand - 15, -90, 90);
      Serial.print("speedTrimCommand=");
      Serial.println(speedTrimCommand);
    } else if (command == 'h' || command == 'H' || command == '?') {
      printHelp();
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
      printHelp();
    }
  }
}

void maybePrintLiveTelemetry() {
  if (!monitorTelemetry) {
    return;
  }

  bool active = reviveState == STATE_APPROACH ||
                reviveState == STATE_DECEL ||
                reviveState == STATE_CREEP;

  if (!active) {
    return;
  }

  if (millis() - lastTelemetryMs >= 250) {
    lastTelemetryMs = millis();
    printTelemetry();
  }
}

void updateKillSwitch() {
  bool killDown = digitalRead(KILL_SWITCH_PIN) == LOW;

  if (killDown && !previousKillSwitchDown) {
    stopReviveRun();
    Serial.println("Task 8 killed by D53 switch.");
  }

  previousKillSwitchDown = killDown;
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
  Serial.begin(115200);

  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {
  }

  Serial.println();
  Serial.println("Trial Run 2 Task 8 touch-based robot revival starting...");
  Serial.println("Place the active robot facing the stranded robot. Start with D32 or Serial 'g'.");

  pinMode(START_REVIVE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUX_REVIVE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  updateStatusLED();

  setupEncoders();
  setupFrontDistanceSensor();
  setupMotorControllers();

  imuReady = setupIMU();

  stopDrive();
  resetEncoders();
  resetYaw();

  startReviveButtonDown = digitalRead(START_REVIVE_BUTTON_PIN) == LOW;
  previousStartReviveButtonDown = startReviveButtonDown;
  auxReviveButtonDown = digitalRead(AUX_REVIVE_BUTTON_PIN) == LOW;
  previousKillSwitchDown = digitalRead(KILL_SWITCH_PIN) == LOW;
  d32ReleasedAfterStart = false;
  reviveRawContact = false;
  reviveContact = false;
  reviveRawChangedMs = millis();

  printHelp();
}

void loop() {
  handleSerialCommands();

  updateAllEncoders();
  updateEncoderSpeeds();
  updateIMU();
  updateFrontDistance();
  updateReviveContact();
  updateKillSwitch();
  handleD32StartButton();
  updateReviveStateMachine();
  updateStatusLED();
  maybePrintLiveTelemetry();

  delay(8);
}

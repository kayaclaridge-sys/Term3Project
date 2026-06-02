#include <Wire.h>
#include <Motoron.h>
#include <math.h>
#include <string.h>

// =====================================================
// Trial Run 2 - Task 5 + Task 6
// Ramped Incline / Decline Control + Wall Following
// Arduino GIGA + Motoron + 4 encoders + MPU6050
//
// Goal:
// - Drive over the airlock ramp with stable speed.
// - Hold the airlock centerline using left/right ultrasonic wall following.
// - Add torque on the incline, brake on the decline.
// - Hold heading using gyro yaw.
// - Stop safely on sustained excessive pitch/roll, stall, or timeout.
// =====================================================

// -------------------- Motoron motor controllers --------------------
MotoronI2C leftController;   // 0x10: left side
MotoronI2C rightController;  // 0x11: right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

// Positive speed means robot-forward.
// If the robot drives backward, flip both signs together.
const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;

const int MOTOR_MAX_SPEED = 550;
const int MOTOR_MIN_CLIMB_COMMAND = 300;
const int MOTOR_MIN_DESCENT_COMMAND = -300;

// Smooth acceleration prevents the robot from jumping on the ramp.
const uint16_t MOTOR_MAX_ACCEL = 400;
const uint16_t MOTOR_MAX_DECEL = 400;

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

// If a wheel count decreases when the robot moves forward, flip only that sign.
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

const float ACCEL_LSB_PER_G = 16384.0;  // +/-2g
const float GYRO_LSB_PER_DPS = 131.0;   // +/-250 dps
const float RAD_TO_DEG_F = 57.2957795;

// Tune these if pitch/roll signs are reversed after mounting the MPU6050.
int PITCH_SIGN = 1;
int ROLL_SIGN = 1;
int YAW_SIGN = 1;

const float IMU_FILTER_ALPHA = 0.20;

bool imuReady = false;
bool imuFilterPrimed = false;
float gyroZBiasDps = 0.0;
float pitchOffsetDeg = 0.0;
float rollOffsetDeg = 0.0;

float pitchDeg = 0.0;
float rollDeg = 0.0;
float yawDeg = 0.0;
float gyroZDps = 0.0;
unsigned long lastImuMicros = 0;

struct ImuRaw {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gz;
};

// -------------------- HC-SR04 ultrasonic distance sensors --------------------
// Pin mapping matches wall_following/wall_following.ino.
const int LEFT_TRIG_PIN = 41;
const int LEFT_ECHO_PIN = 40;
const int RIGHT_TRIG_PIN = 39;
const int RIGHT_ECHO_PIN = 38;

const float SOUND_SPEED_CM_PER_US = 0.0343;
const float ULTRASONIC_MIN_VALID_CM = 1.5;
const float ULTRASONIC_MAX_VALID_CM = 120.0;

// Target is equal raw ultrasonic sensor-to-wall readings on both sides.
const float WALL_EQUALITY_TARGET_DELTA_CM = 0.0;
const float WALL_ENTRY_MAX_CM = 10.0;
const float WALL_EXIT_MIN_CM = 15.0;
const float WALL_DEADBAND_CM = 0.35;
const float WALL_KP = 70.0;
const float WALL_KD = 1.5;
const int WALL_MIN_CORRECTION = 150;
const int WALL_MAX_CORRECTION = 380;
const int WALL_PIVOT_CORRECTION_THRESHOLD = 999;
const int WALL_PIVOT_REVERSE_SPEED = 0;
const int WALL_PIVOT_OUTER_MIN_SPEED = 0;
const float WALL_YAW_HOLD_SCALE = 0.0;
const float WALL_ENCODER_BALANCE_SCALE = 0.3;

const unsigned long DISTANCE_SLOT_MS = 28;

float leftDistanceCm = -1.0;
float rightDistanceCm = -1.0;
float previousWallErrorCm = 0.0;
float lastWallErrorCm = 0.0;
float lastWallCorrection = 0.0;
int wallSteeringSign = -1;
int lastBaseCommand = 0;
int lastYawCorrection = 0;
int lastBalanceCorrection = 0;
bool lastPivotAssistActive = false;

unsigned long lastDistanceMs = 0;
unsigned long lastWallConditionFrameMs = 0;
unsigned long previousWallMs = 0;
unsigned long lastLeftDistanceSeenMs = 0;
unsigned long lastRightDistanceSeenMs = 0;
uint8_t distanceSlot = 0;

// -------------------- Buttons and LEDs --------------------
const int START_BUTTON_PIN = 32;
const int STOP_BUTTON_PIN = 33;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

bool previousStartDown = false;
bool previousStopDown = false;

// -------------------- Ramp control tuning --------------------
enum RampProfile {
  PROFILE_ASCEND,
  PROFILE_DESCEND
};

enum RampState {
  STATE_IDLE,
  STATE_APPROACH,
  STATE_ON_RAMP,
  STATE_EXIT_CLEAR,
  STATE_DONE,
  STATE_FAULT
};

RampProfile selectedProfile = PROFILE_ASCEND;
RampState rampState = STATE_IDLE;

// Encoder-speed targets. Adjust after one short bench run.
int approachTargetTicksPerSec = 320;
int ascendTargetTicksPerSec = 280;
int descendTargetTicksPerSec = 320;
int exitTargetTicksPerSec = 300;
int speedTrimTicksPerSec = 0;

// Feed-forward motor command for level ground, then pitch compensation.
const int LEVEL_FEEDFORWARD_COMMAND = 480;
const float SPEED_KP = 0.30;
const float ENCODER_BALANCE_KP = 0.16;
const float YAW_HOLD_KP = 7.0;
const float UPHILL_PITCH_GAIN = 30.0;
const float DOWNHILL_PITCH_BRAKE_GAIN = 14.0;
const int OVERSPEED_BRAKE_COMMAND = 180;
const int MAX_SIDE_CORRECTION = 240;

// Pitch thresholds in degrees.
const float RAMP_ENTER_PITCH_DEG = 5.0;
const float FLAT_EXIT_PITCH_DEG = 3.0;
const float CLIMB_MODE_ENTER_PITCH_DEG = 4.5;
const float CLIMB_MODE_EXIT_PITCH_DEG = 3.5;

const uint8_t ENTER_CONFIRM_FRAMES = 5;
const uint8_t EXIT_CONFIRM_FRAMES = 3;
const uint8_t CLIMB_MODE_ENTER_FRAMES = 3;
const uint8_t CLIMB_MODE_EXIT_FRAMES = 3;
const uint8_t WALL_ENTRY_CONFIRM_FRAMES = 3;
const uint8_t WALL_EXIT_CONFIRM_FRAMES = 5;

// Distance/time limits protect against a missed sensor reading.
const long APPROACH_MAX_TICKS = 2200;
const long RAMP_MIN_TICKS = 800;
const long RAMP_MAX_TICKS = 6500;
const long EXIT_CLEAR_TICKS = 700;
const unsigned long RUN_TIMEOUT_MS = 45000;
const unsigned long EXIT_CLEAR_MAX_MS = 2500;

// Stall protection while climbing.
const float STALL_TICKS_PER_SEC = 80.0;
const unsigned long STALL_BOOST_DELAY_MS = 450;
const unsigned long STALL_FAULT_MS = 2600;
const int STALL_BOOST_COMMAND = 220;

long runStartTicks = 0;
long rampStartTicks = 0;
long exitStartTicks = 0;
float targetYawDeg = 0.0;
unsigned long missionStartMs = 0;
unsigned long stateStartedMs = 0;
unsigned long stallStartedMs = 0;

uint8_t rampEnterFrames = 0;
uint8_t flatExitFrames = 0;
uint8_t climbModeEnterFrames = 0;
uint8_t climbModeExitFrames = 0;
uint8_t wallEntryFrames = 0;
uint8_t wallExitFrames = 0;
bool climbModeActive = false;

int currentLeftCommand = 0;
int currentRightCommand = 0;
char faultMessage[80] = "";

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

const char *profileName(RampProfile profile) {
  if (profile == PROFILE_ASCEND) {
    return "ASCEND";
  }

  return "DESCEND";
}

const char *stateName(RampState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_APPROACH:
      return "APPROACH";
    case STATE_ON_RAMP:
      return "ON_RAMP";
    case STATE_EXIT_CLEAR:
      return "EXIT_CLEAR";
    case STATE_DONE:
      return "DONE";
    case STATE_FAULT:
      return "FAULT";
  }

  return "?";
}

void setFault(const char *message) {
  strncpy(faultMessage, message, sizeof(faultMessage) - 1);
  faultMessage[sizeof(faultMessage) - 1] = '\0';
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

  lastEncoderSpeedMs = millis();
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

float leftTicksPerSec() {
  return (lfEnc.speedTicksPerSec + lbEnc.speedTicksPerSec) * 0.5;
}

float rightTicksPerSec() {
  return (rfEnc.speedTicksPerSec + rbEnc.speedTicksPerSec) * 0.5;
}

float averageTicksPerSec() {
  return (leftTicksPerSec() + rightTicksPerSec()) * 0.5;
}

// =====================================================
// MPU6050 functions
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

bool i2cAddressPresent(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

bool findMPU6050() {
  if (i2cAddressPresent(MPU_BUS, 0x68)) {
    MPU_ADDR = 0x68;
    return true;
  }

  if (i2cAddressPresent(MPU_BUS, 0x69)) {
    MPU_ADDR = 0x69;
    return true;
  }

  return false;
}

bool readImuRaw(ImuRaw &raw) {
  uint8_t data[14];

  if (!imuReadBytes(ACCEL_XOUT_H, 14, data)) {
    return false;
  }

  raw.ax = (int16_t)((data[0] << 8) | data[1]);
  raw.ay = (int16_t)((data[2] << 8) | data[3]);
  raw.az = (int16_t)((data[4] << 8) | data[5]);
  raw.gz = (int16_t)((data[12] << 8) | data[13]);
  return true;
}

float rawPitchFromAccel(const ImuRaw &raw) {
  float ax = raw.ax / ACCEL_LSB_PER_G;
  float ay = raw.ay / ACCEL_LSB_PER_G;
  float az = raw.az / ACCEL_LSB_PER_G;
  return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG_F * PITCH_SIGN;
}

float rawRollFromAccel(const ImuRaw &raw) {
  float ay = raw.ay / ACCEL_LSB_PER_G;
  float az = raw.az / ACCEL_LSB_PER_G;
  return atan2(ay, az) * RAD_TO_DEG_F * ROLL_SIGN;
}

void setupMPU6050Registers() {
  imuWriteRegister(PWR_MGMT_1, 0x00);
  delay(100);
  imuWriteRegister(SMPLRT_DIV, 0x07);    // 125 Hz
  imuWriteRegister(CONFIG, 0x06);        // DLPF
  imuWriteRegister(GYRO_CONFIG, 0x00);   // +/-250 dps
  imuWriteRegister(ACCEL_CONFIG, 0x00);  // +/-2g
}

void calibrateIMU() {
  Serial.println("IMU calibration: keep robot flat and still.");

  const uint16_t samples = 350;
  float gyroSum = 0.0;
  float pitchSum = 0.0;
  float rollSum = 0.0;
  uint16_t goodSamples = 0;

  for (uint16_t i = 0; i < samples; i++) {
    ImuRaw raw;

    if (readImuRaw(raw)) {
      gyroSum += raw.gz / GYRO_LSB_PER_DPS;
      pitchSum += rawPitchFromAccel(raw);
      rollSum += rawRollFromAccel(raw);
      goodSamples++;
    }

    bool blinkOn = ((millis() / 140) % 2) == 0;
    digitalWrite(LED_RED_PIN, blinkOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, blinkOn ? LOW : HIGH);
    delay(5);
  }

  if (goodSamples > 0) {
    gyroZBiasDps = gyroSum / goodSamples;
    pitchOffsetDeg = pitchSum / goodSamples;
    rollOffsetDeg = rollSum / goodSamples;
  }

  yawDeg = 0.0;
  gyroZDps = 0.0;
  pitchDeg = 0.0;
  rollDeg = 0.0;
  imuFilterPrimed = false;
  lastImuMicros = micros();

  Serial.print("IMU calibration done. gyroZBias=");
  Serial.print(gyroZBiasDps, 4);
  Serial.print(" pitchOffset=");
  Serial.print(pitchOffsetDeg, 2);
  Serial.print(" rollOffset=");
  Serial.println(rollOffsetDeg, 2);
}

bool setupIMU() {
  MPU_BUS.begin();
  MPU_BUS.setClock(100000);

  if (!findMPU6050()) {
    Serial.println("WARNING: MPU6050 not found on Wire D20/D21.");
    return false;
  }

  uint8_t whoAmI = imuReadRegister(WHO_AM_I);

  Serial.print("MPU6050 found at 0x");
  Serial.print(MPU_ADDR, HEX);
  Serial.print(" WHO_AM_I=0x");
  Serial.println(whoAmI, HEX);

  setupMPU6050Registers();
  calibrateIMU();
  return true;
}

void updateIMU() {
  if (!imuReady) {
    return;
  }

  ImuRaw raw;

  if (!readImuRaw(raw)) {
    return;
  }

  float correctedPitch = rawPitchFromAccel(raw) - pitchOffsetDeg;
  float correctedRoll = rawRollFromAccel(raw) - rollOffsetDeg;

  if (!imuFilterPrimed) {
    pitchDeg = correctedPitch;
    rollDeg = correctedRoll;
    imuFilterPrimed = true;
  } else {
    pitchDeg = pitchDeg * (1.0 - IMU_FILTER_ALPHA) + correctedPitch * IMU_FILTER_ALPHA;
    rollDeg = rollDeg * (1.0 - IMU_FILTER_ALPHA) + correctedRoll * IMU_FILTER_ALPHA;
  }

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastImuMicros) / 1000000.0;
  lastImuMicros = nowMicros;

  gyroZDps = ((raw.gz / GYRO_LSB_PER_DPS) - gyroZBiasDps) * YAW_SIGN;
  yawDeg = wrap180(yawDeg + gyroZDps * dt);
}

void resetYaw() {
  yawDeg = 0.0;
  gyroZDps = 0.0;
  lastImuMicros = micros();
}

// =====================================================
// Ultrasonic distance functions
// =====================================================

bool isDistanceValid(float distanceCm) {
  return distanceCm >= 0.0 && distanceCm < ULTRASONIC_MAX_VALID_CM;
}

unsigned long ultrasonicTimeoutFromDistance(float cm) {
  return (unsigned long)(cm * 2.0 / SOUND_SPEED_CM_PER_US) + 800;
}

float readUltrasonicRawCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long timeoutUs = ultrasonicTimeoutFromDistance(ULTRASONIC_MAX_VALID_CM);
  unsigned long duration = pulseIn(echoPin, HIGH, timeoutUs);

  if (duration == 0) {
    return -1.0;
  }

  float distance = duration * SOUND_SPEED_CM_PER_US / 2.0;

  if (distance < ULTRASONIC_MIN_VALID_CM || distance > ULTRASONIC_MAX_VALID_CM) {
    return -1.0;
  }

  return distance;
}

float smoothedDistance(float currentCm, float readingCm) {
  if (!isDistanceValid(readingCm)) {
    return currentCm;
  }

  if (!isDistanceValid(currentCm)) {
    return readingCm;
  }

  return currentCm * 0.65 + readingCm * 0.35;
}

void updateStoredDistance(float readingCm, float &storedCm, unsigned long &lastSeenMs) {
  unsigned long now = millis();

  if (isDistanceValid(readingCm)) {
    storedCm = smoothedDistance(storedCm, readingCm);
    lastSeenMs = now;
  } else if (now - lastSeenMs > 220) {
    storedCm = -1.0;
  }
}

void setupDistanceSensors() {
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  digitalWrite(LEFT_TRIG_PIN, LOW);
  digitalWrite(RIGHT_TRIG_PIN, LOW);
}

void refreshAllDistancesBlocking() {
  unsigned long now = millis();

  leftDistanceCm = readUltrasonicRawCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  if (isDistanceValid(leftDistanceCm)) {
    lastLeftDistanceSeenMs = now;
  }

  delay(8);

  rightDistanceCm = readUltrasonicRawCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  if (isDistanceValid(rightDistanceCm)) {
    lastRightDistanceSeenMs = now;
  }
}

void updateDistanceSensors() {
  unsigned long now = millis();

  if (now - lastDistanceMs < DISTANCE_SLOT_MS) {
    return;
  }

  lastDistanceMs = now;

  if (distanceSlot == 0) {
    float reading = readUltrasonicRawCm(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    updateStoredDistance(reading, leftDistanceCm, lastLeftDistanceSeenMs);
  } else {
    float reading = readUltrasonicRawCm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    updateStoredDistance(reading, rightDistanceCm, lastRightDistanceSeenMs);
  }

  distanceSlot = (distanceSlot + 1) % 2;
}

void selectWallForRun() {
  refreshAllDistancesBlocking();
  previousWallErrorCm = 0.0;
  lastWallCorrection = 0.0;
  previousWallMs = millis();

  Serial.print("Dual ultrasonic equality hold target delta=");
  Serial.print(WALL_EQUALITY_TARGET_DELTA_CM, 1);
  Serial.print("cm left=");
  Serial.print(leftDistanceCm, 1);
  Serial.print("cm right=");
  Serial.print(rightDistanceCm, 1);
  Serial.println("cm");
}

int calculateWallCorrection() {
  bool leftValid = isDistanceValid(leftDistanceCm);
  bool rightValid = isDistanceValid(rightDistanceCm);

  if (!leftValid || !rightValid) {
    lastWallErrorCm = 0.0;
    lastWallCorrection = 0.0;
    previousWallMs = millis();
    return 0;
  }

  unsigned long now = millis();
  bool hasPreviousWallSample = previousWallMs != 0;
  float dt = (now - previousWallMs) / 1000.0;

  if (dt < 0.02) {
    dt = 0.02;
  }

  previousWallMs = now;

  float error = (leftDistanceCm - rightDistanceCm) - WALL_EQUALITY_TARGET_DELTA_CM;

  if (fabs(error) < WALL_DEADBAND_CM) {
    error = 0.0;
  }

  float derivative = hasPreviousWallSample ? (error - previousWallErrorCm) / dt : 0.0;
  previousWallErrorCm = error;
  lastWallErrorCm = error;

  float correction = WALL_KP * error + WALL_KD * derivative;

  if (error != 0.0 && fabs(correction) < WALL_MIN_CORRECTION) {
    correction = error > 0.0 ? WALL_MIN_CORRECTION : -WALL_MIN_CORRECTION;
  }

  correction *= wallSteeringSign;

  lastWallCorrection = correction;
  lastWallCorrection = constrain(lastWallCorrection, -WALL_MAX_CORRECTION, WALL_MAX_CORRECTION);

  return (int)lastWallCorrection;
}

bool bothSideDistancesAtMost(float distanceCm) {
  return isDistanceValid(leftDistanceCm) &&
         isDistanceValid(rightDistanceCm) &&
         leftDistanceCm <= distanceCm &&
         rightDistanceCm <= distanceCm;
}

bool bothSideDistancesAtLeast(float distanceCm) {
  return isDistanceValid(leftDistanceCm) &&
         isDistanceValid(rightDistanceCm) &&
         leftDistanceCm >= distanceCm &&
         rightDistanceCm >= distanceCm;
}

// =====================================================
// Motor helpers
// =====================================================

bool isI2cDevicePresent(TwoWire &bus, byte address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
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

  Serial.print(name);
  Serial.println(" Motoron ready.");
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

void applyDriveSpeeds(int leftSpeed, int rightSpeed) {
  currentLeftCommand = constrain(leftSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  currentRightCommand = constrain(rightSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  writeLeftSideSpeed(currentLeftCommand);
  writeRightSideSpeed(currentRightCommand);
}

void stopDrive() {
  applyDriveSpeeds(0, 0);
}

// =====================================================
// Ramp controller
// =====================================================

void enterState(RampState nextState) {
  if (rampState != nextState) {
    Serial.print("State: ");
    Serial.print(stateName(rampState));
    Serial.print(" -> ");
    Serial.println(stateName(nextState));
  }

  rampState = nextState;
  stateStartedMs = millis();

  if (nextState == STATE_ON_RAMP) {
    rampStartTicks = forwardTicks();
    rampEnterFrames = 0;
    flatExitFrames = 0;
    wallExitFrames = 0;
    lastWallConditionFrameMs = 0;
    stallStartedMs = 0;
  } else if (nextState == STATE_EXIT_CLEAR) {
    exitStartTicks = forwardTicks();
    flatExitFrames = 0;
  }
}

void stopWithFault(const char *message) {
  setFault(message);
  stopDrive();
  enterState(STATE_FAULT);
  Serial.print("FAULT: ");
  Serial.println(faultMessage);
}

void finishRun() {
  stopDrive();
  enterState(STATE_DONE);
  Serial.println("Ramp run complete.");
}

void setClimbModeActive(bool active) {
  if (climbModeActive == active) {
    return;
  }

  climbModeActive = active;
  Serial.print("Climb mode ");
  Serial.println(climbModeActive ? "ON" : "OFF");
}

void updateRampCounters(float absPitch) {
  if (absPitch >= RAMP_ENTER_PITCH_DEG) {
    if (rampEnterFrames < 255) {
      rampEnterFrames++;
    }
  } else if (rampEnterFrames > 0) {
    rampEnterFrames--;
  }

  if (absPitch <= FLAT_EXIT_PITCH_DEG) {
    if (flatExitFrames < 255) {
      flatExitFrames++;
    }
  } else {
    flatExitFrames = 0;
  }
}

void updateClimbModeDetection(float absPitch) {
  if (absPitch >= CLIMB_MODE_ENTER_PITCH_DEG) {
    if (climbModeEnterFrames < 255) {
      climbModeEnterFrames++;
    }
  } else {
    climbModeEnterFrames = 0;
  }

  if (!climbModeActive && climbModeEnterFrames >= CLIMB_MODE_ENTER_FRAMES) {
    climbModeExitFrames = 0;
    setClimbModeActive(true);
  }

  if (absPitch <= CLIMB_MODE_EXIT_PITCH_DEG) {
    if (climbModeExitFrames < 255) {
      climbModeExitFrames++;
    }
  } else {
    climbModeExitFrames = 0;
  }

  if (climbModeActive && climbModeExitFrames >= CLIMB_MODE_EXIT_FRAMES) {
    climbModeEnterFrames = 0;
    setClimbModeActive(false);
  }
}

bool updateStallBoost(float avgTicksPerSec, int &baseCommand) {
  bool climbing = selectedProfile == PROFILE_ASCEND;
  bool onSlope = fabs(pitchDeg) >= RAMP_ENTER_PITCH_DEG;
  bool slow = avgTicksPerSec < STALL_TICKS_PER_SEC;

  if (!climbing || !onSlope || !slow) {
    stallStartedMs = 0;
    return false;
  }

  if (stallStartedMs == 0) {
    stallStartedMs = millis();
    return false;
  }

  unsigned long stalledFor = millis() - stallStartedMs;

  if (stalledFor > STALL_FAULT_MS) {
    stopWithFault("Climb stalled for too long.");
    return false;
  }

  if (stalledFor > STALL_BOOST_DELAY_MS) {
    baseCommand += STALL_BOOST_COMMAND;
    return true;
  }

  return false;
}

void applyStraightApproachDrive(int targetTicksPerSec) {
  float leftRate = leftTicksPerSec();
  float rightRate = rightTicksPerSec();
  float avgRate = (leftRate + rightRate) * 0.5;

  int target = max(120, targetTicksPerSec + speedTrimTicksPerSec);
  float speedError = target - avgRate;
  int baseCommand = LEVEL_FEEDFORWARD_COMMAND + (int)(SPEED_KP * speedError);
  baseCommand = constrain(baseCommand, MOTOR_MIN_DESCENT_COMMAND, MOTOR_MAX_SPEED);

  lastBaseCommand = baseCommand;
  lastYawCorrection = 0;
  lastBalanceCorrection = 0;
  lastWallCorrection = 0.0;
  lastPivotAssistActive = false;

  applyDriveSpeeds(baseCommand, baseCommand);
}

void applyRampDrive(int targetTicksPerSec) {
  float leftRate = leftTicksPerSec();
  float rightRate = rightTicksPerSec();
  float avgRate = (leftRate + rightRate) * 0.5;
  float absPitch = fabs(pitchDeg);

  int target = max(120, targetTicksPerSec + speedTrimTicksPerSec);
  float speedError = target - avgRate;
  int baseCommand = LEVEL_FEEDFORWARD_COMMAND + (int)(SPEED_KP * speedError);

  if (selectedProfile == PROFILE_ASCEND) {
    baseCommand += (int)(UPHILL_PITCH_GAIN * absPitch);
    baseCommand = max(baseCommand, MOTOR_MIN_CLIMB_COMMAND);
  } else {
    baseCommand -= (int)(DOWNHILL_PITCH_BRAKE_GAIN * absPitch);

    if (avgRate > target * 1.25) {
      baseCommand -= OVERSPEED_BRAKE_COMMAND;
    }

    baseCommand = max(baseCommand, MOTOR_MIN_DESCENT_COMMAND);
  }

  bool boostActive = updateStallBoost(avgRate, baseCommand);
  if (rampState == STATE_FAULT) {
    return;
  }

  baseCommand = constrain(baseCommand, MOTOR_MIN_DESCENT_COMMAND, MOTOR_MAX_SPEED);
  lastBaseCommand = baseCommand;

  float headingError = angleDiffDeg(targetYawDeg, yawDeg);
  int yawCorrection = imuReady
    ? constrain((int)(headingError * YAW_HOLD_KP), -MAX_SIDE_CORRECTION, MAX_SIDE_CORRECTION)
    : 0;

  int wallCorrection = calculateWallCorrection();

  if (wallCorrection != 0) {
    yawCorrection = (int)(yawCorrection * WALL_YAW_HOLD_SCALE);
  }
  lastYawCorrection = yawCorrection;

  float balanceError = leftRate - rightRate;
  float balanceScale = wallCorrection != 0 ? WALL_ENCODER_BALANCE_SCALE : 1.0;
  int balanceCorrection = constrain((int)(balanceError * ENCODER_BALANCE_KP),
                                    -MAX_SIDE_CORRECTION,
                                    MAX_SIDE_CORRECTION);
  balanceCorrection = (int)(balanceCorrection * balanceScale);
  lastBalanceCorrection = balanceCorrection;

  int leftCommand = baseCommand + yawCorrection - balanceCorrection;
  int rightCommand = baseCommand - yawCorrection + balanceCorrection;

  leftCommand -= wallCorrection;
  rightCommand += wallCorrection;
  lastPivotAssistActive = false;

  if (abs(wallCorrection) >= WALL_PIVOT_CORRECTION_THRESHOLD) {
    lastPivotAssistActive = true;

    if (wallCorrection > 0) {
      leftCommand = min(leftCommand, -WALL_PIVOT_REVERSE_SPEED);
      rightCommand = max(rightCommand, WALL_PIVOT_OUTER_MIN_SPEED);
    } else {
      leftCommand = max(leftCommand, WALL_PIVOT_OUTER_MIN_SPEED);
      rightCommand = min(rightCommand, -WALL_PIVOT_REVERSE_SPEED);
    }
  }

  if (boostActive) {
    leftCommand += 30;
    rightCommand += 30;
  }

  applyDriveSpeeds(leftCommand, rightCommand);
}

void resetRampRunVariables() {
  resetEncoders();
  resetYaw();

  runStartTicks = forwardTicks();
  rampStartTicks = runStartTicks;
  exitStartTicks = runStartTicks;
  targetYawDeg = yawDeg;
  missionStartMs = millis();
  stateStartedMs = missionStartMs;
  stallStartedMs = 0;
  rampEnterFrames = 0;
  flatExitFrames = 0;
  climbModeEnterFrames = 0;
  climbModeExitFrames = 0;
  wallEntryFrames = 0;
  wallExitFrames = 0;
  lastWallConditionFrameMs = 0;
  climbModeActive = false;
  faultMessage[0] = '\0';
}

void startRampRun(RampProfile profile) {
  selectedProfile = profile;

  if (!imuReady) {
    Serial.println("MPU6050 is not ready; using ultrasonic entry/exit.");
  }

  resetRampRunVariables();
  selectWallForRun();
  enterState(STATE_APPROACH);

  Serial.print("Ramp run started. Profile=");
  Serial.println(profileName(selectedProfile));
}

void stopRampRun() {
  stopDrive();
  enterState(STATE_IDLE);
  Serial.println("Ramp run stopped.");
}

void updateRampStateMachine() {
  if (rampState == STATE_IDLE || rampState == STATE_DONE || rampState == STATE_FAULT) {
    stopDrive();
    return;
  }

  unsigned long now = millis();
  if (now - missionStartMs > RUN_TIMEOUT_MS) {
    stopWithFault("Run timeout.");
    return;
  }

  float absPitch = fabs(pitchDeg);
  updateRampCounters(absPitch);
  updateClimbModeDetection(absPitch);

  if (rampState == STATE_APPROACH) {
    applyStraightApproachDrive(approachTargetTicksPerSec);

    if (lastDistanceMs != lastWallConditionFrameMs) {
      lastWallConditionFrameMs = lastDistanceMs;

      if (bothSideDistancesAtMost(WALL_ENTRY_MAX_CM)) {
        if (wallEntryFrames < 255) {
          wallEntryFrames++;
        }
      } else {
        wallEntryFrames = 0;
      }
    }

    if (wallEntryFrames >= WALL_ENTRY_CONFIRM_FRAMES) {
      enterState(STATE_ON_RAMP);
      return;
    }

    long approachTicks = abs(forwardTicks() - runStartTicks);
    if (approachTicks > APPROACH_MAX_TICKS) {
      stopWithFault("Both side walls were not detected during approach.");
    }
  } else if (rampState == STATE_ON_RAMP) {
    int target = selectedProfile == PROFILE_ASCEND
      ? ascendTargetTicksPerSec
      : descendTargetTicksPerSec;

    applyRampDrive(target);

    long rampTicks = abs(forwardTicks() - rampStartTicks);

    if (lastDistanceMs != lastWallConditionFrameMs) {
      lastWallConditionFrameMs = lastDistanceMs;

      if (bothSideDistancesAtLeast(WALL_EXIT_MIN_CM)) {
        if (wallExitFrames < 255) {
          wallExitFrames++;
        }
      } else {
        wallExitFrames = 0;
      }
    }

    if (wallExitFrames >= WALL_EXIT_CONFIRM_FRAMES) {
      finishRun();
      return;
    }

    if (rampTicks > RAMP_MAX_TICKS) {
      Serial.println("Ramp max ticks reached; stopping.");
      finishRun();
      return;
    }
  } else if (rampState == STATE_EXIT_CLEAR) {
    applyRampDrive(exitTargetTicksPerSec);

    long exitTicks = abs(forwardTicks() - exitStartTicks);
    unsigned long exitMs = now - stateStartedMs;

    if (exitTicks >= EXIT_CLEAR_TICKS || exitMs >= EXIT_CLEAR_MAX_MS) {
      finishRun();
    }
  }
}

// =====================================================
// Controls and telemetry
// =====================================================

void updateStatusLED() {
  if (rampState == STATE_FAULT) {
    bool blinkOn = ((millis() / 200) % 2) == 0;
    digitalWrite(LED_RED_PIN, blinkOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }

  bool running = rampState == STATE_APPROACH ||
                 rampState == STATE_ON_RAMP ||
                 rampState == STATE_EXIT_CLEAR;

  digitalWrite(LED_RED_PIN, running ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, running ? HIGH : LOW);
}

void handleButtons() {
  bool startDown = digitalRead(START_BUTTON_PIN) == LOW;
  bool stopDown = digitalRead(STOP_BUTTON_PIN) == LOW;

  if (startDown && !previousStartDown) {
    startRampRun(selectedProfile);
  }

  if (stopDown && !previousStopDown) {
    stopRampRun();
  }

  previousStartDown = startDown;
  previousStopDown = stopDown;
}

void printTelemetry() {
  Serial.print("state=");
  Serial.print(stateName(rampState));
  Serial.print(" profile=");
  Serial.print(profileName(selectedProfile));
  Serial.print(" climbMode=");
  Serial.print(climbModeActive ? "Y" : "N");
  Serial.print(" cmdL=");
  Serial.print(currentLeftCommand);
  Serial.print(" cmdR=");
  Serial.print(currentRightCommand);

  Serial.print(" ticks L=");
  Serial.print(leftTicks());
  Serial.print(" R=");
  Serial.print(rightTicks());
  Serial.print(" F=");
  Serial.print(forwardTicks());

  Serial.print(" tps L=");
  Serial.print(leftTicksPerSec(), 1);
  Serial.print(" R=");
  Serial.print(rightTicksPerSec(), 1);
  Serial.print(" avg=");
  Serial.print(averageTicksPerSec(), 1);

  Serial.print(" pitch=");
  Serial.print(pitchDeg, 1);
  Serial.print(" roll=");
  Serial.print(rollDeg, 1);
  Serial.print(" yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" imu=");
  Serial.print(imuReady ? "Y" : "N");

  Serial.print(" | dist L=");
  if (isDistanceValid(leftDistanceCm)) {
    Serial.print(leftDistanceCm, 1);
  } else {
    Serial.print("---");
  }
  Serial.print(" R=");
  if (isDistanceValid(rightDistanceCm)) {
    Serial.print(rightDistanceCm, 1);
  } else {
    Serial.print("---");
  }
  Serial.print("cm");
  Serial.print(" targetDelta=");
  Serial.print(WALL_EQUALITY_TARGET_DELTA_CM, 1);
  Serial.print(" err=");
  Serial.print(lastWallErrorCm, 1);
  Serial.print(" corr=");
  Serial.print(lastWallCorrection, 1);
  Serial.print(" base=");
  Serial.print(lastBaseCommand);
  Serial.print(" yawCorr=");
  Serial.print(lastYawCorrection);
  Serial.print(" balCorr=");
  Serial.print(lastBalanceCorrection);
  Serial.print(" steerSign=");
  Serial.print(wallSteeringSign);
  Serial.print(" pivot=");
  Serial.print(lastPivotAssistActive ? "Y" : "N");

  if (rampState == STATE_FAULT) {
    Serial.print(" fault=");
    Serial.print(faultMessage);
  }

  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("--- Trial Run 2 Task 5 + Task 6 Ramp + Wall Control ---");
  Serial.println("u = select ascend profile");
  Serial.println("n = select descend profile");
  Serial.println("g = start selected profile");
  Serial.println("s = stop");
  Serial.println("Task 6 left/right ultrasonic wall equality hold is always on");
  Serial.println("wall following starts after both side distances are under 10cm for 3 frames");
  Serial.println("run stops after both side distances are over 15cm for 5 frames");
  Serial.println("c = recalibrate IMU on flat ground");
  Serial.println("z = reset encoders and yaw");
  Serial.println("p = print telemetry");
  Serial.println("d = refresh and print left/right ultrasonic distances");
  Serial.println("f = flip wall steering correction direction");
  Serial.println("+/- = change target encoder speed trim");
  Serial.println("h/? = help");
  Serial.print("Selected profile: ");
  Serial.println(profileName(selectedProfile));
  Serial.print("Speed trim ticks/sec: ");
  Serial.println(speedTrimTicksPerSec);
  Serial.print("Wall hold: dual sensor equality, target delta=");
  Serial.print(WALL_EQUALITY_TARGET_DELTA_CM, 1);
  Serial.println("cm");
  Serial.print("Wall steering sign: ");
  Serial.println(wallSteeringSign);
  Serial.println("D32 starts, D33 stops.");
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char command = Serial.read();

    if (command == '\n' || command == '\r' || command == ' ') {
      continue;
    }

    if (command == 'u' || command == 'U') {
      selectedProfile = PROFILE_ASCEND;
      Serial.println("Selected profile: ASCEND");
    } else if (command == 'n' || command == 'N') {
      selectedProfile = PROFILE_DESCEND;
      Serial.println("Selected profile: DESCEND");
    } else if (command == 'g' || command == 'G') {
      startRampRun(selectedProfile);
    } else if (command == 's' || command == 'S') {
      stopRampRun();
    } else if (command == 'c' || command == 'C') {
      stopRampRun();
      calibrateIMU();
      Serial.println("IMU recalibrated.");
    } else if (command == 'z' || command == 'Z') {
      resetEncoders();
      resetYaw();
      Serial.println("Encoders and yaw reset.");
    } else if (command == 'p' || command == 'P') {
      printTelemetry();
    } else if (command == 'd' || command == 'D') {
      refreshAllDistancesBlocking();
      calculateWallCorrection();
      printTelemetry();
    } else if (command == 'f' || command == 'F') {
      wallSteeringSign *= -1;
      previousWallErrorCm = 0.0;
      lastWallErrorCm = 0.0;
      lastWallCorrection = 0.0;
      previousWallMs = millis();
      Serial.print("Wall steering sign=");
      Serial.println(wallSteeringSign);
    } else if (command == '+' || command == '=') {
      speedTrimTicksPerSec = constrain(speedTrimTicksPerSec + 40, -240, 360);
      Serial.print("Speed trim ticks/sec=");
      Serial.println(speedTrimTicksPerSec);
    } else if (command == '-' || command == '_') {
      speedTrimTicksPerSec = constrain(speedTrimTicksPerSec - 40, -240, 360);
      Serial.print("Speed trim ticks/sec=");
      Serial.println(speedTrimTicksPerSec);
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
  unsigned long startWait = millis();

  while (!Serial && millis() - startWait < 3000) {
  }

  Serial.println();
  Serial.println("Trial Run 2 Task 5 + Task 6 ramp and wall control starting...");
  Serial.println("Keep robot flat and still during IMU calibration.");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  updateStatusLED();

  setupEncoders();
  setupDistanceSensors();
  setupMotorControllers();

  imuReady = setupIMU();

  stopDrive();
  resetEncoders();
  resetYaw();
  enterState(STATE_IDLE);
  printHelp();
}

void loop() {
  handleSerialCommands();
  handleButtons();

  updateAllEncoders();
  updateEncoderSpeeds();
  updateIMU();
  updateDistanceSensors();
  updateRampStateMachine();
  updateStatusLED();

  static unsigned long lastTelemetryMs = 0;
  if (millis() - lastTelemetryMs >= 300 && rampState != STATE_IDLE) {
    lastTelemetryMs = millis();
    printTelemetry();
  }

  delay(4);
}

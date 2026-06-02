/*
 * dead_reckoning_hop.ino
 * Arduino Giga R1 WiFi - Task 4: Open-Field Dead Reckoning
 *
 * Main modes:
 *   1) SINGLE HOP    - 'g'. Robot drives 25 cm forward with encoder + IMU
 *                      and stops only after an RFID marker is present.
 *   2) FIXED PATH    - Start button or 'm'. Executes the Fig. 1 staircase:
 *                        Seg 1: FIXED_NORTH_1 nodes north
 *                        Seg 2: FIXED_EAST_NODES sideways (east or west)
 *                        Seg 3: FIXED_NORTH_2 nodes north
 *                      RFID is presence-only; tag identity is ignored.
 *
 * Compass convention:
 *   N / E / S / W are relative to how the robot is placed at the start.
 *
 * Sensors / hardware:
 *   Wire   (D20/D21)  : MPU6050 0x68
 *   Wire1  (SDA1/SCL1): RFID MFRC522 0x28, Motoron 0x10 (left), 0x11 (right)
 *   Encoders H1 only RISING: RF=D48 LF=D50 RB=D42 LB=D44
 *   Ultrasonics: Front 36/37, Left 39/38, Right 41/40
 *   UI: Start D32, Stop D33, Red LED D34, Green LED D35
 *
 * Serial commands (line-based, end with newline):
 *   m                       FIXED PATH mission (Fig. 1 staircase) ← main Task 4
 *   g                       single 25 cm hop (debug)
 *   s                       abort current motion
 *   d                       sensor telemetry
 *   r                       RFID presence read
 *   i                       IMU live read
 *   c                       recalibrate gyro bias
 *   z                       zero encoders
 */

#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include "types.h"

// ===========================================================================
//  TUNING SECTION
// ===========================================================================

// ---- Mission distances -----------------------------------------------------
const float TARGET_DIST_CM   = 25.0f;
const float MAX_DIST_CM      = 35.0f;
const unsigned long HOP_TIMEOUT_MS  = 8000UL;
const unsigned long PATH_TIMEOUT_MS = 60000UL;  // 1-minute task budget

// ---- Fixed path (Task 4 / Fig. 1 staircase — TUNE THESE) ------------------
//   Segment 1: drive FIXED_NORTH_1 nodes north
//   Segment 2: turn and drive FIXED_EAST_NODES nodes sideways
//   Segment 3: turn back north and drive FIXED_NORTH_2 nodes
//   FIXED_TURN_RIGHT: true = turn east (+col), false = turn west (-col)
const int  FIXED_NORTH_1    = 1;     // 1 node forward
const int  FIXED_EAST_NODES = 1;     // 1 node right (east)
const int  FIXED_NORTH_2    = 1;     // 1 node forward
const bool FIXED_TURN_RIGHT = true;  // right turn = east (+col)

// ---- Robot mechanics -------------------------------------------------------
const float WHEEL_DIAMETER_MM = 65.0f;
const int   ENCODER_CPR       = 144;
const float MM_PER_COUNT      = (WHEEL_DIAMETER_MM * 3.14159265f) / (float)ENCODER_CPR;

// ---- Forward drive speeds (scaled for 11 V supply, was 7.4 V) -------------
const int   CRUISE_SPEED      = 350;   // 300 × 0.67
const int   SCAN_SPEED        = 170;   // 250 × 0.67
const float SLOWDOWN_FRACTION = 0.75f;

// ---- Heading correction during forward motion ------------------------------
const float HEADING_KP_IMU   = 22.0f;
const float HEADING_KP_ENC   = 1.5f;
const int   MAX_HEADING_CORR = 170;   // 250 × 0.67
const float IMU_YAW_SIGN     = -1.0f;   // flip to -1.0 if gyro Z sign is reversed

// ---- Turn-in-place primitive ----------------------------------------------
const int   TURN_SPEED_FAST       = 550;   // needs to be high enough to pivot all 4 wheels
const int   TURN_SPEED_SLOW       = 150;   // must be high enough to overcome stiction
const float TURN_SLOW_THRESHOLD_DEG   = 40.0f;
const float TURN_OVERSHOOT_BUFFER_DEG =  4.0f;
// Gyro overreads on this hardware: physical 50° = ~90° heading.
// TURN_90_DEG is the heading target that produces a physical 90° turn.
// Formula: TURN_90_DEG = 90 * (observed_heading / observed_physical)
//          = 90 * (86 / 50) ≈ 155  (+4 for the buffer) = 158
// If turns are still short → increase; if they overshoot → decrease.
const float TURN_90_DEG = 120.0f;

// ---- Pre-turn alignment ------------------------------------------------
// Before each 90-degree turn the robot drives this far, then turns into
// the next segment. Tune freely.
const float PRE_TURN_CM = 2.0f;
const unsigned long TURN_TIMEOUT_MS = 4000UL;
const unsigned long TURN_SETTLE_MS  = 200UL;

// ---- IMU calibration -------------------------------------------------------
const int  GYRO_CAL_SAMPLES   = 200;
const int  GYRO_CAL_DELAY_MS  = 5;

// ---- RFID timing -----------------------------------------------------------
// Ignore RFID until part-way through a hop so the start marker is not counted
// as the next marker.
const float RFID_ARM_FRACTION = 0.60f;

// ---- Safety ----------------------------------------------------------------
const float FRONT_SAFETY_CM   = 8.0f;
const float SENSOR_OFFSET_CM  = 4.0f;

// ---- Ultrasonic limits -----------------------------------------------------
const float MAX_VALID_CM   = 180.0f;
const float MIN_VALID_CM   = 1.5f;
const int   SENSOR_SAMPLES = 3;

// ===========================================================================
//  PIN DEFINITIONS
// ===========================================================================

const int ENC_RF_H1 = 48;
const int ENC_LF_H1 = 50;
const int ENC_RB_H1 = 42;
const int ENC_LB_H1 = 44;

#define FRONT_TRIG  36
#define FRONT_ECHO  37
#define LEFT_TRIG   39
#define LEFT_ECHO   38
#define RIGHT_TRIG  41
#define RIGHT_ECHO  40

#define BTN_START   32
#define BTN_STOP    33
#define LED_RED     34
#define LED_GREEN   35

// ---- Front bumper key-cap switches — CHANGE pins to match your wiring -----
#define BUMPER_L_PIN  32   // ← left  key cap
#define BUMPER_R_PIN  33  // ← right key cap
// Either switch pressed (LOW with INPUT_PULLUP) starts the staircase mission.

#define ADDR_LEFT   0x10
#define ADDR_RIGHT  0x11
const int LEFT_MOTOR_SIGN  =  1;
const int RIGHT_MOTOR_SIGN = -1;

const byte RFID_I2C_ADDRESS = 0x28;
const byte MPU_I2C_ADDRESS  = 0x68;

#define MPU_PWR_MGMT_1   0x6B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG       0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_GYRO_ZOUT_H  0x47
#define MPU_WHO_AM_I     0x75

// ===========================================================================
//  RFID NODE DETECTION
//  RFID is used as a presence sensor only. Tag identity is ignored.
// ===========================================================================

// ===========================================================================
//  COMPASS
// ===========================================================================

Facing currentFacing = F_NORTH;

// ===========================================================================
//  GLOBALS
// ===========================================================================

MotoronI2C motorLeft;
MotoronI2C motorRight;
MFRC522_I2C rfid(RFID_I2C_ADDRESS, -1, &Wire1);  // Wire1 — same bus as Motoron

const long TARGET_COUNTS = (long)((TARGET_DIST_CM * 10.0f) / MM_PER_COUNT);
const long MAX_COUNTS    = (long)((MAX_DIST_CM    * 10.0f) / MM_PER_COUNT);

volatile long encCountRF = 0, encCountLF = 0, encCountRB = 0, encCountLB = 0;
void isrRF() { encCountRF++; } void isrLF() { encCountLF++; }
void isrRB() { encCountRB++; } void isrLB() { encCountLB++; }

bool  imuPresent = false;
float gyroZBias  = 0.0f;
float headingDeg = 0.0f;
unsigned long lastImuMs = 0;

// Top-level state: drives LED, blocks new commands while running
enum SystemState { SYS_IDLE, SYS_BUSY };
volatile SystemState sysState = SYS_IDLE;
volatile bool abortRequested = false;

// Button state is now tracked via static locals inside handleButtons() — see teammate pattern.

// ===========================================================================
//  MPU6050 helpers
// ===========================================================================

void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_I2C_ADDRESS);
  Wire.write(reg); Wire.write(value);
  Wire.endTransmission();
}
uint8_t mpuRead(uint8_t reg) {
  Wire.beginTransmission(MPU_I2C_ADDRESS); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_I2C_ADDRESS, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
bool mpuReadBytes(uint8_t reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_I2C_ADDRESS); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)MPU_I2C_ADDRESS, count);
  uint8_t i = 0;
  while (Wire.available() && i < count) buf[i++] = Wire.read();
  return i == count;
}
bool mpuProbe() {
  Wire.beginTransmission(MPU_I2C_ADDRESS);
  return Wire.endTransmission() == 0;
}
void mpuInit() {
  mpuWrite(MPU_PWR_MGMT_1, 0x00); delay(100);
  mpuWrite(MPU_SMPLRT_DIV, 0x07);
  mpuWrite(MPU_CONFIG, 0x06);
  mpuWrite(MPU_GYRO_CONFIG, 0x00);
  mpuWrite(MPU_ACCEL_CONFIG, 0x00);
}
float mpuReadGyroZ_raw() {
  uint8_t data[2];
  if (!mpuReadBytes(MPU_GYRO_ZOUT_H, 2, data)) return 0.0f;
  int16_t raw = ((int16_t)data[0] << 8) | data[1];
  return (float)raw / 131.0f;
}
void calibrateGyroBias() {
  Serial.print("Gyro cal ("); Serial.print(GYRO_CAL_SAMPLES); Serial.print(" samples)...");
  float sum = 0.0f;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    sum += mpuReadGyroZ_raw();
    if ((i & 0x07) == 0) {
      digitalWrite(LED_RED,   (i & 0x10) ? LOW  : HIGH);
      digitalWrite(LED_GREEN, (i & 0x10) ? HIGH : LOW);
    }
    delay(GYRO_CAL_DELAY_MS);
  }
  gyroZBias = sum / (float)GYRO_CAL_SAMPLES;
  Serial.print(" bias="); Serial.print(gyroZBias, 4); Serial.println(" deg/s");
}
void resetHeading() { headingDeg = 0.0f; lastImuMs = 0; }
void updateHeading() {
  if (!imuPresent) return;
  unsigned long now = millis();
  if (lastImuMs == 0) { lastImuMs = now; return; }
  float dt = (now - lastImuMs) / 1000.0f;
  lastImuMs = now;
  float gz = (mpuReadGyroZ_raw() - gyroZBias) * IMU_YAW_SIGN;
  headingDeg += gz * dt;
}

// ===========================================================================
//  Encoder / ultrasonic / motor / RFID helpers
// ===========================================================================

void resetEncoders() {
  noInterrupts();
  encCountRF = encCountLF = encCountRB = encCountLB = 0;
  interrupts();
}
void readEncoders(long &lf, long &lb, long &rf, long &rb) {
  noInterrupts(); lf=encCountLF; lb=encCountLB; rf=encCountRF; rb=encCountRB; interrupts();
}
long avgLeftCounts()  { long lf,lb,rf,rb; readEncoders(lf,lb,rf,rb); return (lf+lb)/2; }
long avgRightCounts() { long lf,lb,rf,rb; readEncoders(lf,lb,rf,rb); return (rf+rb)/2; }

float singlePing(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return MAX_VALID_CM;
  float dist = duration * 0.0343f / 2.0f;
  return (dist < MIN_VALID_CM || dist > MAX_VALID_CM) ? MAX_VALID_CM : dist;
}
float readDistance(int trigPin, int echoPin) {
  float r[SENSOR_SAMPLES];
  for (int i = 0; i < SENSOR_SAMPLES; i++) { r[i] = singlePing(trigPin, echoPin); delay(3); }
  if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
  if (r[1] > r[2]) { float t = r[1]; r[1] = r[2]; r[2] = t; }
  if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
  float median = r[1];
  if (median >= MAX_VALID_CM) return median;
  float adjusted = median - SENSOR_OFFSET_CM;
  return (adjusted < 0.0f) ? 0.0f : adjusted;
}

int clampSpeed(int s) { return constrain(s, -800, 800); }
void drive(int leftSpd, int rightSpd) {
  int l = clampSpeed(leftSpd)  * LEFT_MOTOR_SIGN;
  int r = clampSpeed(rightSpd) * RIGHT_MOTOR_SIGN;
  motorLeft.setSpeed(1, l);  motorLeft.setSpeed(2, l);  motorLeft.setSpeed(3, l);
  motorRight.setSpeed(1, r); motorRight.setSpeed(2, r); motorRight.setSpeed(3, r);
}
void stopMotors() {
  motorLeft.setSpeed(1, 0);  motorLeft.setSpeed(2, 0);  motorLeft.setSpeed(3, 0);
  motorRight.setSpeed(1, 0); motorRight.setSpeed(2, 0); motorRight.setSpeed(3, 0);
}
void initMotor(MotoronI2C &mc, const char *label) {
  mc.reinitialize(); delay(20);
  mc.disableCrc(); mc.clearResetFlag();
  mc.clearMotorFaultUnconditional(); mc.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 3; ch++) {
    mc.setMaxAcceleration(ch, 800);  // matches sensor_motor_integration_test
    mc.setMaxDeceleration(ch, 800);
  }
  Serial.print(label); Serial.println(" motor ready.");
}

// driveNow: identical sign logic to drive(), but uses setSpeedNow() which
// bypasses the acceleration ramp — required for turns (from sensor_motor_integration_test).
void driveNow(int leftSpd, int rightSpd) {
  int l = clampSpeed(leftSpd)  * LEFT_MOTOR_SIGN;
  int r = clampSpeed(rightSpd) * RIGHT_MOTOR_SIGN;
  motorLeft.setSpeedNow(1, l);  motorLeft.setSpeedNow(2, l);  motorLeft.setSpeedNow(3, l);
  motorRight.setSpeedNow(1, r); motorRight.setSpeedNow(2, r); motorRight.setSpeedNow(3, r);
}

bool readRfidPresent() {
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial()) return false;

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return true;
}

// Uses 50 ms between attempts — matches reference code delay(50).
bool captureTagBriefly(int attempts = 25, int delayMs = 50) {
  for (int i = 0; i < attempts; i++) {
    if (readRfidPresent()) return true;
    delay(delayMs);
  }
  return false;
}

// ===========================================================================
//  Forward declarations
// ===========================================================================
void handleButtons();
void handleSerial();
void dumpStatus(const char *prefix);
void dumpImu();

// ===========================================================================
//  PRIMITIVE 1: turn 90 degrees (CW = right, CCW = left)
//  Returns true on success. Caller must have set sysState = SYS_BUSY.
// ===========================================================================
bool turnDegrees(float targetDeltaDeg) {
  Serial.print("  turn "); Serial.print(targetDeltaDeg, 1); Serial.println(" deg");
  resetHeading();
  unsigned long startMs = millis();

  while (true) {
    handleButtons();
    handleSerial();
    updateHeading();
    if (abortRequested) { driveNow(0, 0); return false; }
    if (millis() - startMs > TURN_TIMEOUT_MS) {
      driveNow(0, 0);
      Serial.println("  TURN TIMEOUT");
      return false;
    }

    float remaining = targetDeltaDeg - headingDeg;
    if (fabs(remaining) <= TURN_OVERSHOOT_BUFFER_DEG) { driveNow(0, 0); break; }

    int spd = (fabs(remaining) > TURN_SLOW_THRESHOLD_DEG) ? TURN_SPEED_FAST : TURN_SPEED_SLOW;
    // On this robot driveNow(-,+) is physically CW (right) and makes headingDeg increase.
    // So: remaining > 0 → turn RIGHT (CW),  remaining < 0 → turn LEFT (CCW).
    // Angle convention in turnToFacing() matches: right = +90°, left = -90°.
    // Uses setSpeedNow (from sensor_motor_integration_test) to bypass accel ramp.
    if (remaining > 0) driveNow(-spd,  spd);   // CW  — right — headingDeg increases
    else               driveNow( spd, -spd);   // CCW — left  — headingDeg decreases

    static unsigned long lastP = 0;
    if (millis() - lastP > 120) {
      lastP = millis();
      Serial.print("    hdg="); Serial.print(headingDeg, 1);
      Serial.print(" rem=");    Serial.println(remaining, 1);
    }
    delay(2);
  }
  delay(TURN_SETTLE_MS);
  return true;
}

bool turnToFacing(Facing target) {
  if (target == currentFacing) return true;
  // Compute shortest signed angle
  int delta = ((int)target - (int)currentFacing + 4) % 4;  // 1, 2, or 3
  float angle;
  // With IMU_YAW_SIGN=-1: CW = positive heading, CCW = negative heading.
  // TURN_90_DEG is calibrated so that the robot physically turns 90° (gyro overreads).
  if      (delta == 1) angle =  TURN_90_DEG;          // one step CW  = right
  else if (delta == 3) angle = -TURN_90_DEG;          // one step CCW = left
  else                 angle =  TURN_90_DEG * 2.0f;   // about-face
  bool ok = turnDegrees(angle);
  if (ok) currentFacing = target;
  return ok;
}

// ===========================================================================
//  Short alignment drive — encoder only, no RFID required
// ===========================================================================
void driveStraight(float distCm) {
  if (distCm <= 0.0f) return;
  resetEncoders();
  resetHeading();
  long targetCounts = (long)((distCm * 10.0f) / MM_PER_COUNT);
  unsigned long startMs = millis();

  while (true) {
    handleButtons();
    handleSerial();
    updateHeading();
    if (abortRequested) { stopMotors(); return; }
    if (millis() - startMs > 6000UL) { stopMotors(); return; }

    long lc = avgLeftCounts();
    long rc = avgRightCounts();
    if ((lc + rc) / 2 >= targetCounts) { stopMotors(); break; }

    int corrImu = (int)(-HEADING_KP_IMU * headingDeg);
    int corrEnc = (int)( HEADING_KP_ENC * (float)(lc - rc));
    int correction = constrain(corrImu + corrEnc, -MAX_HEADING_CORR, MAX_HEADING_CORR);
    drive(SCAN_SPEED - correction, SCAN_SPEED + correction);
    delay(3);
  }
  delay(100);  // brief pause before next action
}

// ===========================================================================
//  PRIMITIVE 2: drive forward one cell (25 cm + RFID presence confirm)
//  Returns true when distance is reached and an RFID marker is detected.
// ===========================================================================
bool driveOneHop() {
  Serial.println("  hop 25 cm, waiting for RFID marker");

  bool tagSeen = false;
  resetEncoders();
  resetHeading();
  unsigned long hopStartMs  = millis();
  unsigned long lastRfidPollMs = 0;    // gate RFID to 50 ms (reference code cadence)

  while (true) {
    handleButtons();
    handleSerial();
    updateHeading();
    if (abortRequested) { stopMotors(); return false; }

    if (millis() - hopStartMs > HOP_TIMEOUT_MS) {
      stopMotors(); Serial.println("  HOP TIMEOUT"); return false;
    }

    float frontDist = readDistance(FRONT_TRIG, FRONT_ECHO);
    if (frontDist > 0.0f && frontDist < FRONT_SAFETY_CM) {
      stopMotors(); Serial.println("  FRONT OBSTACLE"); return false;
    }

    long lc = avgLeftCounts();
    long rc = avgRightCounts();
    long avgCounts = (lc + rc) / 2;
    float distCm = avgCounts * MM_PER_COUNT / 10.0f;

    bool rfidArmed = avgCounts >= (long)(TARGET_COUNTS * RFID_ARM_FRACTION);
    // Poll RFID every 50 ms after the hop is far enough from the previous tag.
    if (rfidArmed && millis() - lastRfidPollMs >= 50) {
      lastRfidPollMs = millis();
      if (readRfidPresent()) {
        tagSeen = true;
        Serial.println("    [RFID] tag detected");
      }
    }

    if (avgCounts > MAX_COUNTS) {
      stopMotors(); Serial.println("  DISTANCE CAP, no RFID marker"); return false;
    }

    bool distOK = (avgCounts >= TARGET_COUNTS);
    if (distOK && tagSeen) { stopMotors(); break; }

    int baseSpeed = (avgCounts < (long)(TARGET_COUNTS * SLOWDOWN_FRACTION))
                    ? CRUISE_SPEED : SCAN_SPEED;

    long encDiff = lc - rc;
    int corrImu = (int)(-HEADING_KP_IMU * headingDeg);
    int corrEnc = (int)( HEADING_KP_ENC * (float)encDiff);
    int correction = constrain(corrImu + corrEnc, -MAX_HEADING_CORR, MAX_HEADING_CORR);

    drive(baseSpeed - correction, baseSpeed + correction);

    static unsigned long lastP = 0;
    if (millis() - lastP > 120) {
      lastP = millis();
      Serial.print("    d=");   Serial.print(distCm, 1);
      Serial.print(" hdg=");    Serial.print(headingDeg, 1);
      Serial.print(" corr=");   Serial.print(correction);
      Serial.print(" rfid=");
      Serial.println(tagSeen ? "seen" : (rfidArmed ? "armed" : "waiting"));
    }
    delay(3);
  }

  Serial.println("  hop complete");
  return true;
}

// ===========================================================================
//  FIXED PATH MISSION  (Task 4 / Fig. 1 staircase)
//  Start button (D32) or serial 'm'
//
//  Path shape:
//    Seg 1: face NORTH, drive FIXED_NORTH_1 nodes
//    Seg 2: face EAST (or WEST), drive FIXED_EAST_NODES nodes
//    Seg 3: face NORTH, drive FIXED_NORTH_2 nodes
//
//  RFID only confirms that a marker exists at the end of each hop.
// ===========================================================================

void runFixedPath() {
  if (sysState != SYS_IDLE) return;

  Serial.println();
  Serial.println("=== Task 4: Fixed Path Mission (Fig. 1 staircase) ===");
  Serial.print  ("  Seg 1 : North  x"); Serial.println(FIXED_NORTH_1);
  Serial.print  ("  Seg 2 : "); Serial.print(FIXED_TURN_RIGHT ? "East" : "West");
  Serial.print  ("   x"); Serial.println(FIXED_EAST_NODES);
  Serial.print  ("  Seg 3 : North  x"); Serial.println(FIXED_NORTH_2);
  Serial.println();

  // Fixed Task 4 assumes the robot is physically placed facing north.
  currentFacing = F_NORTH;

  sysState       = SYS_BUSY;
  abortRequested = false;
  digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, HIGH);

  if (imuPresent) { calibrateGyroBias(); resetHeading(); }

  unsigned long pathStartMs = millis();
  bool ok = true;

  // ---- Segment 1: face NORTH, drive FIXED_NORTH_1 nodes -------------------
  Serial.println("--- SEG 1: North ---");
  if (ok && !turnToFacing(F_NORTH)) ok = false;
  for (int i = 0; i < FIXED_NORTH_1 && ok; i++) {
    if (abortRequested || millis() - pathStartMs > PATH_TIMEOUT_MS) { ok = false; break; }
    if (!driveOneHop()) { ok = false; break; }
    delay(200);
  }

  // ---- Segment 2: face EAST / WEST, drive FIXED_EAST_NODES nodes ----------
  if (ok) {
    Facing sideFacing = FIXED_TURN_RIGHT ? F_EAST : F_WEST;
    Serial.print("--- SEG 2: ");
    Serial.print(FIXED_TURN_RIGHT ? "East" : "West"); Serial.println(" ---");
    Serial.println("  pre-turn align");
    driveStraight(PRE_TURN_CM);
    if (!turnToFacing(sideFacing)) ok = false;
  }
  for (int i = 0; i < FIXED_EAST_NODES && ok; i++) {
    if (abortRequested || millis() - pathStartMs > PATH_TIMEOUT_MS) { ok = false; break; }
    if (!driveOneHop()) { ok = false; break; }
    delay(200);
  }

  // ---- Segment 3: face NORTH, drive FIXED_NORTH_2 nodes -------------------
  if (ok) {
    Serial.println("--- SEG 3: North ---");
    Serial.println("  pre-turn align");
    driveStraight(PRE_TURN_CM);
    if (!turnToFacing(F_NORTH)) ok = false;
  }
  for (int i = 0; i < FIXED_NORTH_2 && ok; i++) {
    if (abortRequested || millis() - pathStartMs > PATH_TIMEOUT_MS) { ok = false; break; }
    if (!driveOneHop()) { ok = false; break; }
    delay(200);
  }

  // ---- Done ----------------------------------------------------------------
  stopMotors();
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);
  sysState = SYS_IDLE;

  Serial.println();
  if (ok) {
    Serial.print(">>> FIXED PATH SUCCESS  ");
    Serial.print(millis() - pathStartMs);
    Serial.println(" ms <<<");
  } else {
    Serial.println(">>> FIXED PATH FAILED <<<");
  }
}

// ===========================================================================
//  SINGLE-HOP MISSION (button D32 / 'g')
// ===========================================================================

void runSingleHop() {
  if (sysState != SYS_IDLE) return;
  sysState = SYS_BUSY;
  abortRequested = false;
  digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, HIGH);

  Serial.println();
  Serial.println("=== Single hop ===");

  if (imuPresent) calibrateGyroBias();

  bool ok = driveOneHop();

  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);
  sysState = SYS_IDLE;
  Serial.println(ok ? ">>> HOP SUCCESS <<<" : ">>> HOP FAILED <<<");
}

// ===========================================================================
//  Telemetry
// ===========================================================================

void dumpStatus(const char *prefix) {
  long lf, lb, rf, rb; readEncoders(lf, lb, rf, rb);
  float distCm = ((lf + lb + rf + rb) / 4.0f) * MM_PER_COUNT / 10.0f;
  if (prefix) Serial.print(prefix);
  Serial.print("ENC LF="); Serial.print(lf);
  Serial.print(" LB="); Serial.print(lb);
  Serial.print(" RF="); Serial.print(rf);
  Serial.print(" RB="); Serial.print(rb);
  Serial.print("  d=");  Serial.print(distCm, 1); Serial.print("cm");
  Serial.print("  hdg="); Serial.print(headingDeg, 2);
  Serial.print("  US F="); Serial.print(readDistance(FRONT_TRIG, FRONT_ECHO), 1);
  Serial.print(" L="); Serial.print(readDistance(LEFT_TRIG, LEFT_ECHO), 1);
  Serial.print(" R="); Serial.print(readDistance(RIGHT_TRIG, RIGHT_ECHO), 1);
  Serial.print("  RFID_now="); Serial.println(readRfidPresent() ? "detected" : "none");
}

void dumpImu() {
  if (!imuPresent) { Serial.println("IMU not present."); return; }
  float gz_raw = mpuReadGyroZ_raw();
  float gz_cal = (gz_raw - gyroZBias) * IMU_YAW_SIGN;
  Serial.print("IMU gz_raw="); Serial.print(gz_raw, 3); Serial.print(" deg/s  ");
  Serial.print("bias=");       Serial.print(gyroZBias, 4); Serial.print("  ");
  Serial.print("gz_cal=");     Serial.print(gz_cal, 3); Serial.print("  ");
  Serial.print("heading=");    Serial.print(headingDeg, 2); Serial.println(" deg");
}

// ===========================================================================
//  Command parsing (line-based)
// ===========================================================================

String serialBuf = "";

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // single-char commands first
  if (cmd.length() == 1) {
    char c = cmd.charAt(0);
    if      (c == 'm' || c == 'M') runFixedPath();
    else if (c == 'g' || c == 'G') runSingleHop();
    else if (c == 's' || c == 'S') { abortRequested = true; Serial.println("ABORT requested"); }
    else if (c == 'd' || c == 'D') dumpStatus("[manual] ");
    else if (c == 'i' || c == 'I') dumpImu();
    else if (c == 'c' || c == 'C') {
      if (imuPresent) { calibrateGyroBias(); resetHeading(); }
      else Serial.println("IMU not present.");
    }
    else if (c == 'r' || c == 'R') {
      Serial.println(readRfidPresent() ? "RFID: detected" : "RFID: none");
    }
    else if (c == 'z' || c == 'Z') { resetEncoders(); Serial.println("Encoders zeroed."); }
    else Serial.print("?cmd: "); Serial.println(cmd);
    return;
  }

  Serial.print("?cmd: "); Serial.println(cmd);
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) { processCommand(serialBuf); serialBuf = ""; }
    } else {
      serialBuf += c;
      if (serialBuf.length() > 80) serialBuf = "";
    }
  }
}

void handleButtons() {
  // Teammate pattern: static locals, no delay(), simple rising-edge detection.
  static bool prevStart = false;
  static bool prevStop  = false;

  bool startNow = (digitalRead(BTN_START) == LOW);  // D32 — bumper key caps
  bool stopNow  = (digitalRead(BTN_STOP)  == LOW);  // D33 — standalone stop

  // Rising edge on D32 → start staircase
  if (startNow && !prevStart) {
    if (sysState == SYS_IDLE) {
      Serial.println("Start / bumper pressed — running staircase");
      runFixedPath();
    }
  }

  // Rising edge on D33, only when D32 is NOT also pressed → abort
  // (if both are LOW simultaneously it is a bumper press, not a stop press)
  if (stopNow && !prevStop && !startNow) {
    abortRequested = true;
    Serial.println("ABORT (stop button)");
  }

  prevStart = startNow;
  prevStop  = stopNow;
}

// ===========================================================================
//  setup()
// ===========================================================================

void setup() {
  Serial.begin(115200);
  unsigned long sw = millis();
  while (!Serial && millis() - sw < 2000) {}

  Serial.println();
  Serial.println("=== Task 4: fixed path + RFID presence ===");
  Serial.println("RFID mode: presence only");
  Serial.print("MM/count: "); Serial.println(MM_PER_COUNT, 4);
  Serial.print("25 cm = "); Serial.print(TARGET_COUNTS); Serial.print(" counts (max ");
  Serial.print(MAX_COUNTS); Serial.println(")");

  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BUMPER_L_PIN, INPUT_PULLUP); pinMode(BUMPER_R_PIN, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);

  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG,  OUTPUT); pinMode(LEFT_ECHO,  INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);
  digitalWrite(FRONT_TRIG, LOW); digitalWrite(LEFT_TRIG, LOW); digitalWrite(RIGHT_TRIG, LOW);

  pinMode(ENC_RF_H1, INPUT_PULLUP); pinMode(ENC_LF_H1, INPUT_PULLUP);
  pinMode(ENC_RB_H1, INPUT_PULLUP); pinMode(ENC_LB_H1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_H1), isrRF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LF_H1), isrLF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RB_H1), isrRB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LB_H1), isrLB, RISING);

  Wire.begin();
  Wire.setClock(400000);

  imuPresent = mpuProbe();
  if (imuPresent) {
    mpuInit();
    Serial.print("MPU6050 ready (WHO_AM_I=0x"); Serial.print(mpuRead(MPU_WHO_AM_I), HEX); Serial.println(")");
    calibrateGyroBias();
    resetHeading();
  } else {
    Serial.println("WARNING: MPU6050 NOT found.");
  }

  // Wire1: RFID (0x28) + Motoron left (0x10) + Motoron right (0x11)
  Wire1.begin();
  Wire1.setClock(400000);   // RFID needs 400 kHz; Motoron M3S256 also handles it
  delay(100);

  Wire1.beginTransmission(RFID_I2C_ADDRESS);
  if (Wire1.endTransmission() == 0) {
    rfid.PCD_Init();
    Serial.println("RFID ready (0x28 on Wire1).");
  } else {
    Serial.println("WARNING: RFID not found on Wire1 0x28 — check SDA1/SCL1 wiring.");
  }
  delay(200);  // let Wire1 bus fully settle after PCD_Init before Motoron init

  motorLeft.setBus(&Wire1);  motorLeft.setAddress(ADDR_LEFT);
  initMotor(motorLeft, "Left");
  motorRight.setBus(&Wire1); motorRight.setAddress(ADDR_RIGHT);
  initMotor(motorRight, "Right");
  stopMotors();

  Serial.println();
  Serial.println("Commands (newline-terminated):");
  Serial.println("  m                       FIXED PATH mission (Fig 1 staircase) <-- Task 4");
  Serial.println("  g                       single 25 cm hop (debug)");
  Serial.println("  s                       abort current motion");
  Serial.println("  d / r / i / c / z       dump / RFID presence / IMU / recal / zero enc");
  Serial.print  ("  Fixed path: North x"); Serial.print(FIXED_NORTH_1);
  Serial.print  (" -> "); Serial.print(FIXED_TURN_RIGHT ? "East" : "West");
  Serial.print  (" x"); Serial.print(FIXED_EAST_NODES);
  Serial.print  (" -> North x"); Serial.println(FIXED_NORTH_2);
  Serial.println("  (edit FIXED_* constants in sketch to change node counts / direction)");
}

// ===========================================================================
//  loop()
// ===========================================================================

void loop() {
  handleButtons();
  handleSerial();
  updateHeading();

  // Idle telemetry while not busy
  if (sysState == SYS_IDLE) {
    static unsigned long lastIdle = 0;
    if (millis() - lastIdle >= 600) {
      lastIdle = millis();
      dumpStatus("[idle] ");
    }
  }
  delay(8);
}

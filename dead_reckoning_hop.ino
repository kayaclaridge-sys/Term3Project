/*
 * dead_reckoning_hop.ino
 * Arduino Giga R1 WiFi - Task 4: Open-Field Dead Reckoning
 *
 * Two modes:
 *   1) SINGLE HOP   - press D32 (or send 'g'). Robot drives 25 cm forward
 *                     using encoders + IMU heading hold + RFID confirm.
 *   2) PATH MISSION - send "go R7C2" (or "path R3C5 R7C2 N"). Robot turns to
 *                     face the target and chains multiple 25 cm hops.
 *
 * Compass convention (grid-relative):
 *   N = toward HIGHER row numbers (R1 -> R9 direction)
 *   S = toward LOWER  row numbers
 *   E = toward HIGHER column numbers (C1 -> C9 direction)
 *   W = toward LOWER  column numbers
 *
 * Sensors / hardware:
 *   Wire   (D20/D21): MPU6050 0x68, RFID MFRC522 0x28
 *   Wire1           : Motoron 0x10 (left), 0x11 (right)
 *   Encoders H1 only RISING: RF=D48 LF=D50 RB=D42 LB=D44
 *   Ultrasonics: Front 36/37, Left 39/38, Right 41/40
 *   UI: Start D32, Stop D33, Red LED D34, Green LED D35
 *
 * Serial commands (line-based, end with newline):
 *   g                       single 25 cm hop
 *   s                       abort current motion
 *   face N|E|S|W            set current facing
 *   at R#C#                 set current cell
 *   pose R#C# N|E|S|W       set full pose in one line
 *   go R#C#                 pathfind from current pose to R#C#
 *   path R#C# R#C# [F]      shortcut: set pose, then go
 *   d                       sensor + pose telemetry
 *   r                       single RFID read
 *   i                       IMU live read
 *   c                       recalibrate gyro bias
 *   z                       zero encoders
 */

#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>

// ===========================================================================
//  TUNING SECTION
// ===========================================================================

// ---- Mission distances -----------------------------------------------------
const float TARGET_DIST_CM   = 25.0f;
const float MAX_DIST_CM      = 35.0f;
const unsigned long HOP_TIMEOUT_MS = 8000UL;
const unsigned long PATH_TIMEOUT_MS = 45000UL;

// ---- Robot mechanics -------------------------------------------------------
const float WHEEL_DIAMETER_MM = 65.0f;
const int   ENCODER_CPR       = 144;
const float MM_PER_COUNT      = (WHEEL_DIAMETER_MM * 3.14159265f) / (float)ENCODER_CPR;

// ---- Forward drive speeds --------------------------------------------------
const int   CRUISE_SPEED      = 400;
const int   SCAN_SPEED        = 250;
const float SLOWDOWN_FRACTION = 0.75f;

// ---- Heading correction during forward motion ------------------------------
const float HEADING_KP_IMU   = 22.0f;
const float HEADING_KP_ENC   = 1.5f;
const int   MAX_HEADING_CORR = 250;
const float IMU_YAW_SIGN     = 1.0f;   // flip to -1.0 if gyro Z sign is reversed

// ---- Turn-in-place primitive ----------------------------------------------
const int   TURN_SPEED_FAST       = 380;
const int   TURN_SPEED_SLOW       = 220;
const float TURN_SLOW_THRESHOLD_DEG = 25.0f;
const float TURN_OVERSHOOT_BUFFER_DEG = 2.0f;
const unsigned long TURN_TIMEOUT_MS = 4000UL;
const unsigned long TURN_SETTLE_MS  = 200UL;

// ---- IMU calibration -------------------------------------------------------
const int  GYRO_CAL_SAMPLES   = 200;
const int  GYRO_CAL_DELAY_MS  = 5;

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
//  GRID MAP (UID -> row, col)
//   Rows 9-6: lined area (top of arena)
//   Rows 5-1: DEAD ZONE
// ===========================================================================

struct CellEntry {
  const char *uid;
  uint8_t row;
  uint8_t col;
};

const CellEntry GRID[] = {
  // Row 9 (top, lined)
  {"C3DFAA41",9,1},{"671BAB41",9,2},{"855AAB41",9,3},{"70CBAA41",9,4},{"6E54A641",9,5},
  {"7447AB41",9,6},{"1B0AAB41",9,7},{"418BAB41",9,8},{"43DB2CDD",9,9},
  // Row 8 (lined)
  {"2802AB41",8,1},{"7074AB41",8,2},{"DF54A941",8,3},{"03CCAA41",8,4},{"1D65AA41",8,5},
  {"F6B6A941",8,6},{"A42DAB41",8,7},{"F164AB41",8,8},{"A335126A",8,9},
  // Row 7 (lined)
  {"4A12AB41",7,1},{"685EAB41",7,2},{"E7F7AA41",7,3},{"9C01AB41",7,4},{"E238A941",7,5},
  {"54C4AA41",7,6},{"8CE5AA41",7,7},{"4E4DAB41",7,8},{"28E3AA41",7,9},
  // Row 6 (lined)
  {"BD47AB41",6,1},{"F94FAB41",6,2},{"CE9CAA41",6,3},{"060DAB41",6,4},{"6666AA41",6,5},
  {"B3DA2ADD",6,6},{"D3DDAA41",6,7},{"0D46AB41",6,8},{"A142AB41",6,9},
  // Row 5 (DEAD ZONE begins)
  {"5663AB41",5,1},{"0077AB41",5,2},{"48CBAA41",5,3},{"F85EAB41",5,4},{"4FC0AA41",5,5},
  {"AE55AA41",5,6},{"41AB4141",5,7},{"FCD6AA41",5,8},{"D157AB41",5,9},
  // Row 4
  {"9259AB41",4,1},{"3D84AB41",4,2},{"70D7AA41",4,3},{"B811AB41",4,4},{"3ACEAA41",4,5},
  {"6C5FAB41",4,6},{"F459AB41",4,7},{"47FAAA41",4,8},{"773DAB41",4,9},
  // Row 3
  {"7451AB41",3,1},{"B493AB41",3,2},{"6D19AB41",3,3},{"8A45AB41",3,4},{"9312AB41",3,5},
  {"AC5CAB41",3,6},{"E840AB41",3,7},{"F052AB41",3,8},{"10C7AA41",3,9},
  // Row 2
  {"7C88AB41",2,1},{"2A60AB41",2,2},{"E74BA941",2,3},{"C47CAB41",2,4},{"BCCFAA41",2,5},
  {"07F6AA41",2,6},{"3385AB41",2,7},{"573DAB41",2,8},{"F642AB41",2,9},
  // Row 1 (bottom)
  {"F07EAB41",1,1},{"528AAB41",1,2},{"375CAB41",1,3},{"8145A941",1,4},{"76F0AA41",1,5},
  {"F63BAB41",1,6},{"9017AB41",1,7},{"390DAB41",1,8},{"1F27AB41",1,9},
};
const int GRID_SIZE = sizeof(GRID) / sizeof(GRID[0]);

bool lookupUid(const String &uid, uint8_t &row, uint8_t &col) {
  for (int i = 0; i < GRID_SIZE; i++) {
    if (uid.equalsIgnoreCase(GRID[i].uid)) {
      row = GRID[i].row; col = GRID[i].col; return true;
    }
  }
  return false;
}

bool inDeadZone(uint8_t row) { return row >= 1 && row <= 5; }

// ===========================================================================
//  COMPASS / POSE
// ===========================================================================

enum Facing { F_NORTH = 0, F_EAST = 1, F_SOUTH = 2, F_WEST = 3 };
const char *FACING_NAME[] = { "N", "E", "S", "W" };

uint8_t currentRow    = 0;
uint8_t currentCol    = 0;
Facing  currentFacing = F_NORTH;
bool    poseKnown     = false;

// Increment (dr, dc) when stepping one cell forward in the given facing
void facingDelta(Facing f, int &dr, int &dc) {
  dr = 0; dc = 0;
  switch (f) {
    case F_NORTH: dr =  1; break;
    case F_SOUTH: dr = -1; break;
    case F_EAST:  dc =  1; break;
    case F_WEST:  dc = -1; break;
  }
}

// Parse "R3C5" / "r3c5" / "3,5"
bool parseCell(String s, uint8_t &r, uint8_t &c) {
  s.trim(); s.toUpperCase();
  int rIdx = s.indexOf('R');
  int cIdx = s.indexOf('C');
  if (rIdx >= 0 && cIdx > rIdx) {
    long rr = s.substring(rIdx+1, cIdx).toInt();
    long cc = s.substring(cIdx+1).toInt();
    if (rr >= 1 && rr <= 9 && cc >= 1 && cc <= 9) { r=(uint8_t)rr; c=(uint8_t)cc; return true; }
  }
  int comma = s.indexOf(',');
  if (comma > 0) {
    long rr = s.substring(0, comma).toInt();
    long cc = s.substring(comma+1).toInt();
    if (rr >= 1 && rr <= 9 && cc >= 1 && cc <= 9) { r=(uint8_t)rr; c=(uint8_t)cc; return true; }
  }
  return false;
}

bool parseFacingChar(char ch, Facing &f) {
  ch = toupper(ch);
  if (ch == 'N') { f = F_NORTH; return true; }
  if (ch == 'E') { f = F_EAST;  return true; }
  if (ch == 'S') { f = F_SOUTH; return true; }
  if (ch == 'W') { f = F_WEST;  return true; }
  return false;
}

// ===========================================================================
//  GLOBALS
// ===========================================================================

MotoronI2C motorLeft;
MotoronI2C motorRight;
MFRC522_I2C rfid(RFID_I2C_ADDRESS, -1);

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

String latestRfidUid = "";

bool lastStartBtn = HIGH;
bool lastStopBtn  = HIGH;

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
  motorLeft.setSpeed(1, l);  motorLeft.setSpeed(2, l);
  motorRight.setSpeed(1, r); motorRight.setSpeed(2, r);
}
void stopMotors() {
  motorLeft.setSpeed(1, 0);  motorLeft.setSpeed(2, 0);
  motorRight.setSpeed(1, 0); motorRight.setSpeed(2, 0);
}
void initMotor(MotoronI2C &mc, const char *label) {
  mc.reinitialize(); delay(20);
  mc.disableCrc(); mc.clearResetFlag();
  mc.clearMotorFaultUnconditional(); mc.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 3; ch++) {
    mc.setMaxAcceleration(ch, 220); mc.setMaxDeceleration(ch, 350);
  }
  Serial.print(label); Serial.println(" motor ready.");
}

String uidToString() {
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}
String readRfidUid() {
  if (!rfid.PICC_IsNewCardPresent()) return "";
  if (!rfid.PICC_ReadCardSerial())   return "";
  String uid = uidToString();
  rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
  return uid;
}
String captureTagBriefly(int attempts = 10, int delayMs = 60) {
  for (int i = 0; i < attempts; i++) {
    String u = readRfidUid();
    if (u.length() > 0) return u;
    delay(delayMs);
  }
  return "";
}

// ===========================================================================
//  Forward declarations
// ===========================================================================
void handleButtons();
void handleSerial();
void dumpStatus(const char *prefix);
void dumpImu();
void printPose(const char *prefix);

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
    if (abortRequested) { stopMotors(); return false; }
    if (millis() - startMs > TURN_TIMEOUT_MS) {
      stopMotors();
      Serial.println("  TURN TIMEOUT");
      return false;
    }

    float remaining = targetDeltaDeg - headingDeg;
    if (fabs(remaining) <= TURN_OVERSHOOT_BUFFER_DEG) { stopMotors(); break; }

    int spd = (fabs(remaining) > TURN_SLOW_THRESHOLD_DEG) ? TURN_SPEED_FAST : TURN_SPEED_SLOW;
    // remaining > 0: need to turn LEFT (CCW, heading increases) -> drive(-,+)
    // remaining < 0: need to turn RIGHT (CW, heading decreases) -> drive(+,-)
    if (remaining > 0) drive(-spd,  spd);
    else               drive( spd, -spd);

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
  if      (delta == 1) angle = -90.0f;   // one step CW = right
  else if (delta == 3) angle =  90.0f;   // one step CCW = left
  else                 angle = -180.0f;  // about-face (do as CW)
  bool ok = turnDegrees(angle);
  if (ok) currentFacing = target;
  return ok;
}

// ===========================================================================
//  PRIMITIVE 2: drive forward one cell (25 cm + RFID confirm)
//  Returns true on success. Updates latestRfidUid as it goes.
// ===========================================================================
bool driveOneHop(uint8_t expectedRow, uint8_t expectedCol) {
  Serial.print("  hop towards R"); Serial.print(expectedRow); Serial.print("C"); Serial.println(expectedCol);

  String hopStartUid = latestRfidUid;
  resetEncoders();
  resetHeading();
  unsigned long hopStartMs = millis();

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

    String uid = readRfidUid();
    if (uid.length() > 0) latestRfidUid = uid;

    long lc = avgLeftCounts();
    long rc = avgRightCounts();
    long avgCounts = (lc + rc) / 2;
    float distCm = avgCounts * MM_PER_COUNT / 10.0f;

    if (avgCounts > MAX_COUNTS) {
      stopMotors(); Serial.println("  DISTANCE CAP, no new RFID"); return false;
    }

    bool distOK = (avgCounts >= TARGET_COUNTS);
    bool tagOK  = (latestRfidUid.length() > 0) && (latestRfidUid != hopStartUid);
    if (distOK && tagOK) { stopMotors(); break; }

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
      Serial.println(latestRfidUid.length() ? latestRfidUid : String("(none)"));
    }
    delay(3);
  }

  // Update current pose based on RFID if known, else dead reckon
  uint8_t r, c;
  if (latestRfidUid.length() > 0 && lookupUid(latestRfidUid, r, c)) {
    currentRow = r; currentCol = c;
  } else {
    int dr, dc; facingDelta(currentFacing, dr, dc);
    currentRow = (uint8_t)constrain((int)currentRow + dr, 1, 9);
    currentCol = (uint8_t)constrain((int)currentCol + dc, 1, 9);
  }
  Serial.print("  arrived at R"); Serial.print(currentRow); Serial.print("C"); Serial.println(currentCol);
  return true;
}

// ===========================================================================
//  PATH EXECUTION
// ===========================================================================

bool planAndExecute(uint8_t targetRow, uint8_t targetCol) {
  if (!poseKnown) {
    Serial.println("ERROR: pose unknown. Send 'face X' and 'at R#C#' first (or use 'pose R#C# F').");
    return false;
  }
  if (targetRow < 1 || targetRow > 9 || targetCol < 1 || targetCol > 9) {
    Serial.println("ERROR: target out of grid"); return false;
  }
  if (targetRow == currentRow && targetCol == currentCol) {
    Serial.println("Already at target."); return true;
  }

  sysState = SYS_BUSY;
  abortRequested = false;
  digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, HIGH);

  unsigned long pathStartMs = millis();
  int dRow = (int)targetRow - (int)currentRow;
  int dCol = (int)targetCol - (int)currentCol;

  Serial.println();
  Serial.print("PATH PLAN: R"); Serial.print(currentRow); Serial.print("C"); Serial.print(currentCol);
  Serial.print(" ("); Serial.print(FACING_NAME[currentFacing]); Serial.print(") -> R");
  Serial.print(targetRow); Serial.print("C"); Serial.println(targetCol);
  Serial.print("  dRow="); Serial.print(dRow); Serial.print(" dCol="); Serial.println(dCol);

  // Pick axis order: if both nonzero, do the axis matching current facing first
  bool rowsFirst;
  if (dRow == 0)      rowsFirst = false;
  else if (dCol == 0) rowsFirst = true;
  else                rowsFirst = (currentFacing == F_NORTH || currentFacing == F_SOUTH);

  for (int phase = 0; phase < 2; phase++) {
    if (abortRequested) goto failure;
    if (millis() - pathStartMs > PATH_TIMEOUT_MS) {
      Serial.println("PATH TIMEOUT (45 s)"); goto failure;
    }

    int delta; Facing needed;
    bool doRowPhase = (phase == 0) ? rowsFirst : !rowsFirst;
    if (doRowPhase) {
      delta = dRow;
      needed = (dRow > 0) ? F_NORTH : F_SOUTH;
    } else {
      delta = dCol;
      needed = (dCol > 0) ? F_EAST : F_WEST;
    }
    if (delta == 0) continue;

    Serial.print("PHASE: face "); Serial.print(FACING_NAME[needed]);
    Serial.print(", drive "); Serial.print(abs(delta)); Serial.println(" cells");

    if (!turnToFacing(needed)) goto failure;

    int dr, dc; facingDelta(currentFacing, dr, dc);
    for (int i = 0; i < abs(delta); i++) {
      if (abortRequested) goto failure;
      if (millis() - pathStartMs > PATH_TIMEOUT_MS) {
        Serial.println("PATH TIMEOUT (45 s)"); goto failure;
      }
      uint8_t expR = (uint8_t)constrain((int)currentRow + dr, 1, 9);
      uint8_t expC = (uint8_t)constrain((int)currentCol + dc, 1, 9);
      if (!driveOneHop(expR, expC)) goto failure;
    }
  }

  // Success
  stopMotors();
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);
  sysState = SYS_IDLE;
  Serial.println();
  Serial.print(">>> PATH SUCCESS <<<  arrived at R");
  Serial.print(currentRow); Serial.print("C"); Serial.print(currentCol);
  Serial.print(" facing "); Serial.print(FACING_NAME[currentFacing]);
  Serial.print(" in "); Serial.print(millis() - pathStartMs); Serial.println(" ms");
  return true;

failure:
  stopMotors();
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);
  sysState = SYS_IDLE;
  Serial.print(">>> PATH FAILED <<< at R"); Serial.print(currentRow); Serial.print("C"); Serial.println(currentCol);
  return false;
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

  // Capture starting tag
  String startTag = captureTagBriefly();
  latestRfidUid = startTag;
  uint8_t sr = 0, sc = 0;
  bool startKnown = false;
  if (startTag.length() > 0) {
    startKnown = lookupUid(startTag, sr, sc);
    if (startKnown) {
      currentRow = sr; currentCol = sc; poseKnown = true;
      Serial.print("Start: R"); Serial.print(sr); Serial.print("C"); Serial.println(sc);
    } else {
      Serial.print("Start UID "); Serial.print(startTag); Serial.println(" (not in map)");
    }
  } else {
    Serial.println("No start tag detected.");
  }

  if (imuPresent) calibrateGyroBias();

  // Do one hop in whatever direction the robot is facing
  uint8_t er = currentRow, ec = currentCol;
  int dr, dc; facingDelta(currentFacing, dr, dc);
  er = (uint8_t)constrain((int)currentRow + dr, 1, 9);
  ec = (uint8_t)constrain((int)currentCol + dc, 1, 9);
  bool ok = driveOneHop(er, ec);

  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, LOW);
  sysState = SYS_IDLE;
  Serial.println(ok ? ">>> HOP SUCCESS <<<" : ">>> HOP FAILED <<<");
  printPose("Final pose: ");
}

// ===========================================================================
//  Telemetry
// ===========================================================================

void printPose(const char *prefix) {
  if (prefix) Serial.print(prefix);
  if (poseKnown) {
    Serial.print("R"); Serial.print(currentRow); Serial.print("C"); Serial.print(currentCol);
    Serial.print(" facing "); Serial.println(FACING_NAME[currentFacing]);
  } else {
    Serial.println("(unknown)");
  }
}

void dumpStatus(const char *prefix) {
  long lf, lb, rf, rb; readEncoders(lf, lb, rf, rb);
  float distCm = ((lf + lb + rf + rb) / 4.0f) * MM_PER_COUNT / 10.0f;
  if (prefix) Serial.print(prefix);
  Serial.print("pose=");
  if (poseKnown) {
    Serial.print("R"); Serial.print(currentRow); Serial.print("C"); Serial.print(currentCol);
    Serial.print(FACING_NAME[currentFacing]);
  } else Serial.print("?");
  Serial.print("  ENC LF="); Serial.print(lf);
  Serial.print(" LB="); Serial.print(lb);
  Serial.print(" RF="); Serial.print(rf);
  Serial.print(" RB="); Serial.print(rb);
  Serial.print("  d=");  Serial.print(distCm, 1); Serial.print("cm");
  Serial.print("  hdg="); Serial.print(headingDeg, 2);
  Serial.print("  US F="); Serial.print(readDistance(FRONT_TRIG, FRONT_ECHO), 1);
  Serial.print(" L="); Serial.print(readDistance(LEFT_TRIG, LEFT_ECHO), 1);
  Serial.print(" R="); Serial.print(readDistance(RIGHT_TRIG, RIGHT_ECHO), 1);
  Serial.print("  RFID="); Serial.println(latestRfidUid.length() ? latestRfidUid : "(none)");
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
    if      (c == 'g' || c == 'G') runSingleHop();
    else if (c == 's' || c == 'S') { abortRequested = true; Serial.println("ABORT requested"); }
    else if (c == 'd' || c == 'D') dumpStatus("[manual] ");
    else if (c == 'i' || c == 'I') dumpImu();
    else if (c == 'c' || c == 'C') {
      if (imuPresent) { calibrateGyroBias(); resetHeading(); }
      else Serial.println("IMU not present.");
    }
    else if (c == 'r' || c == 'R') {
      String u = readRfidUid();
      if (u.length() == 0) Serial.println("RFID: (none)");
      else {
        uint8_t r, col; bool known = lookupUid(u, r, col);
        Serial.print("RFID: "); Serial.print(u);
        if (known) { Serial.print(" -> R"); Serial.print(r); Serial.print("C"); Serial.println(col); }
        else Serial.println(" (not in grid)");
      }
    }
    else if (c == 'z' || c == 'Z') { resetEncoders(); Serial.println("Encoders zeroed."); }
    else Serial.print("?cmd: "); Serial.println(cmd);
    return;
  }

  String upper = cmd; upper.toUpperCase();

  // face N|E|S|W
  if (upper.startsWith("FACE ")) {
    Facing f;
    if (cmd.length() >= 6 && parseFacingChar(cmd.charAt(5), f)) {
      currentFacing = f; poseKnown = (currentRow != 0 && currentCol != 0);
      Serial.print("Facing set to "); Serial.println(FACING_NAME[currentFacing]);
      printPose("Pose: ");
    } else Serial.println("usage: face N|E|S|W");
    return;
  }

  // at R#C#
  if (upper.startsWith("AT ")) {
    uint8_t r, c;
    if (parseCell(cmd.substring(3), r, c)) {
      currentRow = r; currentCol = c; poseKnown = true;
      Serial.print("Cell set to R"); Serial.print(r); Serial.print("C"); Serial.println(c);
      printPose("Pose: ");
    } else Serial.println("usage: at R#C#");
    return;
  }

  // pose R#C# F
  if (upper.startsWith("POSE ")) {
    String rest = cmd.substring(5); rest.trim();
    int sp = rest.indexOf(' ');
    if (sp <= 0) { Serial.println("usage: pose R#C# N|E|S|W"); return; }
    uint8_t r, c; Facing f;
    if (parseCell(rest.substring(0, sp), r, c) && parseFacingChar(rest.charAt(sp+1), f)) {
      currentRow = r; currentCol = c; currentFacing = f; poseKnown = true;
      printPose("Pose: ");
    } else Serial.println("usage: pose R#C# N|E|S|W");
    return;
  }

  // go R#C#
  if (upper.startsWith("GO ")) {
    uint8_t r, c;
    if (parseCell(cmd.substring(3), r, c)) planAndExecute(r, c);
    else Serial.println("usage: go R#C#");
    return;
  }

  // path R#C# R#C# [F]
  if (upper.startsWith("PATH ")) {
    String rest = cmd.substring(5); rest.trim();
    int sp1 = rest.indexOf(' ');
    if (sp1 <= 0) { Serial.println("usage: path R#C# R#C# [N|E|S|W]"); return; }
    String fromS = rest.substring(0, sp1);
    String tail = rest.substring(sp1+1); tail.trim();
    int sp2 = tail.indexOf(' ');
    String toS;
    char fch = 'N';
    if (sp2 > 0) { toS = tail.substring(0, sp2); fch = tail.charAt(sp2+1); }
    else         { toS = tail; }
    uint8_t fr, fc, tr, tc; Facing f = currentFacing;
    if (!parseCell(fromS, fr, fc) || !parseCell(toS, tr, tc)) {
      Serial.println("usage: path R#C# R#C# [N|E|S|W]"); return;
    }
    if (sp2 > 0) parseFacingChar(fch, f);
    currentRow = fr; currentCol = fc; currentFacing = f; poseKnown = true;
    Serial.print("Pose set: R"); Serial.print(fr); Serial.print("C"); Serial.print(fc);
    Serial.print(" facing "); Serial.println(FACING_NAME[f]);
    planAndExecute(tr, tc);
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
  bool sNow = digitalRead(BTN_START);
  bool xNow = digitalRead(BTN_STOP);
  if (lastStartBtn == HIGH && sNow == LOW) {
    delay(25);
    if (digitalRead(BTN_START) == LOW && sysState == SYS_IDLE) runSingleHop();
  }
  if (lastStopBtn == HIGH && xNow == LOW) {
    delay(25);
    if (digitalRead(BTN_STOP) == LOW) { abortRequested = true; Serial.println("ABORT (button)"); }
  }
  lastStartBtn = sNow;
  lastStopBtn  = xNow;
}

// ===========================================================================
//  setup()
// ===========================================================================

void setup() {
  Serial.begin(115200);
  unsigned long sw = millis();
  while (!Serial && millis() - sw < 2000) {}

  Serial.println();
  Serial.println("=== Task 4: dead reckoning + pathfinding ===");
  Serial.print("Grid: "); Serial.print(GRID_SIZE); Serial.println(" cells");
  Serial.print("MM/count: "); Serial.println(MM_PER_COUNT, 4);
  Serial.print("25 cm = "); Serial.print(TARGET_COUNTS); Serial.print(" counts (max ");
  Serial.print(MAX_COUNTS); Serial.println(")");

  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_STOP, INPUT_PULLUP);
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
  Wire.setClock(100000);

  imuPresent = mpuProbe();
  if (imuPresent) {
    mpuInit();
    Serial.print("MPU6050 ready (WHO_AM_I=0x"); Serial.print(mpuRead(MPU_WHO_AM_I), HEX); Serial.println(")");
    calibrateGyroBias();
    resetHeading();
  } else {
    Serial.println("WARNING: MPU6050 NOT found.");
  }

  rfid.PCD_Init();
  Serial.println("RFID ready (0x28).");

  Wire1.begin();
  delay(100);
  motorLeft.setBus(&Wire1);  motorLeft.setAddress(ADDR_LEFT);
  initMotor(motorLeft, "Left");
  motorRight.setBus(&Wire1); motorRight.setAddress(ADDR_RIGHT);
  initMotor(motorRight, "Right");
  stopMotors();

  Serial.println();
  Serial.println("Commands (newline-terminated):");
  Serial.println("  g                       single 25 cm hop forward");
  Serial.println("  face N|E|S|W            set current facing");
  Serial.println("  at R#C#                 set current cell");
  Serial.println("  pose R#C# N|E|S|W       set full pose");
  Serial.println("  go R#C#                 plan and drive to R#C#");
  Serial.println("  path R#C# R#C# [F]      set pose, then go to second cell");
  Serial.println("  s                       abort current motion");
  Serial.println("  d / r / i / c / z       telemetry, RFID, IMU, recal, zero enc");
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

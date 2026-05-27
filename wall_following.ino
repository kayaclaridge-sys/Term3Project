/*
 * wall_following.ino
 * Arduino Giga R1 WiFi - Wall Following (Task 6)
 *
 * Motor wiring matches trial_run2_standard_line_tracking.ino:
 *   - LEFT_MOTOR_SIGN  = +1
 *   - RIGHT_MOTOR_SIGN = -1
 *
 * Strategy:
 *  - Picks the closer wall at startup and tracks it
 *  - PD controller keeps the robot at TARGET_DIST_CM from the chosen wall
 *  - Front sensor triggers a turn-away maneuver before a collision
 *  - Ramp boost is wired in but currently disabled for flat-ground testing
 *    (set RAMP_BOOST_SPEED > BASE_SPEED to re-enable it for the ramp run)
 *  - Button 1 (D32) = START, Button 2 (D33) = STOP
 *  - Green LED = running, Red LED = stopped
 *  - Serial: 'g' = start, 's' = stop, 'w' = swap wall side
 *
 * Pin assignments:
 *   Front ultrasonic  : Trig D2,  Echo D3
 *   Left  ultrasonic  : Trig D41, Echo D40
 *   Right ultrasonic  : Trig D39, Echo D38
 *   Left  Motoron I2C : 0x10 on Wire1 (ch1 = front-left, ch2 = rear-left)
 *   Right Motoron I2C : 0x11 on Wire1 (ch1 = front-right, ch2 = rear-right)
 *   Start button      : D32 (INPUT_PULLUP)
 *   Stop  button      : D33 (INPUT_PULLUP)
 *   Red LED           : D34
 *   Green LED         : D35
 *
 * Encoder pins (reserved, not used here):
 *   RB H1=D42 H2=D43
 *   LB H1=D44 H2=D45
 *   RF H1=D48 H2=D49
 *   LF H1=D50 H2=D51
 */

#include <Wire.h>
#include <Motoron.h>

// ---- Ultrasonic pins ------------------------------------------------------
#define FRONT_TRIG  36
#define FRONT_ECHO  37
#define LEFT_TRIG   39
#define LEFT_ECHO   38
#define RIGHT_TRIG  41
#define RIGHT_ECHO  40

// ---- UI pins --------------------------------------------------------------
#define BTN_START   32
#define BTN_STOP    33
#define LED_RED     34
#define LED_GREEN   35

// ---- Motoron I2C addresses ------------------------------------------------
#define ADDR_LEFT   0x10
#define ADDR_RIGHT  0x11

// ---- Motor channel / direction (matches trial_run2) -----------------------
const int LEFT_MOTOR_SIGN  =  1;
const int RIGHT_MOTOR_SIGN = -1;

const uint8_t FRONT_LEFT_MOTOR  = 1;
const uint8_t REAR_LEFT_MOTOR   = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR  = 2;

// ===========================================================================
// ====================  TUNING SECTION  =====================================
// ===========================================================================
// Adjust these and re-upload. See the tuning guide at the bottom of this
// block for what each value does and which direction to move it.
// ---------------------------------------------------------------------------

// ---- Sensor geometry -------------------------------------------------------
// The ultrasonic modules sit ~4 cm inside the robot body, so a raw reading of
// "15 cm" actually means the robot's outer surface is only 11 cm from the wall.
// readDistance() subtracts this offset, so EVERY other distance constant in
// this file refers to the ACTUAL distance from the robot's outer surface to
// the wall / obstacle.
const float SENSOR_OFFSET_CM = 4.0f;

// ---- Target wall distance (actual robot-edge -> wall, not raw sensor) ------
const float TARGET_DIST_CM   = 11.0f;  // ~11 cm clearance from robot body to wall
const float WALL_DEADBAND_CM = 1.5f;   // ignore tiny errors (anti-jitter)

// ---- PD gains --------------------------------------------------------------
// KP = how hard to react to current error (in cm)
// KD = how hard to react to how fast the error is changing (cm/sec)
const float KP               = 26.0f;  // was 14 - bumped for faster reaction
const float KD               = 12.0f;  // was 9  - extra damping for the bigger KP

// ---- Steering aggression ---------------------------------------------------
const int   MAX_CORRECTION   = 380;    // was 240 - cap on PD output, higher = sharper turns allowed
const int   MIN_DRIVE_SPEED  = 80;     // was 320 - lower lets a wheel actually slow down so the robot can turn

// ---- Drive speeds ----------------------------------------------------------
const int   BASE_SPEED       = 600;    // cruising speed (Motoron max 800)
const int   RAMP_BOOST_SPEED = 600;    // set > BASE_SPEED (e.g. 780) to enable ramp boost
const int   MAX_SPEED        = 800;

// ---- Front-obstacle avoidance (actual front-of-robot -> wall) --------------
const float FRONT_STOP_CM    = 24.0f;  // was 28 raw - 4cm offset
const float FRONT_CLEAR_CM   = 34.0f;  // was 38 raw - 4cm offset
const int   TURN_SPEED       = 600;    // in-place spin during avoidance
const unsigned long TURN_MS  = 450;    // duration of the avoidance turn

// ---- Sensor filtering ------------------------------------------------------
const int   SENSOR_SAMPLES   = 3;      // median-filtered readings per call
const float MAX_VALID_CM     = 180.0f; // readings above this = "no wall"
const float MIN_VALID_CM     = 1.5f;   // readings below this = noise

// ---- Ramp-boost arming window (only active if RAMP_BOOST_SPEED > BASE_SPEED)
const unsigned long RAMP_BOOST_MS_ON  = 1200;
const float RAMP_BOOST_FRONT_MIN_CM   = 35.0f;

// ===========================================================================
// TUNING GUIDE
// ---------------------------------------------------------------------------
// Symptom                                  -> what to change
// ---------------------------------------------------------------------------
// Slow to react / drifts off the wall      -> raise KP by ~20%
// Oscillates / wiggles around the target   -> lower KP by ~20%, or raise KD
// Overshoots, then corrects back, repeat   -> raise KD by ~30%
// Jitters/twitches on noisy readings       -> lower KD, or raise WALL_DEADBAND_CM
// Hits side wall before turning            -> raise KP, raise MAX_CORRECTION,
//                                             lower MIN_DRIVE_SPEED
// Hits front wall in corners               -> raise FRONT_STOP_CM, raise TURN_SPEED
// Avoidance turn too small (still facing
//   wall after the turn)                   -> raise TURN_MS
// Avoidance turn too big (over-rotates)    -> lower TURN_MS
// Robot stalls on flat ground              -> raise MIN_DRIVE_SPEED slightly
// ===========================================================================

// ---- Motor controllers ----------------------------------------------------
MotoronI2C motorLeft;
MotoronI2C motorRight;

// ---- State ----------------------------------------------------------------
enum RobotState { STOPPED, FOLLOWING, AVOIDING };
RobotState state = STOPPED;

bool  followLeft       = true;
float prevError        = 0.0f;
unsigned long prevTime = 0;
bool  lastStartState   = HIGH;
bool  lastStopState    = HIGH;

unsigned long rampBoostUntilMs = 0;
unsigned long lastRampCheckMs  = 0;
float lastWallDist             = -1.0f;

// ==========================================================================
//  Ultrasonic helpers
// ==========================================================================

float singlePing(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout ~ 515 cm
  if (duration == 0) return MAX_VALID_CM;
  float dist = duration * 0.0343f / 2.0f;
  return (dist < MIN_VALID_CM || dist > MAX_VALID_CM) ? MAX_VALID_CM : dist;
}

float readDistance(int trigPin, int echoPin) {
  float readings[SENSOR_SAMPLES];
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    readings[i] = singlePing(trigPin, echoPin);
    delay(4);
  }
  for (int i = 1; i < SENSOR_SAMPLES; i++) {
    float key = readings[i];
    int j = i - 1;
    while (j >= 0 && readings[j] > key) { readings[j + 1] = readings[j]; j--; }
    readings[j + 1] = key;
  }
  float median = readings[SENSOR_SAMPLES / 2];

  // Account for the sensor being SENSOR_OFFSET_CM inside the robot body.
  // If the reading is "no wall" (>= MAX_VALID_CM), pass through unchanged.
  if (median >= MAX_VALID_CM) return median;
  float adjusted = median - SENSOR_OFFSET_CM;
  if (adjusted < 0.0f) adjusted = 0.0f;
  return adjusted;
}

// ==========================================================================
//  Motor helpers
// ==========================================================================

int clampSpeed(int spd) {
  return constrain(spd, -MAX_SPEED, MAX_SPEED);
}

// leftSpd / rightSpd in "robot forward" convention (+ve = forward)
void drive(int leftSpd, int rightSpd) {
  int lCmd = clampSpeed(leftSpd)  * LEFT_MOTOR_SIGN;
  int rCmd = clampSpeed(rightSpd) * RIGHT_MOTOR_SIGN;
  motorLeft.setSpeed(FRONT_LEFT_MOTOR,  lCmd);
  motorLeft.setSpeed(REAR_LEFT_MOTOR,   lCmd);
  motorRight.setSpeed(FRONT_RIGHT_MOTOR, rCmd);
  motorRight.setSpeed(REAR_RIGHT_MOTOR,  rCmd);
}

void stopMotors() {
  motorLeft.setSpeed(FRONT_LEFT_MOTOR, 0);
  motorLeft.setSpeed(REAR_LEFT_MOTOR,  0);
  motorRight.setSpeed(FRONT_RIGHT_MOTOR, 0);
  motorRight.setSpeed(REAR_RIGHT_MOTOR,  0);
}

// ==========================================================================
//  Motoron initialisation (mirrors trial_run2)
// ==========================================================================

void initMotorController(MotoronI2C &mc, const char *label) {
  mc.reinitialize();
  delay(20);
  mc.disableCrc();
  mc.clearResetFlag();
  mc.clearMotorFaultUnconditional();
  mc.disableCommandTimeout();
  for (uint8_t channel = 1; channel <= 3; channel++) {
    mc.setMaxAcceleration(channel, 260);
    mc.setMaxDeceleration(channel, 360);
  }
  Serial.print(label);
  Serial.println(" Motoron ready.");
}

// ==========================================================================
//  Wall selection at startup
// ==========================================================================

void selectWall() {
  float leftDist  = readDistance(LEFT_TRIG,  LEFT_ECHO);
  float rightDist = readDistance(RIGHT_TRIG, RIGHT_ECHO);
  Serial.print("Startup - Left wall: ");
  Serial.print(leftDist);
  Serial.print(" cm  Right wall: ");
  Serial.print(rightDist);
  Serial.println(" cm");
  if (leftDist < rightDist && leftDist < MAX_VALID_CM) {
    followLeft = true;
    Serial.println("-> Following LEFT wall");
  } else if (rightDist < MAX_VALID_CM) {
    followLeft = false;
    Serial.println("-> Following RIGHT wall");
  } else {
    followLeft = true;
    Serial.println("-> No wall detected clearly, defaulting to LEFT");
  }
}

// ==========================================================================
//  Robot start/stop
// ==========================================================================

void startRobot() {
  if (state != STOPPED) return;
  selectWall();
  prevError        = 0.0f;
  prevTime         = millis();
  rampBoostUntilMs = 0;
  lastWallDist     = -1.0f;
  state            = FOLLOWING;
  digitalWrite(LED_RED,   LOW);
  digitalWrite(LED_GREEN, HIGH);
  Serial.println("Wall following STARTED");
}

void stopRobot(const char *reason) {
  state = STOPPED;
  stopMotors();
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, LOW);
  Serial.print("Wall following STOPPED");
  if (reason && *reason) { Serial.print(" - "); Serial.print(reason); }
  Serial.println();
}

void handleButtons() {
  bool startNow = digitalRead(BTN_START);
  bool stopNow  = digitalRead(BTN_STOP);

  if (lastStartState == HIGH && startNow == LOW) {
    delay(25);
    if (digitalRead(BTN_START) == LOW) startRobot();
  }
  if (lastStopState == HIGH && stopNow == LOW) {
    delay(25);
    if (digitalRead(BTN_STOP) == LOW) stopRobot("button");
  }
  lastStartState = startNow;
  lastStopState  = stopNow;
}

void printSensorReadings(const char *prefix) {
  float frontDist = readDistance(FRONT_TRIG, FRONT_ECHO);
  float leftDist  = readDistance(LEFT_TRIG,  LEFT_ECHO);
  float rightDist = readDistance(RIGHT_TRIG, RIGHT_ECHO);

  if (prefix) Serial.print(prefix);
  Serial.print("Distance | F: ");
  if (frontDist >= MAX_VALID_CM) Serial.print("---");
  else { Serial.print(frontDist, 1); Serial.print(" cm"); }
  Serial.print("    L: ");
  if (leftDist >= MAX_VALID_CM)  Serial.print("---");
  else { Serial.print(leftDist, 1);  Serial.print(" cm"); }
  Serial.print("    R: ");
  if (rightDist >= MAX_VALID_CM) Serial.print("---");
  else { Serial.print(rightDist, 1); Serial.print(" cm"); }
  Serial.println();
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'g' || c == 'G') startRobot();
  else if (c == 's' || c == 'S') stopRobot("serial");
  else if (c == 'w' || c == 'W') {
    followLeft = !followLeft;
    Serial.print("Wall toggled -> "); Serial.println(followLeft ? "LEFT" : "RIGHT");
  }
  else if (c == 'd' || c == 'D') {
    printSensorReadings("[manual] ");
  }
}

// ==========================================================================
//  setup()
// ==========================================================================

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 2000) { }

  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG,  OUTPUT); pinMode(LEFT_ECHO,  INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);
  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(LEFT_TRIG,  LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);
  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, LOW);

  Serial.println("Initializing Motorons on Wire1...");
  Wire1.begin();
  delay(100);

  motorLeft.setBus(&Wire1);
  motorLeft.setAddress(ADDR_LEFT);
  initMotorController(motorLeft, "Left 0x10");

  motorRight.setBus(&Wire1);
  motorRight.setAddress(ADDR_RIGHT);
  initMotorController(motorRight, "Right 0x11");

  stopMotors();

  selectWall();
  Serial.println("Ready.");
  Serial.println("  D32 / 'g' = start    D33 / 's' = stop");
  Serial.println("  'w' = swap wall side    'd' = one-shot distance dump");
  Serial.println("While stopped, distances print automatically every 250 ms.");
}

// ==========================================================================
//  loop()
// ==========================================================================

void loop() {
  handleButtons();
  handleSerial();

  if (state == STOPPED) {
    // Print distance telemetry every ~250 ms while idle so you can position
    // the robot next to a wall and watch what each sensor actually sees.
    static unsigned long lastIdlePrintMs = 0;
    unsigned long now = millis();
    if (now - lastIdlePrintMs >= 250) {
      lastIdlePrintMs = now;
      printSensorReadings("[idle]   ");
    }
    delay(10);
    return;
  }

  float frontDist = readDistance(FRONT_TRIG, FRONT_ECHO);
  float leftDist  = readDistance(LEFT_TRIG,  LEFT_ECHO);
  float rightDist = readDistance(RIGHT_TRIG, RIGHT_ECHO);

  Serial.print("F:");  Serial.print(frontDist, 1);
  Serial.print(" L:"); Serial.print(leftDist,  1);
  Serial.print(" R:"); Serial.print(rightDist, 1);
  Serial.print(" cm | ");

  // Front obstacle avoidance
  if (state == FOLLOWING && frontDist < FRONT_STOP_CM) {
    state = AVOIDING;
    Serial.println("AVOIDING");
    if (followLeft) {
      // following left wall + obstacle ahead -> turn right
      drive(TURN_SPEED, -TURN_SPEED);
    } else {
      drive(-TURN_SPEED, TURN_SPEED);
    }
    delay(TURN_MS);
    stopMotors();
    delay(80);
  }

  if (state == AVOIDING) {
    frontDist = readDistance(FRONT_TRIG, FRONT_ECHO);
    if (frontDist >= FRONT_CLEAR_CM) {
      state     = FOLLOWING;
      prevError = 0.0f;
      prevTime  = millis();
      Serial.println("FOLLOWING (resumed)");
    } else {
      Serial.println("AVOIDING (waiting)");
      return;
    }
  }

  // PD wall-following controller
  Serial.print("FOLLOWING ");

  float wallDist = followLeft ? leftDist : rightDist;

  // If wall is out of range, creep toward it gently.
  // To veer LEFT  (toward left wall):  right wheel faster than left
  // To veer RIGHT (toward right wall): left  wheel faster than right
  if (wallDist >= MAX_VALID_CM) {
    if (followLeft) drive(BASE_SPEED - 180, BASE_SPEED);
    else            drive(BASE_SPEED, BASE_SPEED - 180);
    Serial.println(" wall=lost, creeping in");
    return;
  }

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  prevTime = now;

  float error      = wallDist - TARGET_DIST_CM;
  if (fabsf(error) < WALL_DEADBAND_CM) error = 0.0f;
  float derivative = (error - prevError) / dt;
  prevError        = error;

  int correction = (int)(KP * error + KD * derivative);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // Ramp boost (disabled while RAMP_BOOST_SPEED == BASE_SPEED)
  int effectiveBase = BASE_SPEED;
  if (RAMP_BOOST_SPEED > BASE_SPEED) {
    bool pathOpen = frontDist > RAMP_BOOST_FRONT_MIN_CM;
    bool wallStable = (lastWallDist >= 0.0f) && (fabsf(wallDist - lastWallDist) < 4.0f);
    if (pathOpen && wallStable && now - lastRampCheckMs > 250) {
      rampBoostUntilMs = now + RAMP_BOOST_MS_ON;
      lastRampCheckMs  = now;
    }
    if (now < rampBoostUntilMs && pathOpen) {
      effectiveBase = RAMP_BOOST_SPEED;
    }
  }
  lastWallDist = wallDist;

  // Differential steering:
  //   +ve correction means "veer toward the tracked wall"
  //   To veer LEFT  (toward left wall):  right wheel faster, left  wheel slower
  //   To veer RIGHT (toward right wall): left  wheel faster, right wheel slower
  int leftSpd, rightSpd;
  if (followLeft) {
    leftSpd  = effectiveBase - correction;
    rightSpd = effectiveBase + correction;
  } else {
    leftSpd  = effectiveBase + correction;
    rightSpd = effectiveBase - correction;
  }

  // Floor so a wheel never stalls mid-correction
  if (leftSpd  < MIN_DRIVE_SPEED) leftSpd  = MIN_DRIVE_SPEED;
  if (rightSpd < MIN_DRIVE_SPEED) rightSpd = MIN_DRIVE_SPEED;
  leftSpd  = constrain(leftSpd,  -MAX_SPEED, MAX_SPEED);
  rightSpd = constrain(rightSpd, -MAX_SPEED, MAX_SPEED);

  drive(leftSpd, rightSpd);

  Serial.print(" wall="); Serial.print(wallDist, 1);
  Serial.print(" err=");  Serial.print(error, 2);
  Serial.print(" corr="); Serial.print(correction);
  Serial.print(" base="); Serial.print(effectiveBase);
  Serial.print(" L=");    Serial.print(leftSpd);
  Serial.print(" R=");    Serial.println(rightSpd);
}
                                                                                                                              
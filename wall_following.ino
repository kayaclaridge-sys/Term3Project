/*
 * wall_following.ino
 * Arduino Giga R1 WiFi — Wall Following (Task 6)
 *
 * Updated for ramp running and tighter (10 cm) wall tracking.
 * Motor wiring conventions are kept in sync with
 * trial_run2_standard_line_tracking.ino:
 *   - LEFT_MOTOR_SIGN  = +1
 *   - RIGHT_MOTOR_SIGN = -1
 *
 * Strategy:
 *  - Detects which wall is closer (left or right) at startup and follows it
 *  - PD controller keeps the robot at TARGET_DIST_CM from the chosen wall
 *  - Front sensor triggers a turn-away manoeuvre before a collision happens
 *  - Ramp boost: when the front is open but the robot is slowing (low speed
 *    output from the controller, OR loss of forward progress), bump the base
 *    speed so it climbs sub-20° ramps without stalling
 *  - Button 1 (D32) = START, Button 2 (D33) = STOP
 *  - Green LED = running, Red LED = stopped
 *  - Serial: 'g' = start, 's' = stop, 'w' = swap wall side
 *
 * Pin assignments:
 *   Front ultrasonic  : Trig D2,  Echo D3
 *   Left  ultrasonic  : Trig D41, Echo D40
 *   Right ultrasonic  : Trig D39, Echo D38
 *   Left  Motoron I2C : address 0x10 on Wire1  (ch1 = front-left, ch2 = rear-left)
 *   Right Motoron I2C : address 0x11 on Wire1  (ch1 = front-right, ch2 = rear-right)
 *   Start button      : D32 (INPUT_PULLUP)
 *   Stop  button      : D33 (INPUT_PULLUP)
 *   Red LED           : D34
 *   Green LED         : D35
 *
 * Encoder pins (reserved – not used here; consistent with the wiring photo):
 *   RB H1=D42  H2=D43
 *   LB H1=D44  H2=D45
 *   RF H1=D48  H2=D49
 *   LF H1=D50  H2=D51
 */

#include <Wire.h>
#include <Motoron.h>

// ─── Ultrasonic pins ────────────────────────────────────────────────────────
#define FRONT_TRIG  2
#define FRONT_ECHO  3
#define LEFT_TRIG   41
#define LEFT_ECHO   40
#define RIGHT_TRIG  39
#define RIGHT_ECHO  38

// ─── UI pins ────────────────────────────────────────────────────────────────
#define BTN_START   32
#define BTN_STOP    33
#define LED_RED     34
#define LED_GREEN   35

// ─── Motoron I2C addresses ──────────────────────────────────────────────────
#define ADDR_LEFT   0x10
#define ADDR_RIGHT  0x11

// ─── Motor channel / direction (matches trial_run2) ─────────────────────────
const int LEFT_MOTOR_SIGN  =  1;
const int RIGHT_MOTOR_SIGN = -1;

const uint8_t FRONT_LEFT_MOTOR  = 1;
const uint8_t REAR_LEFT_MOTOR   = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR  = 2;

// ─── Tunable parameters ─────────────────────────────────────────────────────
// Closer wall tracking — target 4 cm (very tight, mind sensor floor)
const float TARGET_DIST_CM   = 4.0f;
const float WALL_DEADBAND_CM = 0.4f;    // ignore errors smaller than this (anti-jitter)

// Front avoidance thresholds — a little tighter to suit a 10 cm gap from wall
const float FRONT_STOP_CM    = 22.0f;
const float FRONT_CLEAR_CM   = 32.0f;

// Speeds — raised so we can climb the (<20°) ramp without stalling
const int   BASE_SPEED       = 620;     // straight-line drive speed (Motoron max 800)
const int   RAMP_BOOST_SPEED = 780;     // forced speed while ramp boost is active
const int   MIN_DRIVE_SPEED  = 380;     // PD output must never drop a wheel below this on a ramp
const int   MAX_SPEED        = 800;
const int   MAX_CORRECTION   = 260;     // cap on PD correction so neither wheel stalls
const int   TURN_SPEED       = 420;     // speed used during avoidance turns
const unsigned long TURN_MS  = 380;     // duration of an avoidance turn (ms)

// Tighter target ⇒ smaller errors ⇒ a touch less gain so we don't oscillate
const float KP               = 14.0f;
const float KD               = 9.0f;

const int   SENSOR_SAMPLES   = 3;       // readings per sensor call (median filtered)
const float MAX_VALID_CM     = 180.0f;  // readings above this are treated as "no wall"
const float MIN_VALID_CM     = 1.5f;    // tracking at 4cm — sensor floor must sit below it

// Ramp detection: if the front is clear but the chosen wall distance is stable
// and the loop time is long enough, assume we're climbing and apply RAMP_BOOST.
const unsigned long RAMP_BOOST_MS_ON  = 1200;   // how long to hold a boost
const float RAMP_BOOST_FRONT_MIN_CM   = 35.0f;  // only boost when path is open

// ─── Motor controllers ──────────────────────────────────────────────────────
MotoronI2C motorLeft;
MotoronI2C motorRight;

// ─── State ──────────────────────────────────────────────────────────────────
enum RobotState { STOPPED, FOLLOWING, AVOIDING };
RobotState state = STOPPED;

bool  followLeft       = true;   // true = track left wall, false = track right
float prevError        = 0.0f;
unsigned long prevTime = 0;
bool  lastStartState   = HIGH;
bool  lastStopState    = HIGH;

// Ramp boost timing
unsigned long rampBoostUntilMs = 0;
unsigned long lastRampCheckMs  = 0;
float lastWallDist             = -1.0f;

// ════════════════════════════════════════════════════════════════════════════
//  Ultrasonic helpers
// ════════════════════════════════════════════════════════════════════════════

float singlePing(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout ≈ 515 cm
  if (duration == 0) return MAX_VALID_CM;
  float dist = duration * 0.0343f / 2.0f;
  return (dist < MIN_VALID_CM || dist > MAX_VALID_CM) ? MAX_VALID_CM : dist;
}

// Returns the median of SENSOR_SAMPLES readings for reliable distance
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
  return readings[SENSOR_SAMPLES / 2];
}

// ════════════════════════════════════════════════════════════════════════════
//  Motor helpers
// ════════════════════════════════════════════════════════════════════════════

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

// ════════════════════════════════════════════════════════════════════════════
//  Motoron initialisation (mirrors trial_run2)
// ════════════════════════════════════════════════════════════════════════════

void initMotorController(MotoronI2C &mc, const char *label) {
  mc.reinitialize();
  delay(20);
  mc.disableCrc();
  mc.clearResetFlag();
  mc.clearMotorFaultUnconditional();
  mc.disableCommandTimeout();
  // Snappier accel for ramp engagement, but still gentle enough to avoid wheel slip
  for (uint8_t channel = 1; channel <= 3; channel++) {
    mc.setMaxAcceleration(channel, 260);
    mc.setMaxDeceleration(channel, 360);
  }
  Serial.print(label);
  Serial.println(" Motoron ready.");
}

// ════════════════════════════════════════════════════════════════════════════
//  Wall selection at startup
// ════════════════════════════════════════════════════════════════════════════

void selectWall() {
  float leftDist  = readDistance(LEFT_TRIG,  LEFT_ECHO);
  float rightDist = readDistance(RIGHT_TRIG, RIGHT_ECHO);
  Serial.print("Startup — Left wall: ");
  Serial.print(leftDist);
  Serial.print(" cm  Right wall: ");
  Serial.print(rightDist);
  Serial.println(" cm");
  if (leftDist < rightDist && leftDist < MAX_VALID_CM) {
    followLeft = true;
    Serial.println("→ Following LEFT wall");
  } else if (rightDist < MAX_VALID_CM) {
    followLeft = false;
    Serial.println("→ Following RIGHT wall");
  } else {
    followLeft = true;
    Serial.println("→ No wall detected clearly, defaulting to LEFT");
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Robot start/stop
// ════════════════════════════════════════════════════════════════════════════

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
  Serial.println("▶ Wall following STARTED");
}

void stopRobot(const char *reason) {
  state = STOPPED;
  stopMotors();
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, LOW);
  Serial.print("■ Wall following STOPPED");
  if (reason && *reason) { Serial.print(" — "); Serial.print(reason); }
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

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'g' || c == 'G') startRobot();
  else if (c == 's' || c == 'S') stopRobot("serial");
  else if (c == 'w' || c == 'W') {
    followLeft = !followLeft;
    Serial.print("Wall toggled → "); Serial.println(followLeft ? "LEFT" : "RIGHT");
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  setup()
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 2000) { }

  // Ultrasonic pins
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG,  OUTPUT); pinMode(LEFT_ECHO,  INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);
  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(LEFT_TRIG,  LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  // UI
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);
  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, LOW);

  // Motor controllers on Wire1
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
  Serial.println("Ready. Press D32 (or send 'g') to start, D33 (or 's') to stop.");
  Serial.println("Send 'w' to swap which wall to follow.");
}

// ════════════════════════════════════════════════════════════════════════════
//  loop()
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  handleButtons();
  handleSerial();

  if (state == STOPPED) {
    delay(10);
    return;
  }

  // ── Read sensors ──────────────────────────────────────────────────────────
  float frontDist = readDistance(FRONT_TRIG, FRONT_ECHO);
  float leftDist  = readDistance(LEFT_TRIG,  LEFT_ECHO);
  float rightDist = readDistance(RIGHT_TRIG, RIGHT_ECHO);

  Serial.print("F:");  Serial.print(frontDist, 1);
  Serial.print(" L:"); Serial.print(leftDist,  1);
  Serial.print(" R:"); Serial.print(rightDist, 1);
  Serial.print(" cm | ");

  // ── Front obstacle avoidance ─────────────────────────────────────────────
  if (state == FOLLOWING && frontDist < FRONT_STOP_CM) {
    state = AVOIDING;
    Serial.println("AVOIDING");
    if (followLeft) {
      // Following left wall → obstacle ahead → turn right
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

  // ── PD wall-following controller ─────────────────────────────────────────
  Serial.print("FOLLOWING ");

  float wallDist = followLeft ? leftDist : rightDist;

  // If wall is out of range, creep toward it gently (still going forward).
  // To veer LEFT  (toward left wall):  right wheel faster than left
  // To veer RIGHT (toward right wall): left  wheel faster than right
  if (wallDist >= MAX_VALID_CM) {
    if (followLeft) drive(BASE_SPEED - 180, BASE_SPEED);   // veer left
    else            drive(BASE_SPEED, BASE_SPEED - 180);   // veer right
    Serial.println(" wall=lost, creeping in");
    return;
  }

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  prevTime = now;

  // Error: +ve = too far, -ve = too close. Apply a small deadband.
  float error      = wallDist - TARGET_DIST_CM;
  if (fabsf(error) < WALL_DEADBAND_CM) error = 0.0f;
  float derivative = (error - prevError) / dt;
  prevError        = error;

  int correction = (int)(KP * error + KD * derivative);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // ── Ramp-boost logic ──────────────────────────────────────────────────────
  // Idea: if the path ahead is open AND the chosen-wall distance is stable
  // (we're not crashing into anything), pulse the base speed up for a while.
  // This is purely time-based since the encoders weren't returning counts.
  int effectiveBase = BASE_SPEED;
  bool pathOpen = frontDist > RAMP_BOOST_FRONT_MIN_CM;
  bool wallStable = (lastWallDist >= 0.0f) && (fabsf(wallDist - lastWallDist) < 4.0f);
  if (pathOpen && wallStable && now - lastRampCheckMs > 250) {
    // Re-arm a 1.2s boost if we've been cruising cleanly
    rampBoostUntilMs = now + RAMP_BOOST_MS_ON;
    lastRampCheckMs  = now;
  }
  if (now < rampBoostUntilMs && pathOpen) {
    effectiveBase = RAMP_BOOST_SPEED;
  }
  lastWallDist = wallDist;

  // Differential steering:
  //   +ve correction means "veer toward the tracked wall" (we're too far away)
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

  // Never let a wheel drop below MIN_DRIVE_SPEED on a forward path — on a
  // ramp, a wheel that's almost stopped will simply lose traction.
  if (leftSpd  < MIN_DRIVE_SPEED) leftSpd  = MIN_DRIVE_SPEED;
  if (rightSpd < MIN_DRIVE_SPEED) rightSpd = MIN_DRIVE_SPEED;
  leftSpd  = constrain(leftSpd,  -MAX_SPEED, MAX_SPEED);
  rightSpd = constrain(rightSpd, -MAX_SPEED, MAX_SPEED);

  drive(leftSpd, rightSpd);

  // Debug
  Serial.print(" wall="); Serial.print(wallDist, 1);
  Serial.print(" err=");  Serial.print(error, 2);
  Serial.print(" corr="); Serial.print(correction);
  Serial.print(" base="); Serial.print(effectiveBase);
  Serial.print(" L=");    Serial.print(leftSpd);
  Serial.print(" R=");    Serial.println(rightSpd);
}

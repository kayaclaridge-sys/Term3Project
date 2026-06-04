#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Servo.h>
#include <math.h>
#include <string.h>
#include "secrets.h"

// =====================================================
// Final Medium competition flow
// - Medium mission phase orchestration
// - D53 is the only physical run/stop switch
// - RFID fertile check via server
// - Airlock A/B requests via MiniMessenger
// - Non-blocking seed dropper
// - Manual plant/dropper loading and calibration commands
// - Task2 line following, Task5-6 ramp control, and Task4-style staircase route
// =====================================================

// -------------------- Server / messaging --------------------
MiniMessenger messenger;

const char *BOARD_ID = "Robot1";
const char *SERVER_BOARD_ID = "server";

const unsigned long REGISTER_SEND_MS = 10000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;
const unsigned long SEED_PLANTED_RETRY_MS = 2000;
unsigned long lastRegisterSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastSeedPlantedRetryMs = 0;
unsigned long lastTeamStatusMs = 0;
unsigned long lastGridMapMs = 0;

const byte TEAM_STATUS_BYTES = 6;
const byte GRID_MAP_BYTES = 21;

byte latestTeamStatus[TEAM_STATUS_BYTES] = {0};
byte latestGridMap[GRID_MAP_BYTES] = {0};
bool haveTeamStatus = false;
bool haveGridMap = false;

// -------------------- Final mission orchestration --------------------
enum FinalPhase {
  FINAL_PHASE_IDLE,
  FINAL_PHASE_TASK2_TO_AIRLOCK_A,
  FINAL_PHASE_LINE_TO_RAMP,
  FINAL_PHASE_ASCEND_RAMP,
  FINAL_PHASE_TASK4_OPEN_FIELD,
  FINAL_PHASE_EXIT_RFID_AIRLOCK_B,
  FINAL_PHASE_DESCEND_RAMP,
  FINAL_PHASE_POST_RAMP_LINE,
  FINAL_PHASE_WAIT_REVIVE,
  FINAL_PHASE_COMPLETE
};

FinalPhase finalPhase = FINAL_PHASE_IDLE;
String finalExitRfidUid = "";

// -------------------- Motoron motor controllers --------------------
MotoronI2C leftController;   // 0x10, left side
MotoronI2C rightController;  // 0x11, right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

const int LEFT_MOTOR_SIGN = 1;
const int RIGHT_MOTOR_SIGN = -1;
const int TEST_FORWARD_SPEED = 350;
const int MAX_MOTOR_SPEED = 650;

bool robotEnabled = false;
bool driveRequested = false;
unsigned long driveStartMs = 0;
const unsigned long DRIVE_TEST_RUN_MS = 6000;

// -------------------- QTR 9-channel RC line sensor --------------------
const uint8_t QTR_SENSOR_COUNT = 9;
const uint8_t QTR_EMITTER_PIN = 31;
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
const long LINE_PRESENT_MIN = 250;
const int BLACK_SENSOR_THRESHOLD = 600;
const int LINE_BASE_SPEED = 240;
const int LINE_CURVE_BASE_SPEED = 220;
const int LINE_MAX_TURN = 600;
const int LINE_SEARCH_SPEED = 190;
const float LINE_KP = 0.12;
const float LINE_KD = 0.80;

uint16_t lastLinePosition = CENTER_POSITION;
unsigned long lastLineSeenMs = 0;
int lastLineError = 0;

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

enum FinalLineState {
  FINAL_LINE_FOLLOW,
  FINAL_LINE_TURN_LEFT,
  FINAL_LINE_TURN_RIGHT,
  FINAL_LINE_DOOR_GAP,
  FINAL_LINE_FAULT
};

FinalLineState finalLineState = FINAL_LINE_FOLLOW;
unsigned long finalLineStateStartedMs = 0;
bool finalTask2FirstTJunctionHandled = false;
bool finalTurnReleaseArmed = false;
bool finalDoorGapWarned = false;
uint8_t finalTJunctionFrames = 0;
uint8_t finalLeftAngleFrames = 0;
uint8_t finalRightAngleFrames = 0;
uint8_t finalHollowFrames = 0;
uint8_t rampWallEntryFrames = 0;

const uint8_t LINE_SPECIAL_DETECT_FRAMES = 3;
const uint8_t LINE_RIGHT_ANGLE_DETECT_FRAMES = 3;
const unsigned long LINE_PRE_TURN_MS = 120;
const unsigned long LINE_TURN_MIN_MS = 300;
const unsigned long LINE_TURN_MAX_MS = 6000;
const int LINE_TURN_SPEED = 580;
const int LINE_TURN_BOOST_SPEED = 620;
const unsigned long DOOR_GAP_MAX_MS = 850;
const unsigned long LINE_TO_RAMP_GAP_FAILSAFE_MS = 2200;
const int DOOR_GAP_SPEED = 210;
const uint8_t RAMP_WALL_ENTRY_CONFIRM_FRAMES = 3;

enum Task3RouteAction {
  TASK3_ACTION_STRAIGHT,
  TASK3_ACTION_LEFT,
  TASK3_ACTION_RIGHT,
  TASK3_ACTION_STOP
};

enum Task3AutoState {
  TASK3_AUTO_FOLLOW,
  TASK3_AUTO_WAIT_FERTILITY,
  TASK3_AUTO_PLANT_FORWARD,
  TASK3_AUTO_WAIT_DROP,
  TASK3_AUTO_WAIT_SEED_LOAD,
  TASK3_AUTO_TURNING,
  TASK3_AUTO_COMPLETE
};

const Task3RouteAction TASK3_FIXED_ACTIONS[] = {
  TASK3_ACTION_STRAIGHT,
  TASK3_ACTION_RIGHT,
  TASK3_ACTION_LEFT,
  TASK3_ACTION_STRAIGHT
};
const uint8_t TASK3_FIXED_ACTION_COUNT =
  sizeof(TASK3_FIXED_ACTIONS) / sizeof(TASK3_FIXED_ACTIONS[0]);
const char *const TASK3_EXIT_UID = "1B0AAB41";
const unsigned long TASK3_FERTILITY_REPLY_TIMEOUT_MS = 2500;
const int TASK3_PLANT_FORWARD_SPEED = 200;
const unsigned long TASK3_PLANT_FORWARD_MS = 180;

Task3AutoState task3AutoState = TASK3_AUTO_FOLLOW;
Task3RouteAction task3PendingAction = TASK3_ACTION_STRAIGHT;
String task3PendingTagId = "";
String task3LastActionTagId = "";
unsigned long task3AutoStateStartedMs = 0;
uint8_t task3FixedActionIndex = 0;
bool task3WaitingForServerReply = false;

enum MediumTask4State {
  MEDIUM_TASK4_IDLE,
  MEDIUM_TASK4_DRIVE_SEGMENT,
  MEDIUM_TASK4_WAIT_FERTILITY,
  MEDIUM_TASK4_PLANT_FORWARD,
  MEDIUM_TASK4_WAIT_DROP,
  MEDIUM_TASK4_WAIT_SEED_LOAD,
  MEDIUM_TASK4_TURN,
  MEDIUM_TASK4_DONE,
  MEDIUM_TASK4_FAULT
};

const uint8_t MEDIUM_TASK4_NORTH_1 = 2;
const uint8_t MEDIUM_TASK4_SIDE_NODES = 1;
const uint8_t MEDIUM_TASK4_NORTH_2 = 2;
const bool MEDIUM_TASK4_SIDE_TURN_RIGHT = true;

const long MEDIUM_TASK4_CELL_TICKS = 700;
const long MEDIUM_TASK4_CELL_TICK_LIMIT = 980;
const float MEDIUM_TASK4_RFID_ARM_FRACTION = 0.60;
const unsigned long MEDIUM_TASK4_SEGMENT_TIMEOUT_MS = 9000;
const unsigned long MEDIUM_TASK4_TOTAL_TIMEOUT_MS = 60000;
const unsigned long MEDIUM_TASK4_FERTILITY_TIMEOUT_MS = 2500;
const int MEDIUM_TASK4_CRUISE_SPEED = 330;
const int MEDIUM_TASK4_SLOW_SPEED = 190;
const int MEDIUM_TASK4_PLANT_FORWARD_SPEED = 190;
const unsigned long MEDIUM_TASK4_PLANT_FORWARD_MS = 180;
const float MEDIUM_TASK4_YAW_KP = 7.0;
const float MEDIUM_TASK4_ENCODER_BALANCE_KP = 0.12;
const int MEDIUM_TASK4_MAX_CORRECTION = 170;
const int MEDIUM_TASK4_TURN_FAST = 520;
const int MEDIUM_TASK4_TURN_SLOW = 190;
const float MEDIUM_TASK4_TURN_TARGET_DEG = 120.0;
const float MEDIUM_TASK4_TURN_TOLERANCE_DEG = 5.0;
const unsigned long MEDIUM_TASK4_TURN_TIMEOUT_MS = 4500;

MediumTask4State mediumTask4State = MEDIUM_TASK4_IDLE;
uint8_t mediumTask4SegmentIndex = 0;
uint8_t mediumTask4SegmentCells = 0;
long mediumTask4SegmentStartTicks = 0;
long mediumTask4SegmentStartLeftTicks = 0;
long mediumTask4SegmentStartRightTicks = 0;
long mediumTask4TargetTicks = 0;
float mediumTask4TargetYawDeg = 0.0;
unsigned long mediumTask4MissionStartedMs = 0;
unsigned long mediumTask4StateStartedMs = 0;
String mediumTask4PendingTagId = "";
String mediumTask4LastTagId = "";
bool mediumTask4RfidSeen = false;
bool mediumTask4WaitingForFertility = false;
bool mediumTask4PendingPlant = false;
bool mediumTask4FertilityReplyKnown = false;
bool mediumTask4FertilityShouldPlant = false;

// -------------------- RFID reader --------------------
const byte RFID_I2C_ADDRESS = 0x28;
const int RFID_RESET_PIN = -1;
TwoWire &RFID_BUS = Wire1;
MFRC522_I2C rfid(RFID_I2C_ADDRESS, RFID_RESET_PIN, &RFID_BUS);

const unsigned long RFID_POLL_MS = 80;
const unsigned long SAME_TAG_REPEAT_MS = 3000;

bool rfidReady = false;
unsigned long lastRfidPollMs = 0;
unsigned long lastTagSendMs = 0;
String lastTagId = "";
String lastPlantRequestTagId = "";
String lastPlantedTagId = "";
String activeDropTagId = "";
String pendingSeedPlantedTagId = "";
bool activeDropShouldReport = false;

// -------------------- Seed dropper --------------------
const byte SEED_SERVO_PIN = 47;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_RANGE_DEGREES = 300;
const int GATE_CLOSED_DEGREES = 20;
const int GATE_OPEN_DEGREES = 90;
const int INITIAL_SEED_COUNT = 5;
const int MAX_SEED_COUNT = 5;

const unsigned long GATE_OPEN_MS = 450;
const unsigned long GATE_CLOSE_SETTLE_MS = 250;
const int MOVE_STEP_DEGREES = 2;
const unsigned long MOVE_STEP_MS = 8;

enum DropperState {
  DROPPER_IDLE,
  DROPPER_OPENING,
  DROPPER_OPEN,
  DROPPER_CLOSING,
  DROPPER_WAIT_AFTER_CLOSE,
  DROPPER_MANUAL_OPENING,
  DROPPER_MANUAL_CLOSING
};

Servo seedServo;
DropperState dropperState = DROPPER_IDLE;
unsigned long waitStartedMs = 0;
unsigned long lastMoveStepMs = 0;
int currentAngle = GATE_CLOSED_DEGREES;
int targetAngle = GATE_CLOSED_DEGREES;
int seedsRemaining = INITIAL_SEED_COUNT;
bool resumeDriveAfterDrop = false;

// -------------------- Task 5-6 ramp controller state --------------------
const uint8_t RB_A = 42;
const uint8_t RB_B = 43;
const uint8_t LB_A = 44;
const uint8_t LB_B = 45;
const uint8_t RF_A = 48;
const uint8_t RF_B = 49;
const uint8_t LF_A = 50;
const uint8_t LF_B = 51;

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

TwoWire &MPU_BUS = Wire;
uint8_t MPU_ADDR = 0x68;

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

const float ACCEL_LSB_PER_G = 16384.0;
const float GYRO_LSB_PER_DPS = 131.0;
const float RAD_TO_DEG_F = 57.2957795;

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

const int LEFT_TRIG_PIN = 41;
const int LEFT_ECHO_PIN = 40;
const int RIGHT_TRIG_PIN = 39;
const int RIGHT_ECHO_PIN = 38;
const float SOUND_SPEED_CM_PER_US = 0.0343;
const float ULTRASONIC_MIN_VALID_CM = 1.5;
const float ULTRASONIC_MAX_VALID_CM = 120.0;
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

const int MOTOR_MAX_SPEED = 550;
const int MOTOR_MIN_CLIMB_COMMAND = 300;
const int MOTOR_MIN_DESCENT_COMMAND = -300;
const uint16_t MOTOR_MAX_ACCEL = 400;
const uint16_t MOTOR_MAX_DECEL = 400;
int approachTargetTicksPerSec = 320;
int ascendTargetTicksPerSec = 300;
int descendTargetTicksPerSec = 320;
int exitTargetTicksPerSec = 300;
int speedTrimTicksPerSec = 0;

const int LEVEL_FEEDFORWARD_COMMAND = 480;
const float SPEED_KP = 0.30;
const float ENCODER_BALANCE_KP = 0.16;
const float YAW_HOLD_KP = 7.0;
const float UPHILL_PITCH_GAIN = 30.0;
const float DOWNHILL_PITCH_BRAKE_GAIN = 14.0;
const int OVERSPEED_BRAKE_COMMAND = 180;
const int MAX_SIDE_CORRECTION = 240;
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
const long APPROACH_MAX_TICKS = 2200;
const long RAMP_MIN_TICKS = 800;
const long RAMP_MAX_TICKS = 6500;
const long EXIT_CLEAR_TICKS = 700;
const unsigned long RUN_TIMEOUT_MS = 45000;
const unsigned long EXIT_CLEAR_MAX_MS = 2500;
const float STALL_TICKS_PER_SEC = 80.0;
const unsigned long STALL_BOOST_DELAY_MS = 450;
const unsigned long STALL_FAULT_MS = 2600;
const int STALL_BOOST_COMMAND = 220;

long runStartTicks = 0;
long rampStartTicks = 0;
long exitStartTicks = 0;
float targetYawDeg = 0.0;
unsigned long missionStartMs = 0;
unsigned long rampStateStartedMs = 0;
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

// -------------------- Buttons and status LEDs --------------------
const int REVIVE_CONTACT_PIN = 32;
const int AUX_REVIVE_CONTACT_PIN = 33;
const int RUN_TOGGLE_BUTTON_PIN = 53;
const int LED_RED_PIN = 34;
const int LED_GREEN_PIN = 35;

bool previousRunToggleDown = false;
bool previousReviveContactDown = false;
bool previousAuxReviveContactDown = false;
String serialCommandBuffer = "";
unsigned long lastSerialCharMs = 0;
const unsigned long SERIAL_COMMAND_IDLE_MS = 120;

bool isSeedDropperBusy();
bool startSeedDrop();
void printHelp();
void stopDrive();
void setDriveSpeeds(int leftSpeed, int rightSpeed);
void setRobotEnabled(bool enabled, const char *reason);
bool sendSeedPlantedNotice(const String &tagId);
void sendPendingSeedPlantedIfNeeded();
void sendIsFertileRequest(const String &tagId);
void sendOpenAirlockRequest(const String &airlock, const String &tagId);
void handleIsFertileReply(const String &messageText);
void handleTextMessage(const MessageMetadata &metadata, const String &messageText);
void handleBinaryMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length);
void startFinalMission();
void advanceFinalPhase(const char *reason);
void enterFinalPhase(FinalPhase nextPhase);
void handleFinalRfidTag(const String &tagId);
void handleReviveContact(const char *sourceName);
bool finalAutonomousPhaseActive();
void resetTask3Automation();
void updateTask3Automation();
void handleTask3FertilityReply(const String &tagId, bool shouldPlant);
void resetMediumTask4Automation();
void updateMediumTask4Automation();
void handleMediumTask4Rfid(const String &tagId);
void handleMediumTask4FertilityReply(const String &tagId, bool shouldPlant);
void runQTRCalibration();
LineSnapshot readLineSnapshot();
void resetFinalLineController();
void updateFinalAutomation();
void updateLineToRamp();
bool rampEntryWallsDetected();
void startRampRun(RampProfile profile);
void updateRampStateMachine();
void enterRampState(RampState nextState);
void setupEncoders();
void resetEncoders();
void setupDistanceSensors();
bool setupIMU();
void resetYaw();
void updateAllEncoders();
void updateEncoderSpeeds();
void updateIMU();
void updateDistanceSensors();

// =====================================================
// Small helpers
// =====================================================

String payloadToString(const uint8_t *payload, size_t length) {
  String text = "";

  for (size_t i = 0; i < length; i++) {
    text += (char)payload[i];
  }

  text.trim();
  return text;
}

bool payloadLooksLikeTextMessage(const uint8_t *payload, size_t length) {
  return length >= 5 &&
         payload[0] == 't' &&
         payload[1] == 'y' &&
         payload[2] == 'p' &&
         payload[3] == 'e' &&
         payload[4] == '=';
}

String getMessageField(const String &messageText, const char *key) {
  String prefix = String(key) + "=";
  int start = messageText.indexOf(prefix);

  if (start < 0) {
    return "";
  }

  start += prefix.length();
  int end = messageText.indexOf(' ', start);

  if (end < 0) {
    end = messageText.length();
  }

  return messageText.substring(start, end);
}

bool messageTypeIs(const String &messageText, const char *expectedType) {
  String messageType = getMessageField(messageText, "type");
  return messageType == expectedType;
}

bool sendServerText(const char *messageText) {
  if (!messenger.isConnected()) {
    Serial.print("MiniMessenger disconnected, cannot send: ");
    Serial.println(messageText);
    return false;
  }

  if (messenger.sendToBoard(SERVER_BOARD_ID, messageText)) {
    Serial.print("Sent to server: ");
    Serial.println(messageText);
    return true;
  }

  Serial.print("Server send failed: ");
  Serial.println(messageText);
  return false;
}

String popFirstToken(String &text) {
  text.trim();

  if (text.length() == 0) {
    return "";
  }

  int separator = text.indexOf(' ');
  if (separator < 0) {
    String token = text;
    text = "";
    return token;
  }

  String token = text.substring(0, separator);
  text = text.substring(separator + 1);
  text.trim();
  return token;
}

bool isI2cDevicePresent(TwoWire &bus, byte address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

const char *wifiStatusName(int status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID";
    case WL_SCAN_COMPLETED:
      return "SCAN_COMPLETED";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    case WL_NO_MODULE:
      return "NO_MODULE";
    default:
      return "UNKNOWN";
  }
}

// =====================================================
// QTR line helpers
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
    qtrCalibrated[i] = constrain(value, 0, 1000);
  }
}

void runQTRCalibration() {
  stopDrive();
  resetQTRCalibration();

  Serial.println("QTR calibration: 5 seconds.");
  Serial.println("Move all 9 sensors across pale floor and black tape.");

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
    lastLinePosition = snapshot.position;
    lastLineSeenMs = millis();
  } else if (lastLinePosition < CENTER_POSITION) {
    snapshot.position = 0;
  } else {
    snapshot.position = (QTR_SENSOR_COUNT - 1) * 1000;
  }

  return snapshot;
}

bool centerLineFound(const LineSnapshot &snapshot) {
  return snapshot.centerPresent &&
         snapshot.position > 2600 &&
         snapshot.position < 5400;
}

void resetFinalLineDetectors() {
  finalTJunctionFrames = 0;
  finalLeftAngleFrames = 0;
  finalRightAngleFrames = 0;
  finalHollowFrames = 0;
}

void resetFinalLineController() {
  finalLineState = FINAL_LINE_FOLLOW;
  finalLineStateStartedMs = millis();
  finalTask2FirstTJunctionHandled = false;
  finalTurnReleaseArmed = false;
  finalDoorGapWarned = false;
  rampWallEntryFrames = 0;
  lastLineError = 0;
  lastLineSeenMs = millis();
  resetFinalLineDetectors();
}

void enterFinalLineState(FinalLineState nextState) {
  if (finalLineState != nextState) {
    Serial.print("Line state -> ");
    Serial.println(nextState == FINAL_LINE_FOLLOW ? "FOLLOW" :
                   nextState == FINAL_LINE_TURN_LEFT ? "TURN_LEFT" :
                   nextState == FINAL_LINE_TURN_RIGHT ? "TURN_RIGHT" :
                   nextState == FINAL_LINE_DOOR_GAP ? "DOOR_GAP" : "FAULT");
  }

  finalLineState = nextState;
  finalLineStateStartedMs = millis();

  if (nextState == FINAL_LINE_TURN_LEFT || nextState == FINAL_LINE_TURN_RIGHT) {
    finalTurnReleaseArmed = false;
  } else if (nextState == FINAL_LINE_DOOR_GAP) {
    finalDoorGapWarned = false;
  }
}

void updateFinalLineDetectors(const LineSnapshot &snapshot) {
  bool hollowCrossCandidate =
    snapshot.leftWing &&
    snapshot.rightWing &&
    snapshot.midCount <= 1 &&
    snapshot.blackCount >= 2;

  bool leftAngleCandidate =
    !hollowCrossCandidate &&
    snapshot.leftCount == 3 &&
    snapshot.rightCount <= 1 &&
    snapshot.midCount >= 1 &&
    snapshot.position < 4300 &&
    snapshot.blackCount >= 5 &&
    snapshot.blackCount < QTR_SENSOR_COUNT;

  bool rightAngleCandidate =
    !hollowCrossCandidate &&
    snapshot.rightCount == 3 &&
    snapshot.leftCount <= 1 &&
    snapshot.midCount >= 1 &&
    snapshot.position > 3700 &&
    snapshot.blackCount >= 5 &&
    snapshot.blackCount < QTR_SENSOR_COUNT;

  bool tJunctionCandidate =
    !hollowCrossCandidate &&
    !leftAngleCandidate &&
    !rightAngleCandidate &&
    snapshot.blackCount == QTR_SENSOR_COUNT;

  finalHollowFrames = hollowCrossCandidate ? min((int)finalHollowFrames + 1, 255) : 0;
  finalLeftAngleFrames = leftAngleCandidate ? min((int)finalLeftAngleFrames + 1, 255) : 0;
  finalRightAngleFrames = rightAngleCandidate ? min((int)finalRightAngleFrames + 1, 255) : 0;
  finalTJunctionFrames = tJunctionCandidate ? min((int)finalTJunctionFrames + 1, 255) : 0;
}

void followLineSnapshot(const LineSnapshot &snapshot) {
  if (!snapshot.linePresent) {
    enterFinalLineState(FINAL_LINE_DOOR_GAP);
    return;
  }

  int error = snapshot.position - CENTER_POSITION;
  int derivative = error - lastLineError;
  lastLineError = error;

  int base = abs(error) > 2200 ? LINE_CURVE_BASE_SPEED : LINE_BASE_SPEED;
  int turn = (int)(LINE_KP * error + LINE_KD * derivative);
  turn = constrain(turn, -LINE_MAX_TURN, LINE_MAX_TURN);

  setDriveSpeeds(constrain(base + turn, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED),
                 constrain(base - turn, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED));
}

void updateTask2LineFollow(const LineSnapshot &snapshot) {
  updateFinalLineDetectors(snapshot);

  if (finalTJunctionFrames >= LINE_SPECIAL_DETECT_FRAMES && !finalTask2FirstTJunctionHandled) {
    finalTask2FirstTJunctionHandled = true;
    resetFinalLineDetectors();
    enterFinalLineState(FINAL_LINE_TURN_LEFT);
    Serial.println("Task2 first T junction: forced LEFT.");
    return;
  }

  if (finalLeftAngleFrames >= LINE_RIGHT_ANGLE_DETECT_FRAMES) {
    resetFinalLineDetectors();
    enterFinalLineState(FINAL_LINE_TURN_LEFT);
    return;
  }

  if (finalRightAngleFrames >= LINE_RIGHT_ANGLE_DETECT_FRAMES) {
    resetFinalLineDetectors();
    enterFinalLineState(FINAL_LINE_TURN_RIGHT);
    return;
  }

  followLineSnapshot(snapshot);
}

void updateLineTurn(const LineSnapshot &snapshot) {
  unsigned long elapsed = millis() - finalLineStateStartedMs;
  bool turningLeft = finalLineState == FINAL_LINE_TURN_LEFT;
  int pivotSpeed = elapsed < 500 ? LINE_TURN_BOOST_SPEED : LINE_TURN_SPEED;

  if (turningLeft) {
    setDriveSpeeds(-pivotSpeed, pivotSpeed);
  } else {
    setDriveSpeeds(pivotSpeed, -pivotSpeed);
  }

  if (elapsed > LINE_TURN_MIN_MS &&
      (!snapshot.centerPresent || snapshot.position < 2600 || snapshot.position > 5400)) {
    finalTurnReleaseArmed = true;
  }

  if (elapsed > LINE_TURN_MIN_MS && finalTurnReleaseArmed && centerLineFound(snapshot)) {
    lastLineError = 0;
    enterFinalLineState(FINAL_LINE_FOLLOW);
  } else if (elapsed > LINE_TURN_MAX_MS) {
    Serial.println("Line turn timeout.");
    enterFinalLineState(FINAL_LINE_FAULT);
    setRobotEnabled(false, "line turn timeout");
  }
}

const char *finalPhaseName(FinalPhase phase) {
  switch (phase) {
    case FINAL_PHASE_IDLE:
      return "IDLE";
    case FINAL_PHASE_TASK2_TO_AIRLOCK_A:
      return "TASK2_TO_AIRLOCK_A";
    case FINAL_PHASE_LINE_TO_RAMP:
      return "LINE_TO_RAMP";
    case FINAL_PHASE_ASCEND_RAMP:
      return "ASCEND_RAMP";
    case FINAL_PHASE_TASK4_OPEN_FIELD:
      return "TASK4_OPEN_FIELD";
    case FINAL_PHASE_EXIT_RFID_AIRLOCK_B:
      return "EXIT_RFID_AIRLOCK_B";
    case FINAL_PHASE_DESCEND_RAMP:
      return "DESCEND_RAMP";
    case FINAL_PHASE_POST_RAMP_LINE:
      return "POST_RAMP_LINE";
    case FINAL_PHASE_WAIT_REVIVE:
      return "WAIT_REVIVE";
    case FINAL_PHASE_COMPLETE:
      return "COMPLETE";
  }

  return "?";
}

void enterFinalPhase(FinalPhase nextPhase) {
  if (finalPhase != nextPhase) {
    Serial.print("Final phase: ");
    Serial.print(finalPhaseName(finalPhase));
    Serial.print(" -> ");
    Serial.println(finalPhaseName(nextPhase));
  }

  finalPhase = nextPhase;

  if (finalPhase == FINAL_PHASE_TASK2_TO_AIRLOCK_A) {
    resetFinalLineController();
    finalTask2FirstTJunctionHandled = false;
  } else if (finalPhase == FINAL_PHASE_LINE_TO_RAMP ||
             finalPhase == FINAL_PHASE_POST_RAMP_LINE ||
             finalPhase == FINAL_PHASE_WAIT_REVIVE) {
    resetFinalLineController();
  } else if (finalPhase == FINAL_PHASE_TASK4_OPEN_FIELD) {
    resetMediumTask4Automation();
  }

  if (finalPhase == FINAL_PHASE_COMPLETE || finalPhase == FINAL_PHASE_IDLE) {
    driveRequested = false;
    resumeDriveAfterDrop = false;
    stopDrive();
  }
}

void startFinalMission() {
  finalExitRfidUid = "";
  lastTagId = "";
  lastTagSendMs = 0;
  enterFinalPhase(FINAL_PHASE_TASK2_TO_AIRLOCK_A);
  setRobotEnabled(true, "D53 run switch");

  Serial.println("Final Medium flow:");
  Serial.println("Task2 line -> RFID open A -> line to ramp -> ascend -> Task4 staircase + planting -> open B -> descend -> line until revive contact.");
}

void advanceFinalPhase(const char *reason) {
  Serial.print("Final phase advance by ");
  Serial.println(reason);

  switch (finalPhase) {
    case FINAL_PHASE_IDLE:
      startFinalMission();
      break;
    case FINAL_PHASE_TASK2_TO_AIRLOCK_A:
      enterFinalPhase(FINAL_PHASE_LINE_TO_RAMP);
      break;
    case FINAL_PHASE_LINE_TO_RAMP:
      startRampRun(PROFILE_ASCEND);
      enterFinalPhase(FINAL_PHASE_ASCEND_RAMP);
      break;
    case FINAL_PHASE_ASCEND_RAMP:
      enterFinalPhase(FINAL_PHASE_TASK4_OPEN_FIELD);
      break;
    case FINAL_PHASE_TASK4_OPEN_FIELD:
      enterFinalPhase(FINAL_PHASE_EXIT_RFID_AIRLOCK_B);
      break;
    case FINAL_PHASE_EXIT_RFID_AIRLOCK_B:
      startRampRun(PROFILE_DESCEND);
      enterFinalPhase(FINAL_PHASE_DESCEND_RAMP);
      break;
    case FINAL_PHASE_DESCEND_RAMP:
      enterFinalPhase(FINAL_PHASE_POST_RAMP_LINE);
      break;
    case FINAL_PHASE_POST_RAMP_LINE:
      enterFinalPhase(FINAL_PHASE_WAIT_REVIVE);
      break;
    case FINAL_PHASE_WAIT_REVIVE:
      enterFinalPhase(FINAL_PHASE_COMPLETE);
      setRobotEnabled(false, "mission complete");
      break;
    case FINAL_PHASE_COMPLETE:
      Serial.println("Final mission is already complete.");
      break;
  }
}

const char *task3RouteActionName(Task3RouteAction action) {
  switch (action) {
    case TASK3_ACTION_STRAIGHT:
      return "STRAIGHT";
    case TASK3_ACTION_LEFT:
      return "LEFT";
    case TASK3_ACTION_RIGHT:
      return "RIGHT";
    case TASK3_ACTION_STOP:
      return "STOP";
  }

  return "?";
}

const char *task3AutoStateName(Task3AutoState state) {
  switch (state) {
    case TASK3_AUTO_FOLLOW:
      return "FOLLOW";
    case TASK3_AUTO_WAIT_FERTILITY:
      return "WAIT_FERTILITY";
    case TASK3_AUTO_PLANT_FORWARD:
      return "PLANT_FORWARD";
    case TASK3_AUTO_WAIT_DROP:
      return "WAIT_DROP";
    case TASK3_AUTO_WAIT_SEED_LOAD:
      return "WAIT_SEED_LOAD";
    case TASK3_AUTO_TURNING:
      return "TURNING";
    case TASK3_AUTO_COMPLETE:
      return "COMPLETE";
  }

  return "?";
}

void enterTask3AutoState(Task3AutoState nextState) {
  if (task3AutoState != nextState) {
    Serial.print("Task3 auto: ");
    Serial.print(task3AutoStateName(task3AutoState));
    Serial.print(" -> ");
    Serial.println(task3AutoStateName(nextState));
  }

  task3AutoState = nextState;
  task3AutoStateStartedMs = millis();
}

void resetTask3Automation() {
  task3AutoState = TASK3_AUTO_FOLLOW;
  task3PendingAction = TASK3_ACTION_STRAIGHT;
  task3PendingTagId = "";
  task3LastActionTagId = "";
  task3AutoStateStartedMs = millis();
  task3FixedActionIndex = 0;
  task3WaitingForServerReply = false;
  lastPlantRequestTagId = "";
  lastLineError = 0;
  resetFinalLineDetectors();
}

Task3RouteAction nextTask3RouteAction() {
  if (task3FixedActionIndex >= TASK3_FIXED_ACTION_COUNT) {
    return TASK3_ACTION_STOP;
  }

  Task3RouteAction action = TASK3_FIXED_ACTIONS[task3FixedActionIndex];
  task3FixedActionIndex++;
  return action;
}

void performTask3PendingAction() {
  task3WaitingForServerReply = false;
  task3PendingTagId = "";

  Serial.print("Task3 route action: ");
  Serial.println(task3RouteActionName(task3PendingAction));

  if (task3PendingAction == TASK3_ACTION_STRAIGHT) {
    resetFinalLineDetectors();
    enterFinalLineState(FINAL_LINE_FOLLOW);
    enterTask3AutoState(TASK3_AUTO_FOLLOW);
  } else if (task3PendingAction == TASK3_ACTION_LEFT) {
    resetFinalLineDetectors();
    enterFinalLineState(FINAL_LINE_TURN_RIGHT);
    enterTask3AutoState(TASK3_AUTO_TURNING);
  } else if (task3PendingAction == TASK3_ACTION_RIGHT) {
    resetFinalLineDetectors();
    enterFinalLineState(FINAL_LINE_TURN_LEFT);
    enterTask3AutoState(TASK3_AUTO_TURNING);
  } else {
    stopDrive();
    enterTask3AutoState(TASK3_AUTO_COMPLETE);
    enterFinalPhase(FINAL_PHASE_EXIT_RFID_AIRLOCK_B);
    Serial.println("Task3 fixed actions are complete. Next RFID opens airlock B.");
  }
}

void beginTask3RfidAction(const String &tagId) {
  if (task3AutoState != TASK3_AUTO_FOLLOW) {
    Serial.println("Task3 RFID ignored while another plant/turn action is active.");
    return;
  }

  if (tagId == task3LastActionTagId) {
    Serial.print("Task3 duplicate RFID ignored: ");
    Serial.println(tagId);
    return;
  }

  task3LastActionTagId = tagId;
  task3PendingAction = nextTask3RouteAction();
  task3PendingTagId = tagId;
  task3WaitingForServerReply = true;
  stopDrive();

  Serial.print("Task3 RFID pause before ");
  Serial.print(task3RouteActionName(task3PendingAction));
  Serial.print(": ");
  Serial.println(tagId);

  sendIsFertileRequest(tagId);
  enterTask3AutoState(TASK3_AUTO_WAIT_FERTILITY);
}

void handleTask3FertilityReply(const String &tagId, bool shouldPlant) {
  if (finalPhase != FINAL_PHASE_TASK4_OPEN_FIELD ||
      task3AutoState != TASK3_AUTO_WAIT_FERTILITY ||
      !task3WaitingForServerReply) {
    return;
  }

  String normalizedTag = tagId;
  normalizedTag.toUpperCase();

  if (task3PendingTagId.length() > 0 && normalizedTag.length() > 0 &&
      normalizedTag != task3PendingTagId) {
    Serial.print("Task3 fertility reply ignored for non-pending tag: ");
    Serial.println(normalizedTag);
    return;
  }

  task3WaitingForServerReply = false;

  if (shouldPlant) {
    enterTask3AutoState(TASK3_AUTO_PLANT_FORWARD);
  } else {
    performTask3PendingAction();
  }
}

const char *mediumTask4StateName(MediumTask4State state) {
  switch (state) {
    case MEDIUM_TASK4_IDLE:
      return "IDLE";
    case MEDIUM_TASK4_DRIVE_SEGMENT:
      return "DRIVE_SEGMENT";
    case MEDIUM_TASK4_WAIT_FERTILITY:
      return "WAIT_FERTILITY";
    case MEDIUM_TASK4_PLANT_FORWARD:
      return "PLANT_FORWARD";
    case MEDIUM_TASK4_WAIT_DROP:
      return "WAIT_DROP";
    case MEDIUM_TASK4_WAIT_SEED_LOAD:
      return "WAIT_SEED_LOAD";
    case MEDIUM_TASK4_TURN:
      return "TURN";
    case MEDIUM_TASK4_DONE:
      return "DONE";
    case MEDIUM_TASK4_FAULT:
      return "FAULT";
  }

  return "?";
}

void enterMediumTask4State(MediumTask4State nextState) {
  if (mediumTask4State != nextState) {
    Serial.print("Medium Task4: ");
    Serial.print(mediumTask4StateName(mediumTask4State));
    Serial.print(" -> ");
    Serial.println(mediumTask4StateName(nextState));
  }

  mediumTask4State = nextState;
  mediumTask4StateStartedMs = millis();
}

uint8_t mediumTask4CellsForSegment(uint8_t segmentIndex) {
  if (segmentIndex == 0) {
    return MEDIUM_TASK4_NORTH_1;
  }

  if (segmentIndex == 1) {
    return MEDIUM_TASK4_SIDE_NODES;
  }

  return MEDIUM_TASK4_NORTH_2;
}

int mediumTask4TurnAfterSegment(uint8_t segmentIndex) {
  if (segmentIndex == 0) {
    return MEDIUM_TASK4_SIDE_TURN_RIGHT ? 1 : -1;
  }

  if (segmentIndex == 1) {
    return MEDIUM_TASK4_SIDE_TURN_RIGHT ? -1 : 1;
  }

  return 0;
}

void startMediumTask4DriveSegment(uint8_t segmentIndex) {
  mediumTask4SegmentIndex = segmentIndex;
  mediumTask4SegmentCells = mediumTask4CellsForSegment(segmentIndex);
  mediumTask4SegmentStartTicks = forwardTicks();
  mediumTask4SegmentStartLeftTicks = leftTicks();
  mediumTask4SegmentStartRightTicks = rightTicks();
  mediumTask4TargetTicks = (long)mediumTask4SegmentCells * MEDIUM_TASK4_CELL_TICKS;
  mediumTask4RfidSeen = false;
  mediumTask4WaitingForFertility = false;
  mediumTask4PendingPlant = false;
  mediumTask4FertilityReplyKnown = false;
  mediumTask4FertilityShouldPlant = false;
  mediumTask4PendingTagId = "";
  targetYawDeg = yawDeg;

  Serial.print("Medium Task4 segment ");
  Serial.print(segmentIndex + 1);
  Serial.print(" cells=");
  Serial.println(mediumTask4SegmentCells);

  enterMediumTask4State(MEDIUM_TASK4_DRIVE_SEGMENT);
}

void resetMediumTask4Automation() {
  resetEncoders();
  resetYaw();
  mediumTask4MissionStartedMs = millis();
  mediumTask4LastTagId = "";
  mediumTask4PendingTagId = "";
  mediumTask4SegmentIndex = 0;
  mediumTask4TargetYawDeg = yawDeg;
  mediumTask4RfidSeen = false;
  mediumTask4WaitingForFertility = false;
  mediumTask4PendingPlant = false;
  mediumTask4FertilityReplyKnown = false;
  mediumTask4FertilityShouldPlant = false;
  startMediumTask4DriveSegment(0);

  Serial.println("Medium Task4 staircase starts facing north.");
}

long mediumTask4SegmentTicks() {
  return abs(forwardTicks() - mediumTask4SegmentStartTicks);
}

void applyMediumTask4Drive() {
  long ticks = mediumTask4SegmentTicks();
  int base = ticks > (long)(mediumTask4TargetTicks * 0.75)
    ? MEDIUM_TASK4_SLOW_SPEED
    : MEDIUM_TASK4_CRUISE_SPEED;

  float headingError = angleDiffDeg(targetYawDeg, yawDeg);
  int yawCorrection = constrain((int)(headingError * MEDIUM_TASK4_YAW_KP),
                                -MEDIUM_TASK4_MAX_CORRECTION,
                                MEDIUM_TASK4_MAX_CORRECTION);

  long leftDelta = leftTicks() - mediumTask4SegmentStartLeftTicks;
  long rightDelta = rightTicks() - mediumTask4SegmentStartRightTicks;
  int balanceCorrection = constrain((int)((leftDelta - rightDelta) * MEDIUM_TASK4_ENCODER_BALANCE_KP),
                                    -MEDIUM_TASK4_MAX_CORRECTION,
                                    MEDIUM_TASK4_MAX_CORRECTION);

  setDriveSpeeds(base + yawCorrection - balanceCorrection,
                 base - yawCorrection + balanceCorrection);
}

void finishMediumTask4Segment() {
  int turnDirection = mediumTask4TurnAfterSegment(mediumTask4SegmentIndex);

  if (turnDirection == 0) {
    stopDrive();
    mediumTask4State = MEDIUM_TASK4_DONE;
    Serial.println("Medium Task4 staircase complete.");

    if (mediumTask4LastTagId.length() > 0) {
      finalExitRfidUid = mediumTask4LastTagId;
      sendOpenAirlockRequest("B", mediumTask4LastTagId);
      startRampRun(PROFILE_DESCEND);
      enterFinalPhase(FINAL_PHASE_DESCEND_RAMP);
    } else {
      enterFinalPhase(FINAL_PHASE_EXIT_RFID_AIRLOCK_B);
      Serial.println("Task4 complete without endpoint UID; next RFID will open Airlock B.");
    }

    return;
  }

  mediumTask4TargetYawDeg = wrap180(yawDeg + turnDirection * MEDIUM_TASK4_TURN_TARGET_DEG);
  enterMediumTask4State(MEDIUM_TASK4_TURN);
}

void startMediumTask4DropOrWait() {
  stopDrive();

  if (seedsRemaining <= 0) {
    Serial.println("Medium Task4 out of seeds. Load one with serial command l.");
    enterMediumTask4State(MEDIUM_TASK4_WAIT_SEED_LOAD);
    return;
  }

  activeDropTagId = mediumTask4PendingTagId;
  activeDropShouldReport = true;

  if (startSeedDrop()) {
    lastPlantedTagId = mediumTask4PendingTagId;
    enterMediumTask4State(MEDIUM_TASK4_WAIT_DROP);
  } else {
    activeDropTagId = "";
    activeDropShouldReport = false;
    enterMediumTask4State(MEDIUM_TASK4_WAIT_SEED_LOAD);
  }
}

void handleMediumTask4FertilityReply(const String &tagId, bool shouldPlant) {
  if (finalPhase != FINAL_PHASE_TASK4_OPEN_FIELD ||
      !mediumTask4WaitingForFertility) {
    return;
  }

  String normalizedTag = tagId;
  normalizedTag.toUpperCase();

  if (mediumTask4PendingTagId.length() > 0 &&
      normalizedTag.length() > 0 &&
      normalizedTag != mediumTask4PendingTagId) {
    Serial.print("Medium Task4 fertility reply ignored for non-pending tag: ");
    Serial.println(normalizedTag);
    return;
  }

  mediumTask4WaitingForFertility = false;
  mediumTask4FertilityReplyKnown = true;
  mediumTask4FertilityShouldPlant = shouldPlant;

  if (mediumTask4State == MEDIUM_TASK4_WAIT_FERTILITY) {
    if (mediumTask4FertilityShouldPlant) {
      enterMediumTask4State(MEDIUM_TASK4_PLANT_FORWARD);
    } else {
      finishMediumTask4Segment();
    }
  }
}

void handleMediumTask4Rfid(const String &tagId) {
  if (finalPhase != FINAL_PHASE_TASK4_OPEN_FIELD ||
      mediumTask4State != MEDIUM_TASK4_DRIVE_SEGMENT) {
    return;
  }

  long ticks = mediumTask4SegmentTicks();
  long armTicks = (long)(mediumTask4TargetTicks * MEDIUM_TASK4_RFID_ARM_FRACTION);

  if (ticks < armTicks) {
    Serial.print("Medium Task4 RFID ignored before arm point: ");
    Serial.println(tagId);
    return;
  }

  mediumTask4RfidSeen = true;
  mediumTask4LastTagId = tagId;
  mediumTask4PendingTagId = tagId;

  Serial.print("Medium Task4 RFID presence confirmed: ");
  Serial.println(tagId);

  if (!mediumTask4WaitingForFertility) {
    mediumTask4WaitingForFertility = true;
    sendIsFertileRequest(tagId);
  }
}

void updateMediumTask4Automation() {
  if (mediumTask4State == MEDIUM_TASK4_IDLE ||
      mediumTask4State == MEDIUM_TASK4_DONE ||
      mediumTask4State == MEDIUM_TASK4_FAULT) {
    stopDrive();
    return;
  }

  if (millis() - mediumTask4MissionStartedMs > MEDIUM_TASK4_TOTAL_TIMEOUT_MS) {
    stopDrive();
    enterMediumTask4State(MEDIUM_TASK4_FAULT);
    setRobotEnabled(false, "medium Task4 total timeout");
    return;
  }

  if (mediumTask4State == MEDIUM_TASK4_DRIVE_SEGMENT) {
    applyMediumTask4Drive();

    long ticks = mediumTask4SegmentTicks();
    long limitTicks = (long)mediumTask4SegmentCells * MEDIUM_TASK4_CELL_TICK_LIMIT;

    if (mediumTask4RfidSeen && ticks >= mediumTask4TargetTicks) {
      stopDrive();
      if (mediumTask4FertilityReplyKnown) {
        if (mediumTask4FertilityShouldPlant) {
          enterMediumTask4State(MEDIUM_TASK4_PLANT_FORWARD);
        } else {
          finishMediumTask4Segment();
        }
      } else {
        enterMediumTask4State(MEDIUM_TASK4_WAIT_FERTILITY);
      }
      return;
    }

    if (ticks > limitTicks ||
        millis() - mediumTask4StateStartedMs > MEDIUM_TASK4_SEGMENT_TIMEOUT_MS) {
      stopDrive();
      enterMediumTask4State(MEDIUM_TASK4_FAULT);
      setRobotEnabled(false, "medium Task4 segment timeout");
    }
  } else if (mediumTask4State == MEDIUM_TASK4_WAIT_FERTILITY) {
    stopDrive();

    if (mediumTask4FertilityReplyKnown) {
      if (mediumTask4FertilityShouldPlant) {
        enterMediumTask4State(MEDIUM_TASK4_PLANT_FORWARD);
      } else {
        finishMediumTask4Segment();
      }
    } else if (!mediumTask4WaitingForFertility ||
        millis() - mediumTask4StateStartedMs > MEDIUM_TASK4_FERTILITY_TIMEOUT_MS) {
      if (mediumTask4WaitingForFertility) {
        Serial.println("Medium Task4 fertility timeout; skipping plant.");
        mediumTask4WaitingForFertility = false;
      }

      finishMediumTask4Segment();
    }
  } else if (mediumTask4State == MEDIUM_TASK4_PLANT_FORWARD) {
    setDriveSpeeds(MEDIUM_TASK4_PLANT_FORWARD_SPEED, MEDIUM_TASK4_PLANT_FORWARD_SPEED);

    if (millis() - mediumTask4StateStartedMs >= MEDIUM_TASK4_PLANT_FORWARD_MS) {
      startMediumTask4DropOrWait();
    }
  } else if (mediumTask4State == MEDIUM_TASK4_WAIT_DROP) {
    stopDrive();

    if (!isSeedDropperBusy()) {
      finishMediumTask4Segment();
    }
  } else if (mediumTask4State == MEDIUM_TASK4_WAIT_SEED_LOAD) {
    stopDrive();

    if (seedsRemaining > 0) {
      Serial.println("Manual seed loaded; retrying Medium Task4 drop.");
      startMediumTask4DropOrWait();
    }
  } else if (mediumTask4State == MEDIUM_TASK4_TURN) {
    float turnError = angleDiffDeg(mediumTask4TargetYawDeg, yawDeg);

    if (fabs(turnError) <= MEDIUM_TASK4_TURN_TOLERANCE_DEG) {
      stopDrive();
      startMediumTask4DriveSegment(mediumTask4SegmentIndex + 1);
      return;
    }

    int turnSpeed = fabs(turnError) > 40.0 ? MEDIUM_TASK4_TURN_FAST : MEDIUM_TASK4_TURN_SLOW;

    if (turnError > 0.0) {
      setDriveSpeeds(turnSpeed, -turnSpeed);
    } else {
      setDriveSpeeds(-turnSpeed, turnSpeed);
    }

    if (millis() - mediumTask4StateStartedMs > MEDIUM_TASK4_TURN_TIMEOUT_MS) {
      stopDrive();
      enterMediumTask4State(MEDIUM_TASK4_FAULT);
      setRobotEnabled(false, "medium Task4 turn timeout");
    }
  }
}

void handleFinalRfidTag(const String &tagId) {
  String normalizedTag = tagId;
  normalizedTag.toUpperCase();

  switch (finalPhase) {
    case FINAL_PHASE_TASK2_TO_AIRLOCK_A:
      sendOpenAirlockRequest("A", tagId);
      enterFinalPhase(FINAL_PHASE_LINE_TO_RAMP);
      Serial.println("Continue Task2-style line following toward the ramp.");
      break;

    case FINAL_PHASE_TASK4_OPEN_FIELD:
      handleMediumTask4Rfid(tagId);
      break;

    case FINAL_PHASE_EXIT_RFID_AIRLOCK_B:
      finalExitRfidUid = tagId;
      sendOpenAirlockRequest("B", tagId);
      startRampRun(PROFILE_DESCEND);
      enterFinalPhase(FINAL_PHASE_DESCEND_RAMP);
      Serial.println("Exit RFID captured. Airlock B requested; switch to descent profile.");
      break;

    default:
      Serial.print("RFID noted in final phase ");
      Serial.print(finalPhaseName(finalPhase));
      Serial.print(": ");
      Serial.println(tagId);
      break;
  }
}

void handleReviveContact(const char *sourceName) {
  Serial.print("Revive/contact input from ");
  Serial.println(sourceName);

  if (finalPhase == FINAL_PHASE_POST_RAMP_LINE ||
      finalPhase == FINAL_PHASE_WAIT_REVIVE) {
    enterFinalPhase(FINAL_PHASE_COMPLETE);
    setRobotEnabled(false, "revive contact");
    Serial.println("Final flow complete: revive/contact reached after descent.");
    return;
  }

  Serial.println("Revive/contact does not start or stop the robot outside the final revive phase.");
}

bool finalAutonomousPhaseActive() {
  return finalPhase == FINAL_PHASE_TASK2_TO_AIRLOCK_A ||
         finalPhase == FINAL_PHASE_LINE_TO_RAMP ||
         finalPhase == FINAL_PHASE_ASCEND_RAMP ||
         finalPhase == FINAL_PHASE_TASK4_OPEN_FIELD ||
         finalPhase == FINAL_PHASE_EXIT_RFID_AIRLOCK_B ||
         finalPhase == FINAL_PHASE_DESCEND_RAMP ||
         finalPhase == FINAL_PHASE_POST_RAMP_LINE ||
         finalPhase == FINAL_PHASE_WAIT_REVIVE;
}

void updateTask2ToAirlock() {
  LineSnapshot snapshot = readLineSnapshot();

  if (finalLineState == FINAL_LINE_TURN_LEFT ||
      finalLineState == FINAL_LINE_TURN_RIGHT) {
    updateLineTurn(snapshot);
    return;
  }

  if (finalLineState == FINAL_LINE_DOOR_GAP) {
    if (snapshot.linePresent) {
      enterFinalLineState(FINAL_LINE_FOLLOW);
      followLineSnapshot(snapshot);
      return;
    }

    setDriveSpeeds(DOOR_GAP_SPEED, DOOR_GAP_SPEED);

    if (millis() - finalLineStateStartedMs > DOOR_GAP_MAX_MS) {
      Serial.println("Line missing before airlock RFID.");
      enterFinalLineState(FINAL_LINE_FAULT);
      setRobotEnabled(false, "task2 line lost");
    }

    return;
  }

  if (finalLineState == FINAL_LINE_FAULT) {
    stopDrive();
    return;
  }

  updateTask2LineFollow(snapshot);
}

void updateLineToRamp() {
  LineSnapshot snapshot = readLineSnapshot();

  if (rampEntryWallsDetected()) {
    stopDrive();
    startRampRun(PROFILE_ASCEND);
    enterFinalPhase(FINAL_PHASE_ASCEND_RAMP);
    return;
  }

  if (finalLineState == FINAL_LINE_TURN_LEFT ||
      finalLineState == FINAL_LINE_TURN_RIGHT) {
    updateLineTurn(snapshot);
    return;
  }

  if (finalLineState == FINAL_LINE_DOOR_GAP) {
    setDriveSpeeds(DOOR_GAP_SPEED, DOOR_GAP_SPEED);

    if (snapshot.linePresent && centerLineFound(snapshot)) {
      lastLineError = snapshot.position - CENTER_POSITION;
      enterFinalLineState(FINAL_LINE_FOLLOW);
      return;
    }

    unsigned long gapElapsed = millis() - finalLineStateStartedMs;

    if (gapElapsed > DOOR_GAP_MAX_MS && !finalDoorGapWarned) {
      finalDoorGapWarned = true;
      Serial.println("Door gap drive: line still missing, watching for ramp walls.");
    }

    if (gapElapsed > LINE_TO_RAMP_GAP_FAILSAFE_MS) {
      Serial.println("Door gap failsafe: no line and no ramp wall trigger.");
      enterFinalLineState(FINAL_LINE_FAULT);
      setRobotEnabled(false, "line to ramp gap timeout");
    }

    return;
  }

  if (finalLineState == FINAL_LINE_FAULT) {
    stopDrive();
    return;
  }

  if (!snapshot.linePresent) {
    enterFinalLineState(FINAL_LINE_DOOR_GAP);
    setDriveSpeeds(DOOR_GAP_SPEED, DOOR_GAP_SPEED);
    return;
  }

  followLineSnapshot(snapshot);
}

void startTask3DropOrWait() {
  stopDrive();

  if (seedsRemaining <= 0) {
    Serial.println("Task3 out of seeds. Load one seed with serial command l.");
    enterTask3AutoState(TASK3_AUTO_WAIT_SEED_LOAD);
    return;
  }

  activeDropTagId = task3PendingTagId;
  activeDropShouldReport = true;

  if (startSeedDrop()) {
    lastPlantedTagId = task3PendingTagId;
    enterTask3AutoState(TASK3_AUTO_WAIT_DROP);
  } else {
    activeDropTagId = "";
    activeDropShouldReport = false;
    enterTask3AutoState(TASK3_AUTO_WAIT_SEED_LOAD);
  }
}

void updateTask3Automation() {
  LineSnapshot snapshot = readLineSnapshot();
  unsigned long elapsed = millis() - task3AutoStateStartedMs;

  if (task3AutoState == TASK3_AUTO_FOLLOW) {
    if (finalLineState == FINAL_LINE_DOOR_GAP) {
      if (snapshot.linePresent && centerLineFound(snapshot)) {
        enterFinalLineState(FINAL_LINE_FOLLOW);
      } else {
        setDriveSpeeds(LINE_SEARCH_SPEED, LINE_SEARCH_SPEED);
        return;
      }
    }

    followLineSnapshot(snapshot);
  } else if (task3AutoState == TASK3_AUTO_WAIT_FERTILITY) {
    stopDrive();

    if (task3WaitingForServerReply &&
        elapsed > TASK3_FERTILITY_REPLY_TIMEOUT_MS) {
      Serial.println("Task3 fertility reply timeout; skipping plant and continuing route.");
      performTask3PendingAction();
    }
  } else if (task3AutoState == TASK3_AUTO_PLANT_FORWARD) {
    setDriveSpeeds(TASK3_PLANT_FORWARD_SPEED, TASK3_PLANT_FORWARD_SPEED);

    if (elapsed >= TASK3_PLANT_FORWARD_MS) {
      startTask3DropOrWait();
    }
  } else if (task3AutoState == TASK3_AUTO_WAIT_DROP) {
    stopDrive();

    if (!isSeedDropperBusy()) {
      performTask3PendingAction();
    }
  } else if (task3AutoState == TASK3_AUTO_WAIT_SEED_LOAD) {
    stopDrive();

    if (seedsRemaining > 0) {
      Serial.println("Manual seed loaded; retrying Task3 drop.");
      startTask3DropOrWait();
    }
  } else if (task3AutoState == TASK3_AUTO_TURNING) {
    updateLineTurn(snapshot);

    if (finalLineState == FINAL_LINE_FOLLOW) {
      enterTask3AutoState(TASK3_AUTO_FOLLOW);
    }
  } else {
    stopDrive();
  }
}

void updatePostRampLine() {
  LineSnapshot snapshot = readLineSnapshot();

  if (finalLineState == FINAL_LINE_TURN_LEFT ||
      finalLineState == FINAL_LINE_TURN_RIGHT) {
    updateLineTurn(snapshot);
    return;
  }

  if (finalLineState == FINAL_LINE_DOOR_GAP) {
    if (snapshot.linePresent && centerLineFound(snapshot)) {
      enterFinalLineState(FINAL_LINE_FOLLOW);
    } else {
      setDriveSpeeds(DOOR_GAP_SPEED, DOOR_GAP_SPEED);
    }
    return;
  }

  followLineSnapshot(snapshot);
}

void updateFinalAutomation() {
  if (!robotEnabled || isSeedDropperBusy()) {
    stopDrive();
    return;
  }

  switch (finalPhase) {
    case FINAL_PHASE_TASK2_TO_AIRLOCK_A:
      updateTask2ToAirlock();
      break;

    case FINAL_PHASE_LINE_TO_RAMP:
      updateLineToRamp();
      break;

    case FINAL_PHASE_ASCEND_RAMP:
      updateRampStateMachine();
      if (rampState == STATE_DONE) {
        enterFinalPhase(FINAL_PHASE_TASK4_OPEN_FIELD);
      }
      break;

    case FINAL_PHASE_TASK4_OPEN_FIELD:
      updateMediumTask4Automation();
      break;

    case FINAL_PHASE_EXIT_RFID_AIRLOCK_B:
      updatePostRampLine();
      break;

    case FINAL_PHASE_DESCEND_RAMP:
      updateRampStateMachine();
      if (rampState == STATE_DONE) {
        enterFinalPhase(FINAL_PHASE_POST_RAMP_LINE);
      }
      break;

    case FINAL_PHASE_POST_RAMP_LINE:
    case FINAL_PHASE_WAIT_REVIVE:
      updatePostRampLine();
      break;

    case FINAL_PHASE_IDLE:
    case FINAL_PHASE_COMPLETE:
      break;
  }
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
    controller->setMaxAcceleration(channel, MOTOR_MAX_ACCEL);
    controller->setMaxDeceleration(channel, MOTOR_MAX_DECEL);
  }

  Serial.print(name);
  Serial.println(" Motoron ready.");
}

void setLeftSideSpeed(int speed) {
  int command = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) * LEFT_MOTOR_SIGN;
  leftController.setSpeedNow(FRONT_LEFT_MOTOR, command);
  leftController.setSpeedNow(REAR_LEFT_MOTOR, command);
}

void setRightSideSpeed(int speed) {
  int command = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) * RIGHT_MOTOR_SIGN;
  rightController.setSpeedNow(FRONT_RIGHT_MOTOR, command);
  rightController.setSpeedNow(REAR_RIGHT_MOTOR, command);
}

void setDriveSpeeds(int leftSpeed, int rightSpeed) {
  setLeftSideSpeed(leftSpeed);
  setRightSideSpeed(rightSpeed);
}

void stopDrive() {
  setDriveSpeeds(0, 0);
}

void setRobotEnabled(bool enabled, const char *reason) {
  if (robotEnabled == enabled) {
    return;
  }

  robotEnabled = enabled;

  Serial.print("Robot state changed to ");
  Serial.print(robotEnabled ? "ENABLED" : "STOPPED");
  Serial.print(" by ");
  Serial.println(reason);

  if (!robotEnabled) {
    driveRequested = false;
    resumeDriveAfterDrop = false;
    stopDrive();
    Serial.println("Motors stopped by kill/disable.");
  }
}

void applyDriveState() {
  if (!robotEnabled || isSeedDropperBusy()) {
    stopDrive();
    return;
  }

  if (finalAutonomousPhaseActive()) {
    return;
  }

  if (!driveRequested) {
    stopDrive();
    return;
  }

  if (millis() - driveStartMs > DRIVE_TEST_RUN_MS) {
    driveRequested = false;
    stopDrive();
    Serial.println("Test drive complete.");
    return;
  }

  setDriveSpeeds(TEST_FORWARD_SPEED, TEST_FORWARD_SPEED);
}

// =====================================================
// Task 5-6 ramp controller
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
  return profile == PROFILE_ASCEND ? "ASCEND" : "DESCEND";
}

const char *rampStateName(RampState state) {
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
  imuWriteRegister(SMPLRT_DIV, 0x07);
  imuWriteRegister(CONFIG, 0x06);
  imuWriteRegister(GYRO_CONFIG, 0x00);
  imuWriteRegister(ACCEL_CONFIG, 0x00);
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

bool rampEntryWallsDetected() {
  if (lastDistanceMs != lastWallConditionFrameMs) {
    lastWallConditionFrameMs = lastDistanceMs;

    if (bothSideDistancesAtMost(WALL_ENTRY_MAX_CM)) {
      if (rampWallEntryFrames < 255) {
        rampWallEntryFrames++;
      }
    } else {
      rampWallEntryFrames = 0;
    }
  }

  return rampWallEntryFrames >= RAMP_WALL_ENTRY_CONFIRM_FRAMES;
}

// =====================================================
// Ramp controller
// =====================================================

void writeRampLeftSideSpeed(int speed) {
  int command = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED) * LEFT_MOTOR_SIGN;
  leftController.setSpeed(FRONT_LEFT_MOTOR, command);
  leftController.setSpeed(REAR_LEFT_MOTOR, command);
}

void writeRampRightSideSpeed(int speed) {
  int command = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED) * RIGHT_MOTOR_SIGN;
  rightController.setSpeed(FRONT_RIGHT_MOTOR, command);
  rightController.setSpeed(REAR_RIGHT_MOTOR, command);
}

void applyRampDriveSpeeds(int leftSpeed, int rightSpeed) {
  currentLeftCommand = constrain(leftSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  currentRightCommand = constrain(rightSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  writeRampLeftSideSpeed(currentLeftCommand);
  writeRampRightSideSpeed(currentRightCommand);
}

void enterRampState(RampState nextState) {
  if (rampState != nextState) {
    Serial.print("Ramp state: ");
    Serial.print(rampStateName(rampState));
    Serial.print(" -> ");
    Serial.println(rampStateName(nextState));
  }

  rampState = nextState;
  rampStateStartedMs = millis();

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
  enterRampState(STATE_FAULT);
  Serial.print("RAMP FAULT: ");
  Serial.println(faultMessage);
  setRobotEnabled(false, message);
}

void finishRun() {
  stopDrive();
  enterRampState(STATE_DONE);
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

  applyRampDriveSpeeds(baseCommand, baseCommand);
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

  applyRampDriveSpeeds(leftCommand, rightCommand);
}

void resetRampRunVariables() {
  resetEncoders();
  resetYaw();

  runStartTicks = forwardTicks();
  rampStartTicks = runStartTicks;
  exitStartTicks = runStartTicks;
  targetYawDeg = yawDeg;
  missionStartMs = millis();
  rampStateStartedMs = missionStartMs;
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
  enterRampState(STATE_APPROACH);

  Serial.print("Ramp run started. Profile=");
  Serial.println(profileName(selectedProfile));
}

void stopRampRun() {
  stopDrive();
  enterRampState(STATE_IDLE);
  Serial.println("Ramp run stopped.");
}

void updateRampStateMachine() {
  if (rampState == STATE_IDLE || rampState == STATE_DONE || rampState == STATE_FAULT) {
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
      enterRampState(STATE_ON_RAMP);
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

    if (rampTicks >= RAMP_MIN_TICKS && wallExitFrames >= WALL_EXIT_CONFIRM_FRAMES) {
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
    unsigned long exitMs = now - rampStateStartedMs;

    if (exitTicks >= EXIT_CLEAR_TICKS || exitMs >= EXIT_CLEAR_MAX_MS) {
      finishRun();
    }
  }
}

// =====================================================
// Seed dropper
// =====================================================

int angleToPulse(int angle) {
  angle = constrain(angle, 0, SERVO_RANGE_DEGREES);
  long pulse = SERVO_MIN_US +
               (long)(SERVO_MAX_US - SERVO_MIN_US) * angle / SERVO_RANGE_DEGREES;
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

const char *dropperStateName() {
  switch (dropperState) {
    case DROPPER_IDLE:
      return "IDLE";
    case DROPPER_OPENING:
      return "OPENING";
    case DROPPER_OPEN:
      return "OPEN";
    case DROPPER_CLOSING:
      return "CLOSING";
    case DROPPER_WAIT_AFTER_CLOSE:
      return "WAIT_AFTER_CLOSE";
    case DROPPER_MANUAL_OPENING:
      return "MANUAL_OPENING";
    case DROPPER_MANUAL_CLOSING:
      return "MANUAL_CLOSING";
  }

  return "?";
}

void setupSeedDropper() {
  seedServo.attach(SEED_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setServoAngle(GATE_CLOSED_DEGREES);
  targetAngle = GATE_CLOSED_DEGREES;
}

bool startSeedDrop() {
  if (isSeedDropperBusy()) {
    Serial.println("Drop ignored: dropper is busy.");
    return false;
  }

  if (seedsRemaining <= 0) {
    Serial.println("Drop ignored: no seeds remaining.");
    return false;
  }

  startServoMove(GATE_OPEN_DEGREES);
  dropperState = DROPPER_OPENING;
  Serial.println("Seed drop started.");
  return true;
}

void openDropperManual() {
  if (isSeedDropperBusy()) {
    Serial.println("Manual open ignored: dropper is busy.");
    return;
  }

  startServoMove(GATE_OPEN_DEGREES);
  dropperState = DROPPER_MANUAL_OPENING;
  Serial.println("Manual dropper open started.");
}

void closeDropperManual() {
  if (dropperState != DROPPER_IDLE && dropperState != DROPPER_MANUAL_OPENING) {
    Serial.println("Manual close ignored: automatic drop is active.");
    return;
  }

  startServoMove(GATE_CLOSED_DEGREES);
  dropperState = DROPPER_MANUAL_CLOSING;
  Serial.println("Manual dropper close started.");
}

void loadOneSeedManual() {
  if (seedsRemaining >= MAX_SEED_COUNT) {
    Serial.println("Seed count already full.");
    return;
  }

  seedsRemaining++;
  Serial.print("Manual seed loaded. Remaining: ");
  Serial.println(seedsRemaining);
}

void resetSeedDropper() {
  if (isSeedDropperBusy()) {
    Serial.println("Reset ignored: dropper is busy.");
    return;
  }

  seedsRemaining = INITIAL_SEED_COUNT;
  activeDropShouldReport = false;
  activeDropTagId = "";
  setServoAngle(GATE_CLOSED_DEGREES);
  targetAngle = GATE_CLOSED_DEGREES;
  Serial.println("Seed count reset and gate closed.");
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

    if (activeDropShouldReport && activeDropTagId.length() > 0) {
      pendingSeedPlantedTagId = activeDropTagId;
      activeDropShouldReport = false;
      activeDropTagId = "";
      sendPendingSeedPlantedIfNeeded();
    }
  } else if (dropperState == DROPPER_MANUAL_OPENING && updateServoMove()) {
    dropperState = DROPPER_IDLE;
    Serial.println("Manual dropper open complete.");
  } else if (dropperState == DROPPER_MANUAL_CLOSING && updateServoMove()) {
    dropperState = DROPPER_IDLE;
    Serial.println("Manual dropper close complete.");
  }
}

void updateDropperResume() {
  if (!resumeDriveAfterDrop || isSeedDropperBusy()) {
    return;
  }

  resumeDriveAfterDrop = false;

  if (robotEnabled) {
    driveRequested = true;
    driveStartMs = millis();
    Serial.println("Resuming motion after seed drop.");
  }
}

void printSeedDropperStatus() {
  Serial.print("Seeds=");
  Serial.print(seedsRemaining);
  Serial.print("/");
  Serial.print(MAX_SEED_COUNT);
  Serial.print(" state=");
  Serial.print(dropperStateName());
  Serial.print(" angle=");
  Serial.print(currentAngle);
  Serial.print(" target=");
  Serial.print(targetAngle);
  Serial.print(" lastPlanted=");
  Serial.print(lastPlantedTagId.length() ? lastPlantedTagId : "none");
  Serial.print(" pendingSeedReport=");
  Serial.println(pendingSeedPlantedTagId.length() ? pendingSeedPlantedTagId : "none");
}

// =====================================================
// RFID and planting
// =====================================================

String currentRfidTagId() {
  String tagId = "";

  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      tagId += "0";
    }

    tagId += String(rfid.uid.uidByte[i], HEX);
  }

  tagId.toUpperCase();
  return tagId;
}

void sendIsFertileRequest(const String &tagId) {
  char message[128];
  snprintf(message,
           sizeof(message),
           "type=isFertile team_id=%s tag_id=%s board_id=%s",
           GROUP_ID,
           tagId.c_str(),
           BOARD_ID);

  if (sendServerText(message)) {
    lastPlantRequestTagId = tagId;
  } else {
    Serial.print("RFID send failed for tag: ");
    Serial.println(tagId);
  }
}

bool sendSeedPlantedNotice(const String &tagId) {
  if (tagId.length() == 0) {
    Serial.println("Seed planted report skipped: no tag id.");
    return false;
  }

  char message[128];
  snprintf(message,
           sizeof(message),
           "type=seedPlanted team_id=%s tag_id=%s board_id=%s",
           GROUP_ID,
           tagId.c_str(),
           BOARD_ID);

  return sendServerText(message);
}

void sendPendingSeedPlantedIfNeeded() {
  if (pendingSeedPlantedTagId.length() == 0) {
    return;
  }

  if (lastSeedPlantedRetryMs != 0 &&
      millis() - lastSeedPlantedRetryMs < SEED_PLANTED_RETRY_MS) {
    return;
  }

  lastSeedPlantedRetryMs = millis();

  if (sendSeedPlantedNotice(pendingSeedPlantedTagId)) {
    Serial.print("Seed planted report sent for tag: ");
    Serial.println(pendingSeedPlantedTagId);
    pendingSeedPlantedTagId = "";
  }
}

void sendOpenAirlockRequest(const String &airlock, const String &tagId) {
  String airlockId = airlock;
  airlockId.trim();
  airlockId.toUpperCase();

  if (airlockId.length() == 0) {
    airlockId = "A";
  }

  String requestTagId = tagId;
  requestTagId.trim();
  requestTagId.toUpperCase();

  if (requestTagId.length() == 0) {
    requestTagId = lastTagId;
  }

  if (requestTagId.length() == 0) {
    Serial.println("Airlock request skipped: no tag id available.");
    return;
  }

  char message[160];
  snprintf(message,
           sizeof(message),
           "type=openAirlock team_id=%s airlock=%s tag_id=%s board_id=%s",
           GROUP_ID,
           airlockId.c_str(),
           requestTagId.c_str(),
           BOARD_ID);

  sendServerText(message);
}

void sendReviveRequest(const String &targetTeam, const String &targetBoard) {
  String team = targetTeam;
  String board = targetBoard;
  team.trim();
  board.trim();

  if (team.length() == 0) {
    team = GROUP_ID;
  }

  if (board.length() == 0) {
    Serial.println("Revive request skipped: missing target board.");
    return;
  }

  char message[160];
  snprintf(message,
           sizeof(message),
           "type=reviveRequest team_id=%s board_id=%s target_team=%s target_board=%s",
           GROUP_ID,
           BOARD_ID,
           team.c_str(),
           board.c_str());

  sendServerText(message);
}

void sendGetMapRequest() {
  char message[96];
  snprintf(message,
           sizeof(message),
           "type=getMap team_id=%s board_id=%s",
           GROUP_ID,
           BOARD_ID);

  sendServerText(message);
}

void handleRfidScan() {
  if (!rfidReady || isSeedDropperBusy()) {
    return;
  }

  if (!robotEnabled && finalPhase != FINAL_PHASE_IDLE) {
    return;
  }

  if (millis() - lastRfidPollMs < RFID_POLL_MS) {
    return;
  }

  lastRfidPollMs = millis();

  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println("RFID card detected, but UID could not be read.");
    return;
  }

  String tagId = currentRfidTagId();
  bool sameTagTooSoon = tagId == lastTagId && millis() - lastTagSendMs < SAME_TAG_REPEAT_MS;

  if (!sameTagTooSoon) {
    lastTagId = tagId;
    lastTagSendMs = millis();

    Serial.print("RFID tag detected: ");
    Serial.println(tagId);

    handleFinalRfidTag(tagId);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

bool textIsTrue(String text) {
  text.trim();
  text.toLowerCase();
  return text == "true" || text == "1" || text == "yes";
}

void handleIsFertileReply(const String &messageText) {
  String tagId = getMessageField(messageText, "tag_id");
  if (tagId.length() == 0) {
    tagId = getMessageField(messageText, "tag");
  }
  if (tagId.length() == 0) {
    tagId = lastPlantRequestTagId;
  }
  tagId.toUpperCase();

  String fertile = getMessageField(messageText, "fertile");
  String planted = getMessageField(messageText, "planted");
  String x = getMessageField(messageText, "x");
  String y = getMessageField(messageText, "y");

  Serial.print("Plant reply | tag=");
  Serial.print(tagId.length() ? tagId : "unknown");
  Serial.print(" fertile=");
  Serial.print(fertile.length() ? fertile : "unknown");
  Serial.print(" planted=");
  Serial.print(planted.length() ? planted : "unknown");
  Serial.print(" x=");
  Serial.print(x.length() ? x : "unknown");
  Serial.print(" y=");
  Serial.println(y.length() ? y : "unknown");

  bool shouldPlant = textIsTrue(fertile) && !textIsTrue(planted);

  if (tagId.length() > 0 && tagId == lastPlantedTagId) {
    Serial.println("Plant decision: skip duplicate planted tag.");
    shouldPlant = false;
  }

  if (finalPhase == FINAL_PHASE_TASK4_OPEN_FIELD &&
      mediumTask4WaitingForFertility) {
    handleMediumTask4FertilityReply(tagId, shouldPlant);
    return;
  }

  if (!shouldPlant) {
    Serial.println("Plant decision: skip.");
    return;
  }

  bool wasDriving = driveRequested;
  driveRequested = false;
  stopDrive();

  if (startSeedDrop()) {
    lastPlantedTagId = tagId;
    activeDropTagId = tagId;
    activeDropShouldReport = true;
    resumeDriveAfterDrop = wasDriving;
    Serial.println("Plant decision: seed drop accepted.");
  }
}

// =====================================================
// Server messages, buttons, serial commands
// =====================================================

void sendRegisterIfNeeded() {
  if (!messenger.isConnected()) {
    return;
  }

  if (lastRegisterSendMs != 0 && millis() - lastRegisterSendMs < REGISTER_SEND_MS) {
    return;
  }

  lastRegisterSendMs = millis();

  char registerMessage[96];
  snprintf(registerMessage,
           sizeof(registerMessage),
           "type=register team_id=%s board_id=%s",
           GROUP_ID,
           BOARD_ID);

  if (sendServerText(registerMessage)) {
    Serial.println("Register heartbeat sent.");
  } else {
    Serial.println("Register send failed.");
  }
}

uint8_t gridCellStateByIndex(int cellIndex) {
  if (cellIndex < 0 || cellIndex >= 81) {
    return 3;
  }

  int bitIndex = cellIndex * 2;
  int byteIndex = bitIndex / 8;
  int shift = bitIndex % 8;

  if (byteIndex >= GRID_MAP_BYTES) {
    return 3;
  }

  return (latestGridMap[byteIndex] >> shift) & 0x03;
}

void handleTeamStatusPayload(const uint8_t *payload) {
  memcpy(latestTeamStatus, payload, TEAM_STATUS_BYTES);
  haveTeamStatus = true;
  lastTeamStatusMs = millis();

  Serial.print("Team status | queueExit=");
  Serial.print(latestTeamStatus[0]);
  Serial.print(" airlockB=");
  Serial.print(latestTeamStatus[1]);
  Serial.print(" queueEnter=");
  Serial.print(latestTeamStatus[2]);
  Serial.print(" airlockA=");
  Serial.print(latestTeamStatus[3]);
  Serial.print(" emergency=");
  Serial.print(latestTeamStatus[4]);
  Serial.print(" reEntry=");
  Serial.println(latestTeamStatus[5]);

  if (latestTeamStatus[4] == 1) {
    setRobotEnabled(false, "team emergency status");
  }
}

void handleGridMapPayload(const uint8_t *payload) {
  memcpy(latestGridMap, payload, GRID_MAP_BYTES);
  haveGridMap = true;
  lastGridMapMs = millis();

  int stateCounts[4] = {0, 0, 0, 0};
  for (int cell = 0; cell < 81; cell++) {
    stateCounts[gridCellStateByIndex(cell)]++;
  }

  Serial.print("Grid map updated | sterile=");
  Serial.print(stateCounts[0]);
  Serial.print(" fertile=");
  Serial.print(stateCounts[1]);
  Serial.print(" seeded=");
  Serial.print(stateCounts[2]);
  Serial.print(" unknown=");
  Serial.println(stateCounts[3]);
}

void handleBinaryMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length) {
  Serial.print("Binary message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(" length=");
  Serial.println(length);

  if (length == TEAM_STATUS_BYTES) {
    handleTeamStatusPayload(payload);
    return;
  }

  if (length == GRID_MAP_BYTES) {
    handleGridMapPayload(payload);
    return;
  }

  Serial.println("Binary message ignored: unknown payload length.");
}

void handleOpenAirlockReply(const String &messageText) {
  String airlock = getMessageField(messageText, "airlock");
  String accepted = getMessageField(messageText, "accepted");
  if (accepted.length() == 0) {
    accepted = getMessageField(messageText, "success");
  }
  String reason = getMessageField(messageText, "reason");
  String queueEnter = getMessageField(messageText, "queue_enter");
  String queueExit = getMessageField(messageText, "queue_exit");

  Serial.print("Airlock reply | airlock=");
  Serial.print(airlock.length() ? airlock : "unknown");
  Serial.print(" accepted=");
  Serial.print(accepted.length() ? accepted : "unknown");
  Serial.print(" queueEnter=");
  Serial.print(queueEnter.length() ? queueEnter : "unknown");
  Serial.print(" queueExit=");
  Serial.print(queueExit.length() ? queueExit : "unknown");

  if (reason.length() > 0) {
    Serial.print(" reason=");
    Serial.print(reason);
  }

  Serial.println();
}

void handleReviveReply(const String &messageText) {
  String status = getMessageField(messageText, "status");
  String target = getMessageField(messageText, "target");
  if (target.length() == 0) {
    target = getMessageField(messageText, "target_board");
  }
  String accepted = getMessageField(messageText, "accepted");

  Serial.print("Revive reply | target=");
  Serial.print(target.length() ? target : "unknown");
  Serial.print(" status=");
  Serial.print(status.length() ? status : "unknown");
  Serial.print(" accepted=");
  Serial.println(accepted.length() ? accepted : "unknown");
}

void handleSeedPlantedAck(const String &messageText) {
  String tagId = getMessageField(messageText, "tag_id");
  tagId.toUpperCase();

  Serial.print("Seed planted ack | tag=");
  Serial.println(tagId.length() ? tagId : "unknown");
}

void handleTextMessage(const MessageMetadata &metadata, const String &messageText) {
  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(messageText);

  if (messageText.length() == 0) {
    return;
  }

  if (messageTypeIs(messageText, "isFertileReply")) {
    handleIsFertileReply(messageText);
    return;
  }

  if (messageTypeIs(messageText, "seedPlantedAck")) {
    handleSeedPlantedAck(messageText);
    return;
  }

  if (messageTypeIs(messageText, "heartbeat")) {
    lastHeartbeatMs = millis();

    String enable = getMessageField(messageText, "enable");
    if (enable.length() > 0) {
      if (textIsTrue(enable)) {
        Serial.println("Server heartbeat allows motion; D53 remains the run switch.");
      } else {
        setRobotEnabled(false, "server heartbeat");
      }
    }

    return;
  }

  if (messageTypeIs(messageText, "emergency")) {
    String enabled = getMessageField(messageText, "enabled");
    if (enabled.length() == 0 || textIsTrue(enabled)) {
      setRobotEnabled(false, "server emergency");
    }
    return;
  }

  if (messageTypeIs(messageText, "disable")) {
    setRobotEnabled(false, "server disable");
    return;
  }

  if (messageTypeIs(messageText, "enable")) {
    String enabled = getMessageField(messageText, "enabled");
    if (enabled.length() == 0 || textIsTrue(enabled)) {
      Serial.println("Server enable received; D53 remains the run switch.");
    }
    return;
  }

  if (messageTypeIs(messageText, "openAirlockReply") ||
      messageTypeIs(messageText, "openAirlockAck")) {
    handleOpenAirlockReply(messageText);
    return;
  }

  if (messageTypeIs(messageText, "reviveReply") ||
      messageTypeIs(messageText, "reviveAck")) {
    handleReviveReply(messageText);
    return;
  }

  if (messageTypeIs(messageText, "distress")) {
    Serial.print("Distress alert: ");
    Serial.println(messageText);
    return;
  }

  Serial.println("Message ignored: unknown MiniMessenger type.");
}

void onMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length) {
  if ((length == TEAM_STATUS_BYTES || length == GRID_MAP_BYTES) &&
      !payloadLooksLikeTextMessage(payload, length)) {
    handleBinaryMessage(metadata, payload, length);
    return;
  }

  String messageText = payloadToString(payload, length);
  handleTextMessage(metadata, messageText);
}

void applyHeartbeatWatchdog() {
  if (!robotEnabled || lastHeartbeatMs == 0) {
    return;
  }

  if (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    setRobotEnabled(false, "heartbeat timeout");
  }
}

void updateStatusLeds() {
  bool active = robotEnabled && !isSeedDropperBusy();
  digitalWrite(LED_GREEN_PIN, active ? HIGH : LOW);
  digitalWrite(LED_RED_PIN, active ? LOW : HIGH);
}

void handleButtons() {
  bool runToggleDown = digitalRead(RUN_TOGGLE_BUTTON_PIN) == LOW;
  bool reviveContactDown = digitalRead(REVIVE_CONTACT_PIN) == LOW;
  bool auxReviveContactDown = digitalRead(AUX_REVIVE_CONTACT_PIN) == LOW;

  if (runToggleDown && !previousRunToggleDown) {
    if (robotEnabled) {
      setRobotEnabled(false, "D53 run/stop switch");
    } else if (finalPhase == FINAL_PHASE_IDLE || finalPhase == FINAL_PHASE_COMPLETE) {
      startFinalMission();
    } else {
      setRobotEnabled(true, "D53 run/stop switch");
    }
  }

  if (reviveContactDown && !previousReviveContactDown) {
    handleReviveContact("D32");
  }

  if (auxReviveContactDown && !previousAuxReviveContactDown) {
    handleReviveContact("D33");
  }

  previousRunToggleDown = runToggleDown;
  previousReviveContactDown = reviveContactDown;
  previousAuxReviveContactDown = auxReviveContactDown;
}

void printHelp() {
  Serial.println();
  Serial.println("--- Final Medium flow commands ---");
  Serial.println("D53 = only physical run/stop switch");
  Serial.println("next = manually advance final phase for bench debugging");
  Serial.println("phase = print current final phase");
  Serial.println("exit / task4 done = skip to Airlock B RFID phase for bench debugging");
  Serial.println("w = short forward test when enabled");
  Serial.println("x = stop motors");
  Serial.println("D32/D33 = revive/contact input; does not start/stop the robot");
  Serial.println("d = manual seed drop");
  Serial.println("l = manual load one seed into software count");
  Serial.println("r = reset seed count and close gate");
  Serial.println("o = manual open dropper gate");
  Serial.println("c = manual close dropper gate");
  Serial.println("p = print plant/dropper status");
  Serial.println("airlock [A|B] [tag] = request openAirlock; tag defaults to last RFID");
  Serial.println("map = request immediate 21-byte grid map");
  Serial.println("revive <target_team> <target_board> = send reviveRequest");
  Serial.println("h/? = help");
  Serial.println("RFID action depends on final phase: open A, Task4 node/plant check, or open B.");
  Serial.println();
}

void processSerialCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  String normalized = command;
  normalized.toLowerCase();

  if (normalized == "phase" || normalized == "status") {
    Serial.print("Final Medium phase=");
    Serial.println(finalPhaseName(finalPhase));
    printSeedDropperStatus();
  } else if (normalized == "next") {
    advanceFinalPhase("serial next");
  } else if (normalized == "exit" || normalized == "task3 done" || normalized == "task4 done") {
    enterFinalPhase(FINAL_PHASE_EXIT_RFID_AIRLOCK_B);
    Serial.println("Next RFID will be treated as the exit UID and request airlock B.");
  } else if (normalized == "w") {
    if (!robotEnabled) {
      Serial.println("Forward test ignored: robot is disabled.");
    } else if (isSeedDropperBusy()) {
      Serial.println("Forward test ignored: dropper is busy.");
    } else {
      driveRequested = true;
      driveStartMs = millis();
      Serial.println("Short forward test requested.");
    }
  } else if (normalized == "x") {
    driveRequested = false;
    resumeDriveAfterDrop = false;
    stopDrive();
    Serial.println("Manual motor stop.");
  } else if (normalized == "e") {
    Serial.println("Ignored: D53 is the only run/stop switch in final mode.");
  } else if (normalized == "k") {
    Serial.println("Ignored: D53 is the only run/stop switch in final mode.");
  } else if (normalized == "d") {
    bool wasDriving = driveRequested;
    activeDropShouldReport = false;
    activeDropTagId = "";
    driveRequested = false;
    stopDrive();
    if (startSeedDrop()) {
      resumeDriveAfterDrop = wasDriving;
    }
  } else if (normalized == "l") {
    loadOneSeedManual();
  } else if (normalized == "r") {
    resetSeedDropper();
  } else if (normalized == "o") {
    openDropperManual();
  } else if (normalized == "c") {
    closeDropperManual();
  } else if (normalized == "p") {
    printSeedDropperStatus();
  } else if (normalized == "h" || normalized == "?") {
    printHelp();
  } else if (normalized == "map" || normalized == "getmap") {
    sendGetMapRequest();
  } else if (normalized.startsWith("airlock")) {
    String arguments = command.substring(7);
    String airlock = popFirstToken(arguments);
    String tagId = popFirstToken(arguments);

    if (airlock.length() == 0) {
      airlock = "A";
    } else {
      String airlockUpper = airlock;
      airlockUpper.toUpperCase();
      if (airlockUpper != "A" && airlockUpper != "B") {
        tagId = airlock;
        airlock = "A";
      }
    }

    sendOpenAirlockRequest(airlock, tagId);
  } else if (normalized.startsWith("revive")) {
    String arguments = command.substring(6);
    String first = popFirstToken(arguments);
    String second = popFirstToken(arguments);

    if (second.length() == 0) {
      sendReviveRequest(GROUP_ID, first);
    } else {
      sendReviveRequest(first, second);
    }
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    printHelp();
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char incoming = Serial.read();
    lastSerialCharMs = millis();

    if (incoming == '\n' || incoming == '\r') {
      processSerialCommand(serialCommandBuffer);
      serialCommandBuffer = "";
      continue;
    }

    if (serialCommandBuffer.length() < 80) {
      serialCommandBuffer += incoming;
    } else {
      serialCommandBuffer = "";
      Serial.println("Serial command too long; buffer cleared.");
    }
  }

  if (serialCommandBuffer.length() > 0 &&
      millis() - lastSerialCharMs > SERIAL_COMMAND_IDLE_MS) {
    processSerialCommand(serialCommandBuffer);
    serialCommandBuffer = "";
  }
}

void printStatusIfNeeded() {
  static unsigned long lastStatusPrintMs = 0;

  if (millis() - lastStatusPrintMs < 2000) {
    return;
  }

  lastStatusPrintMs = millis();

  Serial.print("Messenger=");
  Serial.print(messenger.isConnected() ? "CONNECTED" : "DISCONNECTED");
  Serial.print(" WiFi=");
  Serial.print(WiFi.status());
  Serial.print("(");
  Serial.print(wifiStatusName(WiFi.status()));
  Serial.print(")");
  Serial.print(" phase=");
  Serial.print(finalPhaseName(finalPhase));
  Serial.print(" robot=");
  Serial.print(robotEnabled ? "ENABLED" : "STOPPED");
  Serial.print(" drive=");
  Serial.print(driveRequested ? "ON" : "OFF");
  Serial.print(" RFID=");
  Serial.print(rfidReady ? "READY" : "NOT_FOUND");
  Serial.print(" lastTag=");
  Serial.print(lastTagId.length() ? lastTagId : "none");
  Serial.print(" teamStatus=");
  if (haveTeamStatus) {
    Serial.print(millis() - lastTeamStatusMs);
    Serial.print("msAgo");
  } else {
    Serial.print("none");
  }
  Serial.print(" map=");
  if (haveGridMap) {
    Serial.print(millis() - lastGridMapMs);
    Serial.print("msAgo");
  } else {
    Serial.print("none");
  }
  Serial.print(" ");
  printSeedDropperStatus();
}

// =====================================================
// setup / loop
// =====================================================

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {
  }

  pinMode(REVIVE_CONTACT_PIN, INPUT_PULLUP);
  pinMode(AUX_REVIVE_CONTACT_PIN, INPUT_PULLUP);
  pinMode(RUN_TOGGLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(QTR_EMITTER_PIN, OUTPUT);
  digitalWrite(QTR_EMITTER_PIN, HIGH);

  previousRunToggleDown = digitalRead(RUN_TOGGLE_BUTTON_PIN) == LOW;
  previousReviveContactDown = digitalRead(REVIVE_CONTACT_PIN) == LOW;
  previousAuxReviveContactDown = digitalRead(AUX_REVIVE_CONTACT_PIN) == LOW;

  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  Serial.println();
  Serial.println("Final Medium flow: RFID airlocks + Task4 staircase + planting + D53 run switch.");
  Serial.println("Keep wheels off the ground for first motor tests.");

  setupSeedDropper();
  setupEncoders();
  setupDistanceSensors();

  Wire1.begin();
  Wire1.setClock(400000);
  delay(100);

  leftController.setBus(&Wire1);
  setupMotoron(&leftController, "Left 0x10");

  rightController.setBus(&Wire1);
  rightController.setAddress(17);
  setupMotoron(&rightController, "Right 0x11");

  stopDrive();
  imuReady = setupIMU();
  runQTRCalibration();
  resetEncoders();
  resetYaw();
  enterRampState(STATE_IDLE);
  resetFinalLineController();
  mediumTask4State = MEDIUM_TASK4_IDLE;

  if (isI2cDevicePresent(RFID_BUS, RFID_I2C_ADDRESS)) {
    rfidReady = true;
    Serial.println("RFID module found at I2C address 0x28.");
  } else {
    rfidReady = false;
    Serial.println("RFID module not found at I2C address 0x28.");
  }

  rfid.PCD_Init();

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);

  printHelp();
  printSeedDropperStatus();
  Serial.println("Setup complete. Waiting for MiniMessenger connection...");
}

void loop() {
  messenger.loop();

  updateSeedDropper();
  updateDropperResume();
  sendRegisterIfNeeded();
  sendPendingSeedPlantedIfNeeded();
  applyHeartbeatWatchdog();
  handleButtons();
  handleSerialCommands();
  updateAllEncoders();
  updateEncoderSpeeds();
  updateIMU();
  updateDistanceSensors();
  handleRfidScan();
  updateFinalAutomation();
  applyDriveState();
  updateStatusLeds();
  printStatusIfNeeded();

  delay(4);
}

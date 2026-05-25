#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>
#include "secrets.h"

MiniMessenger messenger;

MotoronI2C leftController;   // 0x10, left side
MotoronI2C rightController;  // 0x11, right side

const char *BOARD_ID = "Robot1";

const int STATUS_LED_PIN = 4;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

const int LEFT_MOTOR_SIGN = -1;
const int RIGHT_MOTOR_SIGN = 1;
const int FORWARD_SPEED = 500;
const int MAX_SPEED = 800;

const unsigned long LED_BLINK_MS = 500;
const unsigned long STATUS_PRINT_MS = 2000;
const unsigned long REGISTER_SEND_MS = 10000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;
const unsigned long DRIVE_RUN_MS = 10000;

bool robotEnabled = false;
bool driveRequested = false;
bool ledState = false;

unsigned long lastBlinkMs = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long lastRegisterSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long driveStartMs = 0;

String payloadToString(const uint8_t *payload, size_t length) {
  String text = "";

  for (size_t i = 0; i < length; i++) {
    text += (char)payload[i];
  }

  text.trim();
  return text;
}

bool messageHas(const String &messageText, const char *needle) {
  return messageText.indexOf(needle) >= 0;
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

void setupMotoron(MotoronI2C *controller, const char *name) {
  controller->reinitialize();
  delay(20);
  controller->disableCrc();
  controller->clearResetFlag();
  controller->clearMotorFaultUnconditional();
  controller->disableCommandTimeout();

  for (uint8_t channel = 1; channel <= 3; channel++) {
    controller->setMaxAcceleration(channel, 200);
    controller->setMaxDeceleration(channel, 300);
  }

  Serial.print(name);
  Serial.println(" Motoron ready.");
}

void setLeftSideSpeed(int speed) {
  int command = constrain(speed, -MAX_SPEED, MAX_SPEED) * LEFT_MOTOR_SIGN;
  leftController.setSpeedNow(FRONT_LEFT_MOTOR, command);
  leftController.setSpeedNow(REAR_LEFT_MOTOR, command);
}

void setRightSideSpeed(int speed) {
  int command = constrain(speed, -MAX_SPEED, MAX_SPEED) * RIGHT_MOTOR_SIGN;
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
    stopDrive();
    ledState = true;
    lastBlinkMs = millis();
    digitalWrite(STATUS_LED_PIN, LED_ON);
    Serial.println("Motor stopped by kill signal.");
  }
}

void updateStatusLed() {
  if (!messenger.isConnected()) {
    ledState = false;
    digitalWrite(STATUS_LED_PIN, LED_OFF);
    return;
  }

  if (robotEnabled) {
    ledState = true;
    digitalWrite(STATUS_LED_PIN, LED_ON);
    return;
  }

  if (millis() - lastBlinkMs >= LED_BLINK_MS) {
    lastBlinkMs = millis();
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState ? LED_ON : LED_OFF);
  }
}

void applyDriveState() {
  if (!messenger.isConnected() || !robotEnabled || !driveRequested) {
    stopDrive();
    return;
  }

  if (millis() - driveStartMs > DRIVE_RUN_MS) {
    driveRequested = false;
    stopDrive();
    Serial.println("Forward run complete.");
    return;
  }

  setDriveSpeeds(FORWARD_SPEED, FORWARD_SPEED);
}

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

  if (messenger.sendToBoard("server", registerMessage)) {
    Serial.print("Registered with server: ");
    Serial.println(registerMessage);
  } else {
    Serial.println("Register send failed.");
  }
}

void onMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length) {
  String messageText = payloadToString(payload, length);

  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(messageText);

  if (messageText.length() == 0) {
    return;
  }

  if (messageHas(messageText, "type=heartbeat")) {
    lastHeartbeatMs = millis();

    if (messageHas(messageText, "enable=1")) {
      setRobotEnabled(true, "server heartbeat");
    } else if (messageHas(messageText, "enable=0")) {
      setRobotEnabled(false, "server heartbeat");
    }

    return;
  }

  if (messageHas(messageText, "type=emergency enabled=true")) {
    setRobotEnabled(false, "server emergency");
    return;
  }

  if (messageHas(messageText, "type=disable enabled=false")) {
    setRobotEnabled(false, "server disable");
    return;
  }

  if (messageHas(messageText, "type=enable enabled=true")) {
    setRobotEnabled(true, "server enable");
  }
}

void applyHeartbeatWatchdog() {
  if (!robotEnabled || lastHeartbeatMs == 0) {
    return;
  }

  if (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    setRobotEnabled(false, "heartbeat timeout");
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char command = Serial.read();

    if (command == 'w' || command == 'W') {
      if (!messenger.isConnected()) {
        Serial.println("Ignored 'w': MiniMessenger is not connected.");
      } else if (!robotEnabled) {
        Serial.println("Ignored 'w': robot is STOPPED. Press Enable on dashboard first.");
      } else {
        driveRequested = true;
        driveStartMs = millis();
        Serial.println("Forward requested at speed 500.");
      }
    } else if (command == 'x' || command == 'X') {
      driveRequested = false;
      stopDrive();
      Serial.println("Manual stop.");
    } else if (command == 'h' || command == 'H') {
      Serial.println("Commands: w=forward at speed 500, x=manual stop.");
    }
  }
}

void printStatusIfNeeded() {
  if (millis() - lastStatusPrintMs < STATUS_PRINT_MS) {
    return;
  }

  lastStatusPrintMs = millis();

  Serial.print("Messenger: ");
  Serial.print(messenger.isConnected() ? "CONNECTED" : "DISCONNECTED");
  Serial.print(" | WiFi status=");
  Serial.print(WiFi.status());
  Serial.print("(");
  Serial.print(wifiStatusName(WiFi.status()));
  Serial.print(")");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" IP=");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI=");
    Serial.print(WiFi.RSSI());
    Serial.print("dBm");
  }

  Serial.print(" | Robot: ");
  Serial.print(robotEnabled ? "ENABLED" : "STOPPED");
  Serial.print(" | drive=");
  Serial.println(driveRequested ? "FORWARD" : "STOPPED");
}

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {}

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LED_OFF);

  Serial.println("WiFi + motor kill switch test starting...");
  Serial.println("Keep wheels off the ground for the first test.");
  Serial.println("Commands: w=forward at speed 500, x=manual stop.");

  Wire1.begin();
  delay(100);

  leftController.setBus(&Wire1);
  setupMotoron(&leftController, "Left 0x10");

  rightController.setBus(&Wire1);
  rightController.setAddress(17);  // 17 decimal = 0x11
  setupMotoron(&rightController, "Right 0x11");

  stopDrive();

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);

  Serial.println("Setup complete. Waiting for MiniMessenger connection...");
}

void loop() {
  messenger.loop();

  sendRegisterIfNeeded();
  applyHeartbeatWatchdog();
  handleSerialCommands();
  applyDriveState();
  updateStatusLed();
  printStatusIfNeeded();

  delay(20);
}

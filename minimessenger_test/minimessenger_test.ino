#include <MiniMessenger.h>
#include "secrets.h"

MiniMessenger messenger;

const char *BOARD_ID = "Robot1145";

const int STATUS_LED_PIN = 4;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

const unsigned long STOP_BLINK_MS = 500;
const unsigned long STATUS_PRINT_MS = 2000;
const unsigned long REGISTER_SEND_MS = 10000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

bool robotEnabled = false;
bool ledState = false;
unsigned long lastBlinkMs = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long lastRegisterSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long receivedCount = 0;
unsigned long sentCount = 0;

String payloadToString(const uint8_t *payload, size_t length) {
  String text = "";

  for (size_t i = 0; i < length; i++) {
    text += (char)payload[i];
  }

  text.trim();
  return text;
}

String lowercaseCopy(String text) {
  text.toLowerCase();
  return text;
}

bool containsAnyCommand(String text, const char *const commands[], size_t commandCount) {
  String lowerText = lowercaseCopy(text);

  for (size_t i = 0; i < commandCount; i++) {
    if (lowerText.indexOf(commands[i]) >= 0) {
      return true;
    }
  }

  return false;
}

void setRobotEnabled(bool enabled, const char *reason) {
  if (robotEnabled == enabled) {
    return;
  }

  robotEnabled = enabled;

  if (!robotEnabled) {
    ledState = true;
    lastBlinkMs = millis();
    digitalWrite(STATUS_LED_PIN, LED_ON);
  }

  Serial.print("Robot state changed to ");
  Serial.print(robotEnabled ? "ENABLED" : "STOPPED");
  Serial.print(" by ");
  Serial.println(reason);
}

bool messageHas(const String &messageText, const char *needle) {
  return messageText.indexOf(needle) >= 0;
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

  if (millis() - lastBlinkMs >= STOP_BLINK_MS) {
    lastBlinkMs = millis();
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState ? LED_ON : LED_OFF);
  }
}

void sendAckToSender(const MessageMetadata &metadata, const String &messageText) {
  if (!messenger.isConnected()) {
    return;
  }

  String fromBoardId = String(metadata.fromBoardId);

  if (fromBoardId == BOARD_ID) {
    Serial.println("Message came from this board, so no ACK reply was sent.");
    return;
  }

  String reply = "ACK from ";
  reply += BOARD_ID;
  reply += " | state=";
  reply += (robotEnabled ? "ENABLED" : "STOPPED");
  reply += " | received=\"";
  reply += messageText;
  reply += "\"";

  messenger.sendToBoard(fromBoardId.c_str(), reply.c_str());
  sentCount++;

  Serial.print("Reply sent to Board ");
  Serial.print(fromBoardId);
  Serial.print(": ");
  Serial.println(reply);
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
    sentCount++;
    Serial.print("Registered with server: ");
    Serial.println(registerMessage);
  } else {
    Serial.println("Register send failed.");
  }
}

void onMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length) {
  receivedCount++;

  String messageText = payloadToString(payload, length);

  Serial.print("Message #");
  Serial.print(receivedCount);
  Serial.print(" from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  Serial.println(messageText);

  if (messageText.length() == 0) {
    Serial.println("Empty message ignored.");
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
    return;
  }

  const char *const stopCommands[] = {
    "disable",
    "disabled",
    "stop",
    "kill",
    "off",
    "emergency"
  };

  const char *const enableCommands[] = {
    "enable",
    "enabled",
    "go",
    "start",
    "on",
    "resume",
    "revive"
  };

  if (containsAnyCommand(messageText, stopCommands, sizeof(stopCommands) / sizeof(stopCommands[0]))) {
    setRobotEnabled(false, "text command");
  } else if (containsAnyCommand(messageText, enableCommands, sizeof(enableCommands) / sizeof(enableCommands[0]))) {
    setRobotEnabled(true, "text command");
  }

  sendAckToSender(metadata, messageText);
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
  Serial.print(" | sent=");
  Serial.print(sentCount);
  Serial.print(" | received=");
  Serial.println(receivedCount);
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

void applyHeartbeatWatchdog() {
  if (!robotEnabled) {
    return;
  }

  if (lastHeartbeatMs == 0) {
    return;
  }

  if (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    setRobotEnabled(false, "heartbeat timeout");
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {}

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LED_ON);

  Serial.println("MiniMessenger communication test starting...");
  Serial.print("Board ID: ");
  Serial.println(BOARD_ID);
  Serial.print("Group ID: ");
  Serial.println(GROUP_ID);

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BOARD_ID);

  Serial.println("Messenger setup complete. Waiting for connection...");
}

void loop() {
  messenger.loop();

  sendRegisterIfNeeded();
  applyHeartbeatWatchdog();
  updateStatusLed();
  printStatusIfNeeded();

  if (!robotEnabled) {
    // In the full robot code, call stopDrive() here and skip movement logic.
  }

  delay(10);
}

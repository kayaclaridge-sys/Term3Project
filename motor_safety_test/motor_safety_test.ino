#include <Wire.h>
#include <Motoron.h>

MotoronI2C leftController;   // 0x10, left side
MotoronI2C rightController;  // 0x11, right side

const uint8_t FRONT_LEFT_MOTOR = 1;
const uint8_t REAR_LEFT_MOTOR = 2;
const uint8_t FRONT_RIGHT_MOTOR = 1;
const uint8_t REAR_RIGHT_MOTOR = 2;

const int LEFT_MOTOR_SIGN = -1;
const int RIGHT_MOTOR_SIGN = 1;

const int MAX_SPEED = 800;
const int SPEED_STEP = 100;
const unsigned long COMMAND_TIMEOUT_MS = 5000;
const uint16_t MOTORON_REFERENCE_MV = 5000;

int testSpeed = 500;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
unsigned long lastMotionCommandMs = 0;

void printHex16(uint16_t value) {
  if (value < 0x1000) Serial.print("0");
  if (value < 0x0100) Serial.print("0");
  if (value < 0x0010) Serial.print("0");
  Serial.print(value, HEX);
}

bool isI2cDevicePresent(TwoWire &bus, byte address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void scanI2cBus(TwoWire &bus, const char *name) {
  Serial.print("Scanning ");
  Serial.println(name);

  bool foundAny = false;
  for (byte address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    byte error = bus.endTransmission();

    if (error == 0) {
      foundAny = true;
      Serial.print("Found I2C device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (!foundAny) {
    Serial.println("No I2C devices found on this bus.");
  }
}

void printMotoronFlags(MotoronI2C *controller, const char *name) {
  uint16_t statusFlags = controller->getStatusFlags();
  uint8_t lastError = controller->getLastError();
  uint32_t vinMv = controller->getVinVoltageMv(MOTORON_REFERENCE_MV, MotoronVinSenseType::Motoron550);

  Serial.print(name);
  Serial.print(" status=0x");
  printHex16(statusFlags);
  Serial.print(" lastError=");
  Serial.print(lastError);
  Serial.print(" VIN=");
  Serial.print(vinMv);
  Serial.print("mV");

  if (controller->getNoPowerFlag()) Serial.print(" NO_POWER");
  if (controller->getNoPowerLatchedFlag()) Serial.print(" NO_POWER_LATCHED");
  if (controller->getMotorFaultingFlag()) Serial.print(" MOTOR_FAULTING");
  if (controller->getMotorFaultLatchedFlag()) Serial.print(" MOTOR_FAULT_LATCHED");
  if (controller->getResetFlag()) Serial.print(" RESET");
  if (controller->getErrorActiveFlag()) Serial.print(" ERROR_ACTIVE");
  if (controller->getMotorOutputEnabledFlag()) Serial.print(" OUTPUT_ENABLED");
  if (controller->getMotorDrivingFlag()) Serial.print(" DRIVING");

  Serial.println();
}

void printBothMotoronFlags() {
  printMotoronFlags(&leftController, "Left 0x10");
  printMotoronFlags(&rightController, "Right 0x11");
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

  uint16_t productId = 0;
  uint16_t firmwareVersion = 0;
  controller->getFirmwareVersion(&productId, &firmwareVersion);
  Serial.print(name);
  Serial.print(" product=0x");
  printHex16(productId);
  Serial.print(" firmware=0x");
  printHex16(firmwareVersion);
  Serial.print(" lastError=");
  Serial.println(controller->getLastError());

  printMotoronFlags(controller, name);
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
  currentLeftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  currentRightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setLeftSideSpeed(currentLeftSpeed);
  setRightSideSpeed(currentRightSpeed);

  lastMotionCommandMs = millis();

  Serial.print("Drive command | left=");
  Serial.print(currentLeftSpeed);
  Serial.print(" right=");
  Serial.println(currentRightSpeed);
  printBothMotoronFlags();
}

void stopDrive() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  setLeftSideSpeed(0);
  setRightSideSpeed(0);
  Serial.println("Drive stopped.");
}

void printHelp() {
  Serial.println();
  Serial.println("--- Motor safety test commands ---");
  Serial.println("w = forward");
  Serial.println("s = reverse");
  Serial.println("a = turn left in place");
  Serial.println("d = turn right in place");
  Serial.println("q = left side forward only");
  Serial.println("e = right side forward only");
  Serial.println("z = left side reverse only");
  Serial.println("c = right side reverse only");
  Serial.println("x = stop");
  Serial.println("+ = speed up by 100");
  Serial.println("- = speed down by 100");
  Serial.println("h = help");
  Serial.println("p = print Motoron status");
  Serial.print("Current test speed: ");
  Serial.println(testSpeed);
  Serial.println("Safety: motors auto-stop after 5 seconds without a motion command.");
  Serial.println();
}

void handleSerialCommand(char command) {
  switch (command) {
    case 'w':
    case 'W':
      Serial.println("Forward");
      setDriveSpeeds(testSpeed, testSpeed);
      break;

    case 's':
    case 'S':
      Serial.println("Reverse");
      setDriveSpeeds(-testSpeed, -testSpeed);
      break;

    case 'a':
    case 'A':
      Serial.println("Turn left in place");
      setDriveSpeeds(-testSpeed, testSpeed);
      break;

    case 'd':
    case 'D':
      Serial.println("Turn right in place");
      setDriveSpeeds(testSpeed, -testSpeed);
      break;

    case 'q':
    case 'Q':
      Serial.println("Left side forward only");
      setDriveSpeeds(testSpeed, 0);
      break;

    case 'e':
    case 'E':
      Serial.println("Right side forward only");
      setDriveSpeeds(0, testSpeed);
      break;

    case 'z':
    case 'Z':
      Serial.println("Left side reverse only");
      setDriveSpeeds(-testSpeed, 0);
      break;

    case 'c':
    case 'C':
      Serial.println("Right side reverse only");
      setDriveSpeeds(0, -testSpeed);
      break;

    case 'x':
    case 'X':
      stopDrive();
      break;

    case '+':
    case '=':
      testSpeed = constrain(testSpeed + SPEED_STEP, 0, MAX_SPEED);
      Serial.print("Test speed now ");
      Serial.println(testSpeed);
      break;

    case '-':
    case '_':
      testSpeed = constrain(testSpeed - SPEED_STEP, 0, MAX_SPEED);
      Serial.print("Test speed now ");
      Serial.println(testSpeed);
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    case 'p':
    case 'P':
      printBothMotoronFlags();
      break;

    case '\n':
    case '\r':
      break;

    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
      printHelp();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {}

  Serial.println("Motor safety test starting...");
  Serial.println("Keep robot wheels off the ground for the first test.");

  Wire1.begin();
  delay(100);

  scanI2cBus(Wire1, "Wire1");
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

  stopDrive();
  printHelp();
}

void loop() {
  while (Serial.available()) {
    handleSerialCommand((char)Serial.read());
  }

  bool moving = currentLeftSpeed != 0 || currentRightSpeed != 0;
  if (moving && millis() - lastMotionCommandMs > COMMAND_TIMEOUT_MS) {
    Serial.println("Safety timeout.");
    stopDrive();
  }

  delay(10);
}

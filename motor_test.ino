#include <Wire.h>
#include <Motoron.h>

MotoronI2C *mc = nullptr;
TwoWire *motorBus = nullptr;
uint8_t motorAddress = 0;

// Encoder wiring
const int encoderA = 2;  // orange
const int encoderB = 3;  // yellow

volatile long encoderCount = 0;

void readEncoder() {
  int b = digitalRead(encoderB);

  if (b == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

bool scanBus(TwoWire &bus, const char *busName) {
  Serial.print("Scanning ");
  Serial.println(busName);

  uint8_t firstFound = 0;
  bool foundDefault = false;

  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    byte error = bus.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);

      if (addr == 0x10) {
        motorAddress = addr;
        motorBus = &bus;
        foundDefault = true;
      }

      if (firstFound == 0) {
        firstFound = addr;
      }
    }
  }

  if (foundDefault) return true;

  if (firstFound != 0) {
    motorAddress = firstFound;
    motorBus = &bus;
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Motoron + encoder test starting...");

  // Encoder setup
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, CHANGE);

  // Motoron setup
  Wire.begin();
  Wire1.begin();

  bool found = scanBus(Wire, "Wire");

  if (!found) {
    found = scanBus(Wire1, "Wire1");
  }

  if (!found) {
    Serial.println("No I2C device found.");
    while (true) {}
  }

  mc = new MotoronI2C(motorAddress);
  mc->setBus(motorBus);

  mc->reinitialize();
  delay(20);

  mc->disableCrc();
  mc->clearResetFlag();
  mc->clearMotorFaultUnconditional();
  mc->disableCommandTimeout();

  mc->setMaxAcceleration(1, 200);
  mc->setMaxDeceleration(1, 300);

  Serial.println("Setup complete.");
}

void loop() {
  Serial.println("M1 Forward");
  encoderCount = 0;

  mc->setSpeed(1, 400);

  for (int i = 0; i < 30; i++) {
    Serial.print("Encoder count: ");
    Serial.println(encoderCount);
    delay(100);
  }

  Serial.println("M1 Stop");
  mc->setSpeed(1, 0);
  delay(2000);

  Serial.println("M1 Reverse");
  encoderCount = 0;

  mc->setSpeed(1, -400);

  for (int i = 0; i < 30; i++) {
    Serial.print("Encoder count: ");
    Serial.println(encoderCount);
    delay(100);
  }

  Serial.println("M1 Stop");
  mc->setSpeed(1, 0);
  delay(2000);
}
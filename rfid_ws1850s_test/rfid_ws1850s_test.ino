#include <Wire.h>
#include <MFRC522_I2C.h>

const byte RFID_I2C_ADDRESS = 0x28;
const int RFID_RESET_PIN = -1;

const unsigned long SAME_CARD_REPEAT_MS = 1500;
const unsigned long STATUS_PRINT_MS = 2000;

MFRC522_I2C rfid(RFID_I2C_ADDRESS, RFID_RESET_PIN);

String lastUid = "";
unsigned long lastUidPrintMs = 0;
unsigned long lastStatusPrintMs = 0;

bool isI2cDevicePresent(byte address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printHexByte(byte value) {
  if (value < 0x10) {
    Serial.print("0");
  }
  Serial.print(value, HEX);
}

String uidToString() {
  String uid = "";

  for (byte i = 0; i < rfid.uid.size; i++) {
    if (i > 0) {
      uid += ":";
    }

    if (rfid.uid.uidByte[i] < 0x10) {
      uid += "0";
    }

    uid += String(rfid.uid.uidByte[i], HEX);
  }

  uid.toUpperCase();
  return uid;
}

void printUidBytes() {
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (i > 0) {
      Serial.print(" ");
    }
    printHexByte(rfid.uid.uidByte[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) {}

  Serial.println("WS1850S RFID2 test starting...");
  Serial.println("Wiring: black=GND, red=5V, yellow=SDA, white=SCL.");
  Serial.println("I2C address: 0x28");

  Wire.begin();
  Wire.setClock(400000);

  if (!isI2cDevicePresent(RFID_I2C_ADDRESS)) {
    Serial.println("RFID module not found on I2C address 0x28.");
    Serial.println("Check power, GND, SDA/SCL wiring, and that the cable is fully seated.");
  } else {
    Serial.println("RFID module found on I2C address 0x28.");
  }

  rfid.PCD_Init();
  Serial.println("Place an RFID/NFC card near the reader.");
}

void loop() {
  if (millis() - lastStatusPrintMs >= STATUS_PRINT_MS) {
    lastStatusPrintMs = millis();
    Serial.println("Waiting for card...");
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    delay(50);
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println("Card detected, but UID could not be read.");
    delay(100);
    return;
  }

  String currentUid = uidToString();
  if (currentUid == lastUid && millis() - lastUidPrintMs < SAME_CARD_REPEAT_MS) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    delay(50);
    return;
  }

  lastUid = currentUid;
  lastUidPrintMs = millis();

  byte piccType = rfid.PICC_GetType(rfid.uid.sak);

  Serial.println("Card detected.");
  Serial.print("Type: ");
  Serial.println(rfid.PICC_GetTypeName(piccType));
  Serial.print("UID: ");
  Serial.println(currentUid);
  Serial.print("UID bytes: ");
  printUidBytes();
  Serial.println();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  delay(100);
}

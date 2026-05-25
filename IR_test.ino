#include <kvstore_global_api.h>

const int sensorPins[9] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
const int ctrlPin = 12;
const int SensorCount = 9;
const unsigned int timeout = 2500;

uint16_t minValues[9];
uint16_t maxValues[9];
uint16_t lastPosition = 0;

unsigned int readPrivate(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);

  unsigned long start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > timeout) return timeout;
  }
  return micros() - start;
}

void saveCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    char key[20];

    sprintf(key, "/kv/min%d", i);
    kv_set(key, &minValues[i], sizeof(uint16_t), 0);

    sprintf(key, "/kv/max%d", i);
    kv_set(key, &maxValues[i], sizeof(uint16_t), 0);
  }
  Serial.println("Calibration saved.");
}

bool loadCalibration() {
  size_t actual_size;

  for (int i = 0; i < SensorCount; i++) {
    char key[20];

    sprintf(key, "/kv/min%d", i);
    if (kv_get(key, &minValues[i], sizeof(uint16_t), &actual_size) != 0) {
      return false;
    }

    sprintf(key, "/kv/max%d", i);
    if (kv_get(key, &maxValues[i], sizeof(uint16_t), &actual_size) != 0) {
      return false;
    }
  }

  Serial.println("Calibration loaded.");
  return true;
}

void runCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = timeout;
    maxValues[i] = 0;
  }

  Serial.println("--- CALIBRATION STARTING (10 SECONDS) ---");
  Serial.println("Slide sensors over the black line repeatedly!");

  for (int j = 0; j < 400; j++) {
    for (int i = 0; i < SensorCount; i++) {
      unsigned int val = readPrivate(sensorPins[i]);

      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }

    if (j % 40 == 0) Serial.println("Still calibrating...");
    delay(10);
  }

  saveCalibration();
  Serial.println("--- CALIBRATION COMPLETE ---");
}

void setup() {
  Serial.begin(115200);

  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  pinMode(ctrlPin, OUTPUT);
  digitalWrite(ctrlPin, HIGH);

  if (!loadCalibration()) {
    Serial.println("No saved calibration found, running calibration...");
    runCalibration();
  } else {
    Serial.println("Send 'c' within 3 seconds to recalibrate...");

    unsigned long start = millis();
    while (millis() - start < 3000) {
      if (Serial.available() && Serial.read() == 'c') {
        runCalibration();
        break;
      }
    }
  }

  Serial.println("Starting Live Data...");
  delay(1000);
}

void loop() {
  long avg = 0;
  long sum = 0;

  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);

    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);

    avg += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;

    Serial.print(calibratedVal);
    Serial.print("\t");
  }

  if (sum > 200) {
    lastPosition = avg / sum;
  } else {
    if (lastPosition < (SensorCount - 1) * 1000 / 2) {
      lastPosition = 0;
    } else {
      lastPosition = (SensorCount - 1) * 1000;
    }
  }

  Serial.print("| Pos: ");
  Serial.println(lastPosition);

  delay(20);
}
#include <Wire.h>
#include <math.h>

TwoWire &MPU_BUS = Wire;

uint8_t MPU_ADDR = 0x68;

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

void writeRegister(uint8_t reg, uint8_t value) {
  MPU_BUS.beginTransmission(MPU_ADDR);
  MPU_BUS.write(reg);
  MPU_BUS.write(value);
  MPU_BUS.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  MPU_BUS.beginTransmission(MPU_ADDR);
  MPU_BUS.write(reg);
  MPU_BUS.endTransmission(false);

  MPU_BUS.requestFrom(MPU_ADDR, (uint8_t)1);

  if (MPU_BUS.available()) {
    return MPU_BUS.read();
  }

  return 0xFF;
}

bool readBytes(uint8_t reg, uint8_t count, uint8_t *buffer) {
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

bool checkAddress(uint8_t address) {
  MPU_BUS.beginTransmission(address);
  return MPU_BUS.endTransmission() == 0;
}

bool findMPU6050() {
  if (checkAddress(0x68)) {
    MPU_ADDR = 0x68;
    return true;
  }

  if (checkAddress(0x69)) {
    MPU_ADDR = 0x69;
    return true;
  }

  return false;
}

void setupMPU6050() {
  // Wake up MPU6050
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Sample rate = 1kHz / (1 + 7) = 125Hz
  writeRegister(SMPLRT_DIV, 0x07);

  // Digital low-pass filter
  writeRegister(CONFIG, 0x06);

  // Gyro range: ±250 deg/s
  writeRegister(GYRO_CONFIG, 0x00);

  // Accelerometer range: ±2g
  writeRegister(ACCEL_CONFIG, 0x00);
}

void setup() {
  Serial.begin(115200);

  unsigned long startTime = millis();
  while (!Serial && millis() - startTime < 3000) {
    // Wait for Serial Monitor, but not forever
  }

  MPU_BUS.begin();
  MPU_BUS.setClock(100000);

  Serial.println("Arduino GIGA + MPU6050 Test");
  Serial.println("Using Wire: D20 = SDA, D21 = SCL");
  Serial.println();

  if (!findMPU6050()) {
    Serial.println("MPU6050 not found on Wire.");
    Serial.println("Check wiring: SDA=D20, SCL=D21, VCC=3.3V, GND=GND.");
    while (1) {
      delay(1000);
    }
  }

  Serial.print("MPU6050 found at address 0x");
  Serial.println(MPU_ADDR, HEX);

  uint8_t id = readRegister(WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(id, HEX);

  if (id != 0x68) {
    Serial.println("Warning: address exists, but WHO_AM_I is not 0x68.");
    Serial.println("This may not be an MPU6050.");
  }

  setupMPU6050();

  Serial.println("MPU6050 initialized.");
  Serial.println();
}

void loop() {
  uint8_t data[14];

  if (!readBytes(ACCEL_XOUT_H, 14, data)) {
    Serial.println("Failed to read MPU6050 data.");
    delay(500);
    return;
  }

  int16_t rawAx = (data[0] << 8) | data[1];
  int16_t rawAy = (data[2] << 8) | data[3];
  int16_t rawAz = (data[4] << 8) | data[5];

  int16_t rawTemp = (data[6] << 8) | data[7];

  int16_t rawGx = (data[8] << 8) | data[9];
  int16_t rawGy = (data[10] << 8) | data[11];
  int16_t rawGz = (data[12] << 8) | data[13];

  // ±2g range: 16384 LSB/g
  float ax = rawAx / 16384.0;
  float ay = rawAy / 16384.0;
  float az = rawAz / 16384.0;

  // ±250 deg/s range: 131 LSB/(deg/s)
  float gx = rawGx / 131.0;
  float gy = rawGy / 131.0;
  float gz = rawGz / 131.0;

  float temperature = rawTemp / 340.0 + 36.53;

  float roll = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  Serial.println("========== MPU6050 Data ==========");

  Serial.print("Accel X: ");
  Serial.print(ax, 3);
  Serial.print(" g\t");

  Serial.print("Y: ");
  Serial.print(ay, 3);
  Serial.print(" g\t");

  Serial.print("Z: ");
  Serial.print(az, 3);
  Serial.println(" g");

  Serial.print("Gyro  X: ");
  Serial.print(gx, 3);
  Serial.print(" deg/s\t");

  Serial.print("Y: ");
  Serial.print(gy, 3);
  Serial.print(" deg/s\t");

  Serial.print("Z: ");
  Serial.print(gz, 3);
  Serial.println(" deg/s");

  Serial.print("Roll: ");
  Serial.print(roll, 2);
  Serial.print(" deg\t");

  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.println(" deg");

  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" C");

  Serial.println();

  delay(500);
}

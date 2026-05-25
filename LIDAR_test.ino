/*
 * TF-Luna LiDAR - UART Mode
 * For Arduino GIGA R1 WiFi
 * 
 * --- WIRING ---
 * TF-Luna 5V/VCC -> GIGA 5V
 * TF-Luna GND    -> GIGA GND
 * TF-Luna TX     -> GIGA Serial1 RX
 * TF-Luna RX     -> GIGA Serial1 TX
 */

const int HEADER = 0x59;

int uart_data[9];

int current_distance = 0;
int current_strength = 0;
float current_temp = 0.0;

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  Serial1.begin(115200);

  Serial.println("=====================================");
  Serial.println(" TF-Luna LiDAR Initialized on GIGA   ");
  Serial.println("=====================================");
}

void loop() {
  if (readTFLuna()) {
    Serial.print("Distance: ");
    Serial.print(current_distance);
    Serial.print(" cm \t| Strength: ");
    Serial.print(current_strength);
    Serial.print(" \t| Temp: ");
    Serial.print(current_temp);
    Serial.println(" C");
  }
}

bool readTFLuna() {
  if (Serial1.available() < 9) {
    return false;
  }

  if (Serial1.read() != HEADER) {
    return false;
  }

  if (Serial1.read() != HEADER) {
    return false;
  }

  uart_data[0] = HEADER;
  uart_data[1] = HEADER;

  for (int i = 2; i < 9; i++) {
    uart_data[i] = Serial1.read();
  }

  int check = 0;
  for (int i = 0; i < 8; i++) {
    check += uart_data[i];
  }

  if (uart_data[8] != (check & 0xff)) {
    return false;
  }

  current_distance = uart_data[2] + (uart_data[3] * 256);
  current_strength = uart_data[4] + (uart_data[5] * 256);

  int temp_raw = uart_data[6] + (uart_data[7] * 256);
  current_temp = (temp_raw / 8.0) - 256.0;

  return true;
}
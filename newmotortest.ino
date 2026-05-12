#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc1; // Shield 0x10 — LEFT side  (M1 = front-left, M2 = rear-left)
MotoronI2C mc2; // Shield 0x11 — RIGHT side (M1 = front-right, M2 = rear-right)

int defaultSpeed = 600;


void setLeftTrack(int speed) {
  mc1.setSpeed(1, speed);  // Front-left
  mc1.setSpeed(2, speed);  // Rear-left
}

void setRightTrack(int speed) {
  mc2.setSpeed(1, speed);  // Front-right
  mc2.setSpeed(2, speed);  // Rear-right
}

void stopTracks() {
  setLeftTrack(0);
  setRightTrack(0);
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire1.begin();
  delay(100);

  // Shield 1 — 0x10
  mc1.setBus(&Wire1);
  mc1.reinitialize();
  delay(10);
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc1.disableCommandTimeout();

  // Shield 2 — 0x11
  mc2.setBus(&Wire1);
  mc2.setAddress(17); // 17 decimal = 0x11
  mc2.reinitialize();
  delay(10);
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.disableCommandTimeout();

  Serial.println("--- TANK CONTROL ONLINE ---");
  Serial.println("W=All Fwd | S=All Rev | A=Left only | D=Right only | X=Stop");
}


void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'w') {
      Serial.println("All forward");
      setLeftTrack(-defaultSpeed);
      setRightTrack(defaultSpeed);
    }
    else if (cmd == 's') {
      Serial.println("All reverse");
      setLeftTrack(defaultSpeed);
      setRightTrack(-defaultSpeed);
    }
    else if (cmd == 'a') {
      Serial.println("Left motors only");
      setLeftTrack(-defaultSpeed);
      setRightTrack(0);
    }
    else if (cmd == 'd') {
      Serial.println("Right motors only");
      setLeftTrack(0);
      setRightTrack(defaultSpeed);
    }
    else if (cmd == 'x') {
      Serial.println("Stop");
      stopTracks();
    }
  }
}
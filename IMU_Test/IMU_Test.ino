#include <Arduino_LSM9DS1.h>

int buttonPin = 5;
int state = 0;

byte lastButtonPressed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU.begin();

  pinMode(5, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:

  float x, y, z;
  switch (state) {
    case 0:
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x, y, z);
        Serial.println("MagneticField," + String(x) + "," + String(y) + "," + String(z));
      }
      break;
    case 1:
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        Serial.println("Acceleration," + String(x) + "," + String(y) + "," + String(z));
      }
      break;
    case 2:
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        Serial.println("Gyroscope," + String(x) + "," + String(y) + "," + String(z));
      }
      break;
  }

  if (isButtonPressed() && !lastButtonPressed) {
    state = (state + 1) % 3;
  }
  lastButtonPressed = isButtonPressed();
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

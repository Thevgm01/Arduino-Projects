#include <Arduino_LSM9DS1.h>

int buttonPin = 5;
int state = 1;
int filter = 0;

float emaX, emaY, emaZ;
const float emaAlpha = 0.3f;

byte lastButtonPressed = 0;
unsigned long buttonHoldTime = 0;
bool stateChanged = false;

String title;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU.begin();

  pinMode(5, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:

  title = "";
  float x, y, z;
  switch (state) {
    case 0:
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x, y, z);
        title = "MagneticField";
      }
      break;
    case 1:
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        title = "Acceleration";
      }
      break;
    case 2:
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        title = "Gyroscope";
      }
      break;
  }

  if (title != "") {
    filterXYZ(x, y, z);
    Serial.print(title + ",");
    Serial.println(String(emaX) + "," + String(emaY) + "," + String(emaZ));
  }

  if (isButtonPressed()) {
    if (!lastButtonPressed) {
      buttonHoldTime = millis() + 1000;
    } else if (millis() >= buttonHoldTime) {
      buttonHoldTime = millis() + 1000;
      state = (state + 1) % 3;
      stateChanged = true;
    }
  } else if(lastButtonPressed) {
    buttonHoldTime = 0;
    if (!stateChanged) {
      filter = (filter + 1) % 3;
    }
    stateChanged = false;
  }
  lastButtonPressed = isButtonPressed();
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void filterXYZ(float x, float y, float z) {
  switch (filter) {
    case 0: // None
      emaX = x;
      emaY = y;
      emaZ = z;
      title += "Unfiltered";
      break;
    case 1: // Low-pass
      emaX = x * emaAlpha + emaX * (1 - emaAlpha);
      emaY = y * emaAlpha + emaY * (1 - emaAlpha);
      emaZ = z * emaAlpha + emaZ * (1 - emaAlpha);
      title += "LowPass";
      break;
    case 2: // High-pass
      emaX = x - (x * emaAlpha + emaX * (1 - emaAlpha));
      emaY = y - (y * emaAlpha + emaY * (1 - emaAlpha));
      emaZ = z - (z * emaAlpha + emaZ * (1 - emaAlpha));
      title += "HighPass";
      break;
  }
}

#include <Arduino_LSM9DS1.h>

int buttonPin = 5;
int state = 1;
int axis = 0;

float emaReading = 0.0f;
const float emaAlpha = 0.1f;
float reading, lowpass, highpass;

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
    switch (axis) {
      case 0:
        reading = x;
        title += "X";
        break;
      case 1:
        reading = y;
        title += "Y";
        break;
      case 2:
        reading = z;
        title += "Z";
        break;
    }
    filterReading();
    Serial.print(title + ",");
    Serial.println(String(reading) + "," + String(lowpass) + "," + String(highpass));
  }

  handleButton();
}

void filterReading() {
  emaReading = (emaAlpha * reading) + ((1 - emaAlpha) * emaReading);
  lowpass = emaReading;
  highpass = reading - emaReading;
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void handleButton() {
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
      axis = (axis + 1) % 3;
    }
    stateChanged = false;
  }
  lastButtonPressed = isButtonPressed();
}

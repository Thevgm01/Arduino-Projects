#include <Arduino_LSM9DS1.h>

int buttonPin = 5;
int buttonMode = 2;
int sensor = 1; // Acceleration
int axis = 1; // Y axis
int emaButtonPressesLow = 3;
int emaButtonPressesHigh = 5;

float emaAlphaLow = 0.3f;
float emaAlphaHigh = 0.5f;
float emaReadingLow, emaReadingHigh, lowpass, highpass, bandpass;

byte lastButtonPressed = 0;
unsigned long buttonHoldTime = 0;
bool modeChanged = false;

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
  switch (sensor) {
    case 0:
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x, y, z);
        title = "MagneticField_";
      }
      break;
    case 1:
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        title = "Acceleration_";
      }
      break;
    case 2:
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        title = "Gyroscope_";
      }
      break;
  }

  float f;
  if (title != "") {
    switch (axis) {
      case 0:
        f = x;
        title += "X_";
        break;
      case 1:
        f = y;
        title += "Y_";
        break;
      case 2:
        f = z;
        title += "Z_";
        break;
    }

    filterReading(f);
    title += "emaLow=" + String(emaAlphaLow) + "_";
    title += "emaHigh=" + String(emaAlphaHigh) + "_";

    switch (buttonMode) {
      case 0:
        title += "Button=Sensor";
        break;
      case 1:
        title += "Button=Axis";
        break;
      case 2:
        title += "Button=EmaAlphaLow";
        break;
      case 3:
        title += "Button=EmaAlphaHigh";
        break;
    }
    Serial.print(title);
    Serial.println("," + String(f) + "," + String(lowpass) + "," + String(highpass)/* + "," + String(bandpass)*/);
  }

  handleButton();
}

void filterReading(float f) {
  emaReadingLow = (emaAlphaLow * f) + ((1 - emaAlphaLow) * emaReadingLow);
  emaReadingHigh = (emaAlphaHigh * f) + ((1 - emaAlphaHigh) * emaReadingHigh);
  lowpass = emaReadingLow;
  highpass = f - emaReadingHigh;
  bandpass = emaReadingHigh - emaReadingLow;
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void handleButton() {
  if (isButtonPressed()) {
    if (!lastButtonPressed) {
      buttonHoldTime = millis() + 1000;
    } else if (millis() >= buttonHoldTime) {
      buttonMode = (buttonMode + 1) % 4;
      modeChanged = true;
      buttonHoldTime = millis() + 1000;
    }
  } else if(lastButtonPressed) {
    buttonHoldTime = 0;
    if (!modeChanged) {
      switch (buttonMode) {
        case 0:
          sensor = (sensor + 1) % 3;
          break;
        case 1:
          axis = (axis + 1) % 3;
          break;
        case 2:
          emaButtonPressesLow = (emaButtonPressesLow + 1) % 11;
          emaAlphaLow = emaButtonPressesLow / 10.0f;
          break;
        case 3:
          emaButtonPressesHigh = (emaButtonPressesHigh + 1) % 11;
          emaAlphaHigh = emaButtonPressesHigh / 10.0f;
          break;
      }
    }
    modeChanged = false;
  }
  lastButtonPressed = isButtonPressed();
}

#include <Arduino_LSM9DS1.h>

int buttonPin = 5;
int buttonMode = 2;
int sensor = 1; // Acceleration
int axis = 1; // Y axis
int emaButtonPresses = 5;

float emaReading = 0.0f;
float emaAlpha = 0.5f;
float reading, lowpass, highpass;

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
    title += "_ema=" + String(emaAlpha);
    Serial.print(title + ",");

    switch (buttonMode) {
      case 0:
        Serial.print("Button=Sensor,");
        break;
      case 1:
        Serial.print("Button=Axis,");
        break;
      case 2:
        Serial.print("Button=EmaAlpha,");
        break;
    }
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
      buttonMode = (buttonMode + 1) % 3;
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
          emaButtonPresses = (emaButtonPresses + 1) % 11;
          emaAlpha = emaButtonPresses / 10.0f;
          break;
      }
    }
    modeChanged = false;
  }
  lastButtonPressed = isButtonPressed();
}

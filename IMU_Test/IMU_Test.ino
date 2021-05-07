#include <Arduino_LSM9DS1.h>

bool xAxis = true, yAxis = true, zAxis = true;
const byte accelSense = 0, gyroSense = 1, magSense = 2;

byte sensor = 1; // Acceleration

float emaAlphaLow = 0.3f;
float emaAlphaHigh = 0.5f;
bool reading = true, lowpass = true, highpass = true, bandpass = true;
const byte rIndex = 0, lIndex = 3, hIndex = 4, bIndex = 5;
float xPasses[6], yPasses[6], zPasses[6];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU.begin();

  pinMode(5, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  processSerial();

  String title = "";
  float x, y, z;
  switch (sensor) {
    case accelSense:
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        title = "Acceleration_";
      }
      break;
    case gyroSense:
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        title = "Gyroscope_";
      }
      break;
    case magSense:
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x, y, z);
        title = "MagneticField_";
      }
      break;
  }
  if (title == "") {
    return;
  }

  if (xAxis) {
    title += 'x';
    calculatePasses(x, xPasses);
  }
  if (yAxis) {
    title += 'y';
    calculatePasses(y, yPasses);
  }
  if (zAxis) {
    title += 'z';
    calculatePasses(z, zPasses);
  }
  title += '_';

  String values = "";
  if (reading) {
    title += 'r';
    if (xAxis) values += String(xPasses[rIndex]) + ',';
    if (yAxis) values += String(yPasses[rIndex]) + ',';
    if (zAxis) values += String(zPasses[rIndex]) + ',';
  }
  if (lowpass) {
    title += 'l';
    if (xAxis) values += String(xPasses[lIndex]) + ',';
    if (yAxis) values += String(yPasses[lIndex]) + ',';
    if (zAxis) values += String(zPasses[lIndex]) + ',';
  }
  if (highpass) {
    title += 'h';
    if (xAxis) values += String(xPasses[hIndex]) + ',';
    if (yAxis) values += String(yPasses[hIndex]) + ',';
    if (zAxis) values += String(zPasses[hIndex]) + ',';
  }
  if (bandpass) {
    title += 'b';
    if (xAxis) values += String(xPasses[bIndex]) + ',';
    if (yAxis) values += String(yPasses[bIndex]) + ',';
    if (zAxis) values += String(zPasses[bIndex]) + ',';
  }
  title += '_';
  title += "a1=" + String(emaAlphaLow) + '_';
  title += "a2=" + String(emaAlphaHigh);

  Serial.println(title + "," + values);
}

void calculatePasses(float f, float arr[]) {
  arr[rIndex] = f;
  arr[1] = (emaAlphaLow * f) + ((1 - emaAlphaLow) * arr[1]);
  arr[2] = (emaAlphaHigh * f) + ((1 - emaAlphaHigh) * arr[2]);
  arr[lIndex] = arr[1];
  arr[hIndex] = f - arr[2];
  arr[bIndex] = arr[4] - arr[3];
}

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'x': xAxis = !xAxis; break;
      case 'y': yAxis = !yAxis; break;
      case 'z': zAxis = !zAxis; break;
      
      case 'a': sensor = accelSense; break;
      case 'g': sensor = gyroSense; break;
      case 'm': sensor = magSense; break;
      
      case 'r': reading = !reading; break;
      case 'l': lowpass = !lowpass; break;
      case 'h': highpass = !highpass; break;
      case 'b': bandpass = !bandpass; break;
    }
    if (c == '1' || c == '2') {
      float number = 0.0f;
      int numNumbers = 0;
      char c2 = ' ';
      while (c2 != '\n') {
        if (!Serial.available()) {
          delay(1);
        }
        c2 = Serial.read();
        if (c2 == '.') { // Ignore everything before a period
          number = 0.0f;
          numNumbers = 0;
        }
        else if (c2 >= '0' && c2 <= '9') { // If a number was sent
          int n = c2 - '0';
          number += (float)n / pow(10, ++numNumbers);
        }
      }
      switch (c) {
        case '1': emaAlphaLow = number; break;
        case '2': emaAlphaHigh = number; break;
      }
    }
  }
}

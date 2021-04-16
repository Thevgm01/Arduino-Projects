// Arduino_LSM6DS3 - Version: Latest 
#include <Arduino_LSM9DS1.h>

const int ledPin = 3; // pin to use for the LED
byte state = 0;

const float tolerance = 200;
int rotationFrames = 0;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
}

void loop() {
  checkGyroscope();
  delay(100);
}

void checkGyroscope() {
  if (rotationFrames > 0) {
    --rotationFrames;
    return;
  }
  
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    if(abs(y) > tolerance) {
      rotationFrames = -1;
    } else if(rotationFrames < 0) {
      rotationFrames = 10; // wait 1 second
      state = 1 - state;
      digitalWrite(ledPin, state);
    }
  } 
}

#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <PollFunctions.h>
#include <string>

// Inner cable hole
// Radius = 10.4 mm
// Low height = 1.8 mm
// Cable width = 3.5 mm

// Bee
// Width = 100 mm
// Length = 150 mm
// 2:3
// 2*2+3*3=4+9=13

const float NORMALIZED_WIDTH = 2.0f/sqrt(13);
const float NORMALIZED_LENGTH = 3.0f/sqrt(13);

const int ANALOG_RESOLUTION = 12;
const int MAX_BRIGHTNESS = (1 << ANALOG_RESOLUTION) - 1;

enum { 
  BACK_RIGHT  = 0, 
  FRONT_RIGHT = 1, 
  FRONT_LEFT  = 2, 
  BACK_LEFT   = 3 
};

const int LED_PINS[] = { 
  D2, 
  D3, 
  D4, 
  D5 
};

const float LED_COORDS[4][2] = { 
  {  NORMALIZED_WIDTH, -NORMALIZED_LENGTH }, // BACK_RIGHT
  {  NORMALIZED_WIDTH,  NORMALIZED_LENGTH }, // FRONT_RIGHT
  { -NORMALIZED_WIDTH,  NORMALIZED_LENGTH }, // FRONT_LEFT
  { -NORMALIZED_WIDTH, -NORMALIZED_LENGTH }  // BACK_LEFT
};

const int AUDIO_PIN = A6;

void logarithmicWrite(int pin, float brightness) {
  analogWrite(pin, pow(brightness, 3)*MAX_BRIGHTNESS);
}

void writeToAllPins(int br, int fr, int fl, int bl) {
  analogWrite(LED_PINS[BACK_RIGHT],  br);
  analogWrite(LED_PINS[FRONT_RIGHT], fr);
  analogWrite(LED_PINS[FRONT_LEFT],  fl);
  analogWrite(LED_PINS[BACK_LEFT],   bl);
}

void writeToAllPins(int brightness) {
  writeToAllPins(brightness, brightness, brightness, brightness);
}

class Sensors : public Polls {
  public:
    Sensors(unsigned long updateDelay) : Polls(updateDelay) {
      if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
      }
    }
  
    bool update() override {
      if (!Polls::update()) return false;
      
      if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return false;
  
      enum { xAccel, yAccel, zAccel,
             xGyro, yGyro, zGyro };
      float readings[6];
      IMU.readAcceleration(readings[xAccel], readings[yAccel], readings[zAccel]);
      IMU.readGyroscope(readings[xGyro], readings[yGyro], readings[zGyro]);
  
      //Serial.println("X:" + String(readings[xAccel]) + ",Y:" + String(readings[yAccel]) + ",Z:" + String(readings[zAccel]));
  
      xAccelLowpass = xAccelLowpass * accelLowpassEma + readings[xAccel] * (1 - accelLowpassEma);
      yAccelLowpass = yAccelLowpass * accelLowpassEma + readings[yAccel] * (1 - accelLowpassEma);
  
      float len = sqrt(xAccelLowpass * xAccelLowpass + yAccelLowpass * yAccelLowpass);
      float normalized[] = { -yAccelLowpass / len, -xAccelLowpass / len };
  
      for (int i = 0; i < 4; ++i) {
        float dot = LED_COORDS[i][0] * normalized[0] + LED_COORDS[i][1] * normalized[1];
        logarithmicWrite(LED_PINS[i], dot * len);
      }
    }
  
  private:
    float xAccelLowpass = 0.0f;
    float yAccelLowpass = 0.0f;
    float accelLowpassEma = 0.8f;
};

Sensors* sensors;

void setup() {
  analogWriteResolution(12);
  analogReadResolution(12);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PWR, LOW);

  randomSeed(analogRead(A5));

  sensors = new Sensors(16);
}

void loop() {
    Polls::setMillis(millis());

    sensors->update();
    Serial.println("Volume:" + String(analogRead(AUDIO_PIN)));

    delay(1);
}

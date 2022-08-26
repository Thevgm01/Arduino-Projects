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

const float NORMALIZED_WIDTH = 1.0f/sqrt(2);//2.0f/sqrt(13);
const float NORMALIZED_LENGTH = 1.0f/sqrt(2);//3.0f/sqrt(13);

const int ANALOG_RESOLUTION = 12;
const int ANALOG_MAX = (1 << ANALOG_RESOLUTION) - 1;

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
  analogWrite(pin, pow(brightness, 3) * ANALOG_MAX);
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
  private:
    float xAccelLowpass = 0.0f;
    float xAccelBandpass = 0.0f;
    float yAccelLowpass = 0.0f;
    float yAccelBandpass = 0.0f;
    float zAccelLowpass = 0.0f;
    float zAccelBandpass = 0.0f;
    
    float accelLowpassEma = 0.85f;
    float accelBandpassEma = 0.85f;

    enum { xAccel, yAccel, zAccel,
           xGyro, yGyro, zGyro };
           
    float readings[6];
    int readingCount = 0;
    
  public:
    Sensors(unsigned long updateDelay) : Polls(updateDelay) {
      if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
      }
    }

    void read() {
      if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

      float newReadings[6];
      IMU.readAcceleration(newReadings[xAccel], newReadings[yAccel], newReadings[zAccel]);
      IMU.readGyroscope(newReadings[xGyro], newReadings[yGyro], newReadings[zGyro]);
      for (int i = 0; i < 6; ++i) readings[i] += newReadings[i];

      ++readingCount;
    }

    bool update() override {      
      if (!Polls::update()) return false;

      if (readingCount == 0) return false;

      for (int i = 0; i < 6; ++i) readings[i] /= readingCount;

      //Serial.println("X:" + String(readings[xAccel]) + ",Y:" + String(readings[yAccel]) + ",Z:" + String(readings[zAccel]));
  
      xAccelLowpass = xAccelLowpass * accelLowpassEma + readings[xAccel] * (1 - accelLowpassEma);
      float xAccelHighpass = readings[xAccel] - xAccelLowpass;
      xAccelBandpass = xAccelBandpass * accelBandpassEma + xAccelHighpass * (1 - accelBandpassEma);

      yAccelLowpass = yAccelLowpass * accelLowpassEma + readings[yAccel] * (1 - accelLowpassEma);
      float yAccelHighpass = readings[yAccel] - yAccelLowpass;
      yAccelBandpass = yAccelBandpass * accelBandpassEma + yAccelHighpass * (1 - accelBandpassEma);

      zAccelLowpass = zAccelLowpass * accelLowpassEma + readings[zAccel] * (1 - accelLowpassEma);
      float zAccelHighpass = readings[zAccel] - zAccelLowpass;
      zAccelBandpass = zAccelBandpass * accelBandpassEma + abs(zAccelHighpass) * (1 - accelBandpassEma);

      float absHighpassSum = abs(xAccelHighpass) + abs(yAccelHighpass) + abs(zAccelHighpass);
      
      /*
      Serial.println(
         "Lowpass:"+String(zAccelLowpass)+
        ",Highpass:"+String(zAccelHighpass)+
        ",Bandpass:"+String(zAccelBandpass)+
        ",HighpassSum:"+String(absHighpassSum));
      */
      Serial.print("accelerometer:" + String(25*absHighpassSum));
      
      float len = sqrt(xAccelBandpass * xAccelBandpass + yAccelBandpass * yAccelBandpass + zAccelBandpass * zAccelBandpass);
      float normalized[] = { -yAccelBandpass / len, -xAccelBandpass / len, zAccelBandpass / len };

      // Light up according to accelerometer data
      for (int i = 0; i < 4; ++i) {
        //          X                                  Y                                  Z             Divide because all 4 lights are on together
        float dot = LED_COORDS[i][0] * normalized[0] + LED_COORDS[i][1] * normalized[1] + normalized[2] / 3;
        logarithmicWrite(LED_PINS[i], dot * len);
      }

      // Clear all accumulated readings
      for (int i = 0; i < 6; ++i) readings[i] = 0.0f;
      readingCount = 0;

      return true;
    }
};

class Sounds : public Polls {
  private:
    float reading = 0.0f;
    int readingCount = 0;
  
    float soundLowpass = 0.0f;
    
    float soundLowpassEma = 0.85f;
    
  public:
    Sounds(unsigned long updateDelay) : Polls(updateDelay) {

    }
  
    void read() {
      reading += abs(analogRead(AUDIO_PIN)) / ANALOG_MAX;
      ++readingCount;
    }

    bool update() override {
      //if (!Polls::update()) return false;

      if (readingCount == 0) return false;

      reading /= readingCount;
      
      //Serial.println("Volume:" + String(analogRead(AUDIO_PIN)));
    
      soundLowpass = soundLowpass * soundLowpassEma + reading * (1 - soundLowpassEma);
      float soundHighpass = abs(reading - soundLowpass);

      Serial.println(",sound:" + String(25*soundHighpass));

      reading = 0.0f;
      readingCount = 0;

      return true;
    }
};

Sensors* sensors;
Sounds* sounds;

void setup() {
  analogWriteResolution(12);
  analogReadResolution(12);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PWR, LOW);

  randomSeed(analogRead(A5));

  sensors = new Sensors(16);
  sounds = new Sounds(16);
}

void loop() {
    Polls::setMillis(millis());

    sensors->read();
    sounds->read();

    if (sensors->update()) {
      sounds->update();
    }
}

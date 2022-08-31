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

template<typename T>
int sign(T val) { 
  return (T(0) < val) - (val < T(0));
}

float map(float value, float istart, float istop, float ostart, float ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

float random(float min, float max) {
  return (random(2000000000) / 2000000000.0f) * (max - min) + min;
}

static constexpr float NORMALIZED_WIDTH = 1.0f/sqrt(2);//2.0f/sqrt(13);
static constexpr float NORMALIZED_LENGTH = 1.0f/sqrt(2);//3.0f/sqrt(13);

static constexpr int ANALOG_RESOLUTION = 12;
static constexpr int ANALOG_MAX = (1 << ANALOG_RESOLUTION) - 1;

static constexpr float
  DEBUG_ACCEL_SCALE = 25.0f,
  DEBUG_GYRO_SCALE  = 1.5f,
  DEBUG_SOUND_SCALE = 10.0f;

class Lights : public Polls {
  private:
    int maxBrightness = ANALOG_MAX;

    const byte LED_PINS[4] = { 
      D2, // BACK_RIGHT
      D3, // FRONT_RIGHT
      D4, // FRONT_LEFT
      D5  // BACK_LEFT
    };
  
    void logarithmicWrite(int pin, float brightness) {
      analogWrite(LED_PINS[pin], pow(brightness, 3) * maxBrightness);
    }

    bool toggleState = false;
    float brightnessModifiers[4];

    class LightPattern {
      public:
        float speedMult = 1.0f;
        virtual void update(float in[4]) = 0;
    };
    class Static : public LightPattern {
      public:
        void update(float in[4]) override {
          for (int i = 0; i < 4; ++i) {
            in[i] = 1.0f;
          }
        }
    };
    class Lighthouse : public LightPattern {
      private:
        const float speed = 0.03f;
        float position;
        float direction = 1;

      public:
        Lighthouse(float direction) : direction(direction) {}
      
        void update(float in[4]) override {
          position += speed * speedMult;
          if (position >= TWO_PI) position -= TWO_PI;
          for (int i = 0; i < 4; ++i) {
            in[i] = map(cos(position * direction + i*PI/2), -1, 1, 0.2f, 1.0f);
          }
        }
    };
    class Breathe : public LightPattern {
      private:
        const float speed = 0.02f;
        float position = 0.0f;

      public:
        void update(float in[4]) override {
          position += speed * speedMult;
          if (position >= TWO_PI) position -= TWO_PI;

          for (int i = 0; i < 4; ++i) {
            in[i] = map(cos(position), -1, 1, 0.6f, 1.0f);
          }
        }
    };
    class Flicker : public LightPattern {
      private:
        const float speed = 0.04f ;
        float values[4];

      public:
        void update(float in[4]) override {
          for (int i = 0; i < 4; ++i) {
            values[i] += random(-1.0f, 1.0f) * speed;
            values[i] = constrain(values[i], 0.6f, 1.0f);
            in[i] = values[i];
          }
        }
    };
    class Wave : public LightPattern {
      private:
        const float speed = 0.03f;
        float position = 0.0f;
        const byte offset = 1;

      public:
        Wave(byte offset) : offset(offset) {}
      
        void update(float in[4]) override {
          position += speed * speedMult;
          if (position >= TWO_PI) position -= TWO_PI;
          for (int i = 0; i < 4; ++i) {
            in[(i + offset) % 4] = map(cos(position + floor(i/2.0f) * PI), -1, 1, 0.4f, 1.0f);
          }
        }
    };
    class Pulse : public LightPattern {
      private:
        const float speed = 0.04f ;
        float positions[2] = { 0.0f, PI/3.0f };
        const byte offset = 1;

      public:
        Pulse(byte offset) : offset(offset) {}
      
        void update(float in[4]) override {
          for (int i = 0; i < 2; ++i) {
            positions[i] += speed * speedMult;
            if (positions[i] >= PI * 1.5f) positions[i] -= PI * 1.5f;
          }
          for (int i = 0; i < 4; ++i) {
            in[(i + offset) % 4] = max(sin(positions[i/2]), 0) * 0.3f + 0.7f;
          }
        }
    };
    
    LightPattern* pattern;

  public:

    const float LED_COORDS[4][2] = { 
      {  NORMALIZED_WIDTH, -NORMALIZED_LENGTH }, // BACK_RIGHT
      {  NORMALIZED_WIDTH,  NORMALIZED_LENGTH }, // FRONT_RIGHT
      { -NORMALIZED_WIDTH,  NORMALIZED_LENGTH }, // FRONT_LEFT
      { -NORMALIZED_WIDTH, -NORMALIZED_LENGTH }  // BACK_LEFT
    };

    float getMaxBrightness() { return maxBrightness; }

    void setMaxBrightness(int max) {
      maxBrightness = max;
    }

    bool getToggleState() { return toggleState; }

    void toggle() {
      toggleState = !toggleState;
    }

    void setPattern(byte num) {
      delete pattern;
      switch (num) {
      //case 0:  pattern = new Static();       break; // Static (handled by default)
        case 1:  pattern = new Breathe();      break; // Breathe
        case 2:  pattern = new Flicker();      break; // Flicker
        case 3:  pattern = new Wave(1);        break; // Wave (forward/backward)
        case 4:  pattern = new Wave(0);        break; // Wave (left/right)
        case 5:  pattern = new Pulse(1);       break; // Pulse (back to front)
        case 6:  pattern = new Pulse(3);       break; // Pulse (front to back)
        case 7:  pattern = new Lighthouse(1);  break; // Lighthouse (clockwise)
        case 8:  pattern = new Lighthouse(-1); break; // Lighthouse (counter-clockwise)
        default: pattern = new Static();       break; // Static
      }
    }

    void setSpeed(byte speed) {
      pattern->speedMult = speed / 32.0f;
    }

    void setAccelModifier(int index, float value) {
      brightnessModifiers[index] = value;
    }

    Lights(unsigned long updateDelay) : Polls(updateDelay) {
      pattern = new Static();
    }

    bool update() override {      
      if (!Polls::update()) return false;

      float brightness[4];
      if (toggleState) {
        pattern->update(brightness);
      }

      for (int i = 0; i < 4; ++i) {
        logarithmicWrite(i, brightness[i] + brightnessModifiers[i]);
      }

      return true;
    }
};

Lights* lights;

class Sensors : public Polls {
  // If the highpass detects that all 3 sensors have hit their bonk threshold, toggle the light
  // Then, wait until all three bandpasses have returned to under a different threshold for a period of time before allowing another bonk
  
  private:
    static constexpr byte AUDIO_PIN = A6;
  
    static constexpr float 
      accelThresholdHigh = 1.5f / DEBUG_ACCEL_SCALE, accelThresholdLow = 0.5f / DEBUG_ACCEL_SCALE,
      gyroThresholdHigh  = 1.5f / DEBUG_GYRO_SCALE,  gyroThresholdLow  = 1.0f / DEBUG_GYRO_SCALE,
      soundThresholdHigh = 0.8f / DEBUG_SOUND_SCALE, soundThresholdLow = 0.2f / DEBUG_SOUND_SCALE;

    bool readyForBonk = false;
    int accelCounter = 0;
    int gyroCounter = 0;
    int soundCounter = 0;

    static constexpr float 
      bonkGracePeriod = 0.15f, // Seconds
      bonkSettlePeriod = 0.1f;

    const int
      bonkGraceLoops,
      bonkSettleLoops;
  
    static constexpr float 
      accelLowpassEma = 0.85f, accelBandpassEma = 0.85f,
      gyroLowpassEma  = 0.85f, gyroBandpassEma  = 0.85f,
      soundLowpassEma = 0.85f, soundBandpassEma = 0.85f;

    int readingCount = 0;
    int soundReadingCount = 0;

    static constexpr byte
      xAccel = 0, yAccel = 1, zAccel = 2, 
      xGyro = 3, yGyro = 4, zGyro = 5, 
      sound = 6,

      accelAbsSum = 7,
      gyroAbsSum = 8,

      reading  = 0 << 4, // 00_0000
      lowpass  = 1 << 4, // 01_0000
      highpass = 2 << 4, // 10_0000
      bandpass = 3 << 4; // 11_0000

    static constexpr byte largestNumber = gyroAbsSum|bandpass;
    float data[largestNumber];

    void runLowpass(float &lowpass, const float reading, const float ema) { lowpass = lowpass * ema + reading * (1 - ema); }
    void runHighpass(float &highpass, const float lowpass, const float reading) { highpass = reading - lowpass; }
    void runHighpassAbs(float &highpass, const float lowpass, const float reading) { highpass = abs(reading - lowpass); }
    void runBandpass(float &bandpass, const float highpass, const float ema) { runLowpass(bandpass, highpass, ema); }

    void runFullpass(const float reading, float &lowpass, float &highpass, float &bandpass, const float lowpassEma, const float bandpassEma) {
      runLowpass(lowpass, reading, lowpassEma);
      runHighpass(highpass, lowpass, reading);
      runBandpass(bandpass, highpass, bandpassEma);
    }

    void runFullpassAbs(const float reading, float &lowpass, float &highpass, float &bandpass, const float lowpassEma, const float bandpassEma) {
      runLowpass(lowpass, abs(reading), lowpassEma);
      runHighpassAbs(highpass, lowpass, reading);
      runBandpass(bandpass, highpass, bandpassEma);
    }

    void runFullpass(const int type, const float lowpassEma, const float bandpassEma) {
      runFullpass(data[type|reading], data[type|lowpass], data[type|highpass], data[type|bandpass], lowpassEma, bandpassEma);
    }

    void runFullpassAbs(const int type, const float lowpassEma, const float bandpassEma) {
      runFullpassAbs(abs(data[type|reading]), data[type|lowpass], data[type|highpass], data[type|bandpass], lowpassEma, bandpassEma);
    }
    
  public:
    Sensors(unsigned long updateDelay) : 
        Polls(updateDelay), 
        bonkGraceLoops(ceil(bonkGracePeriod * 1000.0f / (float)updateDelay)),
        bonkSettleLoops(ceil(bonkSettlePeriod * 1000.0f / (float)updateDelay)) {
          
      if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        return;
      }
      
      pinMode(AUDIO_PIN, INPUT);
    }

    void read() {
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        float newReadings[6];
        IMU.readAcceleration(newReadings[xAccel], newReadings[yAccel], newReadings[zAccel]);
        IMU.readGyroscope(newReadings[xGyro], newReadings[yGyro], newReadings[zGyro]);
        for (int i = xAccel; i <= zGyro; ++i) {
          data[i|reading] += newReadings[i];
        }
        ++readingCount;
      }
      
      data[sound|reading] += (float)analogRead(AUDIO_PIN) / ANALOG_MAX;
      ++soundReadingCount;
    }

    bool update() override {      
      if (!Polls::update()) return false;

      if (readingCount == 0 || soundReadingCount == 0) return false;

      for (int i = xAccel; i <= zGyro; ++i) {
        data[i|reading] /= readingCount;
      }
      data[sound|reading] /= soundReadingCount;

      //Serial.println("X:" + String(readings[xAccel]) + ",Y:" + String(readings[yAccel]) + ",Z:" + String(readings[zAccel]));

      runFullpass(xAccel, accelLowpassEma, accelBandpassEma);
      runFullpass(yAccel, accelLowpassEma, accelBandpassEma);
      runFullpass(zAccel, accelLowpassEma, accelBandpassEma);
      
      data[accelAbsSum|lowpass]  = abs(data[xAccel|lowpass])  + abs(data[yAccel|lowpass])  + abs(data[zAccel|lowpass]);
      data[accelAbsSum|highpass] = abs(data[xAccel|highpass]) + abs(data[yAccel|highpass]) + abs(data[zAccel|highpass]);
      data[accelAbsSum|bandpass] = abs(data[xAccel|bandpass]) + abs(data[yAccel|bandpass]) + abs(data[zAccel|bandpass]);

      runFullpass(xGyro, gyroLowpassEma, gyroBandpassEma);
      runFullpass(yGyro, gyroLowpassEma, gyroBandpassEma);
      runFullpass(zGyro, gyroLowpassEma, gyroBandpassEma);

      data[gyroAbsSum|lowpass]  = abs(data[xGyro|lowpass])  + abs(data[yGyro|lowpass])  + abs(data[zGyro|lowpass]);
      data[gyroAbsSum|highpass] = abs(data[xGyro|highpass]) + abs(data[yGyro|highpass]) + abs(data[zGyro|highpass]);
      data[gyroAbsSum|bandpass] = abs(data[xGyro|bandpass]) + abs(data[yGyro|bandpass]) + abs(data[zGyro|bandpass]);

      runFullpassAbs(sound, soundLowpassEma, soundBandpassEma);

      if (false) {
        Serial.print("accelerometer:" + String(DEBUG_ACCEL_SCALE*data[accelAbsSum|highpass]));
        Serial.print(",gyroscope:" + String(DEBUG_GYRO_SCALE*data[gyroAbsSum|highpass]));
        Serial.println(",sound:" + String(DEBUG_SOUND_SCALE*data[sound|highpass]));
      }

      if (readyForBonk) {
        if (data[accelAbsSum|highpass] >= accelThresholdHigh) accelCounter += 2;
        if (data[gyroAbsSum|highpass]  >= gyroThresholdHigh)  gyroCounter  += 2;
        if (data[sound|highpass]       >= soundThresholdHigh) soundCounter += 2;
  
        if (accelCounter > 0 && gyroCounter > 0 && soundCounter > 0) {
          Serial.println("Bonk!");
          lights->toggle();
          readyForBonk = false;
        }
        else if (accelCounter > bonkGraceLoops || gyroCounter > bonkGraceLoops || soundCounter > bonkGraceLoops) {
          Serial.println("Held");
          readyForBonk = false;
        }
      }
      if (!readyForBonk) {
        if (data[accelAbsSum|bandpass] >= accelThresholdLow) accelCounter = bonkSettleLoops;
        if (data[gyroAbsSum|bandpass]  >= gyroThresholdLow)  gyroCounter  = bonkSettleLoops;
        if (data[sound|bandpass]       >= soundThresholdLow) soundCounter = bonkSettleLoops;

        if (accelCounter == 0 && gyroCounter == 0 && soundCounter == 0) {
          Serial.println("Ready for bonk");
          readyForBonk = true;
        }
      }
      accelCounter = max(accelCounter - 1, 0);
      gyroCounter  = max(gyroCounter  - 1, 0);
      soundCounter = max(soundCounter - 1, 0);

      //Serial.println("accel:"+String(accelCounter)+",gyro:"+String(gyroCounter)+",sound:"+String(soundCounter));

      float len = sqrt(data[xAccel|bandpass] * data[xAccel|bandpass] + data[yAccel|bandpass] * data[yAccel|bandpass] + data[zAccel|bandpass] * data[zAccel|bandpass]);
      float normalized[] = { -data[yAccel|bandpass] / len, -data[xAccel|bandpass] / len, data[zAccel|bandpass] / len };

      // Light up according to accelerometer data
      for (int i = 0; i < 4; ++i) {
        //          X                                          Y                                          Z
        float dot = lights->LED_COORDS[i][0] * normalized[0] + lights->LED_COORDS[i][1] * normalized[1] + abs(normalized[2]) / 1.5f;
        lights->setAccelModifier(i, (dot * len * 2 - len) * (lights->getToggleState() ? 4 : 2));
      }

      // Clear all accumulated readings
      for (int i = xAccel; i <= sound; ++i) data[i|reading] = 0.0f;
      readingCount = 0;
      soundReadingCount = 0;

      return true;
    }
};

Sensors* sensors;

BLEService LEDService("7b736c0d-6f4b-41f1-85c4-32cc78edca05");

BLEByteCharacteristic toggleCharacteristic("1f00", BLEWrite);
BLEIntCharacteristic maxBrightnessCharacteristic("1f01", BLEWrite);
BLEByteCharacteristic patternCharacteristic("1f02", BLEWrite);
BLEByteCharacteristic patternSpeedCharacteristic("1f03", BLEWrite);

class Bluetooth : public Polls {
  public:
    Bluetooth(unsigned long updateDelay) : Polls(updateDelay) {
      if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        return;
      }
    
      // Set advertised local name and service UUID:
      BLE.setLocalName("Bee-LED-BLE");
      BLE.setAdvertisedService(LEDService);
    
      // Add the characteristics to the service
      LEDService.addCharacteristic(toggleCharacteristic);
      LEDService.addCharacteristic(maxBrightnessCharacteristic);
      LEDService.addCharacteristic(patternCharacteristic);
      LEDService.addCharacteristic(patternSpeedCharacteristic);
    
      // Add service
      BLE.addService(LEDService);
    
      // Set the initial value for the characeristic:
      //toggleCharacteristic.writeValue(0);
      //timerCharacteristic.writeValue(0);
      //morseCharacteristic.writeValue("");
    
      // Start advertising
      BLE.advertise();
  
      //Serial.println("Bluetooth device active, waiting for connections...");
    }
    
    bool update() override {
      if (!Polls::update()) return false;
      
      // Poll for BLE events
      BLE.poll();
    
      if (toggleCharacteristic.written()) {
        lights->toggle();
      }
    
      if (maxBrightnessCharacteristic.written()) {
        lights->setMaxBrightness(maxBrightnessCharacteristic.value());
        if (!lights->getToggleState()) {
          lights->toggle();
        }
      }
    
      if (patternCharacteristic.written()) {
        lights->setPattern(patternCharacteristic.value());
      }
    
      if (patternSpeedCharacteristic.written()) {
        lights->setSpeed(patternSpeedCharacteristic.value());
      }
    }
};

Bluetooth* bluetooth;

void setup() {
  analogWriteResolution(12);
  analogReadResolution(12);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PWR, LOW);

  randomSeed(analogRead(A5));

  sensors = new Sensors(16);
  lights = new Lights(10);
  bluetooth = new Bluetooth(200);
}

void loop() {
    Polls::setMillis(millis());

    sensors->read();

    sensors->update();
    lights->update();
    bluetooth->update();
}

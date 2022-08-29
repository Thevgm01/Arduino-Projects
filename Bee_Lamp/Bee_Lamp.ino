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
int sign(T val) { return (T(0) < val) - (val < T(0)); }

static constexpr float NORMALIZED_WIDTH = 1.0f/sqrt(2);//2.0f/sqrt(13);
static constexpr float NORMALIZED_LENGTH = 1.0f/sqrt(2);//3.0f/sqrt(13);

static constexpr int ANALOG_RESOLUTION = 12;
static constexpr int ANALOG_MAX = (1 << ANALOG_RESOLUTION) - 1;

static constexpr float
  DEBUG_ACCEL_SCALE = 25.0f,
  DEBUG_GYRO_SCALE  = 1.5f,
  DEBUG_SOUND_SCALE = 10.0f;
            
enum { 
  BACK_RIGHT  = 0, 
  FRONT_RIGHT = 1, 
  FRONT_LEFT  = 2, 
  BACK_LEFT   = 3 
};

static constexpr byte LED_PINS[] = { 
  D2, 
  D3, 
  D4, 
  D5 
};

static constexpr byte AUDIO_PIN = A6;

static constexpr float LED_COORDS[4][2] = { 
  {  NORMALIZED_WIDTH, -NORMALIZED_LENGTH }, // BACK_RIGHT
  {  NORMALIZED_WIDTH,  NORMALIZED_LENGTH }, // FRONT_RIGHT
  { -NORMALIZED_WIDTH,  NORMALIZED_LENGTH }, // FRONT_LEFT
  { -NORMALIZED_WIDTH, -NORMALIZED_LENGTH }  // BACK_LEFT
};

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
  // If the highpass detects that all 3 sensors have hit their bonk threshold, toggle the light
  // Then, wait until all three bandpasses have returned to under a different threshold for a period of time before allowing another bonk
  
  private:
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
        //          X                                  Y                                  Z
        float dot = LED_COORDS[i][0] * normalized[0] + LED_COORDS[i][1] * normalized[1] + abs(normalized[2]) / 2.0f;
        logarithmicWrite(LED_PINS[i], dot * len);
      }

      // Clear all accumulated readings
      for (int i = xAccel; i <= sound; ++i) data[i|reading] = 0.0f;
      readingCount = 0;
      soundReadingCount = 0;

      return true;
    }
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

    sensors->read();

    sensors->update();
}

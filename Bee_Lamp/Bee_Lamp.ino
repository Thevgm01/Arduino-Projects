#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <PollFunctions.h>
#include <string>
#include <unordered_map>

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

const float NORMALIZED_WIDTH = 1.0f/sqrt(2);//2.0f/sqrt(13);
const float NORMALIZED_LENGTH = 1.0f/sqrt(2);//3.0f/sqrt(13);

const int ANALOG_RESOLUTION = 12;
const int ANALOG_MAX = (1 << ANALOG_RESOLUTION) - 1;

const float DEBUG_ACCEL_SCALE = 25.0f,
            DEBUG_GYRO_SCALE  = 1.5f,
            DEBUG_SOUND_SCALE = 10.0f;
            
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
  // If the highpass detects that all 3 sensors have hit their bonk threshold, toggle the light
  // Then, wait until all three bandpasses have returned to under a different threshold for a period of time before allowing another bonk
  
  private:
    const float 
      accelThresholdHigh = 1.5f / DEBUG_ACCEL_SCALE, accelThresholdLow = 0.5f / DEBUG_ACCEL_SCALE,
      gyroThresholdHigh  = 1.5f / DEBUG_GYRO_SCALE,  gyroThresholdLow  = 1.0f / DEBUG_GYRO_SCALE,
      soundThresholdHigh = 0.8f / DEBUG_SOUND_SCALE, soundThresholdLow = 0.2f / DEBUG_SOUND_SCALE;

    bool readyForBonk = false;
    int accelCounter = 0;
    int gyroCounter = 0;
    int soundCounter = 0;

    const float 
      bonkGracePeriod = 0.15f, // Seconds
      bonkSettlePeriod = 0.1f;

    const int bonkGraceLoops, bonkSettleLoops;
  
    float accelLowpassEma  = 0.85f, accelBandpassEma = 0.85f,
          gyroLowpassEma   = 0.85f, gyroBandpassEma  = 0.85f,
          soundLowpassEma  = 0.85f, soundBandpassEma = 0.85f;

    int readingCount = 0;
    int soundReadingCount = 0;

    enum { 
      xAccel = 0, yAccel = 1, zAccel = 2, 
      xGyro = 3, yGyro = 4, zGyro = 5, 
      sound = 6,

      accelAbsSum = 7,
      gyroAbsSum = 8,

      reading  = 0 << 6,
      lowpass  = 1 << 6,
      highpass = 2 << 6,
      bandpass = 3 << 6 };

    std::unordered_map<int, float> map;

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
      runFullpass(map[type|reading], map[type|lowpass], map[type|highpass], map[type|bandpass], lowpassEma, bandpassEma);
    }

    void runFullpassAbs(const int type, const float lowpassEma, const float bandpassEma) {
      runFullpassAbs(abs(map[type|reading]), map[type|lowpass], map[type|highpass], map[type|bandpass], lowpassEma, bandpassEma);
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
          map[i|reading] += newReadings[i];
        }
        ++readingCount;
      }
      
      map[sound|reading] += (float)analogRead(AUDIO_PIN) / ANALOG_MAX;
      ++soundReadingCount;
    }

    bool update() override {      
      if (!Polls::update()) return false;

      if (readingCount == 0 || soundReadingCount == 0) return false;

      for (int i = xAccel; i <= zGyro; ++i) {
        map[i|reading] /= readingCount;
      }
      map[sound|reading] /= soundReadingCount;

      //Serial.println("X:" + String(readings[xAccel]) + ",Y:" + String(readings[yAccel]) + ",Z:" + String(readings[zAccel]));

      runFullpass(xAccel, accelLowpassEma, accelBandpassEma);
      runFullpass(yAccel, accelLowpassEma, accelBandpassEma);
      runFullpass(zAccel, accelLowpassEma, accelBandpassEma);
      
      map[accelAbsSum|lowpass]  = abs(map[xAccel|lowpass])  + abs(map[yAccel|lowpass])  + abs(map[zAccel|lowpass]);
      map[accelAbsSum|highpass] = abs(map[xAccel|highpass]) + abs(map[yAccel|highpass]) + abs(map[zAccel|highpass]);
      map[accelAbsSum|bandpass] = abs(map[xAccel|bandpass]) + abs(map[yAccel|bandpass]) + abs(map[zAccel|bandpass]);

      runFullpass(xGyro, gyroLowpassEma, gyroBandpassEma);
      runFullpass(yGyro, gyroLowpassEma, gyroBandpassEma);
      runFullpass(zGyro, gyroLowpassEma, gyroBandpassEma);

      map[gyroAbsSum|lowpass]  = abs(map[xGyro|lowpass])  + abs(map[yGyro|lowpass])  + abs(map[zGyro|lowpass]);
      map[gyroAbsSum|highpass] = abs(map[xGyro|highpass]) + abs(map[yGyro|highpass]) + abs(map[zGyro|highpass]);
      map[gyroAbsSum|bandpass] = abs(map[xGyro|bandpass]) + abs(map[yGyro|bandpass]) + abs(map[zGyro|bandpass]);

      runFullpassAbs(sound, soundLowpassEma, soundBandpassEma);

      if (false) {
        Serial.print("accelerometer:" + String(DEBUG_ACCEL_SCALE*map[accelAbsSum|highpass]));
        Serial.print(",gyroscope:" + String(DEBUG_GYRO_SCALE*map[gyroAbsSum|highpass]));
        Serial.println(",sound:" + String(DEBUG_SOUND_SCALE*map[sound|highpass]));
      }

      if (readyForBonk) {
        if (map[accelAbsSum|highpass] >= accelThresholdHigh) accelCounter += 2;
        if (map[gyroAbsSum|highpass]  >= gyroThresholdHigh)  gyroCounter  += 2;
        if (map[sound|highpass]       >= soundThresholdHigh) soundCounter += 2;
  
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
        if (map[accelAbsSum|bandpass] >= accelThresholdLow) accelCounter = bonkSettleLoops;
        if (map[gyroAbsSum|bandpass]  >= gyroThresholdLow)  gyroCounter  = bonkSettleLoops;
        if (map[sound|bandpass]       >= soundThresholdLow) soundCounter = bonkSettleLoops;

        if (accelCounter == 0 && gyroCounter == 0 && soundCounter == 0) {
          Serial.println("Ready for bonk");
          readyForBonk = true;
        }
      }

      accelCounter = max(accelCounter - 1, 0);
      gyroCounter  = max(gyroCounter  - 1, 0);
      soundCounter = max(soundCounter - 1, 0);

      //Serial.println("accel:"+String(accelCounter)+",gyro:"+String(gyroCounter)+",sound:"+String(soundCounter));

      float len = sqrt(map[xAccel|bandpass] * map[xAccel|bandpass] + map[yAccel|bandpass] * map[yAccel|bandpass] + map[zAccel|bandpass] * map[zAccel|bandpass]);
      float normalized[] = { -map[yAccel|bandpass] / len, -map[xAccel|bandpass] / len, map[zAccel|bandpass] / len };

      // Light up according to accelerometer data
      for (int i = 0; i < 4; ++i) {
        //          X                                  Y                                  Z
        float dot = LED_COORDS[i][0] * normalized[0] + LED_COORDS[i][1] * normalized[1] + abs(normalized[2]) / 2.0f;
        logarithmicWrite(LED_PINS[i], dot * len);
      }

      // Clear all accumulated readings
      for (int i = xAccel; i <= sound; ++i) map[i|reading] = 0.0f;
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

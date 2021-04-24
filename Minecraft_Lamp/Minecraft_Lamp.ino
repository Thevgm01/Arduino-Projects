#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

BLEByteCharacteristic toggleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLEUnsignedIntCharacteristic timerCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLENotify | BLEWrite);
BLEStringCharacteristic morseCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite, 256);
BLEByteCharacteristic gameCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = 3;
int ledState = 0;
const int confirmationDelay = 100;

// Button presses
const int buttonPin = 5;
byte lastButtonState = 1;
int buttonHoldLoops = 0;
int buttonHoldFlash = 0;
const int delayBetweenHoldFlashes = 500;

// Bonking
const int maxBonkLoops = 10;
const float bonkTolerance = 2.0f;
int bonkLoops = 0;
int bonkFreeze = 0;
float lastGyroscopeX, lastGyroscopeY, lastGyroscopeZ;

// Tilting
const float tiltThreshold = 0.05f;
bool tilting = false;
bool stableAfterTilt = false;
bool stableAfterTiming = false;
const int tiltHistoryLength = 8;
int tiltHistoryIndex = 0;
float yAccelerationHistory[tiltHistoryLength];

// Timing
const unsigned long defaultTimerMillis = 15 * 60 * 1000;
unsigned long timerMillis = defaultTimerMillis;
unsigned long timer = 0;

// Morse code
const int morseDelay = 100;
const String morseTable[] = 
//A     B       C       D      E    F       G      H       I     J       K      L       M
{"01", "1000", "1010", "100", "0", "0010", "110", "0000", "00", "0111", "101", "0100", "11",
//N     O      P       Q       R      S      T    U      V       W      X       Y       Z
 "10", "111", "0110", "1101", "010", "000", "1", "001", "0001", "011", "1001", "1011", "1100",
//0,       1        2        3        4        5        6        7        8        9
 "11111", "01111", "00111", "00011", "00001", "00000", "10000", "11000", "11100", "11110" };

// Game
// Wait a random amount of time from minDelay to maxDelay
// Turn on the light for the specified reaction time
// If the player bonks while the light is on, add a point and reduce the next reaction time, go to step 1
// If the player bonks while the light is off, game over
const unsigned long minDelay = 500;
const unsigned long maxDelay = 4000;
const unsigned long startReactionTime = 1000;
const unsigned long minReactionTime = 100;
unsigned long reactionTime = 0;
unsigned long offTime = 0;
byte points = 0;

// Polling
const unsigned long buttonPollInterval = 50;     // 20 times per second
const unsigned long IMUPollInterval = 33;        // 30 times per second
const unsigned long timerPollInterval = 1000;    // 1 time per second
const unsigned long bluetoothPollInterval = 200; // 5 times per second
unsigned long buttonPollStart = 7;
unsigned long IMUPollStart = 13;
unsigned long timerPollStart = 17;
unsigned long bluetoothPollStart = 19;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  //pinMode(ledPin, OUTPUT); // This gets called when the LED is toggled
  pinMode(buttonPin, INPUT_PULLUP);

  // Bluetooth initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    errorLoop();
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Minecraft Lamp");
  BLE.setAdvertisedService(ledService);

  // add the characteristics to the service
  ledService.addCharacteristic(toggleCharacteristic);
  ledService.addCharacteristic(timerCharacteristic);
  ledService.addCharacteristic(morseCharacteristic);
  ledService.addCharacteristic(gameCharacteristic);

  // Add service
  BLE.addService(ledService);

  // Set the initial value for the characeristic:
  //toggleCharacteristic.writeValue(0);
  //timerCharacteristic.writeValue(0);
  //morseCharacteristic.writeValue("");

  // Start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  // IMU initialization
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    errorLoop();
  }
}

void errorLoop() {
  while (1) {
    confirmationFlash(0);
    delay(delayBetweenHoldFlashes);
  }
}

void loop() {
  unsigned long curMillis = millis();
  awaitPoll(bluetoothPollStart, bluetoothPollInterval, curMillis, &checkBluetooth);
  awaitPoll(buttonPollStart,    buttonPollInterval,    curMillis, &checkButton);
  awaitPoll(IMUPollStart,       IMUPollInterval,       curMillis, &checkIMU);
  awaitPoll(timerPollStart,     timerPollInterval,     curMillis, &checkTimer);
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void checkButton() {
  byte buttonPressed = isButtonPressed();
  if (buttonPressed) {
    ++buttonHoldLoops;
    if(buttonHoldLoops * buttonPollInterval >= 2000) {
      if(buttonHoldFlash <= 0) {
        confirmationFlash(0);
        buttonHoldFlash = delayBetweenHoldFlashes;
      }
      buttonHoldFlash -= buttonPollInterval;
    }
  } else {
    if (buttonHoldLoops * buttonPollInterval >= 2000) {
      //resetFunc();
    }
    buttonHoldLoops = 0;
  }
  
  bool pressed = buttonPressed && !lastButtonState;
  if (pressed) {
    toggleLED();
  }
  lastButtonState = buttonPressed;

}

void checkIMU() {
  float ax, ay, az, gx, gy, gz;
  tiltHistoryIndex = (tiltHistoryIndex + 1) % tiltHistoryLength;
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    yAccelerationHistory[tiltHistoryIndex] = ay;
    //printXYZ(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    //printXYZ(gx, gy, gz);
  }

  // Get the average of recent accelerometer y values to cancel out bonking fluctuations
  float tilt = 0;
  for (int i = 0; i < tiltHistoryLength; ++i) {
    tilt += yAccelerationHistory[i];
  }
  tilt /= (float)tiltHistoryLength;
  tilt = tilt + 1;
  //Serial.print(tilt);
  //Serial.print("\t");

  // Handle tilting to activate the timer
  if (tilt >= tiltThreshold) {
    tilting = true;
    if (stableAfterTilt) {
      stableAfterTilt = false;
      Serial.println("IMU: Tilting");
    }
    if (tilt >= 0.975f && stableAfterTiming) {
      stableAfterTiming = false;
      startTimer(defaultTimerMillis);
    } else if (timer == 0) {
      float frac = tiltLightFunc(tilt);
      setLEDBrightnessFloat(frac);
    }
  } else {
    if (tilting && !timer) {
      toggleLED();
      toggleLED();
    }
    tilting = false;
  }

  // Handle bonking
  if (!tilting) {
    // Take the absolute values
    float x = abs(gx);
    float y = abs(gy);
    float z = abs(gz);
    
    // Filter out data spikes
    // These spikes seem to happen randomly while at rest, to all axes, and the value is always near 15.5
    if (lastGyroscopeX < bonkTolerance && x >= 15.25f && x <= 15.75f) x = 0.0f;
    if (lastGyroscopeY < bonkTolerance && y >= 15.25f && y <= 15.75f) y = 0.0f;
    if (lastGyroscopeZ < bonkTolerance && z >= 15.25f && z <= 15.75f) z = 0.0f;

    lastGyroscopeX = x;
    lastGyroscopeY = y;
    lastGyroscopeZ = z;

    //printXYZ(x, y, z);
    float diff = x + /*y +*/ z;
    //Serial.print(diff);
    
    // Bonking
    if(bonkFreeze > 0) {
      --bonkFreeze;
    } else {
      if (diff > bonkTolerance) {
        ++bonkLoops;
      } else {
        if (bonkLoops > 0 && bonkLoops < maxBonkLoops) {
          Serial.println("IMU: Bonk triggered");
          toggleLED();
          bonkFreeze = maxBonkLoops;
        }
        if (!stableAfterTilt || !stableAfterTiming) {
          stableAfterTilt = true;
          stableAfterTiming = true;
          bonkFreeze = maxBonkLoops;
        }
        bonkLoops = 0;
      }
    }
    
  }

  //Serial.println();
}

void startTimer(unsigned long millisToWait) {
  confirmationFlash(1);
  timerMillis = millisToWait;
  timer = millis() + millisToWait;
  Serial.println("Timer: Start");
}

void checkTimer() {
  if (timer) {
    unsigned long diff = (timer - millis());
    Serial.println("Timer: " + String(diff) + " ms remaining");
    if (diff < timerMillis) { // Still timing
      float frac = timerLightFunc((float)diff / timerMillis);
      setLEDBrightnessFloat(frac);
    } else {
      toggleLED(); // Automatically sets timer to 0
    }
  }
}

void morseCode(String str) {
  byte originalState = ledState;
  confirmationFlash(0);
  delay(1000);

  // Looking through each character of the message
  for (int i = 0; i < str.length(); ++i) {
    char c = str.charAt(i);

    if (c >= 'a' && c <= 'z') {
      c = c - 'a';
    } else if (c >= 'A' && c <= 'Z') {
      c = c - 'A';
    } else if (c >= '0' && c <= '9') {
      c = c - '0' + 26;
    } else {
      delay(7 * morseDelay); // Delay 7 between words (with a space, or any unrecognized symbol)
      continue;
    }

    // Looking through the corresponding morse code
    for (int j = 0; j < morseTable[c].length(); ++j) {
      toggleLED();
      delay(((morseTable[c].charAt(j) - '0') * 2 + 1) * morseDelay); // Delay 1 unit if dot, 3 if dash
      toggleLED();
      delay(morseDelay);

      if (isButtonPressed()) { // Abort if the button is pressed
        return;
      }
    }

    delay(morseDelay * 2); // Delay 3 between letters (including the delay from the for loop)
    
    if (isButtonPressed()) { // Abort if the button is pressed
      return;
    }
  }

  delay(1000);
  if(originalState) {
    toggleLED();
  }
}

void checkBluetooth() {
  // Poll for BLE events
  BLE.poll();

  if (toggleCharacteristic.written()) {
    toggleLED();
  }

  if (timerCharacteristic.written()) {
    unsigned long millisToWait = (long)timerCharacteristic.value() * 60 * 1000;
    startTimer(millisToWait);
  }

  if (morseCharacteristic.written()) {
    morseCode(morseCharacteristic.value());
  }
}

// All these values are hard-coded to feel good, check links for details
// Make sure 'x' is between 0 and 1

// https://www.desmos.com/calculator/in73bophjw
// Spend more time dim to counteract the logarithmic nature of light while tilting
float tiltLightFunc(float x) {
  const float a = 0.5f, b = 0.45f, c = 0.025f;
  return a * tan(b * PI * x) / PI - c;
}

// https://www.desmos.com/calculator/bo4g58kh6h
// https://www.desmos.com/calculator/0m9bis4prk
// Spend more time bright, then quickly fade towards the end of the timer
float timerLightFunc(float x) {
  //const float a = 0.07f, b = -0.005f, c = 0.065f, d = 1.0f;
  //const float a = 0.25f, b = -0.04f, c = 0.2f, d = 1.0f;
  const float a = 0.3f, b = -0.787f, c = -1.614f, d = 0.3333f;
  return x <= d ? (x + b) / (x + a + b) + c : 1;
}

void setLEDBrightnessInt(int val) {
  analogWrite(ledPin, val);
  Serial.println("LED Brightness: " + String(val));
}

void setLEDBrightnessFloat(float frac) {
  setLEDBrightnessInt(round(constrain(frac * 255, 0, 255)));
}

void toggleLED() {
  ledState = 1 - ledState;
  setLEDBrightnessInt(ledState * 255);
  digitalWrite(LED_BUILTIN, ledState);
  timer = 0;
}

void confirmationFlash(int desiredState) {
  // 3 cycles, 6 loops
  //         Cur on  Cur off
  // New on    6       5
  // New off   5       6
  int startState = ledState;
  for (int i = 0; i < 6 - abs(startState - desiredState); ++i) {
    toggleLED();
    delay(confirmationDelay);
  }
}

void awaitPoll(unsigned long & timer, unsigned long interval, unsigned long curMillis, void (*function)()) {
  if (curMillis - timer >= interval) {
    function();
    timer = curMillis;
  }
}

void printXYZ(float x, float y, float z) {
  Serial.print("\t");
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t");
  Serial.println(ledState);
}

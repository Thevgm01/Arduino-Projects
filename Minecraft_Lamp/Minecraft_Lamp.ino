#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

BLEService LEDService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

BLEByteCharacteristic toggleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLEByteCharacteristic valueCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic timerCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLEStringCharacteristic morseCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite, 256);
BLEByteCharacteristic gameCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int LEDPin = 3;
int LEDValue = 0;
int maxLEDValue = 255;
const int confirmationDelay = 100;
const int errorDelay = 500;

int ignoreFirstIMULoops = 50;

const bool printMessages = false, printGraphs = false;

// Button presses
const int buttonPin = 5;
byte lastButtonState = 1;

// IMU
const byte xAccel = 0, yAccel = 1, zAccel = 2,
           xGyro  = 3, yGyro  = 4, zGyro  = 5;
const byte highpassMask = 1 << yAccel;
const byte lowpassMask  = 1 << zAccel | highpassMask;
const float emaAlphas[] = 
         { 0.1f,       0.99f,      0.05f,
           0.1f,       0.1f,       0.1f };
float highpasses[6], lowpasses[6];

// Bonking
const float bonkThreshold = 0.003f;
const unsigned long bonkDelayTimerDefault = 100;
unsigned long bonkDelayTimer = 0;

// Tilting
const float tiltThreshold = 0.1f;

// Timing
const unsigned long defaultTimerMillis = 15 * 60 * 1000; // 15 minutes

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
const unsigned long reactionTimeChange = 50;
unsigned long reactionTime = 0;
byte points = 0;
bool expectedLEDValue = false;
bool gaming = false;

// Polling
const byte IMU = 0, BUTTON = 1, TIMER = 2, BLUETOOTH = 3, GAME = 4, 
           BONK = 5;
unsigned long pollTimers[] = {
  0,  // IMU
  5,  // BUTTON
  10, // TIMER
  15, // BLUETOOTH
  0,  // GAME
  0   // BONK
};
unsigned long pollLengths[] = {
  0,  // IMU (every loop)
  50, // BUTTON (20 times per second)
  33, // TIMER (30 times per second)
  200,// BLUETOOTH (5 times per second)
  -1, // GAME (variable)
  100 // BONK (1/10th of a second)
};
void (*pollFunctions[5])(unsigned long curMillis);

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  //pinMode(LEDPin, OUTPUT); // Not needed because of analogWrite
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PWR, LOW);

  // Bluetooth initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE faiLED!");
    errorLoop();
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Minecraft Lamp");
  BLE.setAdvertisedService(LEDService);

  // add the characteristics to the service
  LEDService.addCharacteristic(toggleCharacteristic);
  LEDService.addCharacteristic(valueCharacteristic);
  LEDService.addCharacteristic(timerCharacteristic);
  LEDService.addCharacteristic(morseCharacteristic);
  LEDService.addCharacteristic(gameCharacteristic);

  // Add service
  BLE.addService(LEDService);

  // Set the initial value for the characeristic:
  //toggleCharacteristic.writeValue(0);
  //timerCharacteristic.writeValue(0);
  //morseCharacteristic.writeValue("");

  // Start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  // IMU initialization
  if (!IMU.begin()) {
    Serial.println("FaiLED to initialize IMU!");
    errorLoop();
  }

  pollFunctions[IMU] = checkBluetooth;
  pollFunctions[BUTTON] = checkButton;
  pollFunctions[TIMER] = checkTimer;
  pollFunctions[BLUETOOTH] = checkBluetooth;
  pollFunctions[GAME] = checkGame;
}

void errorLoop() {
  while (1) {
    confirmationFlash(0);
    delay(errorDelay);
  }
}

void loop() {
  unsigned long curMillis = millis();
  awaitPoll(IMU,       curMillis);
  awaitPoll(BUTTON,    curMillis);
  awaitPoll(TIMER,     curMillis);
  awaitPoll(BLUETOOTH, curMillis);
  awaitPoll(GAME,      curMillis);
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void checkButton(unsigned long curMillis) {
  byte buttonState = isButtonPressed();  
  bool pressed = buttonState && !lastButtonState;
  bool released = !buttonState && lastButtonState;
  if (pressed) {
    maxLEDValue = 255;
    toggleLED();
  }
  lastButtonState = buttonState;

}

void checkIMU(unsigned long curMillis) {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    return;
  }

  float readings[6];
  IMU.readAcceleration(readings[0], readings[1], readings[2]);
  IMU.readGyroscope(readings[3], readings[4], readings[5]);

  for (int i = 0; i < 6; ++i) {
    if (bitRead(lowpassMask, i)) {
      lowpasses[i] = emaAlphas[i] * readings[i] + (1 - emaAlphas[i]) * lowpasses[i];
    }
    if (bitRead(highpassMask, i)) {
      highpasses[i] = readings[i] - lowpasses[i];
    }
  }
  if (printGraphs) {
    Serial.println();
  }

  // Wait for the initial values to settle down
  if (ignoreFirstIMULoops > 0) {
    --ignoreFirstIMULoops;
    return;
  }

  // Handle tilting forward to activate the timer
  if (lowpasses[zAccel] >= 0.975f && !timer) {
    startTimer(curMillis, defaultTimerMillis);
  } else if (lowpasses[zAccel] >= tiltThreshold) {
    resetPoll(BONK, curMillis); // Ensure we can't bonk when setting the lamp back down
  }

  // Handle bonking
  if (abs(highpasses[yAccel]) >= bonkThreshold) { // Moved enough to trigger a bonk
    if (checkPoll(BONK, curMillis)) { // Enough time has passed since the last bonk
      printMessage("IMU: Bonk triggered");
      toggleLED();
    }
    resetPoll(BONK, curMillis);
  }
}


void startTimer(unsigned long curMillis, unsigned long millisToWait) {
  confirmationFlash(1);
  timer = curMillis;
  timerMillis = millisToWait;
  printMessage("Timer: Start");
}

void checkTimer(unsigned long curMillis) {
  if (timer) {
    unsigned long diff = curMillis - timer;
    printMessage("Timer: " + String(diff) + "/" + String(timerMillis) + " ms");
    if (diff < timerMillis) { // Still timing
      float frac = timerLightFunc(((float)timerMillis - diff) / timerMillis);
      setLEDBrightnessFloat(frac);
    } else {
      toggleLED(); // Automatically sets timer to 0
    }
  }
}

void morseCode(String str) {
  byte originalState = LEDValue;
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

void checkBluetooth(unsigned long curMillis) {
  // Poll for BLE events
  BLE.poll();

  if (toggleCharacteristic.written()) {
    toggleLED();
  }

  if (valueCharacteristic.written()) {
    maxLEDValue = valueCharacteristic.value();
    setLEDBrightnessInt(maxLEDValue);
  } else if (valueCharacteristic.value() != LEDValue) {
    valueCharacteristic.writeValue(LEDValue);
  }

  if (timerCharacteristic.written()) {
    unsigned long millisToWait = (long)timerCharacteristic.value() * 60 * 1000;
    startTimer(curMillis, millisToWait);
  }

  if (morseCharacteristic.written()) {
    morseCode(morseCharacteristic.value());
  }

  if (gameCharacteristic.written()) {
    startGame(curMillis);
  }
}

void startGame(unsigned long curMillis) {
  gaming = true;
  points = 0;
  gamePollStart = curMillis;
  gamePollInterval = 0;
  expectedLEDValue = true;
  printMessage("Game: Start");
  confirmationFlash(0);
}

void checkGame(unsigned long curMillis) {
  if (!gaming) {
    return;
  }
  
  if (LEDValue && !expectedLEDValue) { // LED turned on while off, game over
    gaming = false;
    gamePollInterval = -1;
    printMessage("Game: Finished with " + String(points) + " points");
    confirmationFlash(0);
    return;
  } else if (!LEDValue && expectedLEDValue) { // LED turned off while on, add point, get harder
    ++points;
    gameCharacteristic.writeValue(points);
    reactionTime = startReactionTime - reactionTimeChange * points;
    printMessage("Game: " + String(points) + " points");
    printMessage("Game: " + String(gamePollInterval) + " ms reaction time ");
  } else {
    toggleLED();
  }

  expectedLEDValue = LEDValue;
  if (expectedLEDValue) {
    gamePollInterval = reactionTime;
  } else {
    gamePollInterval = random(minDelay, maxDelay);
  }

  printMessage("Game: Wait for " + String(gamePollInterval) + " ms");
}

// All these values are hard-coded to feel good, check links for details
// Make sure 'x' is between 0 and 1

// https://www.desmos.com/calculator/in73bophjw
// Spend more time dim to counteract the logarithmic nature of light while tilting
float tanLightFunc(float x) {
  const float a = 0.5f, b = 0.45f, c = 0.025f;
  return a * tan(b * PI * x) / PI - c;
}

// Spend more time bright, then quickly fade towards the end of the timer
float timerLightFunc(float x) {
  const float cutoff = 1/3.0f;
  return x < cutoff ? tanLightFunc(x / cutoff) : 1;
}

void setLEDBrightnessInt(int val) {
  analogWrite(LEDPin, val);
  printMessage("LED Brightness: " + String(val));
}

void setLEDBrightnessFloat(float frac) {
  setLEDBrightnessInt(round(constrain(frac, 0, 1) * 255));
}

void toggleLED() {
  LEDValue = LEDValue ? 0 : maxLEDValue;
  setLEDBrightnessInt(LEDValue);
  timer = 0;
}

void confirmationFlash(int desiredState) {
  // 3 cycles, 6 loops
  //         Cur on  Cur off
  // New on    6       5
  // New off   5       6
  int startState = 0 || LEDValue; // Account for the fact that LEDValue is 0 or 255
  for (int i = 0; i < 6 - abs(startState - desiredState); ++i) {
    toggleLED();
    delay(confirmationDelay);
  }
}

bool checkPoll(int index, unsigned long curMillis) {
  return curMillis - pollTimers[index] >= pollLengths[index];
}

void resetPoll(int index, unsigned long curMillis) {
  pollTimers[index] = curMillis;
}

void setPollLength(int index, unsigned long pollMillis) {
  pollLengths[index] = pollMillis;
}

void awaitFunctionPoll(int index, unsigned long curMillis) {
  if (checkPoll(index, curMillis)) {
    (*pollFunctions[index])(curMillis);
    resetPoll(index, curMillis);
  }
}

void printMessage(String message) {
  if (printMessages) {
    Serial.println(message);
  }
}

void printGraph(float number) {
  if (printGraphs) {
    Serial.print(String(number) + ",");
  }
}

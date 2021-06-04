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

int ignoreFirstIMU_CHECKLoops = 50;

const bool printMessages = false, printGraphs = false;

// Button presses
const int buttonPin = 5;
byte lastButtonState = 1;
float startCycleValue = 0.0f;

// IMU_CHECK
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
bool expectedLEDState = false;

// Polling
const byte IMU_CHECK = 0, BUTTON_CHECK = 1, TIMER_CHECK = 2, BLUETOOTH_CHECK = 3, GAME_CHECK = 4, // Function polls
           BONK_DELAY = 5, BUTTON_HELD = 6, TIMER_ACTUAL = 7; // Other polls
unsigned long pollTimers[] = {
  0,   // IMU_CHECK
  5,   // BUTTON_CHECK
  10,  // TIMER_CHECK
  15,  // BLUETOOTH_CHECK
  0,   // GAME_CHECK
  0,   // BONK_DELAY
  0,   // BUTTON_HELD 
  0,   // TIMER_ACTUAL
};
unsigned long pollLengths[] = {
  1,   // IMU_CHECK (every loop)
  33,  // BUTTON_CHECK (30 times per second)
  33,  // TIMER_CHECK (30 times per second)
  200, // BLUETOOTH_CHECK (5 times per second)
  0,   // GAME_CHECK (variable)
  200, // BONK_DELAY (1/5th of a second)
  1000,// BUTTON_HELD (1 second)
  0    // TIMER_ACTUAL (variable)
};
void (*pollFunctions[5])(const unsigned long curMillis);

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

  // IMU_CHECK initialization
  if (!IMU.begin()) {
    Serial.println("FaiLED to initialize IMU!");
    errorLoop();
  }

  pollFunctions[BUTTON_CHECK] = checkButton;
  pollFunctions[IMU_CHECK] = checkIMU;
  pollFunctions[TIMER_CHECK] = checkTimer;
  pollFunctions[BLUETOOTH_CHECK] = checkBluetooth;
  pollFunctions[GAME_CHECK] = checkGame;
}

void errorLoop() {
  while (1) {
    confirmationFlash(0);
    delay(errorDelay);
  }
}

void loop() {
  const unsigned long curMillis = millis();
  awaitFunctionPoll(BUTTON_CHECK,    curMillis);
  if (lastButtonState)               return; // Don't check anything else if the button is held
  awaitFunctionPoll(IMU_CHECK,       curMillis);
  awaitFunctionPoll(TIMER_CHECK,     curMillis);
  awaitFunctionPoll(BLUETOOTH_CHECK, curMillis);
  awaitFunctionPoll(GAME_CHECK,      curMillis);
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void checkButton(const unsigned long curMillis) {
  byte buttonState = isButtonPressed();  
  bool pressed = buttonState && !lastButtonState;
  bool released = !buttonState && lastButtonState;
  if (pressed) {
    maxLEDValue = 255;
    toggleLED();
    startCycleValue = asin(map(LEDValue, 0, 255, -1, 1));
    resetPoll(BUTTON_HELD, curMillis);
  } else if (buttonState && checkPoll(BUTTON_HELD, curMillis)) {
    unsigned long holdTime = getPollTimeDiff(BUTTON_HELD, curMillis) - pollLengths[BUTTON_HELD];
    float sinValue = sin(startCycleValue + ((float)holdTime / 255 / PI));
    float newLEDFrac = (sinValue + 1) / 2; //map(sinValue, -1, 1, 0, 1);
    maxLEDValue = round(newLEDFrac * 255);
    setLEDBrightnessInt(maxLEDValue);
  }
  lastButtonState = buttonState;
}

void checkIMU(const unsigned long curMillis) {
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
  if (ignoreFirstIMU_CHECKLoops > 0) {
    --ignoreFirstIMU_CHECKLoops;
    return;
  }

  // Handle tilting forward to activate the timer
  if (lowpasses[zAccel] >= 0.975f && !isTiming()) {
    startTimer(curMillis, defaultTimerMillis);
  } else if (lowpasses[zAccel] >= tiltThreshold) {
    resetPoll(BONK_DELAY, curMillis); // Ensure we can't bonk when setting the lamp back down
  }

  // Handle bonking
  if (abs(highpasses[yAccel]) >= bonkThreshold) { // Moved enough to trigger a bonk
    if (checkPoll(BONK_DELAY, curMillis)) { // Enough time has passed since the last bonk
      printMessage("IMU: Bonk triggered");
      toggleLED();
    }
    resetPoll(BONK_DELAY, curMillis);
  }
}


void startTimer(const unsigned long curMillis, unsigned long millisToWait) {
  confirmationFlash(1);
  resetPoll(TIMER_ACTUAL, curMillis);
  setPollLength(TIMER_ACTUAL, millisToWait);
  printMessage("Timer: Start");
}

bool isTiming() {
  return pollLengths[TIMER_ACTUAL] > 0;
}

void checkTimer(const unsigned long curMillis) {
  if (isTiming()) {
    unsigned long diffMillis = getPollTimeDiff(TIMER_ACTUAL, curMillis);
    printMessage("Timer: " + String(diffMillis) + "/" + String(pollLengths[TIMER_ACTUAL]) + " ms");
    if (!checkPoll(TIMER_ACTUAL, curMillis)) { // Still timing
      float frac = timerLightFunc(((float)pollLengths[TIMER_ACTUAL] - diffMillis) / pollLengths[TIMER_ACTUAL]);
      setLEDBrightnessFloat(frac);
    } else {
      setPollLength(TIMER_ACTUAL, 0);
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

      if (isButtonPressed()) return; // Abort if the button is pressed
    }

    delay(morseDelay * 2); // Delay 3 between letters (including the delay from the for loop)
    
    if (isButtonPressed()) return; // Abort if the button is pressed
  }

  delay(1000);
  if(originalState) {
    toggleLED();
  }
}

void checkBluetooth(const unsigned long curMillis) {
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

void startGame(const unsigned long curMillis) {
  points = 0;
  resetPoll(GAME_CHECK, curMillis);
  setPollLength(GAME_CHECK, 1);
  expectedLEDState = true;
  printMessage("Game: Start");
  confirmationFlash(0);
}

bool isGaming() {
  return pollLengths[GAME_CHECK] > 0;
}

void checkGame(const unsigned long curMillis) {
  if (!isGaming()) {
    return;
  }
  
  if (LEDValue && !expectedLEDState) { // LED turned on while off, game over
    setPollLength(GAME_CHECK, 0);
    printMessage("Game: Finished with " + String(points) + " points");
    confirmationFlash(0);
    return;
  } else if (!LEDValue && expectedLEDState) { // LED turned off while on, add point, get harder
    ++points;
    gameCharacteristic.writeValue(points);
    reactionTime = startReactionTime - reactionTimeChange * points;
    printMessage("Game: " + String(points) + " points");
    printMessage("Game: " + String(pollLengths[GAME_CHECK]) + " ms reaction time ");
  } else {
    toggleLED();
  }

  expectedLEDState = LEDValue;
  if (expectedLEDState) {
    setPollLength(GAME_CHECK, reactionTime);
  } else {
    setPollLength(GAME_CHECK, random(minDelay, maxDelay));
  }

  printMessage("Game: Wait for " + String(pollLengths[GAME_CHECK]) + " ms");
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
  LEDValue = val;
  analogWrite(LEDPin, val);
  printMessage("LED Brightness: " + String(val));
}

void setLEDBrightnessFloat(float frac) {
  setLEDBrightnessInt(round(constrain(frac, 0, 1) * maxLEDValue));
}

void toggleLED() {
  setLEDBrightnessInt(LEDValue ? 0 : maxLEDValue);
  pollLengths[TIMER_ACTUAL] = 0;
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

unsigned long getPollTimeDiff(int index, const unsigned long curMillis) {
  return curMillis - pollTimers[index];
}

bool checkPoll(int index, const unsigned long curMillis) {
  return curMillis - pollTimers[index] >= pollLengths[index] && 
         pollLengths[index] > 0 || 
         pollLengths[index] == 1;
}

void resetPoll(int index, const unsigned long curMillis) {
  pollTimers[index] = curMillis;
}

void setPollLength(int index, unsigned long pollMillis) {
  pollLengths[index] = pollMillis;
}

void awaitFunctionPoll(int index, const unsigned long curMillis) {
  if (checkPoll(index, curMillis)) {
    (*pollFunctions[index])(curMillis);
    resetPoll(index, curMillis);
  }
}

void printMessage(String message) {
  if (Serial && printMessages) {
    Serial.println(message);
  }
}

void printGraph(float number) {
  if (Serial && printGraphs) {
    Serial.print(String(number) + ",");
  }
}

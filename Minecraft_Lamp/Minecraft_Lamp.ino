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
const int errorDelay = 500;

int ignoreFirstIMULoops = 100;

const bool printMessages = true, printGraphs = false;

// Button presses
const int buttonPin = 5;
byte lastButtonState = 1;

// IMU
const byte xAccel = 0, yAccel = 1, zAccel = 2,
           xGyro  = 3, yGyro  = 4, zGyro  = 5;
const byte smoothedHighpassMask = 1 << zGyro;
const byte highpassMask = 1 << yAccel | smoothedHighpassMask;
const byte lowpassMask =  1 << zAccel | highpassMask;
const float emaAlphas[] = 
         { 0.1f,       0.9f,       0.1f,
           0.1f,       0.1f,       0.03f };
float smoothedHighpasses[6], highpasses[6], lowpasses[6];

// Bonking
const float bonkThreshold = 0.05f;
const unsigned long bonkDelayTimerDefault = 50;
unsigned long bonkDelayTimer = 0;

// Tilting
const float tiltThreshold = 0.1f;
const float tiltHandheldThreshold = 2.0f;
bool tilting = false;

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
const unsigned long IMUPollInterval = 0;         // Every loop
const unsigned long buttonPollInterval = 50;     // 20 times per second
const unsigned long timerPollInterval = 500;     // 2 times per second
const unsigned long bluetoothPollInterval = 200; // 5 times per second
unsigned long IMUPollStart = 0;
unsigned long buttonPollStart = 7;
unsigned long timerPollStart = 13;
unsigned long bluetoothPollStart = 19;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  //pinMode(ledPin, OUTPUT); // Not needed because of analogWrite
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
    delay(errorDelay);
  }
}

void loop() {
  unsigned long curMillis = millis();
  awaitPoll(IMUPollStart,       IMUPollInterval,       curMillis, &checkIMU);
  awaitPoll(bluetoothPollStart, bluetoothPollInterval, curMillis, &checkBluetooth);
  awaitPoll(buttonPollStart,    buttonPollInterval,    curMillis, &checkButton);
  awaitPoll(timerPollStart,     timerPollInterval,     curMillis, &checkTimer);
}

bool isButtonPressed() {
  return !digitalRead(buttonPin);
}

void checkButton(unsigned long curMillis) {
  byte buttonState = isButtonPressed();  
  bool pressed = buttonState && !lastButtonState;
  if (pressed) {
    toggleLED();
  }
  lastButtonState = buttonState;

}

void checkIMU(unsigned long curMillis) {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    return;
  }
  
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  float readings[] = { ax, ay, az, gx, gy, gz };
  for (int i = 0; i < 6; ++i) {
    if (bitRead(lowpassMask, i)) {
      lowpasses[i] = emaAlphas[i] * readings[i] + (1 - emaAlphas[i]) * lowpasses[i];
      printGraph(lowpasses[i]);
    }
    if (bitRead(highpassMask, i)) {
      highpasses[i] = readings[i] - lowpasses[i];
      printGraph(highpasses[i]);
    }
    if (bitRead(smoothedHighpassMask, i)) {
      smoothedHighpasses[i] = emaAlphas[i] * abs(highpasses[i]) + (1 - emaAlphas[i]) * smoothedHighpasses[i];
      printGraph(smoothedHighpasses[i]);
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
  // Make sure Gyro Z < some threshold (to prevent detection while being held)
  // Make sure Accel Z > another threshold
  if (lowpasses[zAccel] >= tiltThreshold && smoothedHighpasses[zGyro] < tiltHandheldThreshold) {
    if (!tilting) {
      printMessage("IMU: Tilting began");
      tilting = true;
    }
    if (timer == 0) {
      if (lowpasses[zAccel] >= 0.975f) {
        startTimer(curMillis, defaultTimerMillis);
      } else {
        float frac = tiltLightFunc(lowpasses[zAccel]);
        setLEDBrightnessFloat(frac);
      }
    }
  } else {
    if (tilting) {
      printMessage("IMU: Tilting stopped");
      tilting = false;
      if(!timer) {
        setLEDBrightnessInt(ledState);
      }
    }
  }

  // Handle bonking
  if (!tilting && abs(highpasses[yAccel]) >= bonkThreshold) { // Moved enough to trigger a bonk
    if (curMillis - bonkDelayTimer >= bonkDelayTimerDefault) { // Enough time has passed since the last bonk
      printMessage("IMU: Bonk triggered");
      toggleLED();
    }
    bonkDelayTimer = curMillis;
  }
}

void startTimer(unsigned long curMillis, unsigned long millisToWait) {
  confirmationFlash(1);
  timerMillis = millisToWait;
  timer = curMillis;
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

void checkBluetooth(unsigned long curMillis) {
  // Poll for BLE events
  BLE.poll();

  if (toggleCharacteristic.written()) {
    toggleLED();
  }

  if (timerCharacteristic.written()) {
    unsigned long millisToWait = (long)timerCharacteristic.value() * 60 * 1000;
    startTimer(curMillis, millisToWait);
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

// Spend more time bright, then quickly fade towards the end of the timer
float timerLightFunc(float x) {
  float cutoff = 1/3.0f;
  return x < cutoff ? tiltLightFunc(x / cutoff) : 1;
}

void setLEDBrightnessInt(int val) {
  analogWrite(ledPin, val);
  printMessage("LED Brightness: " + String(val));
}

void setLEDBrightnessFloat(float frac) {
  setLEDBrightnessInt(round(constrain(frac * 255, 0, 255)));
}

void toggleLED() {
  ledState = 255 - ledState;
  setLEDBrightnessInt(ledState);
  timer = 0;
}

void confirmationFlash(int desiredState) {
  // 3 cycles, 6 loops
  //         Cur on  Cur off
  // New on    6       5
  // New off   5       6
  int startState = 0 || ledState; // Account for the fact that ledState is 0 or 255
  for (int i = 0; i < 6 - abs(startState - desiredState); ++i) {
    toggleLED();
    delay(confirmationDelay);
  }
}

void awaitPoll(unsigned long & timer, unsigned long interval, unsigned long curMillis, void (*function)(unsigned long)) {
  if (curMillis - timer >= interval) {
    function(curMillis);
    timer = curMillis;
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

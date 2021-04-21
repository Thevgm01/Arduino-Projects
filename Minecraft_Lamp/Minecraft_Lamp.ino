#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

BLEByteCharacteristic toggleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLEUnsignedIntCharacteristic timerCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLENotify | BLEWrite);
BLEStringCharacteristic morseCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite, 256);

const int ledPin = 3;
byte ledState = 0;

const int buttonPin = 5;
byte lastButtonState = 1;
int buttonHoldLoops = 0;
int buttonHoldFlash = 0;
const int delayBetweenHoldFlashes = 500;

// Gyroscope
const int maxBonkLoops = 10;
const float bonkTolerance = 5.0f;
int bonkLoops = 0;
int bonkTimer = 0;
float lastGyroscopeX, lastGyroscopeY, lastGyroscopeZ;

bool timing = false;

const int morseDelay = 100;
const String morseTable[] = 
//A     B       C       D      E    F       G      H       I     J       K      L       M
{"01", "1000", "1010", "100", "0", "0010", "110", "0000", "00", "0111", "101", "0100", "11",
//N     O      P       Q       R      S      T    U      V       W      X       Y       Z
 "10", "111", "0110", "1101", "010", "000", "1", "001", "0001", "011", "1001", "1011", "1100",
//0,       1        2        3        4        5        6        7        8        9
 "11111", "01111", "00111", "00011", "00001", "00000", "10000", "11000", "11100", "11110" };

const int confirmationDelay = 100;

const unsigned long buttonPollInterval = 50;
const unsigned long gyroscopePollInterval = 20;
const unsigned long timerPollInterval = 1000;
const unsigned long bluetoothPollInterval = 200;
unsigned long buttonPollStart = 3;
unsigned long gyroscopePollStart = 7;
unsigned long timerPollStart = 11;
unsigned long bluetoothPollStart = 17;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  pinMode(ledPin, OUTPUT);
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
  awaitPoll(timerPollStart, timerPollInterval, curMillis, &checkTimer);
  awaitPoll(buttonPollStart, buttonPollInterval, curMillis, &checkButton);
  awaitPoll(gyroscopePollStart, gyroscopePollInterval, curMillis, &checkGyroscope);
  awaitPoll(bluetoothPollStart, bluetoothPollInterval, curMillis, &checkBluetooth);
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
    // gyroscopeTimer = gyroscopeStartTimer;
  }
  lastButtonState = buttonPressed;

}

//int a = 0;
//float total[3];

void checkGyroscope() {
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    // Take the absolute values
    x = abs(x);
    y = abs(y);
    z = abs(z);
    
    // Filter out random spikes
    // These spikes seem to happen randomly while at rest, to all axes, and the value is always near 15.5
    if (lastGyroscopeX < bonkTolerance && x >= 15.25f && x <= 15.75f) x = 0.0f;
    if (lastGyroscopeY < bonkTolerance && y >= 15.25f && y <= 15.75f) y = 0.0f;
    if (lastGyroscopeZ < bonkTolerance && z >= 15.25f && z <= 15.75f) z = 0.0f;

    lastGyroscopeX = x;
    lastGyroscopeY = y;
    lastGyroscopeZ = z;

    //printXYZ(x, y, z);

    /*
    total[a] = x + y + z;
    Serial.print(total[a]);
    Serial.print("\t");
    a = (a + 1) % 3;

    float totalDiff = 0.0f;
    for(int i = 0; i < 3; ++i) {
      totalDiff += total[i];
    }
    totalDiff /= 3.0f;
    Serial.println(totalDiff);
    */
    
    // Bonking
    if(bonkTimer > 0) {
      --bonkTimer;
    } else {
      if (x + y + z > bonkTolerance) {
        ++bonkLoops;
      } else {
        if(bonkLoops > 0 && bonkLoops < maxBonkLoops) {
          toggleLED();
          bonkTimer = maxBonkLoops;
        }
        bonkLoops = 0;
      }
    }
    
  }
}

void checkTimer() {
  if (timerCharacteristic.value() == 0) {
    return;
  }
  timerCharacteristic.writeValue(timerCharacteristic.value() - timerPollInterval);
  if(timerCharacteristic.value() == 0) {
    toggleLED();
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
    timerCharacteristic.writeValue(timerCharacteristic.value() * 1000 * 60);
    confirmationFlash(1);
    timing = true;
  }

  if (morseCharacteristic.written()) {
    morseCode(morseCharacteristic.value());
  }
}

void toggleLED() {
  ledState = 1 - ledState;
  digitalWrite(ledPin, ledState); // toggle the LED
  digitalWrite(LED_BUILTIN, ledState);
  timing = false;
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

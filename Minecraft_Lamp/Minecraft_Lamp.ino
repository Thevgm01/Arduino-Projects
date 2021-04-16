// Arduino_LSM6DS3 - Version: Latest 
#include <Arduino_LSM9DS1.h>

/*
  LED

  This example creates a BLE peripheral with service that contains a
  characteristic to control an LED.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic timerCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLENotify | BLEWrite);
BLEStringCharacteristic morseCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite, 256);
BLEUnsignedIntCharacteristic gyroscopeCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = 3; // pin to use for the LED
byte ledState = 0;

const int btnPin = 5;
byte lastBtnState = 1;

const int gyroscopeStartTimer = 1000;
int gyroscopeTimer = 0;

const float tiltTolerance = 0.15f;
byte tiltSwitch = 0;

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
const int globalDelay = 100;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(btnPin, INPUT_PULLUP);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    errorLoop();
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Minecraft Lamp");
  BLE.setAdvertisedService(ledService);

  // add the characteristics to the service
  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(timerCharacteristic);
  ledService.addCharacteristic(morseCharacteristic);
  ledService.addCharacteristic(gyroscopeCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  //switchCharacteristic.writeValue(0);
  //timerCharacteristic.writeValue(0);
  //morseCharacteristic.writeValue("");
  gyroscopeCharacteristic.writeValue(2000);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    errorLoop();
  }
}

void errorLoop() {
  while (1) {
    toggleLED();
    delay(500);
  }
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
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

      checkTimer();
      checkButton();
      checkGyroscope();
      checkAccelerometer();
      delay(globalDelay);
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }

  checkTimer();
  checkButton();
  checkGyroscope();
  checkAccelerometer();
  delay(globalDelay);
}

bool checkButton() {
  byte curBtnState = digitalRead(btnPin);
  bool result = curBtnState == LOW && lastBtnState == HIGH;
  lastBtnState = curBtnState;
  if (result) {
    toggleLED();
    gyroscopeTimer = gyroscopeStartTimer;
  }
  return result;
}

void checkGyroscope() {
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z); // Y is spin
    if(sqrt(x * x + y * y + z * z) > gyroscopeCharacteristic.value()) {
      if (gyroscopeTimer <= 0) {
        toggleLED();
      }
      gyroscopeTimer = gyroscopeStartTimer;
    }
  }
  if(gyroscopeTimer > 0) {
    gyroscopeTimer -= globalDelay;
  }
}

void checkAccelerometer() {
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    printXYZ(x, y, z);
    // y is forward/back, z is left/right
    // y starts at -1,     z starts at 0
    if(x < -tiltTolerance || x > tiltTolerance ||
       z < -tiltTolerance || z > tiltTolerance) {
      if(tiltSwitch == 0) {
        toggleLED();
        tiltSwitch = 1;
      }
    } else {
      tiltSwitch = 0;
    }
  }
}

void checkTimer() {
  if (timerCharacteristic.value() == 0) {
    return;
  }
  timerCharacteristic.writeValue(timerCharacteristic.value() - globalDelay);
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

      if (checkButton()) { // Abort if the button is pressed
        return;
      }
    }

    delay(morseDelay * 2); // Delay 3 between letters (including the delay from the for loop)
    
    if (checkButton()) { // Abort if the button is pressed
      return;
    }
  }

  if(originalState) {
    toggleLED();
  }
}

void toggleLED() {
  ledState = 1 - ledState;
  digitalWrite(ledPin, ledState); // toggle the LED
  switchCharacteristic.writeValue(ledState);
  timing = false;
}

void confirmationFlash(int desiredState) {
  // Base: 3 cycles, 6 loops
  //         Cur on  Cur off
  // New on    6       5
  // New off   5       6
  int startState = ledState;
  for (int i = 0; i < 6 - abs(startState - desiredState); ++i) {
    toggleLED();
    delay(confirmationDelay);
  }
}

void printXYZ(float x, float y, float z) {
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(",\tY: ");
  Serial.print(y);
  Serial.print(",\tZ: ");
  Serial.println(z);
}

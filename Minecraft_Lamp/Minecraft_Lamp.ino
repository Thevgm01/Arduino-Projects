#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <unordered_map>

//////////////////////////
// Forward Declarations //
//////////////////////////
namespace Debug {
  void printText(String message);
  void printGraph(float number);
  void setup();
}

namespace LED {
  void setBrightnessInt(int val);
  void setBrightnessFloat(float frac);
  void toggle();
  void confirmationFlash(int desiredState);
  void errorLoop();
  bool setup();
}

namespace Button {
  void setup();
  bool isPressed();
  void update();
}

namespace Sensors {
  void setup();
  void update();
}

namespace Timer {
  const unsigned long defaultLength = 15 * 60 * 1000; // 15 minutes
  
  void start(unsigned long millisToWait);
  bool isTiming();
  void stop();
  float tanLightFunc(float x);
  float timerLightFunc(float x);
  void update();
}

namespace Morse {
   void translateString(String str);
}

namespace Bluetooth {
  void setup();
  void update();
}

namespace Game {
  void start();
  bool isGaming();
  void update();
}

enum class PollType { SENSORS, BUTTON, TIMER, BLUETOOTH, GAME, BONK_DELAY, BUTTON_HOLD, TIMER_ACTUAL };

namespace Polling {
  unsigned long getPollElapsedMillis(PollType type);
  unsigned long getPollRemainingMillis(PollType type);
  unsigned long getPollLengthMillis(PollType type);
  bool checkPoll(PollType type);
  void resetPoll(PollType type);
  bool isActive(PollType type);
  void setPollLength(PollType type, const unsigned long millis);
  void awaitFunctionPoll(PollType type);
}

/////////////////
// Definitions //
/////////////////
namespace Debug {
  const bool printMessages = false;
  const bool printGraphs = false;

  const int awaitingSerialDelayMillis = 1000;
  
  void printText(String message) {
    if (Serial && printMessages) {
      Serial.println(message);
    }
  }
  
  void printGraph(float number) {
    if (Serial && printGraphs) {
      Serial.print(String(number) + ",");
    }
  }

  void setup() {
    Serial.begin(9600);
  
    while ((printMessages || printGraphs) && !Serial) {
      LED::toggle();
      delay(awaitingSerialDelayMillis);
    }
  }
}

namespace LED {
  const int PIN = 3;
  const int ANALOG_RESOLUTION = 12;
  const int MAX_BRIGHTNESS = (1 << ANALOG_RESOLUTION) - 1;
  
  int curBrightness = 0;
  int variableMaxBrightness = MAX_BRIGHTNESS;
  
  const int confirmationDelayMillis = 100;
  const int errorDelayMillis = 500;
  
  void setBrightnessInt(int val) {
    curBrightness = val;
    analogWrite(PIN, val);
    Debug::printText("LED Brightness: " + String(val));
  }
  
  void setBrightnessFloat(float frac) {
    setBrightnessInt(ceil(constrain(frac, 0, 1) * variableMaxBrightness));
  }
  
  void toggle() {
    setBrightnessInt(curBrightness ? 0 : variableMaxBrightness);
    Timer::stop();
  }
  
  void confirmationFlash(int desiredState) {
    // 3 cycles, 6 loops
    //         Cur on  Cur off
    // New on    6       5
    // New off   5       6
    int startState = LED::curBrightness ? 1 : 0;
    for (int i = 0; i < 6 - abs(startState - desiredState); ++i) {
      toggle();
      delay(confirmationDelayMillis);
    }
  }

  void errorLoop() {
    while (true) {
      confirmationFlash(0);
      delay(errorDelayMillis);
    }
  }

  bool setup() {
    //pinMode(LED_PIN, OUTPUT); // Not needed because of analogWrite
    analogWriteResolution(ANALOG_RESOLUTION);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PWR, LOW);
  }
}

namespace Button {
  const int PIN = 5;
  byte lastButtonState = 1;
  float startCycleValue = 0.0f;

  void setup() {
      pinMode(PIN, INPUT_PULLUP);
  }
 
  bool isPressed() {
    return !digitalRead(PIN);
  }
  
  void update() {
    byte buttonState = isPressed();  
    bool pressed = buttonState && !lastButtonState;
    bool released = !buttonState && lastButtonState;
    if (pressed) {
      LED::variableMaxBrightness = LED::MAX_BRIGHTNESS;
      LED::toggle();
      startCycleValue = asin(map(LED::curBrightness, 0, LED::MAX_BRIGHTNESS, -1, 1));
      Polling::resetPoll(PollType::BUTTON_HOLD);
    } else if (buttonState && Polling::checkPoll(PollType::BUTTON_HOLD)) {
      unsigned long holdTime = 
        Polling::getPollElapsedMillis(PollType::BUTTON_HOLD) - 
        Polling::getPollLengthMillis(PollType::BUTTON_HOLD); // Ignore the first second of holding
      float sinValue = sin(startCycleValue + (holdTime / 800.0f));
      float newBrightnessFrac = (sinValue + 1) / 2; //map(sinValue, -1, 1, 0, 1);
      LED::variableMaxBrightness = round(newBrightnessFrac * LED::MAX_BRIGHTNESS);
      LED::setBrightnessInt(LED::variableMaxBrightness);
    }
    lastButtonState = buttonState;
  }
}

namespace Sensors {
  unsigned int ignoreLoops = 50;
  
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
  
  // Tilting
  const float tiltStartThreshold = 0.1f;
  const float tiltEndThreshold = 0.975f;

  void setup() {
    if (!IMU.begin()) {
      Debug::printText("Failed to initialize IMU!");
      LED::errorLoop();
    }
  }

  void update() {
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
    if (Debug::printGraphs) {
      Debug::printText("");
    }
  
    // Wait for the initial values to settle down
    if (ignoreLoops > 0) {
      --ignoreLoops;
      return;
    }
  
    // Handle tilting forward to activate the timer
    if (lowpasses[zAccel] >= tiltEndThreshold && !Timer::isTiming()) {
      Timer::start(Timer::defaultLength);
    } else if (lowpasses[zAccel] >= tiltStartThreshold) {
      Polling::resetPoll(PollType::BONK_DELAY); // Ensure we can't bonk when setting the lamp back down
    }
  
    // Handle bonking
    if (abs(highpasses[yAccel]) >= bonkThreshold) { // Moved enough to trigger a bonk
      if (Polling::checkPoll(PollType::BONK_DELAY)) { // Enough time has passed since the last bonk
        Debug::printText("Sensors: Bonk triggered");
        LED::toggle();
      }
      Polling::resetPoll(PollType::BONK_DELAY);
    }
  }
}

namespace Timer {
  void start(unsigned long millisToWait) {
    LED::confirmationFlash(1);
    Polling::resetPoll(PollType::TIMER_ACTUAL);
    Polling::setPollLength(PollType::TIMER_ACTUAL, millisToWait);
    Debug::printText("Timer: Start");
  }
  
  bool isTiming() {
    return Polling::isActive(PollType::TIMER_ACTUAL);
  }

  void stop() {
    Polling::setPollLength(PollType::TIMER_ACTUAL, 0);
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
    const float cutoff = 2/3.0f;
    return x < cutoff ? tanLightFunc(x / cutoff) : 1;
  }
  
  void update() {
    if (isTiming()) {
      unsigned long diffMillis = Polling::getPollElapsedMillis(PollType::TIMER_ACTUAL);
      const unsigned long lengthMillis = Polling::getPollLengthMillis(PollType::TIMER_ACTUAL);
      Debug::printText("Timer: " + String(diffMillis) + "/" + String(lengthMillis) + " ms");
      if (!Polling::checkPoll(PollType::TIMER_ACTUAL)) { // Still timing
        float frac = timerLightFunc(((float)lengthMillis - diffMillis) / lengthMillis);
        LED::setBrightnessFloat(frac);
      } else {
        Polling::setPollLength(PollType::TIMER_ACTUAL, 0);
      }
    }
  }
  
}

namespace Morse {
  const int morseDelay = 100;
  const String morseTable[] = 
  //A     B       C       D      E    F       G      H       I     J       K      L       M
  {"01", "1000", "1010", "100", "0", "0010", "110", "0000", "00", "0111", "101", "0100", "11",
  //N     O      P       Q       R      S      T    U      V       W      X       Y       Z
   "10", "111", "0110", "1101", "010", "000", "1", "001", "0001", "011", "1001", "1011", "1100",
  //0,       1        2        3        4        5        6        7        8        9
   "11111", "01111", "00111", "00011", "00001", "00000", "10000", "11000", "11100", "11110" };

   void translateString(String str) {
    byte originalState = LED::curBrightness ? 1 : 0;
    LED::confirmationFlash(0);
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
        LED::toggle();
        delay(((morseTable[c].charAt(j) - '0') * 2 + 1) * morseDelay); // Delay 1 unit if dot, 3 if dash
        LED::toggle();
        delay(morseDelay);
  
        if (Button::isPressed()) return; // Abort if the button is pressed
      }
  
      delay(morseDelay * 2); // Delay 3 between letters (including the delay from the for loop)
      
      if (Button::isPressed()) return; // Abort if the button is pressed
    }
  
    delay(1000);
    if(originalState) {
      LED::toggle();
    }
  }
}

namespace Bluetooth {
  BLEService LEDService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service
  
  BLEByteCharacteristic toggleCharacteristic("3A00", BLEWrite);
  BLEByteCharacteristic valueCharacteristic("3A10", BLERead | BLEWrite);
  BLEUnsignedIntCharacteristic timerCharacteristic("3A20", BLEWrite);
  BLEStringCharacteristic morseCharacteristic("3A30", BLEWrite, 256);
  BLEByteCharacteristic gameCharacteristic("3A40", BLERead | BLEWrite);

  // Bluetooth initialization
  void setup() {
    if (!BLE.begin()) {
      Serial.println("Starting BLE failed!");
      LED::errorLoop();
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
  }
  
  void update() {
    // Poll for BLE events
    BLE.poll();
  
    if (toggleCharacteristic.written()) {
      LED::toggle();
    }
  
    if (valueCharacteristic.written()) {
      LED::variableMaxBrightness = valueCharacteristic.value();
      LED::setBrightnessInt(LED::variableMaxBrightness);
    } else if (valueCharacteristic.value() != LED::curBrightness) {
      valueCharacteristic.writeValue(LED::curBrightness);
    }
  
    if (timerCharacteristic.written()) {
      unsigned long millisToWait = (long)timerCharacteristic.value() * 60 * 1000;
      Timer::start(millisToWait);
    }
  
    if (morseCharacteristic.written()) {
      Morse::translateString(morseCharacteristic.value());
    }
  
    if (gameCharacteristic.written()) {
      Game::start();
    }
  }
}

namespace Game {
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
  byte expectedState = 0;

  void start() {
    points = 0;
    Polling::resetPoll(PollType::GAME);
    Polling::setPollLength(PollType::GAME, 1);
    expectedState = 1;
    Debug::printText("Game: Start");
    LED::confirmationFlash(0);
  }

  bool isGaming() {
    return Polling::isActive(PollType::GAME);
  }
  
  void update() {
    if (!isGaming()) {
      return;
    }
    
    if (LED::curBrightness && !expectedState) { // LED turned on while off, game over
      Polling::setPollLength(PollType::GAME, 0);
      Debug::printText("Game: Finished with " + String(points) + " points");
      LED::confirmationFlash(0);
      return;
    } else if (!LED::curBrightness && expectedState) { // LED turned off while on, add point, get harder
      ++points;
      Bluetooth::gameCharacteristic.writeValue(points);
      reactionTime = startReactionTime - reactionTimeChange * points;
      Debug::printText("Game: " + String(points) + " points");
      Debug::printText("Game: " + String(Polling::getPollLengthMillis(PollType::GAME)) + " ms reaction time ");
    } else {
      LED::toggle();
    }
  
    expectedState = LED::curBrightness ? 1 : 0;
    if (expectedState) {
      Polling::setPollLength(PollType::GAME, reactionTime);
    } else {
      Polling::setPollLength(PollType::GAME, random(minDelay, maxDelay));
    }
  
    Debug::printText("Game: Wait for " + String(Polling::getPollLengthMillis(PollType::GAME)) + " ms");
  }
}

namespace Polling {
  unsigned long curMillis;

  struct Data {
    Data() : lastTimeMillis(0), lengthMillis(0), update(NULL) {}
    Data(unsigned long ltm, unsigned long lm, void (*u)()) : lastTimeMillis(ltm), lengthMillis(lm), update(u) {}
    
    unsigned long lastTimeMillis; // Offset some of these by a small amount so they don't all run on the same cycle
    unsigned long lengthMillis;
    void (*update)();
  };

  std::unordered_map<PollType, Data> map = {
  //  Key                           Last time/start time
  //  |                             |   Update frequency
  //  |                             |   |     Function
    { PollType::SENSORS,      Data( 0,  16,   Sensors::update   ) },
    { PollType::BUTTON,       Data( 5,  33,   Button::update    ) },
    { PollType::TIMER,        Data( 10, 33,   Timer::update     ) },
    { PollType::BLUETOOTH,    Data( 15, 200,  Bluetooth::update ) },
    { PollType::GAME,         Data( 0,  0,    Game::update      ) },
    { PollType::BONK_DELAY,   Data( 0,  200,  NULL )              },
    { PollType::BUTTON_HOLD,  Data( 0,  1000, NULL )              },
    { PollType::TIMER_ACTUAL, Data( 0,  0,    NULL )              } 
  };

  unsigned long getPollElapsedMillis(PollType type) {
    return curMillis - map[type].lastTimeMillis;
  }

  unsigned long getPollRemainingMillis(PollType type) {
    return map[type].lengthMillis - getPollElapsedMillis(type);
  }

  unsigned long getPollLengthMillis(PollType type) {
    return map[type].lengthMillis;
  }
  
  bool checkPoll(PollType type) {
    return map[type].lengthMillis > 0 && curMillis - map[type].lastTimeMillis >= map[type].lengthMillis || 
           map[type].lengthMillis == 1; // Run every frame
  }
  
  void resetPoll(PollType type) {
    map[type].lastTimeMillis = curMillis;
  }

  bool isActive(PollType type) {
    return map[type].lengthMillis > 0;
  }
  
  void setPollLength(PollType type, const unsigned long millis) {
    map[type].lengthMillis = millis;
  }
  
  void awaitFunctionPoll(PollType type) {
    if (checkPoll(type)) {
      map[type].update();
      resetPoll(type);
    }
  }
}

//////////
// Main //
//////////
void setup() {
  Debug::setup();
  LED::setup();
  Button::setup();
  Sensors::setup();
  Bluetooth::setup();
}

void loop() {
  Polling::curMillis = millis();
  Polling::awaitFunctionPoll(PollType::BUTTON);
  if (!Button::lastButtonState) { // Don't check anything else if the button is held
    Polling::awaitFunctionPoll(PollType::SENSORS);
    Polling::awaitFunctionPoll(PollType::TIMER);
    Polling::awaitFunctionPoll(PollType::BLUETOOTH);
    Polling::awaitFunctionPoll(PollType::GAME);
  }
  delay(10);
}

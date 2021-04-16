const int ledPin = 3; // pin to use for the LED
const int buttonPin = 5;
byte state = 0;
byte previousButton = 0;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  byte curButton = digitalRead(buttonPin);

  if(curButton == LOW && previousButton == HIGH) {
    state = 1 - state;
    digitalWrite(ledPin, state);
  }

  previousButton = curButton;
  delay(100);
}

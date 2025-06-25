#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define FREQUENCY 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pins & channels
const int potPins[4] = {A0, A1, A2, A3};
const int servoChannels[4] = {5, 4, 3, 2};
bool reversed[4] = {true, true, false, true};

const int buttonPin = 8;
const int ledPin = 10;
const int clawChannel = 0;
const int oscillateChannel = 1;

// Modes
enum Mode { LIVE, OSCILLATE, RECORD, PLAYBACK };
Mode currentMode = LIVE;

// Click tracking
unsigned long lastClickTime = 0;
int clickCount = 0;
const unsigned long clickTimeout = 600;

// Oscillation
int oscAngle = 0;
int oscDirection = 1;
unsigned long lastOscTime = 0;

// LED blink
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Playback recording
const int maxFrames = 200; // 10s @ 50ms
byte recordedAngles[maxFrames][4];
int frameCount = 0;
int playbackIndex = 0;
unsigned long lastFrameTime = 0;

// Claw
bool clawOpen = false;

// Button state
bool prevButtonState = HIGH;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(10);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    setServoAngle(servoChannels[i], 90);
  }
  setServoAngle(clawChannel, 0);
  setServoAngle(oscillateChannel, 0);
}

void loop() {
  bool currButtonState = digitalRead(buttonPin);
  unsigned long currentTime = millis();

  // === Detect button press ===
  if (currButtonState == LOW && prevButtonState == HIGH) {
    clickCount++;
    lastClickTime = currentTime;
  }
  prevButtonState = currButtonState;

  // === Handle click timeout ===
  if (clickCount > 0 && (currentTime - lastClickTime > clickTimeout)) {
    handleClicks(clickCount);
    clickCount = 0;
  }

  // === Mode logic ===
  switch (currentMode) {
    case LIVE:
      digitalWrite(ledPin, LOW);
      for (int i = 0; i < 4; i++) {
        int val = analogRead(potPins[i]);
        val = constrain(val, 180, 862);
        int angle = reversed[i] ? map(val, 180, 862, 180, 0) : map(val, 180, 862, 0, 180);
        setServoAngle(servoChannels[i], angle);
      }
      break;

    case OSCILLATE:
  // Continue updating 3 servos from pot values (skip oscillating one)
  for (int i = 0; i < 4; i++) {
    if (servoChannels[i] == oscillateChannel) continue;  // Skip the oscillating one

    int val = analogRead(potPins[i]);
    val = constrain(val, 180, 862);
    int angle = reversed[i] ? map(val, 180, 862, 180, 0) : map(val, 180, 862, 0, 180);
    setServoAngle(servoChannels[i], angle);
  }

  // Oscillate servoChannel 1
  if (currentTime - lastOscTime > 30) {
    oscAngle += oscDirection;
    setServoAngle(oscillateChannel, oscAngle);
    lastOscTime = currentTime;

    if (oscAngle >= 90) oscDirection = -1;
    if (oscAngle <= 0) oscDirection = 1;
  }

  digitalWrite(ledPin, HIGH);
  break;


    case RECORD:
      if (currentTime - lastFrameTime > 50 && frameCount < maxFrames) {
        for (int i = 0; i < 4; i++) {
          int val = analogRead(potPins[i]);
          val = constrain(val, 180, 862);
          byte angle = reversed[i] ? map(val, 180, 862, 180, 0) : map(val, 180, 862, 0, 180);
          recordedAngles[frameCount][i] = angle;
        }
        frameCount++;
        lastFrameTime = currentTime;
      }

      // End recording after 10s
      if (frameCount >= maxFrames) {
        currentMode = PLAYBACK;
        playbackIndex = 0;
        lastFrameTime = currentTime;
        Serial.println(">> RECORDING COMPLETE. PLAYBACK STARTED.");
      }

      // Blink LED
      if (currentTime - lastBlinkTime > 300) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastBlinkTime = currentTime;
      }
      break;

    case PLAYBACK:
      if (currentTime - lastFrameTime > 50 && playbackIndex < frameCount) {
        for (int i = 0; i < 4; i++) {
          setServoAngle(servoChannels[i], recordedAngles[playbackIndex][i]);
        }
        playbackIndex++;
        lastFrameTime = currentTime;
      }

      // Blink LED
      if (currentTime - lastBlinkTime > 300) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastBlinkTime = currentTime;
      }

      if (playbackIndex >= frameCount) {
        currentMode = LIVE;
        digitalWrite(ledPin, LOW);
        Serial.println(">> PLAYBACK COMPLETE. RETURN TO LIVE.");
      }
      break;
  }
}

// === Click behavior handler ===
void handleClicks(int count) {
  Serial.print("Clicks detected: "); Serial.println(count);

  switch (count) {
    case 1:  // Toggle claw
      clawOpen = !clawOpen;
      setServoAngle(clawChannel, clawOpen ? 120 : 0);
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      break;

    case 2:  // Toggle oscillation
      if (currentMode == OSCILLATE) {
        currentMode = LIVE;
        digitalWrite(ledPin, LOW);
        Serial.println(">> Oscillation OFF");
      } else {
        currentMode = OSCILLATE;
        Serial.println(">> Oscillation ON");
      }
      break;

    case 3:  // Start recording
      currentMode = RECORD;
      frameCount = 0;
      lastFrameTime = millis();
      Serial.println(">> RECORDING STARTED");
      break;

    default:
      Serial.println(">> Invalid click combo");
      break;
  }
}

// === Servo angle control ===
void setServoAngle(int channel, int angle) {
  int pulseMicro = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  int pulsePWM = int(float(pulseMicro) / 1000000 * FREQUENCY * 4096);
  pwm.setPWM(channel, 0, pulsePWM);
}



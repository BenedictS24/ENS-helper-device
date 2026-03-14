#include "HX711.h"
#include <math.h>

// -------------------------------------------------------------
// Pins
// -------------------------------------------------------------
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN  = 7;
const int MOTOR_PIN         = 9;

const int BUTTON_PIN = 2;
const int LED_PIN    = 3;

// -------------------------------------------------------------
// Button / LED settings
// -------------------------------------------------------------
const unsigned long debounceDelay = 30;
const int blinkOnTime  = 200;
const int blinkOffTime = 200;

int lastReading = HIGH;
int buttonState = HIGH;
unsigned long lastDebounceTime = 0;

// -------------------------------------------------------------
// HX711 / calibration
// -------------------------------------------------------------
HX711 scale;

long  zeroOffset  = 102000;   // raw reading at no load
float scaleFactor = 1000.0;   // raw units per gram

// -------------------------------------------------------------
// Feedback modes
// -------------------------------------------------------------
enum FeedbackMode {
  MODE_ABSOLUTE = 0,
  MODE_RATE     = 1
};

FeedbackMode currentMode = MODE_ABSOLUTE;

// ----- Absolute-force mode settings ---------------------------
float minWeight = 0.0;        // grams -> no vibration below this
float maxWeight = 2000.0;     // grams -> max vibration at/above this

// ----- Rate-of-change mode settings ---------------------------
float lastWeight = 0.0;       // grams
unsigned long lastTime = 0;   // ms

float maxRate = 200.0;        // g/s mapped to max vibration
float minRate = 0.0;          // vibration starts above this threshold

float rateFiltered = 0.0;
float smoothing = 0.1;        // exponential smoothing factor

// -------------------------------------------------------------
// Helper: float mapping to PWM range
// -------------------------------------------------------------
int mapFloatToPWM(float x, float in_min, float in_max) {
  if (in_max == in_min) {
    return 0;
  }

  float result = (x - in_min) * 255.0 / (in_max - in_min);

  if (result < 0.0) {
    result = 0.0;
  }
  if (result > 255.0) {
    result = 255.0;
  }

  return (int)result;
}

// -------------------------------------------------------------
// Helper: read current weight in grams
// -------------------------------------------------------------
float getWeight() {
  long raw = scale.read();
  return (raw - zeroOffset) / scaleFactor;
}

// -------------------------------------------------------------
// LED feedback for current mode
// 1 blink  = absolute
// 2 blinks = rate
// -------------------------------------------------------------
void blinkNTimes(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(blinkOnTime);

    digitalWrite(LED_PIN, LOW);
    delay(blinkOffTime);
  }
}

void indicateCurrentMode() {
  if (currentMode == MODE_ABSOLUTE) {
    blinkNTimes(1);
  } else if (currentMode == MODE_RATE) {
    blinkNTimes(2);
  }
}

// -------------------------------------------------------------
// Switch to next mode
// -------------------------------------------------------------
void nextMode() {
  if (currentMode == MODE_ABSOLUTE) {
    currentMode = MODE_RATE;
  } else {
    currentMode = MODE_ABSOLUTE;
  }

  // Reset rate calculation when entering/changing mode
  lastTime = millis();
  lastWeight = getWeight();
  rateFiltered = 0.0;

  indicateCurrentMode();
}

// -------------------------------------------------------------
// Setup
// -------------------------------------------------------------
void setup() {
  Serial.begin(57600);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize rate-of-change reference
  lastTime = millis();
  if (scale.is_ready()) {
    lastWeight = getWeight();
  } else {
    lastWeight = 0.0;
  }

  // Show startup mode
  indicateCurrentMode();
}

// -------------------------------------------------------------
// Main loop
// -------------------------------------------------------------
void loop() {
  // -----------------------------------------------------------
  // Button handling with debounce
  // -----------------------------------------------------------
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // Button pressed
      if (buttonState == LOW) {
        nextMode();
      }
    }
  }

  lastReading = reading;

  // -----------------------------------------------------------
  // HX711 check
  // -----------------------------------------------------------
  if (!scale.is_ready()) {
    Serial.println("HX711 not ready.");
    analogWrite(MOTOR_PIN, 0);
    delay(100);
    return;
  }

  // -----------------------------------------------------------
  // Read weight
  // -----------------------------------------------------------
  float weight = getWeight();
  int pwm = 0;

  // -----------------------------------------------------------
  // Mode 1: ABSOLUTE FORCE
  // -----------------------------------------------------------
  if (currentMode == MODE_ABSOLUTE) {
    float w = constrain(weight, minWeight, maxWeight);
    pwm = mapFloatToPWM(w, minWeight, maxWeight);

    analogWrite(MOTOR_PIN, pwm);

    Serial.print("Mode: ABSOLUTE");
    Serial.print('\t');
    Serial.print("Weight: ");
    Serial.print(weight, 2);
    Serial.print('\t');
    Serial.print("PWM: ");
    Serial.println(pwm);
  }

  // -----------------------------------------------------------
  // Mode 2: RATE OF CHANGE
  // -----------------------------------------------------------
  else if (currentMode == MODE_RATE) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    if (dt <= 0.0) {
      dt = 0.001;
    }

    float rate = (weight - lastWeight) / dt;

    // Stabilize signal
    rateFiltered = smoothing * rate + (1.0 - smoothing) * rateFiltered;

    float r = fabs(rateFiltered);
    r = constrain(r, minRate, maxRate);

    pwm = mapFloatToPWM(r, minRate, maxRate);

    analogWrite(MOTOR_PIN, pwm);

    Serial.print("Mode: RATE");
    Serial.print('\t');
    Serial.print("Weight: ");
    Serial.print(weight, 2);
    Serial.print('\t');
    Serial.print("Rate: ");
    Serial.print(rateFiltered, 2);
    Serial.print('\t');
    Serial.print("PWM: ");
    Serial.println(pwm);

    lastWeight = weight;
    lastTime = now;
  }

  delay(100);
}
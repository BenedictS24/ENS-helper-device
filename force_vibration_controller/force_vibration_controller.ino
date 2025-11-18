#include "HX711.h"
#include <math.h>

// -------------------------------------------------------------
// Configuration
// -------------------------------------------------------------

// Set mode to either:
//   "absolute"      → vibration based on total force
//   "rate"          → vibration based on rate-of-change
String mode = "absolute";   // "absolute" or "rate"

// HX711 pins
const int PIN_DOUT = 7;
const int PIN_SCK  = 9;

// Vibration motor
const int MOTOR_PIN = 5;

// Calibration values
long  zeroOffset  = 102000;   // raw reading at no load
float scaleFactor = 1000.0;   // raw units per gram

HX711 scale;

// ----- Absolute-force mode settings --------------------------------
float minWeight = 0.0;        // grams → no vibration below this
float maxWeight = 2000.0;     // grams → max vibration at/above this

// ----- Rate-of-change mode settings -------------------------------
float lastWeight = 0.0;       // grams
unsigned long lastTime = 0;   // ms

float maxRate = 200.0;        // g/s mapped to max vibration
float minRate = 0.0;          // vibration starts above this threshold

float rateFiltered = 0.0;
float smoothing = 0.1;        // exponential smoothing factor


// -------------------------------------------------------------
// Setup
// -------------------------------------------------------------
void setup() {
  Serial.begin(57600);

  scale.begin(PIN_DOUT, PIN_SCK);
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

  // Initialize rate-of-change reference
  lastTime = millis();
  long raw = scale.read();
  lastWeight = (raw - zeroOffset) / scaleFactor;
}


// -------------------------------------------------------------
// Helper: read current weight in grams
// -------------------------------------------------------------
float getWeight() {
  long raw = scale.read();
  return (raw - zeroOffset) / scaleFactor;
}


// -------------------------------------------------------------
// Main loop
// -------------------------------------------------------------
void loop() {
  if (!scale.is_ready()) {
    Serial.println("HX711 not ready.");
    analogWrite(MOTOR_PIN, 0);
    delay(100);
    return;
  }

  float weight = getWeight();
  int pwm = 0;   // motor output (0–255)

  // -------------------------------------------------------------
  // Mode: ABSOLUTE FORCE
  // -------------------------------------------------------------
  if (mode == "absolute") {

    Serial.println(weight);

    float w = constrain(weight, minWeight, maxWeight);
    pwm = map(w, minWeight, maxWeight, 0, 255);
    analogWrite(MOTOR_PIN, pwm);
  }

  // -------------------------------------------------------------
  // Mode: RATE OF CHANGE
  // -------------------------------------------------------------
  else if (mode == "rate") {

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;

    float rate = (weight - lastWeight) / dt;

    // Stabilize signal
    rateFiltered = smoothing * rate + (1.0 - smoothing) * rateFiltered;

    float r = fabs(rateFiltered);
    r = constrain(r, minRate, maxRate);

    pwm = map(r, minRate, maxRate, 0, 255);
    analogWrite(MOTOR_PIN, pwm);

    // For serial plotter: weight, rate, pwm
    Serial.print(weight, 2);
    Serial.print('\t');
    Serial.print(rateFiltered, 2);
    Serial.print('\t');
    Serial.println(pwm);

    lastWeight = weight;
    lastTime = now;
  }

  // -------------------------------------------------------------
  // Unknown mode
  // -------------------------------------------------------------
  else {
    Serial.println("Unknown mode, motor off.");
    analogWrite(MOTOR_PIN, 0);
  }

  delay(100);
}

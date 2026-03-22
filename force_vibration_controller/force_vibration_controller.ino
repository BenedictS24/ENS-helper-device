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
const int PIN_SCK = 9;

// Vibration motor
const int MOTOR_PIN = 5;

// Calibration values
long zero_offset = 102000;   // raw reading at no load
float scale_factor = 1000.0;   // raw units per gram

HX711 scale;

// ----- Absolute-force mode settings --------------------------------
float min_weight = 0.0;        // grams → no vibration below this
float max_weight = 2000.0;     // grams → max vibration at/above this

// ----- Rate-of-change mode settings -------------------------------
float last_weight = 0.0;       // grams
unsigned long last_time = 0;   // ms

float max_rate = 200.0;        // g/s mapped to max vibration
float min_rate = 0.0;          // vibration starts above this threshold

float rate_filtered = 0.0;
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
  last_time = millis();
  long raw = scale.read();
  last_weight = (raw - zero_offset) / scale_factor;
}


// -------------------------------------------------------------
// Helper: read current weight in grams
// -------------------------------------------------------------
float get_weight() {
  long raw = scale.read();
  return (raw - zero_offset) / scale_factor;
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

  float weight = get_weight();
  int pwm = 0;   // motor output (0–255)

  // -------------------------------------------------------------
  // Mode: ABSOLUTE FORCE
  // -------------------------------------------------------------
  if (mode == "absolute") {

    Serial.println(weight);

    float w = constrain(weight, min_weight, max_weight);
    pwm = map(w, min_weight, max_weight, 0, 255);
    analogWrite(MOTOR_PIN, pwm);
  }

  // -------------------------------------------------------------
  // Mode: RATE OF CHANGE
  // -------------------------------------------------------------
  else if (mode == "rate") {

    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    if (dt <= 0) dt = 0.001;

    float rate = (weight - last_weight) / dt;

    // Stabilize signal
    rate_filtered = smoothing * rate + (1.0 - smoothing) * rate_filtered;

    float r = fabs(rate_filtered);
    r = constrain(r, min_rate, max_rate);

    pwm = map(r, min_rate, max_rate, 0, 255);
    analogWrite(MOTOR_PIN, pwm);

    // For serial plotter: weight, rate, pwm
    Serial.print(weight, 2);
    Serial.print('\t');
    Serial.print(rate_filtered, 2);
    Serial.print('\t');
    Serial.println(pwm);

    last_weight = weight;
    last_time = now;
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

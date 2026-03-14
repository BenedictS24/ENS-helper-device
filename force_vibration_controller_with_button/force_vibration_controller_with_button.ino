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

long  zeroOffset  = 102000;
float scaleFactor = 1000.0;

// -------------------------------------------------------------
// Feedback modes
// -------------------------------------------------------------
enum FeedbackMode {
  MODE_ABSOLUTE        = 0,
  MODE_RATE            = 1,
  MODE_NO_BREATH_ALERT = 2
};

FeedbackMode currentMode = MODE_ABSOLUTE;

// -------------------------------------------------------------
// Absolute force mode settings
// -------------------------------------------------------------
float minWeight = 0.0;
float maxWeight = 2000.0;

// -------------------------------------------------------------
// Rate of change settings
// -------------------------------------------------------------
unsigned long lastTime = 0;
float lastWeightFiltered = 0.0;
float rateFiltered = 0.0;

float weightSmoothing = 0.12;
float rateSmoothing   = 0.10;

bool filterInitialized = false;

float maxRate = 200.0;
float minRate = 0.0;

// -------------------------------------------------------------
// No breath alert mode settings
// -------------------------------------------------------------
unsigned long lastBreathDetectedTime = 0;
unsigned long noBreathTimeout = 5000;

float breathDetectThreshold = 1.5;

// Pulsing behavior
unsigned long pulsePeriod = 1600;       // time from one pulse start to the next
unsigned long pulseOnTime = 250;        // pulse duration
unsigned long pulseRampTime = 10000;    // strength ramps up over this time

int alertMinPWM = 60;
int alertMaxPWM = 160;

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
// 3 blinks = no breath alert
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
  } else if (currentMode == MODE_NO_BREATH_ALERT) {
    blinkNTimes(3);
  }
}

// -------------------------------------------------------------
// Reset dynamic signal state
// -------------------------------------------------------------
void resetSignalState() {
  lastTime = millis();

  if (scale.is_ready()) {
    float w = getWeight();
    lastWeightFiltered = w;
  } else {
    lastWeightFiltered = 0.0;
  }

  rateFiltered = 0.0;
  filterInitialized = true;
  lastBreathDetectedTime = millis();
}

// -------------------------------------------------------------
// Switch to next mode
// -------------------------------------------------------------
void nextMode() {
  if (currentMode == MODE_ABSOLUTE) {
    currentMode = MODE_RATE;
  } else if (currentMode == MODE_RATE) {
    currentMode = MODE_NO_BREATH_ALERT;
  } else {
    currentMode = MODE_ABSOLUTE;
  }

  resetSignalState();
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

  resetSignalState();
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
    analogWrite(MOTOR_PIN, 0);

    Serial.print(0.0, 2);
    Serial.print('\t');
    Serial.println(0);

    delay(100);
    return;
  }

  // -----------------------------------------------------------
  // Read and smooth signal
  // -----------------------------------------------------------
  unsigned long now = millis();
  float rawWeight = getWeight();
  float forceAbs = fabs(rawWeight);

  if (!filterInitialized) {
    lastWeightFiltered = rawWeight;
    lastTime = now;
    rateFiltered = 0.0;
    filterInitialized = true;
    lastBreathDetectedTime = now;
  }

  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0.0) {
    dt = 0.001;
  }

  float weightFiltered = weightSmoothing * rawWeight
                       + (1.0 - weightSmoothing) * lastWeightFiltered;

  float rawRate = (weightFiltered - lastWeightFiltered) / dt;

  rateFiltered = rateSmoothing * rawRate
               + (1.0 - rateSmoothing) * rateFiltered;

  float activityAbs = fabs(rateFiltered);

  if (activityAbs > breathDetectThreshold) {
    lastBreathDetectedTime = now;
  }

  lastWeightFiltered = weightFiltered;
  lastTime = now;

  int pwm = 0;

  // -----------------------------------------------------------
  // Mode 1: absolute force
  // -----------------------------------------------------------
  if (currentMode == MODE_ABSOLUTE) {
    float forceForMotor = fabs(weightFiltered);
    float w = constrain(forceForMotor, minWeight, maxWeight);
    pwm = mapFloatToPWM(w, minWeight, maxWeight);
    analogWrite(MOTOR_PIN, pwm);
  }

  // -----------------------------------------------------------
  // Mode 2: rate of change
  // -----------------------------------------------------------
  else if (currentMode == MODE_RATE) {
    float r = constrain(activityAbs, minRate, maxRate);
    pwm = mapFloatToPWM(r, minRate, maxRate);
    analogWrite(MOTOR_PIN, pwm);
  }

  // -----------------------------------------------------------
  // Mode 3: no breath alert with slow pulses
  // -----------------------------------------------------------
  else if (currentMode == MODE_NO_BREATH_ALERT) {
    unsigned long quietTime = now - lastBreathDetectedTime;

    if (quietTime < noBreathTimeout) {
      pwm = 0;
    } else {
      unsigned long alertElapsed = quietTime - noBreathTimeout;

      float rampProgress = (float)alertElapsed / (float)pulseRampTime;
      if (rampProgress > 1.0) {
        rampProgress = 1.0;
      }

      int pulseStrength = alertMinPWM + (int)((alertMaxPWM - alertMinPWM) * rampProgress);

      unsigned long pulsePhase = alertElapsed % pulsePeriod;

      if (pulsePhase < pulseOnTime) {
        pwm = pulseStrength;
      } else {
        pwm = 0;
      }
    }

    analogWrite(MOTOR_PIN, pwm);
  }

  // -----------------------------------------------------------
  // Serial Plotter output
  // 1) absolute force
  // 2) motor power
  // -----------------------------------------------------------
  Serial.print(forceAbs, 2);
  Serial.print('\t');
  Serial.println(pwm);

  delay(100);
}
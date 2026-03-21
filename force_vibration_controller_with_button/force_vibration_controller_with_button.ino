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

// Minimum PWM to help the motor overcome stiction when enabled
const int MIN_MOTOR_PWM = 45;

// -------------------------------------------------------------
// Button / LED settings
// -------------------------------------------------------------
const unsigned long debounceDelay     = 40;
const unsigned long longPressDuration = 5000;

const int blinkOnTime  = 250;
const int blinkOffTime = 250;

int lastReading = HIGH;
int buttonState = HIGH;
unsigned long lastDebounceTime = 0;

unsigned long buttonPressStartTime = 0;
bool longPressHandled = false;

// -------------------------------------------------------------
// HX711 / calibration
// -------------------------------------------------------------
HX711 scale;

long  zeroOffset  = 102000;
float scaleFactor = 1000.0;

// -------------------------------------------------------------
// Dynamic calibration of min / max readings
// -------------------------------------------------------------
float calibratedMin = -150.0;
float calibratedMax =  150.0;

float previousCalibratedMin = -150.0;
float previousCalibratedMax =  150.0;

bool isCalibrating = false;
unsigned long calibrationStartTime = 0;
const unsigned long calibrationDuration = 10000;

float calibrationMinReading = 0.0;
float calibrationMaxReading = 0.0;
bool calibrationHasReading = false;

const float minCalibrationSpan = 30.0;

unsigned long lastCalibrationLedToggle = 0;
bool calibrationLedState = false;
const unsigned long calibrationLedInterval = 120;

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
// Absolute mode
// Uses calibratedMin ... calibratedMax directly
// -------------------------------------------------------------

// -------------------------------------------------------------
// Rate of change mode
// Uses normalized position within calibrated span
// -------------------------------------------------------------
unsigned long lastTime = 0;
float lastWeightFiltered = 0.0;
float lastNormalizedWeight = 0.0;
float rateFiltered = 0.0;

float weightSmoothing = 0.12;
float rateSmoothing   = 0.10;

bool filterInitialized = false;

// normalized units per second
float minRate = 0.0;
float maxRate = 1.2;

// -------------------------------------------------------------
// No breath alert mode
// Breath detection is also based on normalized rate
// -------------------------------------------------------------
unsigned long lastBreathDetectedTime = 0;
unsigned long noBreathTimeout = 5000;

// normalized units per second
float breathDetectThreshold = 0.03;

// Pulsing behavior
unsigned long pulsePeriod   = 1600;
unsigned long pulseOnTime   = 250;
unsigned long pulseRampTime = 10000;

int alertMinPWM = 60;
int alertMaxPWM = 160;

// -------------------------------------------------------------
// Helper
// -------------------------------------------------------------
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int mapFloatToPWM(float x, float in_min, float in_max) {
  if (in_max == in_min) {
    return 0;
  }

  float result = (x - in_min) * 255.0 / (in_max - in_min);
  result = clampFloat(result, 0.0, 255.0);

  return (int)result;
}

// Keep motor off at 0, but enforce a minimum drive once enabled
int applyMotorFloor(int pwm) {
  if (pwm <= 0) {
    return 0;
  }

  if (pwm < MIN_MOTOR_PWM) {
    return MIN_MOTOR_PWM;
  }

  if (pwm > 255) {
    return 255;
  }

  return pwm;
}

// -------------------------------------------------------------
// Read current weight in grams
// -------------------------------------------------------------
float getWeight() {
  long raw = scale.read();
  return (raw - zeroOffset) / scaleFactor;
}

// -------------------------------------------------------------
// Convert current reading to normalized range 0..1
// based on calibrated min / max
// -------------------------------------------------------------
float getNormalizedWeight(float weight) {
  float span = calibratedMax - calibratedMin;

  if (span < minCalibrationSpan) {
    span = minCalibrationSpan;
  }

  float normalized = (weight - calibratedMin) / span;
  return clampFloat(normalized, 0.0, 1.0);
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
    lastNormalizedWeight = getNormalizedWeight(w);
  } else {
    lastWeightFiltered = 0.0;
    lastNormalizedWeight = 0.0;
  }

  rateFiltered = 0.0;
  filterInitialized = true;
  lastBreathDetectedTime = millis();
}

// -------------------------------------------------------------
// Start calibration
// -------------------------------------------------------------
void startCalibration() {
  previousCalibratedMin = calibratedMin;
  previousCalibratedMax = calibratedMax;

  isCalibrating = true;
  calibrationStartTime = millis();
  calibrationHasReading = false;

  calibrationMinReading = 0.0;
  calibrationMaxReading = 0.0;

  analogWrite(MOTOR_PIN, 0);

  calibrationLedState = true;
  lastCalibrationLedToggle = millis();
  digitalWrite(LED_PIN, HIGH);

  filterInitialized = false;

  Serial.println("CALIBRATION_START");
}

// -------------------------------------------------------------
// Finish calibration
// -------------------------------------------------------------
void finishCalibration() {
  isCalibrating = false;
  digitalWrite(LED_PIN, LOW);

  if (calibrationHasReading) {
    float span = calibrationMaxReading - calibrationMinReading;

    if (span >= minCalibrationSpan) {
      calibratedMin = calibrationMinReading;
      calibratedMax = calibrationMaxReading;
    } else {
      calibratedMin = previousCalibratedMin;
      calibratedMax = previousCalibratedMax;
    }
  } else {
    calibratedMin = previousCalibratedMin;
    calibratedMax = previousCalibratedMax;
  }

  resetSignalState();

  Serial.print("CALIBRATION_DONE\tMIN=");
  Serial.print(calibratedMin, 2);
  Serial.print("\tMAX=");
  Serial.println(calibratedMax, 2);

  indicateCurrentMode();
}

// -------------------------------------------------------------
// Update calibration LED
// -------------------------------------------------------------
void updateCalibrationLED(unsigned long now) {
  if (now - lastCalibrationLedToggle >= calibrationLedInterval) {
    calibrationLedState = !calibrationLedState;
    digitalWrite(LED_PIN, calibrationLedState ? HIGH : LOW);
    lastCalibrationLedToggle = now;
  }
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

  // Optional:
  // Wenn du bei jedem Einschalten direkt neu kalibrieren willst,
  // kannst du die nächste Zeile einkommentieren.
  // startCalibration();
}

// -------------------------------------------------------------
// Main loop
// -------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // -----------------------------------------------------------
  // Button handling with debounce
  // Short press  = next mode
  // Long press   = recalibration
  // -----------------------------------------------------------
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        buttonPressStartTime = now;
        longPressHandled = false;
      } else {
        if (!longPressHandled && !isCalibrating) {
          nextMode();
        }
      }
    }
  }

  lastReading = reading;

  if (buttonState == LOW && !longPressHandled && !isCalibrating) {
    if ((now - buttonPressStartTime) >= longPressDuration) {
      startCalibration();
      longPressHandled = true;
    }
  }

  // -----------------------------------------------------------
  // Calibration mode
  // Collect fresh min / max for calibrationDuration
  // -----------------------------------------------------------
  if (isCalibrating) {
    analogWrite(MOTOR_PIN, 0);
    updateCalibrationLED(now);

    if (scale.is_ready()) {
      float rawWeight = getWeight();

      if (!calibrationHasReading) {
        calibrationMinReading = rawWeight;
        calibrationMaxReading = rawWeight;
        calibrationHasReading = true;
      } else {
        if (rawWeight < calibrationMinReading) calibrationMinReading = rawWeight;
        if (rawWeight > calibrationMaxReading) calibrationMaxReading = rawWeight;
      }

      Serial.print(rawWeight, 2);
      Serial.print('\t');
      Serial.println(0);
    } else {
      Serial.print(0.0, 2);
      Serial.print('\t');
      Serial.println(0);
    }

    if ((now - calibrationStartTime) >= calibrationDuration) {
      finishCalibration();
    }

    delay(20);
    return;
  }

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
  float rawWeight = getWeight();
  float forceAbs = fabs(rawWeight);

  if (!filterInitialized) {
    lastWeightFiltered = rawWeight;
    lastNormalizedWeight = getNormalizedWeight(rawWeight);
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

  float normalizedWeight = getNormalizedWeight(weightFiltered);

  float rawRate = (normalizedWeight - lastNormalizedWeight) / dt;

  rateFiltered = rateSmoothing * rawRate
               + (1.0 - rateSmoothing) * rateFiltered;

  float activityAbs = fabs(rateFiltered);

  if (activityAbs > breathDetectThreshold) {
    lastBreathDetectedTime = now;
  }

  lastWeightFiltered = weightFiltered;
  lastNormalizedWeight = normalizedWeight;
  lastTime = now;

  int pwm = 0;

  // -----------------------------------------------------------
  // Mode 1: absolute force
  // Scale linearly between calibratedMin and calibratedMax
  // -----------------------------------------------------------
  if (currentMode == MODE_ABSOLUTE) {
    pwm = mapFloatToPWM(weightFiltered, calibratedMin, calibratedMax);
    pwm = applyMotorFloor(pwm);
    analogWrite(MOTOR_PIN, pwm);
  }

  // -----------------------------------------------------------
  // Mode 2: rate of change
  // Uses normalized rate, therefore adapts to calibration span
  // -----------------------------------------------------------
  else if (currentMode == MODE_RATE) {
    float r = clampFloat(activityAbs, minRate, maxRate);
    pwm = mapFloatToPWM(r, minRate, maxRate);
    pwm = applyMotorFloor(pwm);
    analogWrite(MOTOR_PIN, pwm);
  }

  // -----------------------------------------------------------
  // Mode 3: no breath alert with slow pulses
  // Breath detection based on normalized activity
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

    pwm = applyMotorFloor(pwm);
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
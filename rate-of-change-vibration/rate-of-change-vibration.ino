#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 6;
const int LOADCELL_SCK_PIN  = 7;

// motor control pin
const int MOTOR_PIN = 9;

HX711 scale;

// calibration
long   zeroOffset    = 102000;  // raw reading at “no load”
float  scaleFactor   = 1000.0;  // raw units per gram

// for rate calculation
float  lastWeight    = 0.0;     // grams
unsigned long lastTime = 0;     // ms

// parameters for mapping rate → PWM
float  maxRate       = 200.0;   // maximum expected rate (g/s)
float  minRate       = 0.0;     // zero vibration below this

// low-pass filter for rate (to smooth spikes)
float  rateFiltered  = 0.0;
float  alpha         = 0.2;     // smoothing factor (0 = max smoothing, 1 = no smoothing)

void setup() {
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

  // initialize timing & first weight
  lastTime   = millis();
  lastWeight = (scale.read() - zeroOffset) / scaleFactor;
}

void loop() {
  if (!scale.is_ready()) {
    // If the HX711 isn't responding, just shut the motor off
    analogWrite(MOTOR_PIN, 0);
    delay(100);
    return;
  }

  // 1) read & convert to grams
  long raw    = scale.read();
  float grams = (raw - zeroOffset) / scaleFactor;

  // 2) compute rate (g per second)
  unsigned long now   = millis();
  float          dt    = (now - lastTime) / 1000.0;      // seconds since last loop
  float          rawRate = (grams - lastWeight) / dt;   // g/s

  // 3) smooth the rate with an exponential filter
  rateFiltered = alpha * rawRate + (1.0 - alpha) * rateFiltered;

  // 4) map absolute rate → PWM (0–255)
  float rateAbs = fabs(rateFiltered);
  int   pwm     = map(
    constrain((int)rateAbs, (int)minRate, (int)maxRate),
    (int)minRate, (int)maxRate,
    0, 255
  );

  // 5) drive motor
  analogWrite(MOTOR_PIN, pwm);

  // 6) print for Serial Plotter: weight [g], rate [g/s], pwm [0–255]
  Serial.print(grams, 2);
  Serial.print('\t');
  Serial.print(rateFiltered, 2);
  Serial.print('\t');
  Serial.println(pwm);

  // 7) save for next iteration
  lastWeight = grams;
  lastTime   = now;

  delay(100);
}

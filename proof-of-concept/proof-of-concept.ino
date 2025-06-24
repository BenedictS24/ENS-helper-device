#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 6;
const int LOADCELL_SCK_PIN  = 7;

// motor control pin
const int MOTOR_PIN = 9;

HX711 scale;

// these two you’ll need to tweak to suit your setup:
long   zeroOffset    = 102000;  // raw reading at “no load”
float  scaleFactor   = 1000.0;  // raw units per gram

// map these weight values (in grams) to 0–255
float  minWeight     =   0;    // motor off below this
float  maxWeight     = 2000;   // full-blast at this or above

void setup() {
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);     // motor off to start
}

void loop() {
  if (scale.is_ready()) {
    long raw = scale.read();
    Serial.println(raw);

    // convert raw → grams
    float grams = (raw - zeroOffset) / scaleFactor;

    // map grams → PWM 0–255
    int pwm = map(constrain((int)grams, (int)minWeight, (int)maxWeight),
                  (int)minWeight, (int)maxWeight,
                  0, 255);

    analogWrite(MOTOR_PIN, pwm);
  }
  else {
    Serial.println("HX711 not found.");
    analogWrite(MOTOR_PIN, 0);   // safe fallback
  }

  delay(100);
}

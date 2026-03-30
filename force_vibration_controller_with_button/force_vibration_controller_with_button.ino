#include "HX711.h"
#include <math.h>

// -------------------------------------------------------------
// Pins
// -------------------------------------------------------------
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 7;
const int MOTOR_PIN = 9;

const int BUTTON_PIN = 2;
const int LED_PIN = 3;

// Minimum PWM to help the motor overcome stiction when enabled
const int MIN_MOTOR_PWM = 10;

// -------------------------------------------------------------
// Motor signal for calibration mode
// -------------------------------------------------------------
const int CALIBRATION_SIGNAL_PWM = 110;
const int CALIBRATION_SIGNAL_ON_TIME = 120;
const int CALIBRATION_SIGNAL_OFF_TIME = 100;

// -------------------------------------------------------------
// Button / LED settings
// -------------------------------------------------------------
const unsigned long debounce_delay = 40;
const unsigned long long_press_duration = 5000;

const int blink_on_time = 250;
const int blink_off_time = 250;

int last_reading = HIGH;
int button_state = HIGH;
unsigned long last_debounce_time = 0;

unsigned long button_press_start_time = 0;
bool long_press_handled = false;

// -------------------------------------------------------------
// HX711 / calibration
// -------------------------------------------------------------
HX711 scale;

long zero_offset = 102000;
float scale_factor = 1000.0;

// -------------------------------------------------------------
// Dynamic calibration of min / max readings
// -------------------------------------------------------------
float calibrated_min = -150.0;
float calibrated_max = 150.0;

float previous_calibrated_min = -150.0;
float previous_calibrated_max = 150.0;

bool is_calibrating = false;
unsigned long calibration_start_time = 0;
const unsigned long calibration_duration = 10000;

float calibration_min_reading = 0.0;
float calibration_max_reading = 0.0;
bool calibration_has_reading = false;

const float min_calibration_span = 30.0;

unsigned long last_calibration_led_toggle = 0;
bool calibration_led_state = false;
const unsigned long calibration_led_interval = 120;

// -------------------------------------------------------------
// Feedback modes
// -------------------------------------------------------------
enum FeedbackMode {
  MODE_ABSOLUTE = 0,
  MODE_RATE = 1,
  MODE_NO_BREATH_ALERT = 2
};

FeedbackMode current_mode = MODE_ABSOLUTE;

// -------------------------------------------------------------
// Rate of change mode
// -------------------------------------------------------------
unsigned long last_time = 0;
float last_weight_filtered = 0.0;
float last_normalized_weight = 0.0;
float rate_filtered = 0.0;

float weight_smoothing = 0.12;
float rate_smoothing = 0.10;

bool filter_initialized = false;

float min_rate = 0.0;
float max_rate = 1.2;

// -------------------------------------------------------------
// No breath alert mode
// -------------------------------------------------------------
unsigned long last_breath_detected_time = 0;
unsigned long no_breath_timeout = 10000;

float breath_detect_threshold = 0.04;

unsigned long pulse_period = 1600;
unsigned long pulse_on_time = 350;
unsigned long pulse_ramp_time = 5000;

int alert_min_pwm = 20;
int alert_max_pwm = 120;

float clamp_float(float x, float lo, float hi);
int map_float_to_pwm(float x, float in_min, float in_max);
int apply_motor_floor(int pwm);
float get_weight();
float get_normalized_weight(float weight);
void signal_calibration_mode_with_motor();
void blink_n_times(int n);
void indicate_current_mode();
void reset_signal_state();
void start_calibration();
void finish_calibration();
void update_calibration_led(unsigned long now);
void next_mode();

void setup() {
  Serial.begin(57600);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  reset_signal_state();
  indicate_current_mode();
}

void loop() {
  unsigned long now = millis();

  int reading = digitalRead(BUTTON_PIN);

  if (reading != last_reading) {
    last_debounce_time = now;
  }

  if ((now - last_debounce_time) > debounce_delay) {
    if (reading != button_state) {
      button_state = reading;

      if (button_state == LOW) {
        button_press_start_time = now;
        long_press_handled = false;
      } else {
        if (!long_press_handled && !is_calibrating) {
          next_mode();
        }
      }
    }
  }

  last_reading = reading;

  if (button_state == LOW && !long_press_handled && !is_calibrating) {
    if ((now - button_press_start_time) >= long_press_duration) {
      start_calibration();
      long_press_handled = true;
    }
  }

  if (is_calibrating) {
    analogWrite(MOTOR_PIN, 0);
    update_calibration_led(now);

    if (scale.is_ready()) {
      float raw_weight = get_weight();

      if (!calibration_has_reading) {
        calibration_min_reading = raw_weight;
        calibration_max_reading = raw_weight;
        calibration_has_reading = true;
      } else {
        if (raw_weight < calibration_min_reading) calibration_min_reading = raw_weight;
        if (raw_weight > calibration_max_reading) calibration_max_reading = raw_weight;
      }

      Serial.print(raw_weight, 2);
      Serial.print('\t');
      Serial.println(0);
    } else {
      Serial.print(0.0, 2);
      Serial.print('\t');
      Serial.println(0);
    }

    if ((now - calibration_start_time) >= calibration_duration) {
      finish_calibration();
    }

    delay(20);
    return;
  }

  if (!scale.is_ready()) {
    analogWrite(MOTOR_PIN, 0);

    Serial.print(0.0, 2);
    Serial.print('\t');
    Serial.println(0);

    delay(100);
    return;
  }

  float raw_weight = get_weight();
  float force_abs = fabs(raw_weight);

  if (!filter_initialized) {
    last_weight_filtered = raw_weight;
    last_normalized_weight = get_normalized_weight(raw_weight);
    last_time = now;
    rate_filtered = 0.0;
    filter_initialized = true;
    last_breath_detected_time = now;
  }

  float dt = (now - last_time) / 1000.0;
  if (dt <= 0.0) {
    dt = 0.001;
  }

  float weight_filtered = weight_smoothing * raw_weight
                       + (1.0 - weight_smoothing) * last_weight_filtered;

  float normalized_weight = get_normalized_weight(weight_filtered);

  float raw_rate = (normalized_weight - last_normalized_weight) / dt;

  rate_filtered = rate_smoothing * raw_rate
               + (1.0 - rate_smoothing) * rate_filtered;

  float activity_abs = fabs(rate_filtered);

  if (activity_abs > breath_detect_threshold) {
    last_breath_detected_time = now;
  }

  last_weight_filtered = weight_filtered;
  last_normalized_weight = normalized_weight;
  last_time = now;

  int pwm = 0;

  if (current_mode == MODE_ABSOLUTE) {
    pwm = map_float_to_pwm(weight_filtered, calibrated_min, calibrated_max);
    pwm = apply_motor_floor(pwm);
    analogWrite(MOTOR_PIN, pwm);
  }

  else if (current_mode == MODE_RATE) {
    float r = clamp_float(activity_abs, min_rate, max_rate);
    pwm = map_float_to_pwm(r, min_rate, max_rate);
    pwm = apply_motor_floor(pwm);
    analogWrite(MOTOR_PIN, pwm);
  }

  else if (current_mode == MODE_NO_BREATH_ALERT) {
    unsigned long quiet_time = now - last_breath_detected_time;

    if (quiet_time < no_breath_timeout) {
      pwm = 0;
    } else {
      unsigned long alert_elapsed = quiet_time - no_breath_timeout;

      float ramp_progress = (float)alert_elapsed / (float)pulse_ramp_time;
      if (ramp_progress > 1.0) {
        ramp_progress = 1.0;
      }

      int pulse_strength = alert_min_pwm + (int)((alert_max_pwm - alert_min_pwm) * ramp_progress);
      unsigned long pulse_phase = alert_elapsed % pulse_period;

      if (pulse_phase < pulse_on_time) {
        pwm = pulse_strength;
      } else {
        pwm = 0;
      }
    }

    pwm = apply_motor_floor(pwm);
    analogWrite(MOTOR_PIN, pwm);
  }

  Serial.print(force_abs, 2);
  Serial.print('\t');
  Serial.println(pwm);

  delay(100);
}

// -------------------------------------------------------------
// Helper
// -------------------------------------------------------------
float clamp_float(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int map_float_to_pwm(float x, float in_min, float in_max) {
  if (in_max == in_min) {
    return 0;
  }

  float result = (x - in_min) * 255.0 / (in_max - in_min);
  result = clamp_float(result, 0.0, 255.0);

  return (int)result;
}

int apply_motor_floor(int pwm) {
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

float get_weight() {
  long raw = scale.read();
  return (raw - zero_offset) / scale_factor;
}

float get_normalized_weight(float weight) {
  float span = calibrated_max - calibrated_min;

  if (span < min_calibration_span) {
    span = min_calibration_span;
  }

  float normalized = (weight - calibrated_min) / span;
  return clamp_float(normalized, 0.0, 1.0);
}

// -------------------------------------------------------------
// NEW: motor signal for calibration start
// -------------------------------------------------------------
void signal_calibration_mode_with_motor() {
  int pwm = apply_motor_floor(CALIBRATION_SIGNAL_PWM);

  for (int i = 0; i < 2; i++) {
    analogWrite(MOTOR_PIN, pwm);
    delay(CALIBRATION_SIGNAL_ON_TIME);

    analogWrite(MOTOR_PIN, 0);

    if (i < 1) {
      delay(CALIBRATION_SIGNAL_OFF_TIME);
    }
  }
}

// -------------------------------------------------------------
// LED feedback for current mode
// -------------------------------------------------------------
void blink_n_times(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(blink_on_time);

    digitalWrite(LED_PIN, LOW);
    delay(blink_off_time);
  }
}

void indicate_current_mode() {
  if (current_mode == MODE_ABSOLUTE) {
    blink_n_times(1);
  } else if (current_mode == MODE_RATE) {
    blink_n_times(2);
  } else if (current_mode == MODE_NO_BREATH_ALERT) {
    blink_n_times(3);
  }
}

void reset_signal_state() {
  last_time = millis();

  if (scale.is_ready()) {
    float w = get_weight();
    last_weight_filtered = w;
    last_normalized_weight = get_normalized_weight(w);
  } else {
    last_weight_filtered = 0.0;
    last_normalized_weight = 0.0;
  }

  rate_filtered = 0.0;
  filter_initialized = true;
  last_breath_detected_time = millis();
}

// -------------------------------------------------------------
// Start calibration
// -------------------------------------------------------------
void start_calibration() {
  previous_calibrated_min = calibrated_min;
  previous_calibrated_max = calibrated_max;

  analogWrite(MOTOR_PIN, 0);

  // NEW: two short motor pulses to signal calibration mode
  signal_calibration_mode_with_motor();

  is_calibrating = true;
  calibration_start_time = millis();
  calibration_has_reading = false;

  calibration_min_reading = 0.0;
  calibration_max_reading = 0.0;

  calibration_led_state = true;
  last_calibration_led_toggle = millis();
  digitalWrite(LED_PIN, HIGH);

  filter_initialized = false;

  Serial.println("CALIBRATION_START");
}

void finish_calibration() {
  is_calibrating = false;
  digitalWrite(LED_PIN, LOW);

  if (calibration_has_reading) {
    float span = calibration_max_reading - calibration_min_reading;

    if (span >= min_calibration_span) {
      calibrated_min = calibration_min_reading;
      calibrated_max = calibration_max_reading;
    } else {
      calibrated_min = previous_calibrated_min;
      calibrated_max = previous_calibrated_max;
    }
  } else {
    calibrated_min = previous_calibrated_min;
    calibrated_max = previous_calibrated_max;
  }

  reset_signal_state();

  Serial.print("CALIBRATION_DONE\tMIN=");
  Serial.print(calibrated_min, 2);
  Serial.print("\tMAX=");
  Serial.println(calibrated_max, 2);

  indicate_current_mode();
}

void update_calibration_led(unsigned long now) {
  if (now - last_calibration_led_toggle >= calibration_led_interval) {
    calibration_led_state = !calibration_led_state;
    digitalWrite(LED_PIN, calibration_led_state ? HIGH : LOW);
    last_calibration_led_toggle = now;
  }
}

void next_mode() {
  if (current_mode == MODE_ABSOLUTE) {
    current_mode = MODE_RATE;
  } else if (current_mode == MODE_RATE) {
    current_mode = MODE_NO_BREATH_ALERT;
  } else {
    current_mode = MODE_ABSOLUTE;
  }

  reset_signal_state();
  indicate_current_mode();
}
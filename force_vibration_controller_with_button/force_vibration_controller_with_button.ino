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
const int CALIBRATION_SIGNAL_PWM = 40;
const int CALIBRATION_SIGNAL_ON_TIME = 120;
const int CALIBRATION_SIGNAL_OFF_TIME = 100;

// -------------------------------------------------------------
// Button / LED settings
// -------------------------------------------------------------
const unsigned long debounce_delay = 40;
const unsigned long long_press_duration = 3000;

const int blink_on_time = 250;
const int blink_off_time = 250;

int last_reading = HIGH;
int button_state = HIGH;
unsigned long last_debounce_time = 0;

unsigned long button_press_start_time = 0;

// -------------------------------------------------------------
// HX711 / calibration
// -------------------------------------------------------------
HX711 load_cell;

long zero_offset = 102000;
float scale_factor = 1000.0;

// -------------------------------------------------------------
// Dynamic calibration of min / max readings
// -------------------------------------------------------------
float calibrated_min = -150.0;
float calibrated_max = 150.0;

float previous_calibrated_min = -150.0;
float previous_calibrated_max = 150.0;

bool has_valid_calibration = false;

bool is_calibrating = false;
unsigned long calibration_start_time = 0;
const unsigned long calibration_duration = 10000;

float calibration_min_signal = 0.0;
float calibration_max_signal = 0.0;
bool calibration_has_reading = false;

const float min_calibration_span = 30.0;

unsigned long last_calibration_led_toggle = 0;
bool calibration_led_state = false;
const unsigned long calibration_led_interval = 120;

// -------------------------------------------------------------
// Last valid values for plotting if HX711 is temporarily not ready
// -------------------------------------------------------------
float last_valid_force_abs = 0.0;
bool have_valid_force_abs = false;

float last_valid_calibration_signal = 0.0;
bool have_valid_calibration_signal = false;

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
float last_signal_filtered = 0.0;
float last_normalized_signal = 0.0;
float rate_filtered = 0.0;

float signal_smoothing = 0.12;
float rate_smoothing = 0.10;

bool filter_initialized = false;

float min_rate = 0.0;
float max_rate = 1.2;

// -------------------------------------------------------------
// No breath alert mode
// -------------------------------------------------------------
unsigned long last_breath_detected_time = 0;
unsigned long no_breath_timeout = 8000;

float breath_detect_threshold = 0.025;

unsigned long pulse_period = 1600;
unsigned long pulse_on_time = 350;
unsigned long pulse_ramp_time = 8000;

int alert_min_pwm = 20;
int alert_max_pwm = 100;

float clamp_float(float x, float lo, float hi);
int map_float_to_pwm(float x, float in_min, float in_max);
int apply_motor_floor(int pwm);
float get_load_cell_signal();
float get_normalized_signal(float signal_value);
void signal_calibration_start_or_end();
void blink_n_times(int n);
void indicate_current_mode();
void reset_signal_state();
void start_calibration();
void finish_calibration();
void update_calibration_led(unsigned long now);
void next_mode();

void setup() {
  Serial.begin(57600);

  load_cell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

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
      } else {
        if (!is_calibrating) {
          unsigned long press_duration = now - button_press_start_time;

          if (press_duration >= long_press_duration) {
            start_calibration();
          } else {
            next_mode();
          }
        }
      }
    }
  }

  last_reading = reading;

  if (is_calibrating) {
    analogWrite(MOTOR_PIN, 0);
    update_calibration_led(now);

    if (load_cell.is_ready()) {
      float raw_signal = get_load_cell_signal();

      if (!calibration_has_reading) {
        calibration_min_signal = raw_signal;
        calibration_max_signal = raw_signal;
        calibration_has_reading = true;
      } else {
        if (raw_signal < calibration_min_signal) calibration_min_signal = raw_signal;
        if (raw_signal > calibration_max_signal) calibration_max_signal = raw_signal;
      }

      last_valid_calibration_signal = raw_signal;
      have_valid_calibration_signal = true;

      Serial.print(raw_signal, 2);
      Serial.print('\t');
      Serial.println(0);
    } else {
      float plot_signal = have_valid_calibration_signal ? last_valid_calibration_signal : 0.0;

      Serial.print(plot_signal, 2);
      Serial.print('\t');
      Serial.println(0);
    }

    if ((now - calibration_start_time) >= calibration_duration) {
      finish_calibration();
    }

    delay(20);
    return;
  }

  if (!load_cell.is_ready()) {
    analogWrite(MOTOR_PIN, 0);

    float plot_force = have_valid_force_abs ? last_valid_force_abs : 0.0;

    Serial.print(plot_force, 2);
    Serial.print('\t');
    Serial.println(0);

    delay(100);
    return;
  }

  float raw_signal = get_load_cell_signal();
  float force_abs = fabs(raw_signal);

  last_valid_force_abs = force_abs;
  have_valid_force_abs = true;

  if (!filter_initialized) {
    last_signal_filtered = raw_signal;
    last_normalized_signal = get_normalized_signal(raw_signal);
    last_time = now;
    rate_filtered = 0.0;
    filter_initialized = true;
    last_breath_detected_time = now;
  }

  float dt = (now - last_time) / 1000.0;
  if (dt <= 0.0) {
    dt = 0.001;
  }

  float signal_filtered = signal_smoothing * raw_signal
                        + (1.0 - signal_smoothing) * last_signal_filtered;

  float normalized_signal = get_normalized_signal(signal_filtered);

  float raw_rate = (normalized_signal - last_normalized_signal) / dt;

  rate_filtered = rate_smoothing * raw_rate
                + (1.0 - rate_smoothing) * rate_filtered;

  float activity_abs = fabs(rate_filtered);

  if (activity_abs > breath_detect_threshold) {
    last_breath_detected_time = now;
  }

  last_signal_filtered = signal_filtered;
  last_normalized_signal = normalized_signal;
  last_time = now;

  int pwm = 0;

  if (has_valid_calibration) {
    if (current_mode == MODE_ABSOLUTE) {
      float abs_signal = fabs(signal_filtered);
      float abs_max = fmax(fabs(calibrated_min), fabs(calibrated_max));

      if (abs_max < (min_calibration_span * 0.5)) {
        abs_max = min_calibration_span * 0.5;
      }

      pwm = map_float_to_pwm(abs_signal, 0.0, abs_max);
      pwm = apply_motor_floor(pwm);
    }

    else if (current_mode == MODE_RATE) {
      float r = clamp_float(activity_abs, min_rate, max_rate);
      pwm = map_float_to_pwm(r, min_rate, max_rate);
      pwm = apply_motor_floor(pwm);
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
    }
  } else {
    pwm = 0;
  }

  analogWrite(MOTOR_PIN, pwm);

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

float get_load_cell_signal() {
  long raw = load_cell.read();
  return (raw - zero_offset) / scale_factor;
}

float get_normalized_signal(float signal_value) {
  float span = calibrated_max - calibrated_min;

  if (span < min_calibration_span) {
    span = min_calibration_span;
  }

  float normalized = (signal_value - calibrated_min) / span;
  return clamp_float(normalized, 0.0, 1.0);
}

// -------------------------------------------------------------
// Combined LED + motor signal for calibration start/end
// -------------------------------------------------------------
void signal_calibration_start_or_end() {
  int pwm = apply_motor_floor(CALIBRATION_SIGNAL_PWM);

  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    analogWrite(MOTOR_PIN, pwm);
    delay(CALIBRATION_SIGNAL_ON_TIME);

    digitalWrite(LED_PIN, LOW);
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

  if (load_cell.is_ready()) {
    float s = get_load_cell_signal();
    last_signal_filtered = s;
    last_normalized_signal = get_normalized_signal(s);
    last_valid_force_abs = fabs(s);
    have_valid_force_abs = true;
  } else {
    last_signal_filtered = 0.0;
    last_normalized_signal = 0.0;
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
  digitalWrite(LED_PIN, LOW);

  signal_calibration_start_or_end();

  is_calibrating = true;
  calibration_start_time = millis();
  calibration_has_reading = false;

  calibration_min_signal = 0.0;
  calibration_max_signal = 0.0;

  have_valid_calibration_signal = false;

  calibration_led_state = true;
  last_calibration_led_toggle = millis();
  digitalWrite(LED_PIN, HIGH);

  filter_initialized = false;

  Serial.println("CALIBRATION_START");
}

void finish_calibration() {
  is_calibrating = false;
  digitalWrite(LED_PIN, LOW);
  analogWrite(MOTOR_PIN, 0);

  if (calibration_has_reading) {
    float span = calibration_max_signal - calibration_min_signal;

    if (span >= min_calibration_span) {
      calibrated_min = calibration_min_signal;
      calibrated_max = calibration_max_signal;
      has_valid_calibration = true;
    } else {
      calibrated_min = previous_calibrated_min;
      calibrated_max = previous_calibrated_max;
    }
  } else {
    calibrated_min = previous_calibrated_min;
    calibrated_max = previous_calibrated_max;
  }

  signal_calibration_start_or_end();

  reset_signal_state();

  Serial.print("CALIBRATION_DONE\tMIN=");
  Serial.print(calibrated_min, 2);
  Serial.print("\tMAX=");
  Serial.print(calibrated_max, 2);
  Serial.print("\tVALID=");
  Serial.println(has_valid_calibration ? 1 : 0);

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
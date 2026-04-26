#include "motors.h"

#include <Arduino.h>
#include <math.h>

#include "pins.h"
#include "motor_config.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Encoder + PWM
static const int32_t TICKS_PER_REV = 528;     // 11 PPR * 48:1 gearbox, 1x decoding
static const bool LEFT_DIR_INVERT  = true;
static const bool RIGHT_DIR_INVERT = true;

static const int PWM_CH_RIGHT = 0;
static const int PWM_CH_LEFT  = 1;
static const int PWM_FREQ     = 20000;        // 20 kHz, inaudible
static const int PWM_RES      = 8;

// Duty floors. MIN_DUTY_SPIN keeps an already-rotating robot moving;
// SPIN_KICKSTART_DUTY is briefly applied to break stiction at the start.
static const uint8_t  MIN_DUTY            = 30;
static const uint8_t  MIN_DUTY_SPIN       = 130;
static const uint8_t  SPIN_KICKSTART_DUTY = 200;
static const uint32_t SPIN_KICKSTART_MS   = 200;

// Kinematics
static const float BASE_WIDTH_M   = 0.12f;
static const float WHEEL_RADIUS_M = 0.033f;
static const float MAX_RPM        = 70.0f;
static const float MAX_WHEEL_MS   = (2.0f * M_PI * WHEEL_RADIUS_M) * (MAX_RPM / 60.0f);

static const float LIN_DEADBAND   = 0.02f;    // m/s
static const float ANG_DEADBAND   = 0.05f;    // rad/s
static const float SPIN_FLOOR_MS  = 0.08f;

static const float LIN_GAIN  =  0.4f;
static const float WZ_SIGN   = -1.0f;
static const float WZ_GAIN   =  2.0f;

static const uint32_t CMD_TIMEOUT_MS = 500;
static const float MOTOR_TAU_SEC = 0.10f;     // EMA on the wheel target

// State
static volatile float v_left_cmd  = 0.0f;
static volatile float v_right_cmd = 0.0f;
static volatile bool  spin_mode   = false;
static volatile uint32_t last_cmd_ms     = 0;
static volatile uint32_t spin_started_ms = 0;

static float vL_filt = 0.0f, vR_filt = 0.0f;
static uint32_t last_motor_us = 0;

static volatile int32_t enc_left_ticks  = 0;
static volatile int32_t enc_right_ticks = 0;

static float wheel_vel_left_ms  = 0.0f;
static float wheel_vel_right_ms = 0.0f;
static int32_t prev_enc_left_ticks  = 0;
static int32_t prev_enc_right_ticks = 0;
static uint32_t last_vel_us = 0;
static uint32_t last_vel_update_ms = 0;

static PID pid_left, pid_right;

// The two encoders are physically mirrored across the chassis but wired the
// same way, so one of them counts opposite to the URDF "+forward" convention.
// Flipping the left wheel here keeps published position, velocity, and PID
// feedback all consistently signed.
void IRAM_ATTR enc_left_isr() {
  if (digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) enc_left_ticks--;
  else                                                    enc_left_ticks++;
}
void IRAM_ATTR enc_right_isr() {
  if (digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) enc_right_ticks++;
  else                                                      enc_right_ticks--;
}

// Deadzone-compensated linear FF: zero target -> zero duty, otherwise map
// onto [min_duty, 255] proportionally so any nonzero command immediately
// produces a duty large enough to overcome stiction.
static int ff_duty_signed(float v_ms, int min_duty) {
  if (v_ms == 0.0f) return 0;
  float m = fabsf(v_ms) / MAX_WHEEL_MS;
  if (m > 1.0f) m = 1.0f;
  int d = min_duty + (int)lrintf(m * (255 - min_duty));
  return v_ms > 0 ? d : -d;
}

// Direction inversion is applied here so callers always think in robot-frame.
static void apply_motor_left_duty(int signed_duty) {
  if (signed_duty >  255) signed_duty =  255;
  if (signed_duty < -255) signed_duty = -255;
  bool fwd = signed_duty >= 0;
  if (LEFT_DIR_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW); }
  else     { digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH); }
  ledcWrite(PWM_CH_LEFT, abs(signed_duty));
}

static void apply_motor_right_duty(int signed_duty) {
  if (signed_duty >  255) signed_duty =  255;
  if (signed_duty < -255) signed_duty = -255;
  bool fwd = signed_duty >= 0;
  if (RIGHT_DIR_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); }
  else     { digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH); }
  ledcWrite(PWM_CH_RIGHT, abs(signed_duty));
}

static void stop_motors() {
  ledcWrite(PWM_CH_LEFT,  0);
  ledcWrite(PWM_CH_RIGHT, 0);
}

// Sampled at fixed rate so PID feedback isn't sensitive to main-loop jitter.
static void update_wheel_velocities() {
  uint32_t now_us = micros();
  int32_t left_now  = enc_left_ticks;
  int32_t right_now = enc_right_ticks;

  if (last_vel_us != 0) {
    float dt = (now_us - last_vel_us) * 1e-6f;
    if (dt > 0.0f) {
      float left_radps  = (left_now  - prev_enc_left_ticks)  *
                          (2.0f * (float)M_PI / TICKS_PER_REV) / dt;
      float right_radps = (right_now - prev_enc_right_ticks) *
                          (2.0f * (float)M_PI / TICKS_PER_REV) / dt;
      wheel_vel_left_ms  = left_radps  * WHEEL_RADIUS_M;
      wheel_vel_right_ms = right_radps * WHEEL_RADIUS_M;
    }
  }
  prev_enc_left_ticks  = left_now;
  prev_enc_right_ticks = right_now;
  last_vel_us = now_us;
}

void motors_setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  enc_left_isr,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), enc_right_isr, RISING);

  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_ENB, PWM_CH_LEFT);

  pid_init(&pid_left,  DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD,
           DEFAULT_PID_INTEGRAL_MAX_DUTY);
  pid_init(&pid_right, DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD,
           DEFAULT_PID_INTEGRAL_MAX_DUTY);

  stop_motors();
  last_cmd_ms = millis();
}

void motors_update() {
  uint32_t now_us = micros();

  if (millis() - last_vel_update_ms >= VEL_UPDATE_INTERVAL_MS) {
    update_wheel_velocities();
    last_vel_update_ms = millis();
  }

  // Safety: stop if /cmd_vel goes stale.
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    v_left_cmd = v_right_cmd = 0.0f;
    vL_filt = vR_filt = 0.0f;
    pid_reset(&pid_left);
    pid_reset(&pid_right);
    stop_motors();
    last_motor_us = now_us;
    return;
  }

  // EMA toward latest cmd so a single keypress still converges fully.
  if (last_motor_us != 0) {
    float dt = (now_us - last_motor_us) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) {
      float alpha = 1.0f - expf(-dt / MOTOR_TAU_SEC);
      vL_filt += alpha * (v_left_cmd  - vL_filt);
      vR_filt += alpha * (v_right_cmd - vR_filt);
    }
  }
  last_motor_us = now_us;

  int min_duty = MIN_DUTY;
  if (spin_mode) {
    bool kickstarting = (millis() - spin_started_ms) < SPIN_KICKSTART_MS;
    min_duty = kickstarting ? SPIN_KICKSTART_DUTY : MIN_DUTY_SPIN;
  }

  int ff_left  = ff_duty_signed(vL_filt, min_duty);
  int ff_right = ff_duty_signed(vR_filt, min_duty);
  float corr_left  = pid_step(&pid_left,  vL_filt, wheel_vel_left_ms,  now_us);
  float corr_right = pid_step(&pid_right, vR_filt, wheel_vel_right_ms, now_us);
  apply_motor_left_duty (ff_left  + (int)lrintf(corr_left));
  apply_motor_right_duty(ff_right + (int)lrintf(corr_right));
}

void motors_apply_cmd_vel(float linear_x, float angular_z) {
  float v  = linear_x;
  float wz = angular_z;

  if (!isfinite(v))  v  = 0.0f;
  if (!isfinite(wz)) wz = 0.0f;

  if (fabsf(v)  < LIN_DEADBAND) v  = 0.0f;
  if (fabsf(wz) < ANG_DEADBAND) wz = 0.0f;

  v  = LIN_GAIN * v;
  wz = WZ_SIGN * WZ_GAIN * wz;

  float vL = v - 0.5f * wz * BASE_WIDTH_M;
  float vR = v + 0.5f * wz * BASE_WIDTH_M;

  bool new_spin = (v == 0.0f && wz != 0.0f);
  if (new_spin) {
    float target = 0.5f * fabsf(wz) * BASE_WIDTH_M;
    if (target < SPIN_FLOOR_MS) target = SPIN_FLOOR_MS;
    vL = -copysignf(target, wz);
    vR =  copysignf(target, wz);
  }
  // Edge-trigger kickstart on each fresh transition into spin.
  if (new_spin && !spin_mode) spin_started_ms = millis();
  spin_mode = new_spin;

  vL = fmaxf(fminf(vL,  MAX_WHEEL_MS), -MAX_WHEEL_MS);
  vR = fmaxf(fminf(vR,  MAX_WHEEL_MS), -MAX_WHEEL_MS);

  v_left_cmd  = vL;
  v_right_cmd = vR;
  last_cmd_ms = millis();
}

void motors_set_pid_gains(float kp, float ki, float kd) {
  pid_set_gains(&pid_left,  kp, ki, kd, /*reset_state=*/true);
  pid_set_gains(&pid_right, kp, ki, kd, /*reset_state=*/true);
}

double motors_left_position_rad() {
  return (double)enc_left_ticks * 2.0 * M_PI / TICKS_PER_REV;
}
double motors_right_position_rad() {
  return (double)enc_right_ticks * 2.0 * M_PI / TICKS_PER_REV;
}
double motors_left_velocity_radps()  { return (double)(wheel_vel_left_ms  / WHEEL_RADIUS_M); }
double motors_right_velocity_radps() { return (double)(wheel_vel_right_ms / WHEEL_RADIUS_M); }

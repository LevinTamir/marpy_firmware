// Motor + encoder + per-wheel velocity PID for the differential drive.
// Public contract is in motors.h.
#include "motors.h"

#include <Arduino.h>
#include <math.h>

#include "pins.h"
#include "motor_config.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =================== Encoder specs ====================
// 1x decoding (rising edge of channel A only) gives 11 ticks per encoder
// shaft revolution; the gearbox multiplies by 48 -> 528 ticks per output rev.
static const int32_t TICKS_PER_REV = 528;

// Software direction flips (toggle if a wheel turns the wrong way).
static const bool LEFT_DIR_INVERT  = true;
static const bool RIGHT_DIR_INVERT = true;

// =================== PWM (LEDC) =======================
static const int PWM_CH_RIGHT = 0;
static const int PWM_CH_LEFT  = 1;
static const int PWM_FREQ     = 20000;  // 20 kHz (inaudible)
static const int PWM_RES      = 8;      // 0..255 duty

// Duty floors so small non-zero commands actually overcome static friction.
static const uint8_t MIN_DUTY      = 30;
static const uint8_t MIN_DUTY_SPIN = 80;

// =================== Kinematics =======================
static const float BASE_WIDTH_M   = 0.12f;   // track width (12 cm)
static const float WHEEL_RADIUS_M = 0.033f;
static const float MAX_RPM        = 70.0f;
static const float MAX_WHEEL_MS   = (2.0f * M_PI * WHEEL_RADIUS_M) * (MAX_RPM / 60.0f);

static const float LIN_DEADBAND   = 0.02f;   // m/s
static const float ANG_DEADBAND   = 0.05f;   // rad/s
static const float SPIN_FLOOR_MS  = 0.08f;   // smallest per-wheel speed during in-place spin

// ---- Linear / angular shaping ----
// teleop_twist_keyboard defaults to 0.5 m/s; LIN_GAIN 0.4 -> ~0.2 m/s target.
static const float LIN_GAIN  =  0.4f;
static const float WZ_SIGN   = -1.0f;   // flip angular direction
static const float WZ_GAIN   =  2.0f;   // smaller = slower spin; below ~1.8 the duty drops under the floor needed to rotate the robot on the floor

static const uint32_t CMD_TIMEOUT_MS = 500;  // brake if cmd stream stale

// ---- EMA on the wheel target ----
// Time-constant low-pass so a single /cmd_vel still converges fully without
// needing the host to keep republishing.
static const float MOTOR_TAU_SEC = 0.10f;

// =================== State ============================
// Raw wheel-speed commands from /cmd_vel (already shaped + clamped).
static volatile float v_left_cmd  = 0.0f;
static volatile float v_right_cmd = 0.0f;
static volatile bool  spin_mode   = false;
static volatile uint32_t last_cmd_ms = 0;

// EMA-smoothed setpoint (what the PID tries to track / what FF is computed from).
static float vL_filt = 0.0f, vR_filt = 0.0f;
static uint32_t last_motor_us = 0;

// Encoder state (volatile because they're touched in ISRs).
static volatile int32_t enc_left_ticks  = 0;
static volatile int32_t enc_right_ticks = 0;

// Wheel velocity estimates (m/s), updated at VEL_UPDATE_INTERVAL_MS rate.
static float wheel_vel_left_ms  = 0.0f;
static float wheel_vel_right_ms = 0.0f;
static int32_t prev_enc_left_ticks  = 0;
static int32_t prev_enc_right_ticks = 0;
static uint32_t last_vel_us = 0;
static uint32_t last_vel_update_ms = 0;

// PID state, one per wheel.
static PID pid_left, pid_right;

// =================== Encoder ISRs (1x, channel A rising) ===
// On A rising, B's level tells us direction. This is the original decoder
// from the EMA baseline; known-good with our motors.
void IRAM_ATTR enc_left_isr() {
  if (digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) enc_left_ticks++;
  else                                                    enc_left_ticks--;
}
void IRAM_ATTR enc_right_isr() {
  if (digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) enc_right_ticks++;
  else                                                      enc_right_ticks--;
}

// =================== Low-level motor drive ============
// Feed-forward PWM estimate from a target velocity, signed (negative = reverse).
// Enforces MIN_DUTY so a small but nonzero target still overcomes friction --
// without it, low commands have no effect even when PID is disabled.
static int ff_duty_signed(float v_ms, bool is_spin) {
  if (v_ms == 0.0f) return 0;
  float m = fabsf(v_ms) / MAX_WHEEL_MS;
  if (m > 1.0f) m = 1.0f;
  int d = (int)lrintf(m * 255.0f);
  int min_d = is_spin ? MIN_DUTY_SPIN : MIN_DUTY;
  if (d < min_d) d = min_d;
  return v_ms > 0 ? d : -d;
}

// Drive the left motor at a signed PWM duty in [-255, +255]. Sign sets
// direction, magnitude sets PWM. Direction inversion is applied here so
// callers always think in robot-frame: +duty = forward.
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

// =================== Velocity estimate ================
// Recompute wheel velocities from encoder tick deltas. Called at fixed rate
// so the PID feedback signal isn't sensitive to main-loop jitter.
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

// =================== Public API =======================
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

  // 1. Refresh measured wheel velocities at a fixed rate.
  if (millis() - last_vel_update_ms >= VEL_UPDATE_INTERVAL_MS) {
    update_wheel_velocities();
    last_vel_update_ms = millis();
  }

  // 2. Safety: stop if /cmd_vel goes stale.
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    v_left_cmd = v_right_cmd = 0.0f;
    vL_filt = vR_filt = 0.0f;
    pid_reset(&pid_left);
    pid_reset(&pid_right);
    stop_motors();
    last_motor_us = now_us;
    return;
  }

  // 3. Time-based EMA toward the latest cmd. With one /cmd_vel message per
  //    teleop keypress, the EMA still converges by running here every loop.
  if (last_motor_us != 0) {
    float dt = (now_us - last_motor_us) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) {
      float alpha = 1.0f - expf(-dt / MOTOR_TAU_SEC);
      vL_filt += alpha * (v_left_cmd  - vL_filt);
      vR_filt += alpha * (v_right_cmd - vR_filt);
    }
  }
  last_motor_us = now_us;

  // 4. Feed-forward duty + PID correction. With default zero gains, the PID
  //    returns 0 and the motors run pure FF (matches the EMA baseline).
  int ff_left  = ff_duty_signed(vL_filt, spin_mode);
  int ff_right = ff_duty_signed(vR_filt, spin_mode);
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

  // Shape: scale linear, flip + amplify angular.
  v  = LIN_GAIN * v;
  wz = WZ_SIGN * WZ_GAIN * wz;

  // Differential mapping (unicycle -> wheels).
  float vL = v - 0.5f * wz * BASE_WIDTH_M;
  float vR = v + 0.5f * wz * BASE_WIDTH_M;

  // Force in-place spin when only angular is commanded so small wz still
  // overcomes the per-wheel deadband.
  if (v == 0.0f && wz != 0.0f) {
    float target = 0.5f * fabsf(wz) * BASE_WIDTH_M;
    if (target < SPIN_FLOOR_MS) target = SPIN_FLOOR_MS;
    vL = -copysignf(target, wz);
    vR =  copysignf(target, wz);
    spin_mode = true;
  } else {
    spin_mode = false;
  }

  // Clamp to hardware top speed.
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

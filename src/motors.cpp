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
// 4x quadrature decoding (both edges of A and B):
//   ticks_per_output_rev = 4 * PPR * gear_ratio = 4 * 11 * 48 = 2112
static const int32_t TICKS_PER_REV = 2112;

// Software direction flips (toggle if a wheel turns the wrong way)
static const bool LEFT_DIR_INVERT  = true;
static const bool RIGHT_DIR_INVERT = true;

// =================== PWM (LEDC) =======================
static const int PWM_CH_RIGHT = 0;
static const int PWM_CH_LEFT  = 1;
static const int PWM_FREQ     = 20000;  // 20 kHz (inaudible)
static const int PWM_RES      = 8;      // 0..255 duty

// =================== Kinematics =======================
static const float BASE_WIDTH_M   = 0.12f;   // track width (12 cm)
static const float WHEEL_RADIUS_M = 0.033f;  // wheel radius
static const float MAX_RPM        = 70.0f;
static const float MAX_WHEEL_MS   = (2.0f * M_PI * WHEEL_RADIUS_M) * (MAX_RPM / 60.0f);

static const float LIN_DEADBAND   = 0.02f;   // m/s
static const float ANG_DEADBAND   = 0.05f;   // rad/s
static const float SPIN_FLOOR_MS  = 0.08f;   // ensure in-place spins actually move

// ---- Linear shaping ----
// Scales linear.x so forward/back are gentler than what teleop sends.
// teleop_twist_keyboard defaults to 0.5 m/s; with 0.4 here the wheels
// target 0.2 m/s (below MAX_WHEEL_MS), which feels noticeably calmer.
static const float LIN_GAIN = 0.4f;

// ---- Angular shaping ----
static const float WZ_SIGN = -1.0f;  // flip angular direction
static const float WZ_GAIN = 3.0f;   // gentler spin gain

static const uint32_t CMD_TIMEOUT_MS = 500;  // brake if cmd stream stale

// =================== State ============================
// Raw wheel-speed commands set by /cmd_vel (already shaped).
static volatile float v_left_cmd  = 0.0f;
static volatile float v_right_cmd = 0.0f;

// Slewed setpoints (acceleration-limited). PID drives wheels toward these.
static float v_left_target  = 0.0f;
static float v_right_target = 0.0f;
static uint32_t last_slew_us = 0;

// Encoder state. Volatile because they're touched in ISRs.
static volatile int32_t enc_left_ticks  = 0;
static volatile int32_t enc_right_ticks = 0;
static volatile uint8_t enc_left_state  = 0;  // (A << 1) | B
static volatile uint8_t enc_right_state = 0;

// Wheel velocity estimates (m/s), updated at VEL_UPDATE_INTERVAL_MS rate.
static float wheel_vel_left_ms  = 0.0f;
static float wheel_vel_right_ms = 0.0f;
static int32_t prev_enc_left_ticks  = 0;
static int32_t prev_enc_right_ticks = 0;
static uint32_t last_vel_us = 0;
static uint32_t last_vel_update_ms = 0;

// PID state, one per wheel.
static PID pid_left, pid_right;
static float max_accel_ms2 = DEFAULT_MAX_ACCEL_MS2;

static uint32_t last_cmd_ms = 0;

// =================== Encoder ISRs (4x quadrature) =====
// State-table decoder: every edge of A and B advances the state machine.
// QD_TABLE[(prev_state << 2) | new_state] gives {-1, 0, +1}; 0 means an
// invalid transition (likely a missed pulse) and the count holds.
static const int8_t QD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0,
};

static void IRAM_ATTR enc_left_isr() {
  uint8_t s = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_left_ticks += QD_TABLE[(enc_left_state << 2) | s];
  enc_left_state = s;
}

static void IRAM_ATTR enc_right_isr() {
  uint8_t s = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  enc_right_ticks += QD_TABLE[(enc_right_state << 2) | s];
  enc_right_state = s;
}

// =================== Low-level motor drive ============
// Feed-forward PWM estimate from a target velocity. PID adds a correction.
static inline float ff_duty_from_speed(float v_ms) {
  float m = v_ms / MAX_WHEEL_MS;
  if (m >  1.0f) m =  1.0f;
  if (m < -1.0f) m = -1.0f;
  return m * 255.0f;
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

// Active electric brake: both H-bridge inputs HIGH + ENA HIGH shorts the
// motor terminals through the upper transistors. Crisper stop than coasting.
static void brake_left() {
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, HIGH);
  ledcWrite(PWM_CH_LEFT, 255);
}
static void brake_right() {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, HIGH);
  ledcWrite(PWM_CH_RIGHT, 255);
}

// =================== Control helpers ==================
// Time-based slew rate limit: nudge `target` toward `cmd` at max accel.
static inline void slew_toward(float &target, float cmd, float dt) {
  float max_dv = max_accel_ms2 * dt;
  float delta = cmd - target;
  if (delta >  max_dv) delta =  max_dv;
  if (delta < -max_dv) delta = -max_dv;
  target += delta;
}

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
                          (2.0f * M_PI / TICKS_PER_REV) / dt;
      float right_radps = (right_now - prev_enc_right_ticks) *
                          (2.0f * M_PI / TICKS_PER_REV) / dt;
      // Encoders count raw shaft direction; mirror the wiring inversion so
      // +velocity = robot forward, matching the URDF convention.
      if (LEFT_DIR_INVERT)  left_radps  = -left_radps;
      if (RIGHT_DIR_INVERT) right_radps = -right_radps;
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

  // Seed the quadrature state machines so the first edge isn't a phantom.
  enc_left_state  = (digitalRead(ENC_LEFT_A)  << 1) | digitalRead(ENC_LEFT_B);
  enc_right_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  enc_left_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B),  enc_left_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), enc_right_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), enc_right_isr, CHANGE);

  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_ENB, PWM_CH_LEFT);

  pid_init(&pid_left,  DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD,
           DEFAULT_PID_INTEGRAL_MAX_DUTY);
  pid_init(&pid_right, DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD,
           DEFAULT_PID_INTEGRAL_MAX_DUTY);

  brake_left();
  brake_right();
  last_cmd_ms = millis();
}

void motors_update() {
  uint32_t now_us = micros();

  // 1. Refresh measured wheel velocities at a fixed rate (decouples PID
  //    feedback from main-loop jitter).
  if (millis() - last_vel_update_ms >= VEL_UPDATE_INTERVAL_MS) {
    update_wheel_velocities();
    last_vel_update_ms = millis();
  }

  // 2. Safety: brake on stale /cmd_vel and skip the rest of the pipeline.
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    v_left_cmd    = v_right_cmd    = 0.0f;
    v_left_target = v_right_target = 0.0f;
    pid_reset(&pid_left);
    pid_reset(&pid_right);
    brake_left();
    brake_right();
    last_slew_us = now_us;
    return;
  }

  // 3. Slew per-wheel setpoints toward the latest commands.
  if (last_slew_us != 0) {
    float dt = (now_us - last_slew_us) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) {
      slew_toward(v_left_target,  v_left_cmd,  dt);
      slew_toward(v_right_target, v_right_cmd, dt);
    }
  }
  last_slew_us = now_us;

  // 4. PID-driven motor command. Brake when the slewed target is essentially
  //    zero so the robot stops crisply instead of riding open-loop noise.
  if (fabsf(v_left_target) < 0.001f) {
    brake_left();
    pid_reset(&pid_left);
  } else {
    float ff   = ff_duty_from_speed(v_left_target);
    float corr = pid_step(&pid_left, v_left_target, wheel_vel_left_ms, now_us);
    apply_motor_left_duty((int)lrintf(ff + corr));
  }
  if (fabsf(v_right_target) < 0.001f) {
    brake_right();
    pid_reset(&pid_right);
  } else {
    float ff   = ff_duty_from_speed(v_right_target);
    float corr = pid_step(&pid_right, v_right_target, wheel_vel_right_ms, now_us);
    apply_motor_right_duty((int)lrintf(ff + corr));
  }
}

void motors_set_velocity_from_twist(float linear_x, float angular_z) {
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
  // overcomes the deadband at each wheel.
  if (v == 0.0f && wz != 0.0f) {
    float target = 0.5f * fabsf(wz) * BASE_WIDTH_M;
    if (target < SPIN_FLOOR_MS) target = SPIN_FLOOR_MS;
    vL = -copysignf(target, wz);
    vR =  copysignf(target, wz);
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

float motors_left_position_rad() {
  float p = (float)enc_left_ticks * 2.0f * (float)M_PI / TICKS_PER_REV;
  return LEFT_DIR_INVERT ? -p : p;
}
float motors_right_position_rad() {
  float p = (float)enc_right_ticks * 2.0f * (float)M_PI / TICKS_PER_REV;
  return RIGHT_DIR_INVERT ? -p : p;
}
float motors_left_velocity_radps()  { return wheel_vel_left_ms  / WHEEL_RADIUS_M; }
float motors_right_velocity_radps() { return wheel_vel_right_ms / WHEEL_RADIUS_M; }

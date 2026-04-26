// Generic single-channel PID controller. See pid.h for the public contract.
//
// The classic PID equation is:
//
//   output(t) = Kp * e(t) + Ki * integral(e dt) + Kd * de/dt
//
// where e = setpoint - measured. This file implements that with two practical
// safeguards: (1) anti-windup on the I term, (2) bogus-dt rejection so a
// hiccup in the timing source can't blow up the derivative.
#include "pid.h"

#include <math.h>

void pid_init(PID *pid, float kp, float ki, float kd, float i_term_max) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->i_term_max = i_term_max;
  pid_reset(pid);
}

void pid_reset(PID *pid) {
  pid->integral   = 0.0f;
  pid->prev_error = 0.0f;
  pid->last_us    = 0;
}

void pid_set_gains(PID *pid, float kp, float ki, float kd, bool reset_state) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  if (reset_state) pid_reset(pid);
}

float pid_step(PID *pid, float setpoint, float measured, uint32_t now_us) {
  // Fast path: all gains zero means PID is disabled. Skip the work and
  // return 0 so the caller's feed-forward duty passes through unchanged.
  if (pid->kp == 0.0f && pid->ki == 0.0f && pid->kd == 0.0f) {
    pid->integral = 0.0f;
    pid->last_us  = now_us;
    return 0.0f;
  }

  // First call has no previous timestamp, so we can't compute dt. Skip it
  // (return 0) and arm the timer so the next call works.
  if (pid->last_us == 0) {
    pid->last_us    = now_us;
    pid->prev_error = setpoint - measured;
    return 0.0f;
  }

  float dt = (now_us - pid->last_us) * 1e-6f;
  pid->last_us = now_us;

  // Reject obviously bad dt: zero (clock didn't advance) or absurdly large
  // (e.g. micros() rolled over, or the loop stalled). The derivative would
  // explode, so just return 0 and resync.
  if (dt <= 0.0f || dt > 0.5f) {
    pid->prev_error = setpoint - measured;
    return 0.0f;
  }

  float error = setpoint - measured;

  // ----- Integral term, with anti-windup -----
  // Accumulate error * dt, then clamp the resulting I-contribution
  // (ki * integral) to i_term_max. We clamp the integral (not the output)
  // so changing ki doesn't unexpectedly change the saturation point.
  pid->integral += error * dt;
  if (pid->ki > 1e-6f) {
    float i_clamp = pid->i_term_max / pid->ki;
    if (pid->integral >  i_clamp) pid->integral =  i_clamp;
    if (pid->integral < -i_clamp) pid->integral = -i_clamp;
  } else {
    pid->integral = 0.0f;  // Ki disabled: don't carry stale integral
  }

  // ----- Derivative term -----
  // d(error)/dt. Encoder velocity is noisy, so derivative gain Kd should be
  // small or zero unless you've smoothed the input.
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  return pid->kp * error
       + pid->ki * pid->integral
       + pid->kd * derivative;
}

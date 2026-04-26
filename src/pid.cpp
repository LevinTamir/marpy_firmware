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
  // PID disabled: skip the work and let FF pass through.
  if (pid->kp == 0.0f && pid->ki == 0.0f && pid->kd == 0.0f) {
    pid->integral = 0.0f;
    pid->last_us  = now_us;
    return 0.0f;
  }

  // Need a previous timestamp to compute dt.
  if (pid->last_us == 0) {
    pid->last_us    = now_us;
    pid->prev_error = setpoint - measured;
    return 0.0f;
  }

  float dt = (now_us - pid->last_us) * 1e-6f;
  pid->last_us = now_us;

  // Reject bogus dt (clock didn't advance, rolled over, or loop stalled).
  if (dt <= 0.0f || dt > 0.5f) {
    pid->prev_error = setpoint - measured;
    return 0.0f;
  }

  float error = setpoint - measured;

  // Anti-windup: clamp the integral so the I contribution stays under
  // i_term_max. Clamping the integral (not the output) keeps the saturation
  // point stable when ki is changed.
  pid->integral += error * dt;
  if (pid->ki > 1e-6f) {
    float i_clamp = pid->i_term_max / pid->ki;
    if (pid->integral >  i_clamp) pid->integral =  i_clamp;
    if (pid->integral < -i_clamp) pid->integral = -i_clamp;
  } else {
    pid->integral = 0.0f;
  }

  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  return pid->kp * error
       + pid->ki * pid->integral
       + pid->kd * derivative;
}

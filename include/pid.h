// Generic single-channel PID controller.
//
// One instance per controlled variable; MARPY uses two (left + right wheel).
// Output is in whatever units the gains map to (here: PWM duty).
#pragma once

#include <stdint.h>

struct PID {
  float kp;          // proportional: react to current error
  float ki;          // integral:     react to accumulated error
  float kd;          // derivative:   react to error's rate of change

  // Anti-windup cap on the I-term contribution (in output units). Without
  // it, sustained saturation makes the integrator grow unboundedly and the
  // controller slams the actuator long after the error clears.
  float i_term_max;

  // Internal state.
  float integral;
  float prev_error;
  uint32_t last_us;
};

void pid_init(PID *pid, float kp, float ki, float kd, float i_term_max);
void pid_set_gains(PID *pid, float kp, float ki, float kd, bool reset_state);
void pid_reset(PID *pid);

// Returns Kp*e + Ki*integral(e) + Kd*de/dt. First call (or any with bogus
// dt) returns 0 to avoid a derivative blowup. All-zero gains short-circuit
// to 0 (PID effectively disabled).
float pid_step(PID *pid, float setpoint, float measured, uint32_t now_us);

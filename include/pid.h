// Generic single-channel PID controller.
//
// One PID instance per controlled variable. For MARPY we use two: one for the
// left wheel velocity and one for the right wheel velocity. Each instance
// keeps its own integral / derivative memory; the gains can be shared.
//
// Typical use:
//
//   PID pid;
//   pid_init(&pid, 100.0f, 50.0f, 0.0f, 80.0f);   // kp, ki, kd, i_term_max
//
//   // every loop iteration:
//   float correction = pid_step(&pid, target_speed, measured_speed, micros());
//   apply_motor_duty(feed_forward + correction);
//
// The output is in whatever units make sense for the actuator (here: PWM duty).
// Tune the gains in motor_config.h or live at runtime via the /pid_gains topic.
#pragma once

#include <stdint.h>

struct PID {
  // ----- Tunable gains -----
  float kp;          // proportional: how strongly to react to the current error
  float ki;          // integral:     how strongly to react to accumulated error
  float kd;          // derivative:   how strongly to react to the error's rate

  // Anti-windup cap on the I term (in output units). Without this, sustained
  // saturation (e.g. wheel jammed) makes the integrator grow unboundedly and
  // the controller slams the actuator long after the error clears.
  float i_term_max;

  // ----- Internal state (do not poke from outside) -----
  float integral;
  float prev_error;
  uint32_t last_us;
};

// Initialise gains and zero the internal state.
void pid_init(PID *pid, float kp, float ki, float kd, float i_term_max);

// Update the gains in-place. Does NOT reset the integrator unless you want
// it to: pass `reset_state = true` to clear accumulated history (recommended
// when the gain change is large, to avoid a sudden output jump).
void pid_set_gains(PID *pid, float kp, float ki, float kd, bool reset_state);

// Clear integral / derivative memory so the next step starts fresh.
void pid_reset(PID *pid);

// One update step. Call every loop iteration.
//
//   setpoint  : the value you want   (e.g. target wheel speed in m/s)
//   measured  : the value you have   (e.g. encoder-derived wheel speed)
//   now_us    : current micros() timestamp
//
// Returns the controller output, in the same units as kp/ki/kd map to.
// First call (or any with bogus dt) returns 0 to avoid a derivative blowup.
float pid_step(PID *pid, float setpoint, float measured, uint32_t now_us);

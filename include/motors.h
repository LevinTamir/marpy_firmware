// Motor + encoder layer for the differential drive.
//
// Pipeline (one cycle of motors_update()):
//
//   v_*_cmd           <-- set by motors_apply_cmd_vel() from /cmd_vel
//      |
//      v   EMA toward latest cmd   ->  vL_filt / vR_filt  (smoothed setpoint)
//      |
//      v   ff_duty + PID correction  -- PID's setpoint is the EMA output,
//      |                               feedback is encoder-derived velocity
//      v
//   apply_motor_*_duty(signed PWM)  -> H-bridge
//
// PID is disabled by default (gains = 0 in motor_config.h). With gains=0
// pid_step() returns 0 and the wheels run pure feed-forward, matching the
// behavior of the EMA-only baseline. Tune live via /pid_gains, then bake
// the working values into motor_config.h.
//
// motors_update() also handles the safety brake when /cmd_vel goes stale.
#pragma once

void motors_setup();
void motors_update();   // call every loop iteration

// Apply a Twist command (linear.x m/s, angular.z rad/s) to the wheels.
// Updates the cmd timestamp internally.
void motors_apply_cmd_vel(float linear_x, float angular_z);

// Live PID retune. Resets the integrators so a fresh I term doesn't jolt
// the output at the new gains.
void motors_set_pid_gains(float kp, float ki, float kd);

// For /joint_states publishing (radians, rad/s).
double motors_left_position_rad();
double motors_right_position_rad();
double motors_left_velocity_radps();
double motors_right_velocity_radps();

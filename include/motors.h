// Motor + encoder + per-wheel velocity PID for the differential drive.
//
// motors_update() runs the pipeline each loop iteration: EMA the cmd into a
// smoothed setpoint, compute deadzone-compensated FF duty, add PID
// correction (zero unless gains are set), drive the H-bridge. Also brakes
// when /cmd_vel goes stale.
#pragma once

void motors_setup();
void motors_update();   // call every loop iteration

// Apply a Twist command (linear.x m/s, angular.z rad/s). Updates the cmd
// timestamp internally.
void motors_apply_cmd_vel(float linear_x, float angular_z);

// Live PID retune. Resets the integrators so a fresh I term doesn't jolt.
void motors_set_pid_gains(float kp, float ki, float kd);

// For /joint_states publishing.
double motors_left_position_rad();
double motors_right_position_rad();
double motors_left_velocity_radps();
double motors_right_velocity_radps();

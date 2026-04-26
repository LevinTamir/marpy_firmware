// Motor + encoder + per-wheel velocity PID for the differential drive.
//
// Architecture (one cycle of motors_update()):
//
//   v_*_cmd   <-- set by motors_set_velocity_from_twist()
//      |
//      v
//   slew at MAX_ACCEL_MS2  -->  v_*_target
//      |
//      v
//   PID(setpoint=v_*_target, feedback=wheel_vel_*_ms)  -->  duty correction
//      |
//      v
//   feed-forward duty + correction  -->  apply_motor_*_duty (signed PWM)
//
// If no /cmd_vel arrives for CMD_TIMEOUT_MS, motors_update() active-brakes
// both wheels until a fresh command appears.
#pragma once

void motors_setup();
void motors_update();   // call every loop iteration

// Convert a Twist (linear.x m/s, angular.z rad/s) into per-wheel commands.
// Applies LIN_GAIN / WZ_GAIN shaping, deadbands, in-place-spin handling, and
// the unicycle->wheels mapping. Resets the cmd timestamp internally.
void motors_set_velocity_from_twist(float linear_x, float angular_z);

// Live PID retune. Resets the integrators so a fresh I term doesn't jolt
// the output at the new gains.
void motors_set_pid_gains(float kp, float ki, float kd);

// Joint state, ready to publish on /joint_states. All values are signed in
// the URDF convention (positive = forward), even when the motor wiring is
// flipped via LEFT_DIR_INVERT / RIGHT_DIR_INVERT.
float motors_left_position_rad();
float motors_right_position_rad();
float motors_left_velocity_radps();
float motors_right_velocity_radps();

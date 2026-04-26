// Tunable motor parameters. Defaults are baked in at compile time and can
// be overridden at runtime via the /pid_gains topic:
//
//   ros2 topic pub /pid_gains std_msgs/msg/Float32MultiArray \
//     "{data: [50.0, 10.0, 0.0]}" --once
//
// data layout: [Kp, Ki, Kd]. Live updates do not persist across reboots --
// once you find a setup you like, copy the values into the defaults below.
#pragma once

// ===== Per-wheel velocity PID =====
// PID is disabled by default (zeros) so the firmware ships behaving like the
// open-loop EMA baseline. To enable closed-loop wheel-speed tracking, tune
// live via /pid_gains and then update these values.
//
// Suggested first values to try when tuning live:
//
//   Kp = 50.0  Ki = 10.0  Kd = 0.0
//
// Iterate in small steps. Watch /joint_states velocity while driving --
// audible chatter or oscillation means the gains are too high; halve and
// retry. Keep Kd at 0 unless you've smoothed the velocity feedback (encoder
// derivative is noisy and Kd typically amplifies that noise).
#define DEFAULT_PID_KP                  0.0f
#define DEFAULT_PID_KI                  0.0f
#define DEFAULT_PID_KD                  0.0f
#define DEFAULT_PID_INTEGRAL_MAX_DUTY  50.0f  // anti-windup cap on the I term

// ===== Velocity estimation from encoders =====
// PID's feedback signal is sampled at this rate. Decoupled from the main
// loop so PID feedback isn't sensitive to loop-iteration jitter.
#define VEL_UPDATE_INTERVAL_MS  20   // 50 Hz

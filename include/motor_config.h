// Tunable motor / PID parameters.
//
// Defaults below are baked in at compile time. They can also be overridden
// at runtime by publishing to /pid_gains:
//
//   ros2 topic pub /pid_gains std_msgs/msg/Float32MultiArray \
//     "{data: [200.0, 100.0, 0.0]}" --once
//
// data layout: [Kp, Ki, Kd]. Live updates do not persist across reboots.
// Once you find values you like, bake them into the defaults below.
#pragma once

// ===== Acceleration limit on commanded wheel velocity =====
// Caps how fast the slewed setpoint can change (m/s^2). Lower = gentler.
#define DEFAULT_MAX_ACCEL_MS2 2.0f

// ===== Per-wheel velocity PID =====
// Error is in m/s, output is PWM duty added on top of a feed-forward estimate.
#define DEFAULT_PID_KP                100.0f  // duty per (m/s) error
#define DEFAULT_PID_KI                 50.0f  // duty per (m/s*s) integral
#define DEFAULT_PID_KD                  0.0f  // duty * s per (m/s) deriv
#define DEFAULT_PID_INTEGRAL_MAX_DUTY  80.0f  // anti-windup cap on the I term

// ===== Velocity estimation from encoders =====
#define VEL_UPDATE_INTERVAL_MS  20   // 50 Hz wheel-speed update

// ===== Gyro bias calibration (run at boot while the robot sits still) =====
#define GYRO_CALIB_SAMPLES   200
#define GYRO_CALIB_DT_MS       5

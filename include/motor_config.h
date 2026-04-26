// Tunable motor parameters. Defaults compile in; can be overridden at
// runtime via the /pid_gains topic:
//
//   ros2 topic pub /pid_gains std_msgs/msg/Float32MultiArray \
//     "{data: [Kp, Ki, Kd]}" --once
//
// Live updates do not persist across reboots. Once you find values you
// like, copy them into the defaults below.
#pragma once

// Per-wheel velocity PID. Disabled by default (zeros) so the firmware
// runs pure open-loop FF. Suggested live-tune starting point: 200, 100, 0.
// Kd usually stays at 0 (encoder-derived velocity is too noisy for it).
#define DEFAULT_PID_KP                   0.0f
#define DEFAULT_PID_KI                   0.0f
#define DEFAULT_PID_KD                   0.0f
// Anti-windup cap on the I term (PWM-duty units). Sized so PID can push
// past stiction when the FF underestimates.
#define DEFAULT_PID_INTEGRAL_MAX_DUTY  150.0f

// Velocity sample rate for PID feedback (decoupled from main-loop jitter).
#define VEL_UPDATE_INTERVAL_MS  20   // 50 Hz

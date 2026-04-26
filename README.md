# MARPY Firmware

ESP32 micro-ROS firmware for the [MARPY](https://github.com/LevinTamir/MARPY) differential-drive robot.

Subscribes to `/cmd_vel` and publishes `/joint_states` and `/imu` over Wi-Fi UDP, with deadzone-compensated open-loop FF and an opt-in per-wheel velocity PID.

## Getting started

Full setup and flashing instructions live in the main repo:

**[MARPY - Firmware Setup Guide](https://github.com/LevinTamir/MARPY/blob/main/docs/firmware-setup.md)**

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel`      | `geometry_msgs/Twist`            | PC -> ESP32 |
| `/pid_gains`    | `std_msgs/Float32MultiArray`     | PC -> ESP32 (live PID tune: `[Kp, Ki, Kd]`) |
| `/joint_states` | `sensor_msgs/JointState`         | ESP32 -> PC |
| `/imu`          | `sensor_msgs/Imu`                | ESP32 -> PC |

## Code layout

| File | Concern |
|------|---------|
| `pins.h`           | central GPIO map |
| `motor_config.h`   | PID gain defaults |
| `pid.{h,cpp}`      | generic PID controller |
| `motors.{h,cpp}`   | PWM, encoders, FF + PID, kinematics |
| `imu.{h,cpp}`      | MPU6050 driver |
| `microros.{h,cpp}` | Wi-Fi, pubs/subs, callbacks |
| `main.cpp`         | `setup()` + `loop()` orchestration |

## License

MIT

# MERPY Firmware

ESP32 micro-ROS firmware for the [MERPY](https://github.com/LevinTamir/MERPY) differential-drive robot.

Subscribes to `/cmd_vel` and publishes `/joint_states` over WiFi UDP, turning velocity commands into motor control with encoder feedback.

## What's inside

- Differential-drive kinematics (unicycle model)
- Quadrature encoder reading via hardware interrupts
- 20 Hz joint state publishing
- Command timeout safety stop
- WiFi auto-reconnect

## Getting started

See the full setup and flashing instructions in the main repo:

**[MERPY - Firmware Setup Guide](https://github.com/LevinTamir/MERPY/blob/main/docs/firmware-setup.md)**

## Quick reference

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | PC -> ESP32 |
| `/joint_states` | `sensor_msgs/JointState` | ESP32 -> PC |

## License

MIT

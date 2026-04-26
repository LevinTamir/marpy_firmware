// micro-ROS comms layer: WiFi connection, transport, node, pubs/subs,
// executor, callbacks. Hides all the rcl/rclc boilerplate from main.cpp.
//
// Subscribes:
//   /cmd_vel   (geometry_msgs/Twist)        -> motors_set_velocity_from_twist
//   /pid_gains (std_msgs/Float32MultiArray) -> motors_set_pid_gains [Kp, Ki, Kd]
//
// Publishes:
//   /joint_states (sensor_msgs/JointState)  via microros_publish_joint_state
//   /imu          (sensor_msgs/Imu)         via microros_publish_imu
#pragma once

#include <stdbool.h>

// Connect WiFi, bring up the transport, wait for the agent, sync the clock,
// and create all subscribers/publishers + executor. Returns true on success;
// on failure the function logs and the caller should halt.
bool microros_setup();

// Process incoming messages (cmd_vel, pid_gains) and keep WiFi alive.
// Call once per loop iteration.
void microros_spin_some();

// Publish a joint_state sample. Position / velocity values are URDF-signed
// (positive = forward) and in rad / rad-per-second respectively.
void microros_publish_joint_state(float left_pos_rad,  float right_pos_rad,
                                  float left_vel_radps, float right_vel_radps);

// Publish an IMU sample. Accel in m/s^2, gyro in rad/s. Orientation is
// reported as "not provided" (covariance[0] = -1) since raw MPU6050 has none.
void microros_publish_imu(float ax, float ay, float az,
                          float gx, float gy, float gz);

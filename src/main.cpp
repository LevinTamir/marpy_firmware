// MARPY firmware entry point. All real work lives in:
//   motors.{h,cpp}    motor PWM, encoders, slew + per-wheel PID
//   pid.{h,cpp}       generic PID controller used by motors
//   imu.{h,cpp}       MPU6050 driver + gyro bias calibration
//   microros.{h,cpp}  WiFi, micro-ROS pubs/subs, callbacks
//
// main() just wires those modules together and runs the publish timers.
#include <Arduino.h>

#include "motors.h"
#include "imu.h"
#include "microros.h"

static const uint32_t JOINT_STATE_PUB_INTERVAL_MS = 50;  // 20 Hz
static const uint32_t IMU_PUB_INTERVAL_MS         = 20;  // 50 Hz

void setup() {
  Serial.begin(115200);
  delay(200);

  motors_setup();

  if (!microros_setup()) {
    Serial.println("[Setup] micro-ROS init failed; halting");
    while (true) delay(1000);
  }

  if (imu_setup()) {
    Serial.println("[IMU] MPU6050 ready");
  } else {
    Serial.println("[IMU] MPU6050 init FAILED; continuing without IMU");
  }

  Serial.println("[Setup] Ready: sub /cmd_vel /pid_gains, pub /joint_states /imu");
}

void loop() {
  microros_spin_some();
  motors_update();

  // Publish joint state at ~20 Hz.
  static uint32_t last_js_ms = 0;
  if (millis() - last_js_ms >= JOINT_STATE_PUB_INTERVAL_MS) {
    last_js_ms = millis();
    microros_publish_joint_state(
      motors_left_position_rad(),  motors_right_position_rad(),
      motors_left_velocity_radps(), motors_right_velocity_radps());
  }

  // Publish IMU at ~50 Hz (only if the chip came up).
  static uint32_t last_imu_ms = 0;
  if (imu_initialized && millis() - last_imu_ms >= IMU_PUB_INTERVAL_MS) {
    last_imu_ms = millis();
    float ax, ay, az, gx, gy, gz;
    imu_read(&ax, &ay, &az, &gx, &gy, &gz);
    microros_publish_imu(ax, ay, az, gx, gy, gz);
  }

  delay(2);
}

// micro-ROS comms layer for MARPY. See microros.h for the public contract.
#include "microros.h"

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>

#include "wifi_config.h"
#include "motors.h"

// =================== Handles ==========================
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

static rcl_subscription_t cmd_sub;
static rcl_subscription_t pid_gains_sub;
static rcl_publisher_t    joint_state_pub;
static rcl_publisher_t    imu_pub;

static geometry_msgs__msg__Twist             cmd_msg;
static std_msgs__msg__Float32MultiArray      pid_gains_msg;
static sensor_msgs__msg__JointState          joint_state_msg;
static sensor_msgs__msg__Imu                 imu_msg;

// =================== WiFi helpers =====================
static bool connect_wifi(unsigned long timeout_ms = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > timeout_ms) return false;
    delay(250);
  }
  return true;
}

static void wait_for_agent() {
  while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) {
    delay(500);
  }
}

// =================== Time stamping ====================
// Fill a builtin_interfaces/Time from the synced ROS epoch. Falls back to
// millis()-since-boot if the agent hasn't synced yet. Without sync,
// robot_state_publisher would re-broadcast wheel TFs at boot-relative
// stamps, causing /tf chains rooted at /odom to render wheels at the world
// origin in RViz.
static void stamp_now(builtin_interfaces__msg__Time *out) {
  int64_t ns = rmw_uros_epoch_nanos();
  if (ns > 0) {
    out->sec     = (int32_t)(ns / 1000000000LL);
    out->nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    out->sec     = (int32_t)(ms / 1000U);
    out->nanosec = (uint32_t)((ms % 1000U) * 1000000U);
  }
}

// =================== Callbacks ========================
static void cmd_vel_cb(const void *msgin) {
  const auto *m = (const geometry_msgs__msg__Twist *)msgin;
  motors_apply_cmd_vel((float)m->linear.x, (float)m->angular.z);
}

// Live PID tuning. Float32MultiArray.data layout: [Kp, Ki, Kd].
static void pid_gains_cb(const void *msgin) {
  const auto *m = (const std_msgs__msg__Float32MultiArray *)msgin;
  float kp = (m->data.size >= 1) ? m->data.data[0] : 0.0f;
  float ki = (m->data.size >= 2) ? m->data.data[1] : 0.0f;
  float kd = (m->data.size >= 3) ? m->data.data[2] : 0.0f;
  motors_set_pid_gains(kp, ki, kd);
  Serial.printf("[PID] gains updated: Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
}

// =================== Init helpers =====================
static void init_joint_state_msg() {
  sensor_msgs__msg__JointState__init(&joint_state_msg);

  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.size     = 2;
  joint_state_msg.name.data     = (rosidl_runtime_c__String *)
                                  malloc(2 * sizeof(rosidl_runtime_c__String));
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");

  joint_state_msg.position.capacity = 2;
  joint_state_msg.position.size     = 2;
  joint_state_msg.position.data     = (double *)malloc(2 * sizeof(double));

  joint_state_msg.velocity.capacity = 2;
  joint_state_msg.velocity.size     = 2;
  joint_state_msg.velocity.data     = (double *)malloc(2 * sizeof(double));

  joint_state_msg.effort.capacity = 0;
  joint_state_msg.effort.size     = 0;
  joint_state_msg.effort.data     = NULL;
}

static void init_imu_msg() {
  sensor_msgs__msg__Imu__init(&imu_msg);
  imu_msg.header.frame_id.data     = (char *)"imu_link";
  imu_msg.header.frame_id.size     = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
  // Orientation not provided (raw MPU6050 has none).
  imu_msg.orientation_covariance[0] = -1.0;
  for (int i = 0; i < 9; i++) {
    imu_msg.linear_acceleration_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i]    = 0.0;
  }
  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;
  imu_msg.angular_velocity_covariance[0]    = 0.001;
  imu_msg.angular_velocity_covariance[4]    = 0.001;
  imu_msg.angular_velocity_covariance[8]    = 0.001;
}

static void init_pid_gains_msg() {
  std_msgs__msg__Float32MultiArray__init(&pid_gains_msg);
  pid_gains_msg.data.capacity = 8;
  pid_gains_msg.data.size     = 0;
  pid_gains_msg.data.data     = (float *)malloc(8 * sizeof(float));
}

// =================== Public API =======================
bool microros_setup() {
  if (!connect_wifi()) {
    Serial.println("[WiFi] connect failed");
    return false;
  }
  Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());

  set_microros_wifi_transports(WIFI_SSID, WIFI_PSK, AGENT_IP, AGENT_PORT);
  Serial.println("[micro-ROS] waiting for agent...");
  wait_for_agent();
  Serial.println("[micro-ROS] agent found");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_diffdrive_node", "", &support);

  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("[micro-ROS] time synced with agent");
  } else {
    Serial.println("[micro-ROS] time sync FAILED (will publish best-effort)");
  }

  geometry_msgs__msg__Twist__init(&cmd_msg);
  rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  init_pid_gains_msg();
  rclc_subscription_init_default(
    &pid_gains_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/pid_gains");

  init_joint_state_msg();
  rclc_publisher_init_default(
    &joint_state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states");

  init_imu_msg();
  rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu");

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub,       &cmd_msg,       &cmd_vel_cb,   ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pid_gains_sub, &pid_gains_msg, &pid_gains_cb, ON_NEW_DATA);

  return true;
}

void microros_spin_some() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PSK);
  }
}

void microros_publish_joint_state(double left_pos_rad,  double right_pos_rad,
                                  double left_vel_radps, double right_vel_radps) {
  joint_state_msg.position.data[0] = left_pos_rad;
  joint_state_msg.position.data[1] = right_pos_rad;
  joint_state_msg.velocity.data[0] = left_vel_radps;
  joint_state_msg.velocity.data[1] = right_vel_radps;
  stamp_now(&joint_state_msg.header.stamp);
  rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
}

void microros_publish_imu(float ax, float ay, float az,
                          float gx, float gy, float gz) {
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.angular_velocity.x    = gx;
  imu_msg.angular_velocity.y    = gy;
  imu_msg.angular_velocity.z    = gz;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;
  stamp_now(&imu_msg.header.stamp);
  rcl_publish(&imu_pub, &imu_msg, NULL);
}

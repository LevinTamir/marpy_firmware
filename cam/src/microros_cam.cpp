#include "microros_cam.h"

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/compressed_image.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "wifi_config.h"

// =================== Config ===========================
// Generous: a 320x240 JPEG at quality 12 sits well under 25 KB; QVGA at q=15
// or HQVGA at q=12 are smaller still. Bump if you crank quality / framesize.
static const size_t JPEG_BUF_CAPACITY = 60 * 1024;
static const char  *NODE_NAME         = "marpy_cam";
static const char  *TOPIC_NAME        = "/camera/image_compressed";
static const char  *FRAME_ID          = "camera_optical_frame";
static const char  *FORMAT_JPEG       = "jpeg";

// =================== Handles ==========================
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

static rcl_publisher_t                       image_pub;
static sensor_msgs__msg__CompressedImage     image_msg;

// =================== WiFi =============================
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
// Fall back to millis() if the agent hasn't synced yet, so frames published
// pre-sync still have monotonic stamps (RViz tolerates boot-relative time
// better than zero stamps).
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

// =================== Message init =====================
static void init_image_msg() {
  sensor_msgs__msg__CompressedImage__init(&image_msg);

  rosidl_runtime_c__String__assign(&image_msg.header.frame_id, FRAME_ID);
  rosidl_runtime_c__String__assign(&image_msg.format,          FORMAT_JPEG);

  // Preallocate the JPEG buffer once. Hot path uses memcpy + size update only.
  rosidl_runtime_c__uint8__Sequence__init(&image_msg.data, JPEG_BUF_CAPACITY);
  image_msg.data.size = 0;
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
  rclc_node_init_default(&node, NODE_NAME, "", &support);

  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("[micro-ROS] time synced with agent");
  } else {
    Serial.println("[micro-ROS] time sync FAILED (will publish best-effort)");
  }

  init_image_msg();
  // Best-effort: image-sized payloads can't tolerate retransmits.
  rclc_publisher_init_best_effort(
    &image_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    TOPIC_NAME);

  // No subscriptions, but the executor still needs to exist for clock sync
  // and any future timers we add.
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  return true;
}

void microros_spin_some() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PSK);
  }
}

void microros_publish_jpeg(const uint8_t *jpeg, size_t len) {
  if (!jpeg || len == 0) return;
  if (len > image_msg.data.capacity) {
    static bool warned = false;
    if (!warned) {
      Serial.printf("[micro-ROS] JPEG %u B exceeds buffer %u B; dropping (logged once)\n",
                    (unsigned)len, (unsigned)image_msg.data.capacity);
      warned = true;
    }
    return;
  }

  memcpy(image_msg.data.data, jpeg, len);
  image_msg.data.size = len;
  stamp_now(&image_msg.header.stamp);
  rcl_publish(&image_pub, &image_msg, NULL);
}

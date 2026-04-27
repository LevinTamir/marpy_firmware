// micro-ROS comms layer for the cam firmware. Wi-Fi, transport, node, one
// publisher, executor. Hides the rcl/rclc boilerplate from main.cpp.
//
// Publishes:
//   /camera/image_compressed (sensor_msgs/CompressedImage, BEST_EFFORT QoS)
//
// The publisher is best-effort: reliable QoS retransmits and crushes
// throughput at image-sized payloads. The data buffer is preallocated once
// at startup so the publish path never allocates.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Connect WiFi, bring up the transport, wait for the agent, sync the clock,
// create the publisher and executor. Returns true on success.
bool microros_setup();

// Spin the executor and keep WiFi alive. Call once per loop iteration.
void microros_spin_some();

// Publish a JPEG frame. The buffer is copied into the preallocated message
// data; the caller can release the camera frame as soon as this returns.
// `len` larger than the preallocated capacity is dropped (logged once).
void microros_publish_jpeg(const uint8_t *jpeg, size_t len);

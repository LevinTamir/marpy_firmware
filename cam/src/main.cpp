// MARPY cam firmware entry point. All real work lives in:
//   camera.{h,cpp}        esp32-camera init + frame grab/release
//   microros_cam.{h,cpp}  WiFi, micro-ROS publisher, callbacks
//
// main() wires those modules together and runs the publish hot loop.
#include <Arduino.h>

#include "camera.h"
#include "microros_cam.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!camera_setup()) {
    Serial.println("[Setup] camera init failed; halting");
    while (true) delay(1000);
  }
  Serial.println("[Camera] OV2640 ready");

  if (!microros_setup()) {
    Serial.println("[Setup] micro-ROS init failed; halting");
    while (true) delay(1000);
  }

  Serial.println("[Setup] Ready: pub /camera/image_compressed");
}

void loop() {
  microros_spin_some();

  // Latency-bound, not spin-bound: grab, publish, release on every iteration.
  // CAMERA_GRAB_LATEST means we always get the freshest frame; older frames
  // in the FIFO are discarded by the driver.
  camera_fb_t *fb = camera_grab();
  if (fb) {
    microros_publish_jpeg(fb->buf, fb->len);
    camera_release(fb);
  }
}

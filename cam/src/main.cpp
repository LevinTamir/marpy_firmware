// MARPY cam firmware entry point. The cam acts as a plain MJPEG webcam:
//   camera.{h,cpp}      esp32-camera init + frame grab/release
//   http_server.{h,cpp} Wi-Fi, mDNS, /stream endpoint
//
// A ROS 2 bridge node on the workstation (marpy_cam_bridge) pulls the
// stream and republishes frames as sensor_msgs/Image.
#include <Arduino.h>

#include "camera.h"
#include "http_server.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!camera_setup()) {
    Serial.println("[Setup] camera init failed; halting");
    while (true) delay(1000);
  }
  Serial.println("[Camera] OV2640 ready");

  if (!http_server_setup()) {
    Serial.println("[Setup] http server failed; halting");
    while (true) delay(1000);
  }

  Serial.println("[Setup] Ready: http://marpy-cam.local:81/stream");
}

void loop() {
  // The HTTP server runs in its own task. Nothing to do here; just yield.
  delay(1000);
}

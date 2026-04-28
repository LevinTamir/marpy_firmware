// Wi-Fi + mDNS + MJPEG HTTP server. Replaces the old micro-ROS publisher.
//
// Once started, the cam serves an MJPEG (multipart/x-mixed-replace) stream
// at:
//   http://marpy-cam.local/stream
//   http://<cam-ip>/stream
//
// A ROS 2 bridge node on the workstation pulls this stream and republishes
// frames as sensor_msgs/Image. ESP32-CAM with WiFiUDP can't reliably ship
// JPEG-sized payloads through micro-ROS, so HTTP is the robust path.
#pragma once

#include <stdbool.h>

// Bring up Wi-Fi (blocking until associated), advertise mDNS as
// "marpy-cam", and start the HTTP server. Returns true on success.
bool http_server_setup();

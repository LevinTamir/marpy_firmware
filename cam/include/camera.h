// esp32-camera wrapper: init the OV2640 sensor and hand out JPEG frames.
//
// Frames are owned by the camera driver. Always pair grab() with release().
#pragma once

#include <stdbool.h>
#include <esp_camera.h>  // for camera_fb_t (typedef'd anonymous struct, can't forward-declare)

// Initialise the sensor with the AI-Thinker pin map and a JPEG/QVGA config.
// Returns true on success. Logs the failure reason on Serial otherwise.
bool camera_setup();

// Grab the most recent JPEG frame. Returns nullptr if no frame is ready.
// Caller MUST call camera_release() on every non-null return value.
camera_fb_t *camera_grab();

// Hand a frame back to the driver so its slot can be reused.
void camera_release(camera_fb_t *fb);

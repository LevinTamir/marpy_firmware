#include "camera.h"

#include <Arduino.h>
#include <esp_camera.h>

#include "camera_pins.h"

// =================== Config ===========================
// Tuneable starting point. See cam/README.md for the FPS tuning notes.
// HQVGA + q=20 is small (~2 KB/frame) so it tolerates a mediocre Wi-Fi link.
// Bump framesize to QVGA and quality down to 12 once the link is solid.
static const framesize_t START_FRAMESIZE  = FRAMESIZE_HQVGA; // 240x176
static const int         START_JPEG_Q     = 20;              // higher = smaller, more artifacts
static const size_t      FB_COUNT         = 2;

// =================== Public API =======================
bool camera_setup() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = START_FRAMESIZE;
  cfg.jpeg_quality = START_JPEG_Q;
  cfg.fb_count     = FB_COUNT;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[Camera] init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

camera_fb_t *camera_grab() {
  return esp_camera_fb_get();
}

void camera_release(camera_fb_t *fb) {
  if (fb) esp_camera_fb_return(fb);
}

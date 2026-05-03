#include "http_server.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <esp_wifi.h>

#include "wifi_config.h"

// =================== Config ===========================
static const char *MDNS_HOSTNAME = "marpy-cam";

// Multipart format follows the Espressif CameraWebServer reference. The
// boundary string can be anything that won't appear in JPEG payload.
#define PART_BOUNDARY "123456789000000000000987654321"

static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART     =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

// Two separate httpd instances per the reference: stream on its own port
// so a long-lived stream connection doesn't starve the index page handler.
static httpd_handle_t s_stream_httpd = NULL;
static httpd_handle_t s_index_httpd  = NULL;

// =================== Wi-Fi ============================
static bool connect_wifi(unsigned long timeout_ms = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // no Modem-sleep (Arduino API)
  // Crank TX power. AI-Thinker's PCB antenna is weak; every dB matters.
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > timeout_ms) return false;
    delay(250);
  }
  // Belt + suspenders: also call the IDF API directly. Streaming workloads
  // suffer badly from any power-save state.
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.printf("[WiFi] RSSI %d dBm, BSSID %s, ch %d\n",
                WiFi.RSSI(), WiFi.BSSIDstr().c_str(), WiFi.channel());
  return true;
}

// =================== Stream handler ===================
// Lifted from esp32-camera's CameraWebServer example with minor cleanup.
// The key bits versus a naive implementation:
//   - Re-encode to JPEG only when the sensor isn't already producing JPEG.
//   - Send headers + payload in three separate httpd_resp_send_chunk calls
//     using a single buffer reuse, never holding two frames at once.
//   - Always release the camera framebuffer (or the malloc'd JPEG buf)
//     before bailing on send error. This is what was missing from the
//     previous custom handler that wedged after the first request.
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t jpg_buf_len = 0;
  uint8_t *jpg_buf = NULL;
  char part_buf[128];
  struct timeval ts;

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  Serial.printf("[HTTP] stream client connected, RSSI=%d dBm\n", WiFi.RSSI());
  uint32_t frame_n = 0;
  uint32_t last_log_ms = millis();

  while (true) {
    uint32_t t0 = millis();
    fb = esp_camera_fb_get();
    uint32_t t_grab = millis() - t0;
    if (!fb) {
      Serial.println("[HTTP] camera_fb_get failed");
      res = ESP_FAIL;
      break;
    }
    gettimeofday(&ts, NULL);

    if (fb->format == PIXFORMAT_JPEG) {
      jpg_buf_len = fb->len;
      jpg_buf     = fb->buf;
    } else {
      // Sensor not configured for JPEG. Convert in software, then drop the
      // sensor frame. This branch is unused for our config but kept for
      // parity with the reference.
      bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!ok) {
        Serial.println("[HTTP] frame2jpg failed");
        res = ESP_FAIL;
        break;
      }
    }

    uint32_t t1 = millis();
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART,
                             (unsigned)jpg_buf_len, (int)ts.tv_sec, (int)ts.tv_usec);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
    }
    uint32_t t_send = millis() - t1;

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      jpg_buf = NULL;
    } else if (jpg_buf) {
      free(jpg_buf);
      jpg_buf = NULL;
    }

    if (res != ESP_OK) {
      Serial.printf("[HTTP] stream end after %u frames\n", (unsigned)frame_n);
      break;
    }

    frame_n++;
    if (millis() - last_log_ms >= 2000) {
      Serial.printf("[HTTP] frame %u: grab=%ums send=%ums size=%uB RSSI=%d\n",
                    (unsigned)frame_n, (unsigned)t_grab, (unsigned)t_send,
                    (unsigned)jpg_buf_len, WiFi.RSSI());
      last_log_ms = millis();
    }

    // Yield to lwIP / Wi-Fi tasks so ACKs and incoming packets get serviced
    // promptly. Without this, ping/RTT to the cam blows up to seconds while
    // the stream loop runs.
    vTaskDelay(1);
  }

  return res;
}

// =================== Index handler ====================
static esp_err_t index_handler(httpd_req_t *req) {
  static const char *html =
      "<!doctype html><meta charset=utf-8><title>marpy-cam</title>"
      "<style>body{font-family:sans-serif;margin:2em;max-width:40em}</style>"
      "<h1>marpy-cam</h1>"
      "<p>MJPEG: <a href=\"http://marpy-cam.local:81/stream\">"
      "http://marpy-cam.local:81/stream</a> "
      "(also available at the cam IP)</p>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// =================== Public API =======================
bool http_server_setup() {
  Serial.print("[WiFi] connecting to "); Serial.println(WIFI_SSID);
  if (!connect_wifi()) {
    Serial.println("[WiFi] connect failed");
    return false;
  }
  Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("http", "tcp", 81);
    Serial.printf("[mDNS] http://%s.local/  (stream on :81/stream)\n", MDNS_HOSTNAME);
  } else {
    Serial.println("[mDNS] start failed (use the IP instead)");
  }

  // Index server on :80
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port   = 32768;
  config.max_uri_handlers = 4;
  // Force-evict oldest dead/idle connection when full. Without this, half-
  // closed streaming sockets pile up and the cam stops accepting new GETs.
  config.lru_purge_enable = true;
  // 7 is the practical IDF httpd ceiling with default LWIP (CONFIG_LWIP_MAX_SOCKETS=10
  // minus a few reserved). Larger pool plus shorter waits = faster recovery
  // when a client dies mid-stream.
  config.max_open_sockets = 7;
  config.send_wait_timeout = 2;
  config.recv_wait_timeout = 2;

  if (httpd_start(&s_index_httpd, &config) != ESP_OK) {
    Serial.println("[HTTP] index httpd_start failed");
    return false;
  }
  httpd_uri_t index_uri = {
      .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL,
  };
  httpd_register_uri_handler(s_index_httpd, &index_uri);

  // Stream server on :81 with its own task (matches the reference)
  config.server_port = 81;
  config.ctrl_port   = 32769;

  if (httpd_start(&s_stream_httpd, &config) != ESP_OK) {
    Serial.println("[HTTP] stream httpd_start failed");
    return false;
  }
  httpd_uri_t stream_uri = {
      .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL,
  };
  httpd_register_uri_handler(s_stream_httpd, &stream_uri);

  Serial.println("[HTTP] :80/  and  :81/stream  ready");
  return true;
}

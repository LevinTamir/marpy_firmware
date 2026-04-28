# MARPY Cam Firmware

ESP32-CAM (AI-Thinker, OV2640) vision firmware for the [MARPY](https://github.com/LevinTamir/MARPY) project. Serves an MJPEG stream over plain HTTP. A ROS 2 bridge node on the workstation (`marpy_cam_bridge` in [marpy_ws](https://github.com/LevinTamir/MARPY)) pulls the stream and republishes it as `sensor_msgs/Image`.

micro-ROS was the first attempt and ran into the well-known `WiFiUdp endPacket: 12` (EMSGSIZE) wall: image-sized payloads do not survive the ESP32 WiFiUDP stack, regardless of MTU tuning. HTTP MJPEG is the standard escape hatch for ESP32-CAM and what the rest of this firmware is built around.

## Endpoint

| URL | Content |
|-----|---------|
| `http://marpy-cam.local/`           | tiny landing page (port 80) |
| `http://marpy-cam.local:81/stream`  | `multipart/x-mixed-replace` MJPEG stream (port 81) |

The stream lives on a separate port so its long-lived connection doesn't starve the index page handler (this is the same split the official Espressif `CameraWebServer` example uses).

mDNS name is `marpy-cam`. If your network does not resolve `.local`, use the IP printed on the serial console at boot.

## Hardware

- AI-Thinker ESP32-CAM
- ESP32-CAM-MB programmer/power shield (USB-C, handles auto-DTR/RTS so no GPIO0 jumper)

## Code layout

| File | Description |
|------|-------------|
| `include/camera_pins.h`   | AI-Thinker OV2640 pin map |
| `include/camera.h`        | esp32-camera wrapper API |
| `include/http_server.h`   | Wi-Fi + mDNS + HTTP server API |
| `include/wifi_config.h`   | Wi-Fi credentials (gitignored, copy from `.example`) |
| `src/camera.cpp`          | sensor init, frame grab/release |
| `src/http_server.cpp`     | Wi-Fi connect, mDNS, `esp_http_server` MJPEG handler |
| `src/main.cpp`            | `setup()` + `loop()` orchestration |

## Configure Wi-Fi

```bash
cp include/wifi_config.h.example include/wifi_config.h
```

Fill in your SSID and password. `wifi_config.h` is gitignored. There is no agent IP or port to set: the cam just needs the network.

## Build and flash

```bash
pio run -d cam               # build
pio run -d cam -t upload     # build + flash
pio device monitor -d cam    # serial console
```

In VS Code with the PlatformIO extension, pick `cam` in the PROJECT TASKS sidebar and use Build / Upload / Monitor.

The MB shield handles boot/reset, no GPIO0 jumper or BOOT button mash needed.

## Verify

Open `http://marpy-cam.local:81/stream` in any browser. You should see a live video feed. That confirms the cam is working independently of ROS.

For the ROS path, in [marpy_ws](https://github.com/LevinTamir/MARPY) launch `real.launch.py` (or run the bridge directly):

```bash
ros2 run marpy_cam_bridge mjpeg_bridge --ros-args -p stream_url:=http://marpy-cam.local:81/stream
ros2 topic hz /camera/image_raw
```

In RViz: add an `Image` display and set the topic to `/camera/image_raw`. No image_transport plugins required.

## FPS tuning

The first knob to look at is **the Wi-Fi link**, not the camera. The cam logs RSSI on connect and per-2s timing during a stream:

```
[WiFi] RSSI -33 dBm, BSSID 82:F2:B0:64:62:04, ch 1
[HTTP] frame 132: grab=11ms send=64ms size=1909B RSSI=-33
```

Healthy: `grab` < 30 ms, `send` < 100 ms, RSSI better (closer to 0) than -65 dBm. If `send` is hundreds of milliseconds or more, the cam→AP→PC path is the bottleneck (move closer to router, change channel, swap APs) and no camera tweak will help.

Once the link is healthy, edit [src/camera.cpp](src/camera.cpp):

| Knob | Effect | Notes |
|------|--------|-------|
| `START_FRAMESIZE` | bigger frame, more pixels | `FRAMESIZE_HQVGA` (240x176, current) is small and fast; `FRAMESIZE_QVGA` (320x240) is the classic balance; `FRAMESIZE_VGA` (640x480) needs a strong link |
| `START_JPEG_Q`    | lower number = higher quality, larger frames | 12 is sharp, 20 (current) is small/blocky but plenty for teleop |
| `FB_COUNT`        | extra frame buffers in PSRAM | 2 is fine; more rarely helps |

`FB_COUNT = 2` and `CAMERA_GRAB_LATEST` mean the stream always serves the freshest frame, dropping any backlog automatically.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Camera init failed: 0x105` | Wrong board or PSRAM not detected. Confirm `-DBOARD_HAS_PSRAM` in `platformio.ini` and that the AI-Thinker variant is selected. |
| Browser opens `http://marpy-cam.local/` but stream is broken | Check that the camera grab is succeeding (serial: `[HTTP] camera_fb_get failed` means the sensor isn't producing frames). |
| `marpy-cam.local` doesn't resolve | mDNS isn't working on the network. Use the IP printed on the serial console. On Linux, `avahi-daemon` must be running. |
| Upload fails | The MB shield should auto-reset. If not, press the RST button on the cam board after the IDE shows "Connecting..." |
| Cam reboots mid-stream | Power supply too weak. The ESP32-CAM peaks well above what a thin USB cable from a hub provides; use a quality cable straight to a wall adapter. |
| Browser-side stream stutters / 1-2 s RTT to cam although RSSI is good | The Wi-Fi AP probably has client/AP isolation enabled (common on travel routers like GL.iNet). Disable it in the router admin UI, or put the PC and cam on the same physical AP without isolation. |

# MARPY Cam Firmware

ESP32-CAM (AI-Thinker, OV2640) vision firmware for the [MARPY](https://github.com/LevinTamir/MARPY) project. Streams JPEG frames over micro-ROS to the same agent the base firmware uses.

## Topic

| Topic | Type | QoS | Direction |
|-------|------|-----|-----------|
| `/camera/image_compressed` | `sensor_msgs/CompressedImage` | best-effort | ESP32-CAM -> PC |

`format = "jpeg"`, `header.frame_id = "camera_optical_frame"`. Node name is `marpy_cam` (the base firmware uses `marpy`, so both can connect to the same agent at `192.168.1.10:8888`).

## Hardware

- AI-Thinker ESP32-CAM
- ESP32-CAM-MB programmer/power shield (USB-C, handles auto-DTR/RTS so no GPIO0 jumper)

## Code layout

| File | Description |
|------|-------------|
| `include/camera_pins.h`   | AI-Thinker OV2640 pin map |
| `include/camera.h`        | esp32-camera wrapper API |
| `include/microros_cam.h`  | Wi-Fi + publisher API |
| `include/wifi_config.h`   | Wi-Fi + agent config (gitignored, copy from `.example`) |
| `src/camera.cpp`          | sensor init, frame grab/release |
| `src/microros_cam.cpp`    | Wi-Fi, transport, publisher, executor |
| `src/main.cpp`            | `setup()` + `loop()` orchestration |
| `colcon.meta`             | micro-ROS XRCE-DDS sizing overrides (bigger MTU, longer output stream history) |

## Configure Wi-Fi

```bash
cp include/wifi_config.h.example include/wifi_config.h
```

Edit it with the same SSID, password, and agent IP you used for the base firmware. `wifi_config.h` is gitignored.

## Build and flash

```bash
pio run -d cam               # build
pio run -d cam -t upload     # build + flash
pio device monitor -d cam    # serial console
```

Or in VS Code with the PlatformIO extension: pick `cam` in the PROJECT TASKS sidebar and use its Build / Upload / Monitor buttons.

The MB shield handles boot/reset, no GPIO0 jumper or BOOT button mash needed.

## Verify

With the agent running on the PC:

```bash
ros2 topic list                                           # /camera/image_compressed should appear
ros2 topic hz /camera/image_compressed                    # measure FPS
```

In RViz: add an `Image` display, set the topic to `/camera/image_compressed`, set the transport to `compressed`.

If you want raw frames for a node that does not speak compressed transport:
```bash
ros2 run image_transport republish compressed in:=/camera/image_compressed raw out:=/camera/image_raw
```

## FPS tuning sequence

Tune in this order, measuring `ros2 topic hz /camera/image_compressed` after each step.

1. **Baseline.** QVGA, quality 12, best-effort QoS, default MTU. Expect 5 to 12 FPS.
2. **MTU bump.** Confirm `colcon.meta` overrides applied. After editing `colcon.meta`, force the micro-ROS lib to rebuild:
   ```bash
   rm -rf cam/.pio/build/esp32cam/libmicroros
   pio run -d cam
   ```
   Confirm the agent log is not fragmenting at 1500 bytes. Expect +20 to 50%.
3. **Frame size sweep.** Try `FRAMESIZE_HQVGA` (240x176) for higher FPS, or stay at QVGA if already fine. Edit `START_FRAMESIZE` in [src/camera.cpp](src/camera.cpp).
4. **Quality sweep.** Raise `START_JPEG_Q` from 12 to 15 (smaller frames, more artifacts).
5. **Wi-Fi link.** AI-Thinker is 2.4 GHz only. Reduce 2.4 GHz interference if FPS is unstable.
6. **Decision point.** If after these you cannot reach roughly 10 FPS at QVGA, switch to the MJPEG-over-HTTP fallback (see the plan).

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Camera init failed: 0x105` | Wrong board or PSRAM not detected. Confirm `-DBOARD_HAS_PSRAM` in `platformio.ini` and that the AI-Thinker variant is selected. |
| `JPEG XX B exceeds buffer` log | Lower the framesize or raise the quality number, or bump `JPEG_BUF_CAPACITY` in [src/microros_cam.cpp](src/microros_cam.cpp). |
| Upload fails | The MB shield should auto-reset. If not, press the RST button on the cam board after the IDE shows "Connecting..." |
| Agent not found | Verify the agent is running (`docker ps`) and that the cam IP and the PC IP are on the same 2.4 GHz network. |

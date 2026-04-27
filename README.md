# MARPY Firmware

Micro-ROS firmware for the [MARPY](https://github.com/LevinTamir/MARPY) project. Two independent PlatformIO projects, one per MCU, sharing a Wi-Fi micro-ROS agent on the PC.

| Subfolder | MCU | Role |
|-----------|-----|------|
| [base/](base) | ESP32 (esp32dev) | drive controller: `/cmd_vel` in, `/joint_states` and `/imu` out |
| [cam/](cam)   | AI-Thinker ESP32-CAM (OV2640) | vision: `/camera/image_compressed` out (JPEG) |

Each subfolder is its own PlatformIO project with its own `platformio.ini`, so builds and flashes are independent:

```bash
pio run -d base -t upload
pio run -d cam  -t upload
```

See each subfolder's README for code layout, flashing notes, and troubleshooting:

- [base/README.md](base/README.md): drive-controller overview, topics, build/flash.
- [cam/README.md](cam/README.md): vision firmware, FPS tuning sequence, fallback plan.

The end-to-end setup guide (Wi-Fi config, agent setup on the PC, ROS2 launch) lives in the workspace repo:

**[MARPY - Firmware Setup Guide](https://github.com/LevinTamir/MARPY/blob/main/docs/firmware-setup.md)**

## VS Code env var

`.vscode/settings.json` references `${env:MARPY_WS}` for the ROS2 workspace install path. Point it at wherever you cloned the [MARPY workspace](https://github.com/LevinTamir/MARPY) and export it from your shell rc so the Python analysis paths resolve:

```bash
export MARPY_WS=/path/to/your/marpy_ws
```

## License

MIT

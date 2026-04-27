# MARPY Base Firmware

ESP32 (esp32dev) drive-controller firmware for the [MARPY](https://github.com/LevinTamir/MARPY) project. Subscribes to `/cmd_vel` and publishes `/joint_states` and `/imu` over Wi-Fi via micro-ROS.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel`      | `geometry_msgs/Twist`            | PC -> ESP32 |
| `/pid_gains`    | `std_msgs/Float32MultiArray`     | PC -> ESP32 (live PID tune: `[Kp, Ki, Kd]`) |
| `/joint_states` | `sensor_msgs/JointState`         | ESP32 -> PC |
| `/imu`          | `sensor_msgs/Imu`                | ESP32 -> PC |

Node name is `marpy`.

## Code layout

| File | Description |
|------|-------------|
| `include/pins.h`           | GPIO config |
| `include/motor_config.h`   | PID gain defaults |
| `include/pid.h` / `src/pid.cpp`           | generic PID controller |
| `include/motors.h` / `src/motors.cpp`     | PWM, encoders, kinematics |
| `include/imu.h` / `src/imu.cpp`           | MPU6050 driver |
| `include/microros.h` / `src/microros.cpp` | Wi-Fi, pubs/subs, callbacks |
| `src/main.cpp`             | `setup()` + `loop()` orchestration |

## Configure Wi-Fi

```bash
cp include/wifi_config.h.example include/wifi_config.h
```

Edit `include/wifi_config.h` with your SSID, password, and PC IP. The file is gitignored.

The full guide (with shell snippets to find your SSID and IP) is in the workspace repo: [docs/firmware-setup.md](https://github.com/LevinTamir/MARPY/blob/main/docs/firmware-setup.md).

## Build and flash

```bash
pio run -d base               # build
pio run -d base -t upload     # build + flash
pio device monitor -d base    # serial console
```

Or in VS Code with the PlatformIO extension: pick `base` in the PROJECT TASKS sidebar and use its Build / Upload / Monitor buttons.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Upload fails | Hold the **BOOT** button while uploading |
| WiFi won't connect | Check SSID/password, ensure 2.4 GHz (ESP32 does not support 5 GHz) |
| Agent not found | Verify PC IP, check firewall (`sudo ufw allow 8888/udp`) |
| Motors do not spin | Re-check the wiring guide. Usually a swapped IN1/IN2, missing common ground, or ENA/ENB jumpers still in place |

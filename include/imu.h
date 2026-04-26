// MPU6050 accel/gyro driver. Raw readings (no bias correction here -- if
// you want bias removal, add it in a fusion node downstream or a follow-up).
#pragma once

#include <stdbool.h>

extern bool imu_initialized;

// Returns true if the chip is detected.
bool imu_setup();

// Read the latest accel (m/s^2) and gyro (rad/s) sample.
void imu_read(float *ax, float *ay, float *az,
              float *gx, float *gy, float *gz);

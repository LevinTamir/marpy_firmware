// MPU6050 accel/gyro driver with at-boot gyro bias calibration.
//
// imu_setup() initialises I2C, wakes the chip, and (if it responds) averages
// GYRO_CALIB_SAMPLES samples while the robot sits still to estimate the gyro
// bias. Every call to imu_read() returns bias-corrected gyro readings; accel
// is returned raw.
#pragma once

#include <stdbool.h>

extern bool imu_initialized;

// Returns true if MPU6050 is detected and calibration completes.
bool imu_setup();

// Read the latest accel (m/s^2) and gyro (rad/s, bias-corrected) sample.
void imu_read(float *ax, float *ay, float *az,
              float *gx, float *gy, float *gz);

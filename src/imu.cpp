// MPU6050 driver. See imu.h.
#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "pins.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MPU6050_ADDR              0x68
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

// Conversion factors at the default ranges:
//   Accel: +/-2g  -> 16384 LSB/g  -> multiply by 9.80665 / 16384
//   Gyro:  +/-250 deg/s -> 131 LSB/(deg/s) -> multiply by (PI/180) / 131
static const float ACCEL_SCALE = 9.80665f / 16384.0f;
static const float GYRO_SCALE  = (M_PI / 180.0f) / 131.0f;

bool imu_initialized = false;

static void mpu6050_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool imu_setup() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);  // 400 kHz fast I2C

  // Wake up MPU6050 (clear sleep bit), use gyro X as clock source.
  mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);
  delay(100);

  // WHO_AM_I sanity check (register 0x75, expects 0x68).
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  if (!Wire.available()) return false;
  if (Wire.read() != 0x68) return false;

  // Default ranges: accel +/-2g, gyro +/-250 deg/s.
  mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00);
  mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00);

  imu_initialized = true;
  return true;
}

void imu_read(float *ax, float *ay, float *az,
              float *gx, float *gy, float *gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  *ax = raw_ax * ACCEL_SCALE;
  *ay = raw_ay * ACCEL_SCALE;
  *az = raw_az * ACCEL_SCALE;
  *gx = raw_gx * GYRO_SCALE;
  *gy = raw_gy * GYRO_SCALE;
  *gz = raw_gz * GYRO_SCALE;
}

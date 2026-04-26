// Central pin map for the MARPY ESP32 board. All other modules include
// this so there is a single source of truth.
#pragma once

// =================== Motor pins ====================
#define MOTOR_ENA  25   // Right motor PWM
#define MOTOR_IN1  26   // Right motor dir A
#define MOTOR_IN2  27   // Right motor dir B
#define MOTOR_IN3  32   // Left  motor dir A
#define MOTOR_IN4  33   // Left  motor dir B
#define MOTOR_ENB  14   // Left  motor PWM

// =================== Encoder pins ==================
#define ENC_RIGHT_A  18
#define ENC_RIGHT_B  19
#define ENC_LEFT_A   21
#define ENC_LEFT_B   22

// =================== IMU (MPU6050) pins ============
#define IMU_SDA  13   // I2C SDA (custom, default 21 used by encoder)
#define IMU_SCL  15   // I2C SCL (custom, default 22 used by encoder)

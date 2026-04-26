// ESP32 + L298N + micro-ROS (Wi-Fi UDP) + PlatformIO

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>
#include <Wire.h>
#include <math.h>

#include "motor_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =================== Wi-Fi + micro-ROS agent ===================
// Copy include/wifi_config.h.example → include/wifi_config.h and fill in your values
#include "wifi_config.h"

// =================== Motor pins (your wiring) ==================
#define ENA 25   // Right motor PWM
#define IN1 26   // Right motor dir A
#define IN2 27   // Right motor dir B
#define IN3 32   // Left  motor dir A
#define IN4 33   // Left  motor dir B
#define ENB 14   // Left  motor PWM

// =================== Encoder pins =================================
#define ENC_RIGHT_A  18  // Right encoder channel A
#define ENC_RIGHT_B  19  // Right encoder channel B
#define ENC_LEFT_A   21  // Left encoder channel A
#define ENC_LEFT_B   22  // Left encoder channel B

// =================== IMU (MPU6050) pins & config ==================
#define IMU_SDA      13  // I2C SDA (custom, default 21 used by encoder)
#define IMU_SCL      15  // I2C SCL (custom, default 22 used by encoder)
#define MPU6050_ADDR 0x68
// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
// Conversion factors
// Accel: ±2g range → 16384 LSB/g → multiply by 9.80665/16384
static const float ACCEL_SCALE = 9.80665f / 16384.0f;
// Gyro: ±250 deg/s range → 131 LSB/(deg/s) → multiply by (PI/180)/131
static const float GYRO_SCALE  = (M_PI / 180.0f) / 131.0f;

// Encoder specs. With 4x quadrature decoding (both edges of A and B):
//   ticks_per_output_rev = 4 * PPR * gear_ratio = 4 * 11 * 48 = 2112
static const int32_t TICKS_PER_REV = 2112;

// Software direction flips (toggle if a wheel turns the wrong way)
static const bool LEFT_DIR_INVERT  = true;
static const bool RIGHT_DIR_INVERT = true;

// =================== PWM (LEDC) ================================
static const int PWM_CH_RIGHT = 0;
static const int PWM_CH_LEFT  = 1;
static const int PWM_FREQ     = 20000;  // 20 kHz (inaudible)
static const int PWM_RES      = 8;      // 0..255 duty

// Duty floors so small non-zero commands actually move the wheels
static const uint8_t MIN_DUTY      = 30;
// ↓ Gentler spin: lower PWM floor so it doesn't jump too hard
static const uint8_t MIN_DUTY_SPIN = 80;   // was 100

// =================== Kinematics / Scaling ======================
static const float BASE_WIDTH_M    = 0.12f;  // your track width (12 cm)
static const float WHEEL_RADIUS_M  = 0.033f; // wheel radius
static const float MAX_RPM         = 70.0f;
static const float MAX_WHEEL_MS    = (2.0f * M_PI * WHEEL_RADIUS_M) * (MAX_RPM / 60.0f);

static const float LIN_DEADBAND    = 0.02f;  // m/s
static const float ANG_DEADBAND    = 0.05f;  // rad/s
// ↓ Gentler spin: lower per-wheel floor so small angular commands work
static const float SPIN_FLOOR_MS   = 0.08f;  // was 0.10

// ---- Linear shaping ----
// Scales linear.x so forward/back are gentler than what teleop sends.
// teleop_twist_keyboard defaults to 0.5 m/s; with 0.4 here the wheels
// target 0.2 m/s (below MAX_WHEEL_MS), which feels noticeably calmer.
static const float LIN_GAIN        = 0.4f;

// ---- Angular shaping ----
static const float WZ_SIGN         = -1.0f;  // flip angular direction
static const float WZ_GAIN         = 3.0f;   // gentler spin gain (was 5.0f)

static const uint32_t CMD_TIMEOUT_MS = 500;  // stop if cmd stream stale

// =================== micro-ROS handles =========================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_sub;
rcl_subscription_t pid_gains_sub;
rcl_publisher_t joint_state_pub;
rcl_publisher_t imu_pub;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray pid_gains_msg;

// =================== State ====================================
// Raw wheel-speed commands set by /cmd_vel (after LIN_GAIN/WZ_GAIN shaping).
volatile float v_left_cmd  = 0.0f;
volatile float v_right_cmd = 0.0f;
// Slewed setpoints (acceleration-limited). PID drives wheels toward these.
float v_left_target  = 0.0f;
float v_right_target = 0.0f;
bool spin_mode = false;
uint32_t last_cmd_ms = 0;
uint32_t last_slew_us = 0;

// Encoder state (volatile because they're touched in ISRs)
volatile int32_t enc_left_ticks  = 0;
volatile int32_t enc_right_ticks = 0;
volatile uint8_t enc_left_state  = 0;  // (A << 1) | B, for 4x quadrature
volatile uint8_t enc_right_state = 0;
uint32_t last_encoder_pub_ms = 0;
static const uint32_t ENCODER_PUB_INTERVAL_MS = 50; // 20 Hz

// Wheel velocity estimates (m/s), updated at VEL_UPDATE_INTERVAL_MS rate.
float wheel_vel_left_ms  = 0.0f;
float wheel_vel_right_ms = 0.0f;
int32_t prev_enc_left_ticks  = 0;
int32_t prev_enc_right_ticks = 0;
uint32_t last_vel_us = 0;
uint32_t last_vel_update_ms = 0;

// PID state. Live-tunable via /pid_gains topic.
struct PIDState { float integral; uint32_t last_us; };
PIDState pid_left  = {0.0f, 0};
PIDState pid_right = {0.0f, 0};
float pid_kp = DEFAULT_PID_KP;
float pid_ki = DEFAULT_PID_KI;
float pid_kd = DEFAULT_PID_KD;
float max_accel_ms2 = DEFAULT_MAX_ACCEL_MS2;

// IMU state
uint32_t last_imu_pub_ms = 0;
static const uint32_t IMU_PUB_INTERVAL_MS = 20; // 50 Hz
bool imu_initialized = false;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// =================== Encoder ISRs (4x quadrature) =============
// State-table decoder: every edge of A and B advances the state. The
// table maps (prev_state << 2 | new_state) to {-1, 0, +1}, where 0 means
// an invalid transition (likely a missed pulse) and the count is held.
static const int8_t QD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0,
};

void IRAM_ATTR enc_left_isr() {
  uint8_t s = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_left_ticks += QD_TABLE[(enc_left_state << 2) | s];
  enc_left_state = s;
}

void IRAM_ATTR enc_right_isr() {
  uint8_t s = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  enc_right_ticks += QD_TABLE[(enc_right_state << 2) | s];
  enc_right_state = s;
}

// =================== MPU6050 helpers ============================
static void mpu6050_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool mpu6050_init() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);  // 400 kHz fast I2C

  // Wake up MPU6050 (clear sleep bit), use gyro X as clock source
  mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);
  delay(100);

  // Verify connection by reading WHO_AM_I (register 0x75, should return 0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  if (!Wire.available()) return false;
  uint8_t who = Wire.read();
  if (who != 0x68) return false;

  // Accel range ±2g (default), Gyro range ±250 deg/s (default)
  mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00);
  mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00);

  return true;
}

static void mpu6050_read(float *ax, float *ay, float *az,
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

// =================== Helpers ==================================
static bool connectWiFi(unsigned long timeout_ms = 30000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > timeout_ms) return false;
    delay(250);
  }
  return true;
}

static void waitForAgent() {
  while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) {
    delay(500);
  }
}

// Fill a builtin_interfaces/Time from synced ROS epoch.
// Falls back to millis()-since-boot if the agent hasn't synced yet. Without
// sync, robot_state_publisher would re-broadcast wheel TFs at boot-relative
// stamps, causing /tf chains rooted at /odom to render the wheels stuck at
// the world origin.
static void stamp_now(builtin_interfaces__msg__Time *out) {
  int64_t ns = rmw_uros_epoch_nanos();
  if (ns > 0) {
    out->sec = (int32_t)(ns / 1000000000LL);
    out->nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    out->sec = (int32_t)(ms / 1000U);
    out->nanosec = (uint32_t)((ms % 1000U) * 1000000U);
  }
}

// Feed-forward PWM estimate from a target velocity. The PID adds a
// correction on top of this.
static inline float ff_duty_from_speed(float v_ms) {
  float m = v_ms / MAX_WHEEL_MS;
  if (m > 1.0f) m = 1.0f;
  if (m < -1.0f) m = -1.0f;
  return m * 255.0f;
}

// Drive the left motor at a signed PWM duty in [-255, +255]. Sign sets
// direction, magnitude sets PWM. Direction inversion (LEFT_DIR_INVERT) is
// applied here so callers always think in robot-frame: +duty = forward.
static void apply_motor_left_duty(int signed_duty) {
  if (signed_duty > 255)  signed_duty =  255;
  if (signed_duty < -255) signed_duty = -255;
  bool fwd = signed_duty >= 0;
  if (LEFT_DIR_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else     { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  ledcWrite(PWM_CH_LEFT, abs(signed_duty));
}

static void apply_motor_right_duty(int signed_duty) {
  if (signed_duty > 255)  signed_duty =  255;
  if (signed_duty < -255) signed_duty = -255;
  bool fwd = signed_duty >= 0;
  if (RIGHT_DIR_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else     { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  ledcWrite(PWM_CH_RIGHT, abs(signed_duty));
}

// Active electric brake: both H-bridge inputs HIGH + ENA HIGH shorts the
// motor terminals through the upper transistors, dissipating kinetic energy
// as a stopping force. Far crisper than coasting (PWM=0) when you want the
// robot to actually stop where you tell it to.
static void brake_left() {
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CH_LEFT, 255);
}
static void brake_right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CH_RIGHT, 255);
}

static void stop_motors() {
  brake_left();
  brake_right();
}

// =================== /cmd_vel callback ========================
static void cmd_vel_cb(const void * msgin) {
  const auto *m = (const geometry_msgs__msg__Twist *)msgin;

  float v  = (float)m->linear.x;
  float wz = (float)m->angular.z;

  if (!isfinite(v))  v  = 0.0f;
  if (!isfinite(wz)) wz = 0.0f;

  if (fabsf(v)  < LIN_DEADBAND) v  = 0.0f;
  if (fabsf(wz) < ANG_DEADBAND) wz = 0.0f;

  // ----- shape inputs: scale linear, flip+amplify angular -----
  v  = LIN_GAIN * v;
  wz = WZ_SIGN * WZ_GAIN * wz;

  // Default differential mapping (unicycle → wheels)
  float vL = v - 0.5f * wz * BASE_WIDTH_M;
  float vR = v + 0.5f * wz * BASE_WIDTH_M;

  // Force in-place spin when only angular is commanded
  if (v == 0.0f && wz != 0.0f) {
    float target = 0.5f * fabsf(wz) * BASE_WIDTH_M;  // ideal |vL|=|vR|
    if (target < SPIN_FLOOR_MS) target = SPIN_FLOOR_MS;
    // wz>0 (CCW): left backward, right forward
    vL = -copysignf(target, wz);
    vR =  copysignf(target, wz);
    spin_mode = true;
  } else {
    spin_mode = false;
  }

  // Clamp
  vL = fmaxf(fminf(vL,  MAX_WHEEL_MS), -MAX_WHEEL_MS);
  vR = fmaxf(fminf(vR,  MAX_WHEEL_MS), -MAX_WHEEL_MS);

  // Stash raw commands. The loop slews v_left_target / v_right_target
  // toward these at MAX_ACCEL, then PID drives the wheels.
  v_left_cmd  = vL;
  v_right_cmd = vR;

  last_cmd_ms = millis();
}

// =================== /pid_gains callback =====================
// Live PID tuning: publish a Float32MultiArray with [Kp, Ki, Kd]. Resets the
// integrators so a fresh I term doesn't carry over old wind-up at new gains.
static void pid_gains_cb(const void * msgin) {
  const auto *m = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (m->data.size >= 1) pid_kp = m->data.data[0];
  if (m->data.size >= 2) pid_ki = m->data.data[1];
  if (m->data.size >= 3) pid_kd = m->data.data[2];
  pid_left.integral  = 0.0f;
  pid_right.integral = 0.0f;
  Serial.printf("[PID] gains updated: Kp=%.2f Ki=%.2f Kd=%.2f\n",
                pid_kp, pid_ki, pid_kd);
}

// =================== Control helpers =========================
// Time-based slew rate limit: nudge `target` toward `cmd` at max accel.
static inline void slew_toward(float &target, float cmd, float dt) {
  float max_dv = max_accel_ms2 * dt;
  float delta  = cmd - target;
  if (delta >  max_dv) delta =  max_dv;
  if (delta < -max_dv) delta = -max_dv;
  target += delta;
}

// PI controller, output is a PWM-duty correction added on top of the
// feed-forward estimate. Anti-windup clamps the I term, not the raw
// integral, so changing Ki doesn't suddenly change the saturation point.
static float pid_step(float setpoint, float measured, PIDState &state, uint32_t now_us) {
  if (state.last_us == 0) {
    state.last_us = now_us;
    return 0.0f;
  }
  float dt = (now_us - state.last_us) * 1e-6f;
  state.last_us = now_us;
  if (dt <= 0.0f || dt > 0.5f) return 0.0f;

  float error = setpoint - measured;
  state.integral += error * dt;
  if (pid_ki > 1e-6f) {
    float i_max = DEFAULT_PID_INTEGRAL_MAX_DUTY / pid_ki;
    if (state.integral >  i_max) state.integral =  i_max;
    if (state.integral < -i_max) state.integral = -i_max;
  } else {
    state.integral = 0.0f;
  }
  return pid_kp * error + pid_ki * state.integral;
}

// Recompute wheel velocities from encoder tick deltas. Called at fixed rate
// (VEL_UPDATE_INTERVAL_MS) so the PID feedback signal is consistent.
static void update_wheel_velocities() {
  uint32_t now_us = micros();
  int32_t  left_now  = enc_left_ticks;
  int32_t  right_now = enc_right_ticks;

  if (last_vel_us != 0) {
    float dt = (now_us - last_vel_us) * 1e-6f;
    if (dt > 0.0f) {
      float left_radps  = (left_now  - prev_enc_left_ticks)  *
                          (2.0f * M_PI / TICKS_PER_REV) / dt;
      float right_radps = (right_now - prev_enc_right_ticks) *
                          (2.0f * M_PI / TICKS_PER_REV) / dt;
      // Encoders count the raw motor shaft direction; the URDF/firmware sign
      // convention has +velocity = robot forward. LEFT/RIGHT_DIR_INVERT flip
      // the motor wiring; mirror that here so velocity matches commanded sign.
      if (LEFT_DIR_INVERT)  left_radps  = -left_radps;
      if (RIGHT_DIR_INVERT) right_radps = -right_radps;
      wheel_vel_left_ms  = left_radps  * WHEEL_RADIUS_M;
      wheel_vel_right_ms = right_radps * WHEEL_RADIUS_M;
    }
  }
  prev_enc_left_ticks  = left_now;
  prev_enc_right_ticks = right_now;
  last_vel_us = now_us;
}

// =================== Arduino setup/loop =======================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  
  // 4x quadrature: trigger on every edge of A and B, both ISRs feed the
  // same per-wheel state machine.
  enc_left_state  = (digitalRead(ENC_LEFT_A)  << 1) | digitalRead(ENC_LEFT_B);
  enc_right_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  enc_left_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B),  enc_left_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), enc_right_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), enc_right_isr, CHANGE);

  // PWM setup (LEDC)
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CH_LEFT);

  stop_motors();  // ensure no movement at boot

  // Wi-Fi
  if (!connectWiFi()) {
    Serial.println("[WiFi] connect failed; halting");
    while (true) { delay(1000); }
  }
  Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());

  // micro-ROS transport (Wi-Fi UDP)
  set_microros_wifi_transports(WIFI_SSID, WIFI_PSK, AGENT_IP, AGENT_PORT);
  Serial.println("[micro-ROS] waiting for agent...");
  waitForAgent();
  Serial.println("[micro-ROS] agent found");

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_diffdrive_node", "", &support);

  // Sync ESP32 clock with the agent so /joint_states and /imu carry valid
  // ROS epoch stamps. robot_state_publisher uses these stamps when re-
  // broadcasting wheel TFs; without sync, the wheels render at the world
  // origin in RViz when the fixed frame is /odom.
  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("[micro-ROS] time synced with agent");
  } else {
    Serial.println("[micro-ROS] time sync FAILED (will retry in loop)");
  }

  geometry_msgs__msg__Twist__init(&cmd_msg);
  rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // Initialize joint_state publisher
  sensor_msgs__msg__JointState__init(&joint_state_msg);
  
  // Allocate arrays for 2 joints
  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(2 * sizeof(rosidl_runtime_c__String));
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");
  
  joint_state_msg.position.capacity = 2;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.data = (double*)malloc(2 * sizeof(double));
  
  joint_state_msg.velocity.capacity = 2;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.data = (double*)malloc(2 * sizeof(double));
  
  joint_state_msg.effort.capacity = 0;
  joint_state_msg.effort.size = 0;
  joint_state_msg.effort.data = NULL;
  
  rclc_publisher_init_default(
    &joint_state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states");

  // Initialize IMU publisher
  sensor_msgs__msg__Imu__init(&imu_msg);
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
  // Set orientation covariance to -1 (orientation not provided)
  imu_msg.orientation_covariance[0] = -1.0;
  // Set linear acceleration and angular velocity covariances (diagonal)
  for (int i = 0; i < 9; i++) {
    imu_msg.linear_acceleration_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
  }
  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;

  rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu");

  // Initialize MPU6050
  imu_initialized = mpu6050_init();
  if (imu_initialized) {
    Serial.println("[IMU] MPU6050 initialized on I2C (GPIO 13/15)");
    // Gyro bias calibration. Average GYRO_CALIB_SAMPLES samples while the
    // robot sits still; subtract the mean from every subsequent reading.
    // Cuts the slow drift any future Madgwick/EKF would otherwise accumulate.
    Serial.println("[IMU] calibrating gyro bias (keep robot still)...");
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
      float ax, ay, az, gx, gy, gz;
      mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz);
      sum_gx += gx; sum_gy += gy; sum_gz += gz;
      delay(GYRO_CALIB_DT_MS);
    }
    gyro_bias_x = sum_gx / GYRO_CALIB_SAMPLES;
    gyro_bias_y = sum_gy / GYRO_CALIB_SAMPLES;
    gyro_bias_z = sum_gz / GYRO_CALIB_SAMPLES;
    Serial.printf("[IMU] gyro bias: %.4f %.4f %.4f rad/s\n",
                  gyro_bias_x, gyro_bias_y, gyro_bias_z);
  } else {
    Serial.println("[IMU] MPU6050 init FAILED - check wiring");
  }

  // /pid_gains subscriber for live PID tuning (Float32MultiArray [Kp, Ki, Kd])
  std_msgs__msg__Float32MultiArray__init(&pid_gains_msg);
  pid_gains_msg.data.capacity = 8;
  pid_gains_msg.data.size = 0;
  pid_gains_msg.data.data = (float*)malloc(8 * sizeof(float));
  rclc_subscription_init_default(
    &pid_gains_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/pid_gains");

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pid_gains_sub, &pid_gains_msg, &pid_gains_cb, ON_NEW_DATA);

  last_cmd_ms = millis();
  last_encoder_pub_ms = millis();
  last_imu_pub_ms = millis();
  Serial.println("[Setup] Ready: sub /cmd_vel, pub /joint_states, pub /imu");
}

void loop() {
  // Process incoming ROS messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  uint32_t now_us = micros();

  // Update measured wheel velocities at a fixed rate so PID feedback isn't
  // dependent on jitter in the main loop period.
  if (millis() - last_vel_update_ms >= VEL_UPDATE_INTERVAL_MS) {
    update_wheel_velocities();
    last_vel_update_ms = millis();
  }

  // Slew the per-wheel setpoint toward the latest command. Caps acceleration
  // so a hard turn-then-forward keypress doesn't yank current.
  if (last_slew_us != 0) {
    float dt = (now_us - last_slew_us) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) {
      slew_toward(v_left_target,  v_left_cmd,  dt);
      slew_toward(v_right_target, v_right_cmd, dt);
    }
  }
  last_slew_us = now_us;

  // PID-driven motor command: feed-forward + correction.
  // Brake when the slewed setpoint is essentially zero so the robot stops
  // crisply (and doesn't ride open-loop noise).
  if (fabsf(v_left_target) < 0.001f) {
    brake_left();
    pid_left.integral = 0.0f;
  } else {
    float ff   = ff_duty_from_speed(v_left_target);
    float corr = pid_step(v_left_target, wheel_vel_left_ms, pid_left, now_us);
    apply_motor_left_duty((int)lrintf(ff + corr));
  }
  if (fabsf(v_right_target) < 0.001f) {
    brake_right();
    pid_right.integral = 0.0f;
  } else {
    float ff   = ff_duty_from_speed(v_right_target);
    float corr = pid_step(v_right_target, wheel_vel_right_ms, pid_right, now_us);
    apply_motor_right_duty((int)lrintf(ff + corr));
  }

  // Publish joint states at regular intervals
  if (millis() - last_encoder_pub_ms >= ENCODER_PUB_INTERVAL_MS) {
    // Position in radians (cumulative wheel angle)
    joint_state_msg.position.data[0] = (double)enc_left_ticks * 2.0 * M_PI / TICKS_PER_REV;
    joint_state_msg.position.data[1] = (double)enc_right_ticks * 2.0 * M_PI / TICKS_PER_REV;
    if (LEFT_DIR_INVERT)  joint_state_msg.position.data[0] = -joint_state_msg.position.data[0];
    if (RIGHT_DIR_INVERT) joint_state_msg.position.data[1] = -joint_state_msg.position.data[1];

    // Velocity in rad/s (from the same value PID uses, converted back from m/s)
    joint_state_msg.velocity.data[0] = (double)(wheel_vel_left_ms  / WHEEL_RADIUS_M);
    joint_state_msg.velocity.data[1] = (double)(wheel_vel_right_ms / WHEEL_RADIUS_M);

    stamp_now(&joint_state_msg.header.stamp);

    rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
    last_encoder_pub_ms = millis();
  }

  // Publish IMU data at 50 Hz
  if (imu_initialized && millis() - last_imu_pub_ms >= IMU_PUB_INTERVAL_MS) {
    float ax, ay, az, gx, gy, gz;
    mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz);

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx - gyro_bias_x;
    imu_msg.angular_velocity.y = gy - gyro_bias_y;
    imu_msg.angular_velocity.z = gz - gyro_bias_z;

    // No orientation estimate from raw MPU6050
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    stamp_now(&imu_msg.header.stamp);

    rcl_publish(&imu_pub, &imu_msg, NULL);
    last_imu_pub_ms = millis();
  }

  // Safety: stop if cmd stream goes stale. Collapse the cmd, slewed target,
  // and PID integrators so a fresh command starts from a clean state.
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    v_left_cmd     = v_right_cmd     = 0.0f;
    v_left_target  = v_right_target  = 0.0f;
    pid_left.integral = pid_right.integral = 0.0f;
    stop_motors();
  }

  // Keep Wi-Fi alive
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PSK);
  }

  delay(2);
}

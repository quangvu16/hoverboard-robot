// mpu9250.h - InvenSense MPU-9250 / MPU-6500 IMU over I2C, raw registers.
//
// The chip's own DMP is NOT used: the GY-9250 boards on this robot carry
// clone chips that accept the DMP firmware and then never deliver a FIFO
// packet (USER_CTRL 0xC0, FIFO count 0, INT_STATUS never shows DMP_INT),
// while the raw accel/gyro registers work fine. So this file reads those
// registers once per HBT1 datagram and runs a 6-axis Mahony filter here
// (gyro + gravity, like the BNO055's IMUPLUS: yaw relative to power-on, no
// compass). No third-party library. Everything after setup() is called from
// hoverboardTask (core 0), so nothing here is shared or locked.
//
// Wiring (GY-9250 / GY-6500 style breakouts, which carry their own 3.3 V
// regulator and I2C pull-ups):
//
//   MPU9250  ESP32
//   VCC      3V3
//   GND      GND
//   SDA      IMU_SDA_PIN (GPIO8)
//   SCL      IMU_SCL_PIN (GPIO9)
//   AD0      open or GND (0x68)
//   NCS      open (or 3V3) = I2C
//
// Mounting: any orientation; the URDF's imu_joint rpy maps the chip's axes
// onto base_link (REP-103), so the firmware never needs to know. The filter
// starts level-referenced from the first accel sample, whatever the tilt.
//
// HBT1 is unchanged from the BNO055 firmware: the samples are converted to
// the BNO055's units below, so hoverboard_udp_bridge_node needs no change.
//
// A missing chip is not an error: the robot runs without it and HBT1 carries
// imu_status 0. Starting the chip is a handful of register writes (no
// blocking), so it is probed again every IMU_RETRY_MS whenever it is absent.
// A chip that stops answering, or whose registers freeze (it reset itself on
// a brown-out and went back to sleep), is dropped and started again.
//
// Gyro bias: estimated while the robot stands still (no wheel command or
// motion) and gravity's direction does not change (not carried or tilted),
// over CAL_WINDOW_MS windows, and refreshed at every later rest. The gyro is
// NOT required to be quiet: the spinning LiDAR shakes the chassis by tens of
// deg/s, and that averages out over a window. Until the first window completes, yaw drifts by the raw
// bias (a few deg/s at most). There is no accel or compass calibration and
// nothing is kept in NVS.

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace mpu {
static const uint8_t ADDR         = 0x68;
static const uint8_t SMPLRT_DIV   = 0x19;
static const uint8_t CONFIG       = 0x1A;
static const uint8_t GYRO_CONFIG  = 0x1B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t ACCEL_CONFIG2 = 0x1D;
static const uint8_t INT_ENABLE   = 0x38;
static const uint8_t ACCEL_XOUT_H = 0x3B;   // accel, temp, gyro: 14 bytes
static const uint8_t USER_CTRL    = 0x6A;
static const uint8_t PWR_MGMT_1   = 0x6B;
static const uint8_t PWR_MGMT_2   = 0x6C;
static const uint8_t WHO_AM_I     = 0x75;
// 0x71 MPU-9250, 0x73 MPU-9255, 0x70 MPU-6500 (the same chip without the
// AK8963 compass, often sold as "MPU9250"; fine here, the compass is unused).
static const uint8_t ID_9250 = 0x71, ID_9255 = 0x73, ID_6500 = 0x70;

static const int MAX_ERRORS = 5;
// This many identical 14-byte reads in a row = the chip is asleep after a
// reset (live sensors never repeat bit for bit). 25 x 20 ms = 0.5 s.
static const int MAX_FROZEN = 25;

// +-500 deg/s (a robot never turns faster) and +-2 g.
static const uint8_t GYRO_FS_500  = 0x08;
static const uint8_t ACCEL_FS_2G  = 0x00;
static const float GYRO_LSB_PER_DPS = 65.5f;
static const float ACCEL_LSB_PER_G  = 16384.0f;

// Gyro bias window: robot still, gravity within STEADY_DEG of its direction
// at the window's start, and the window's mean below MAX_BIAS_DPS on every
// axis (the MPU-9250's spec is +-5; more = someone turned the robot by hand).
static const uint32_t CAL_WINDOW_MS = 2000;
static const float    STEADY_DEG    = 2.0f;
static const float    MAX_BIAS_DPS  = 8.0f;

// Mahony proportional gain: how fast roll/pitch follow gravity. Gravity is
// only trusted while |accel| is within ACCEL_TRUST of 1 g (no bumps).
static const float KP          = 1.0f;
static const float ACCEL_TRUST = 0.15f;
}  // namespace mpu

// Latest sample, in the BNO055's units (= HBT1's), chip axes:
//   quat  1/16384    gyro  1/16 deg/s    accel  1/100 m/s^2 (gravity included)
struct ImuSample {
  int16_t quat[4];    // w, x, y, z
  int16_t gyro[3];
  int16_t accel[3];
};

static ImuSample imu_sample = {};
static uint8_t   imu_id = 0;            // WHO_AM_I; 0 = not running
static bool      imu_fresh = false;     // last imuRead() got a new sample
static bool      imu_gyro_cal = false;  // a bias window has completed
static int       imu_errors = 0;        // consecutive failed reads
static int       imu_frozen = 0;        // consecutive identical reads
static uint32_t  imu_err_total = 0;
static uint32_t  imu_next_probe_ms = 0;
static uint8_t   imu_last_raw[14] = {};

// Filter state: q rotates chip axes into the level, power-on-yaw frame.
static float     imu_q[4] = {1, 0, 0, 0};
static bool      imu_q_init = false;
static uint32_t  imu_last_us = 0;
static float     imu_bias[3] = {};      // deg/s

// Bias window accumulator.
static uint32_t  imu_cal_start_ms = 0;  // 0 = no window open
static float     imu_cal_sum[3], imu_cal_g0[3];  // g0: unit gravity at start
static int       imu_cal_n = 0;

// Raw gyro spread since the last imuLog(), deg/s: shows vibration.
static float     imu_st_min[3], imu_st_max[3];
static int       imu_st_n = 0;

static bool imuReadRegs(uint8_t addr, uint8_t reg, uint8_t * out, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) out[i] = (uint8_t)Wire.read();
  return true;
}

static bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpu::ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static const char * imuName() {
  return imu_id == mpu::ID_6500 ? "MPU6500" :
         imu_id == mpu::ID_9255 ? "MPU9255" : "MPU9250";
}

// One WHO_AM_I read when nothing answers; the register set-up when it does.
// No reset: every register that matters is written, which also undoes a DMP
// left running by older firmware and wakes a chip that reset itself.
static bool imuProbe() {
  uint8_t id = 0;
  if (!imuReadRegs(mpu::ADDR, mpu::WHO_AM_I, &id, 1)) return false;
  if (id != mpu::ID_9250 && id != mpu::ID_9255 && id != mpu::ID_6500) {
    Serial.printf("imu    0x%02X answers WHO_AM_I 0x%02X, not an MPU9250 "
                  "family chip; ignored\n", mpu::ADDR, id);
    return false;
  }

  const bool ok =
      imuWriteReg(mpu::PWR_MGMT_1, 0x01) &&      // awake, gyro PLL clock
      imuWriteReg(mpu::PWR_MGMT_2, 0x00) &&      // all axes on
      imuWriteReg(mpu::USER_CTRL, 0x0C) &&       // DMP and FIFO off + reset
      imuWriteReg(mpu::INT_ENABLE, 0x00) &&
      imuWriteReg(mpu::CONFIG, 0x04) &&          // gyro DLPF 20 Hz (< 25 Hz Nyquist)
      imuWriteReg(mpu::SMPLRT_DIV, 9) &&         // 1 kHz / 10 = 100 Hz
      imuWriteReg(mpu::GYRO_CONFIG, mpu::GYRO_FS_500) &&
      imuWriteReg(mpu::ACCEL_CONFIG, mpu::ACCEL_FS_2G) &&
      imuWriteReg(mpu::ACCEL_CONFIG2, 0x04);     // accel DLPF 20 Hz
  if (!ok) {
    Serial.printf("imu    WHO_AM_I 0x%02X at 0x%02X, but the set-up writes failed\n",
                  id, mpu::ADDR);
    return false;
  }

  imu_id = id;
  imu_errors = 0;
  imu_frozen = 0;
  imu_q_init = false;      // re-level from the next accel sample
  imu_cal_start_ms = 0;
  // The bias survives a re-probe (same chip, same temperature): only a
  // first start begins uncalibrated.
  Serial.printf("imu    %s at 0x%02X, raw accel/gyro + ESP32 6-axis fusion "
                "(no DMP, no compass)\n", imuName(), mpu::ADDR);
  return true;
}

static void imuDrop(const char * why) {
  Serial.printf("imu    %s %s, probing again\n", imuName(), why);
  imu_id = 0;
  imu_next_probe_ms = millis() + IMU_RETRY_MS;
}

// From setup(), before the tasks start.
static void imuBegin() {
  if (!IMU_ENABLED) return;
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_HZ);
  Wire.setTimeOut(10);   // ms
  Serial.printf("imu    MPU9250 on I2C SDA=GPIO%d SCL=GPIO%d @ %lu Hz\n",
                IMU_SDA_PIN, IMU_SCL_PIN, (unsigned long)IMU_I2C_HZ);
  delay(50);   // the chip needs ~35 ms after power-on
  if (!imuProbe()) imu_next_probe_ms = millis() + IMU_RETRY_MS;
}

// Gyro bias: one window of still samples -> new bias. Wheels moving, the
// robot tilted or carried, or an implausible mean discards the window.
static void imuCalStep(bool robot_still, const float g_dps[3], const float a[3],
                       uint32_t now) {
  if (!robot_still) { imu_cal_start_ms = 0; return; }
  const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (an <= 0.0f) return;
  if (!imu_cal_start_ms) {
    imu_cal_start_ms = now ? now : 1;
    imu_cal_n = 0;
    for (int i = 0; i < 3; i++) {
      imu_cal_sum[i] = 0;
      imu_cal_g0[i] = a[i] / an;
    }
  }
  const float dot = (a[0] * imu_cal_g0[0] + a[1] * imu_cal_g0[1] +
                     a[2] * imu_cal_g0[2]) / an;
  if (dot < cosf(mpu::STEADY_DEG * 0.01745329f)) {
    imu_cal_start_ms = 0;      // tilted or carried: start over
    return;
  }
  for (int i = 0; i < 3; i++) imu_cal_sum[i] += g_dps[i];
  imu_cal_n++;
  if (now - imu_cal_start_ms < mpu::CAL_WINDOW_MS) return;
  imu_cal_start_ms = 0;        // next window starts with the next sample

  float m[3];
  for (int i = 0; i < 3; i++) {
    m[i] = imu_cal_sum[i] / imu_cal_n;
    if (fabsf(m[i]) > mpu::MAX_BIAS_DPS) return;   // turned by hand
  }
  for (int i = 0; i < 3; i++) imu_bias[i] = m[i];
  if (!imu_gyro_cal)
    Serial.printf("imu    robot at rest %lu s: gyro bias %+.2f %+.2f %+.2f deg/s\n",
                  (unsigned long)(mpu::CAL_WINDOW_MS / 1000),
                  imu_bias[0], imu_bias[1], imu_bias[2]);
  imu_gyro_cal = true;
}

// Roll/pitch from gravity, yaw 0: the filter's starting point.
static void imuLevelFrom(float ax, float ay, float az) {
  const float roll  = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float cr = cosf(roll / 2), sr = sinf(roll / 2);
  const float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
  imu_q[0] = cr * cp;
  imu_q[1] = sr * cp;
  imu_q[2] = cr * sp;
  imu_q[3] = -sr * sp;
}

// Mahony, proportional term only (the bias is handled by imuCalStep).
// g in rad/s, a in any unit (normalised here), dt in s.
static void imuMahony(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt) {
  float * q = imu_q;
  const float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an > 0.0f && fabsf(an / mpu::ACCEL_LSB_PER_G - 1.0f) < mpu::ACCEL_TRUST) {
    ax /= an; ay /= an; az /= an;
    // Gravity as the current estimate predicts it, in chip axes.
    const float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    const float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    const float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    gx += mpu::KP * (ay * vz - az * vy);
    gy += mpu::KP * (az * vx - ax * vz);
    gz += mpu::KP * (ax * vy - ay * vx);
  }
  const float h = 0.5f * dt;
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  q[0] += h * (-x * gx - y * gy - z * gz);
  q[1] += h * ( w * gx + y * gz - z * gy);
  q[2] += h * ( w * gy - x * gz + z * gx);
  q[3] += h * ( w * gz + x * gy - y * gx);
  const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// Once per HBT1 datagram: one 14-byte burst read, bias and filter update.
// robot_still gates the gyro bias estimate.
static void imuRead(bool robot_still) {
  imu_fresh = false;
  if (!IMU_ENABLED) return;

  const uint32_t now = millis();
  if (!imu_id) {
    if ((int32_t)(now - imu_next_probe_ms) < 0) return;
    imu_next_probe_ms = now + IMU_RETRY_MS;
    if (!imuProbe()) return;
  }

  uint8_t r[14];
  if (!imuReadRegs(mpu::ADDR, mpu::ACCEL_XOUT_H, r, sizeof(r))) {
    imu_err_total++;
    if (++imu_errors >= mpu::MAX_ERRORS) imuDrop("stopped answering");
    return;
  }
  imu_errors = 0;

  if (memcmp(r, imu_last_raw, sizeof(r)) == 0) {
    if (++imu_frozen >= mpu::MAX_FROZEN) imuDrop("registers frozen (chip reset?)");
    return;
  }
  imu_frozen = 0;
  memcpy(imu_last_raw, r, sizeof(r));

  auto s16 = [&](int i) { return (int16_t)((r[i] << 8) | r[i + 1]); };
  const float a[3] = {(float)s16(0), (float)s16(2), (float)s16(4)};
  float g[3] = {s16(8)  / mpu::GYRO_LSB_PER_DPS,
                s16(10) / mpu::GYRO_LSB_PER_DPS,
                s16(12) / mpu::GYRO_LSB_PER_DPS};   // deg/s

  imuCalStep(robot_still, g, a, now);
  for (int i = 0; i < 3; i++) {
    if (!imu_st_n || g[i] < imu_st_min[i]) imu_st_min[i] = g[i];
    if (!imu_st_n || g[i] > imu_st_max[i]) imu_st_max[i] = g[i];
  }
  imu_st_n++;
  for (int i = 0; i < 3; i++) g[i] -= imu_bias[i];

  const uint32_t us = micros();
  if (!imu_q_init) {
    imuLevelFrom(a[0], a[1], a[2]);
    imu_q_init = true;
  } else {
    float dt = (us - imu_last_us) * 1e-6f;
    if (dt > 0.1f) dt = 0.1f;   // a stalled task must not fling the estimate
    const float k = 0.01745329f;
    imuMahony(g[0] * k, g[1] * k, g[2] * k, a[0], a[1], a[2], dt);
  }
  imu_last_us = us;

  for (int i = 0; i < 4; i++)
    imu_sample.quat[i] = (int16_t)lroundf(imu_q[i] * 16384.0f);
  for (int i = 0; i < 3; i++) {
    imu_sample.gyro[i]  = (int16_t)lroundf(g[i] * 16.0f);
    imu_sample.accel[i] = (int16_t)lroundf(a[i] * (980.665f / mpu::ACCEL_LSB_PER_G));
  }
  imu_fresh = true;
}

// HBT1 imu_status, same layout as the BNO055 firmware: bit0 chip found, bit1
// sample fresh, bits 2-9 calibration 0..3 of sys/gyro/accel/mag, bit10 compass
// (never set here). The MPU9250 reports no calibration of its own: gyro and
// sys read 3 once a gyro bias window has completed, accel and mag always 0.
static uint16_t imuStatus() {
  if (!imu_id) return 0;
  uint16_t s = 0x0001;
  if (imu_fresh)    s |= 0x0002;
  if (imu_gyro_cal) s |= (3u << 2) | (3u << 4);
  return s;
}

static void imuLog() {
  if (!IMU_ENABLED) return;
  if (!imu_id) {
    Serial.printf("imu    no MPU9250 at 0x%02X on SDA=GPIO%d SCL=GPIO%d "
                  "(check 3V3, GND, SDA/SCL swapped, AD0 low)\n",
                  mpu::ADDR, IMU_SDA_PIN, IMU_SCL_PIN);
    // An idle bus floats high through the module's pull-ups. Low = module
    // unpowered, no ground, or no pull-ups; scanning then would only stall
    // on timeouts, so it is skipped.
    const bool sda = digitalRead(IMU_SDA_PIN), scl = digitalRead(IMU_SCL_PIN);
    if (!sda || !scl) {
      Serial.printf("imu    I2C idle level SDA %s SCL %s: module unpowered, "
                    "GND missing or no pull-ups\n",
                    sda ? "high" : "LOW", scl ? "high" : "LOW");
      return;
    }
    Serial.print("imu    I2C bus answers at:");
    int n = 0;
    bool at69 = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        Serial.printf(" 0x%02X", a);
        n++;
        at69 |= (a == 0x69);
      }
    }
    Serial.println(!n ? " nothing" : at69 ? "  (0x69 = AD0 high: tie AD0 to GND)" : "");
    return;
  }
  const float * q = imu_q;
  const float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                           1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
  const ImuSample & m = imu_sample;
  Serial.printf("imu    yaw %+6.1f deg  gyro z %+6.1f deg/s  acc z %+5.2f m/s2  "
                "gyro %s  %lu errors\n",
                yaw * 57.29578f, m.gyro[2] / 16.0f, m.accel[2] / 100.0f,
                imu_gyro_cal ? "calibrated" : "not calibrated (rest 2 s)",
                (unsigned long)imu_err_total);
  if (imu_st_n)
    Serial.printf("imu    gyro spread (peak to peak, %d samples): x %.1f  y %.1f  "
                  "z %.1f deg/s\n", imu_st_n, imu_st_max[0] - imu_st_min[0],
                  imu_st_max[1] - imu_st_min[1], imu_st_max[2] - imu_st_min[2]);
  imu_st_n = 0;
}

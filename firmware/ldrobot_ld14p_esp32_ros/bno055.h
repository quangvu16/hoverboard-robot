// bno055.h - Bosch BNO055 IMU over I2C, raw registers, no library.
//
// The chip runs its own sensor fusion; this file only configures it and
// copies quaternion, gyro and accel out once per HBT1 datagram. Everything is
// called from hoverboardTask (core 0), so nothing here is shared or locked.
//
// Wiring (GY-BNO055 / Adafruit breakouts, which carry their own regulator and
// I2C pull-ups):
//
//   BNO055   ESP32
//   VIN      3V3
//   GND      GND
//   SDA      IMU_SDA_PIN (GPIO21)
//   SCL      IMU_SCL_PIN (GPIO22)
//   ADR      open (0x28); tied high = 0x29 - both are probed
//   PS0/PS1  open (= I2C)
//
// Mounting: any orientation; the URDF's imu_joint rpy maps the chip's axes
// onto base_link (REP-103), so the firmware never needs to know.
//
// A missing chip is not an error: the robot runs without it and HBT1 carries
// imu_status 0. The chip is probed again every IMU_RETRY_MS, and dropped
// after a few failed reads in a row (loose wire), so it can be plugged in or
// re-seated with the robot running.
//
// Calibration: the BNO055 forgets it at power-off. Once gyro and accel (and
// the magnetometer, with IMU_USE_COMPASS) all report 3, the 22 offset bytes
// are saved to NVS and written back at every boot. The gyro calibrates by
// itself after a few seconds standing still; the accelerometer only after
// the board has rested in ~6 different orientations (each axis up and down),
// which is easiest to do once, before mounting.

#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include "config.h"

namespace bno {
// Page 0 registers (BNO055 datasheet, section 4.3).
static const uint8_t CHIP_ID     = 0x00;   // reads 0xA0
static const uint8_t PAGE_ID     = 0x07;
static const uint8_t ACC_DATA    = 0x08;   // 0x08..0x27: acc, mag, gyr, eul, qua
static const uint8_t CALIB_STAT  = 0x35;
static const uint8_t UNIT_SEL    = 0x3B;
static const uint8_t OPR_MODE    = 0x3D;
static const uint8_t PWR_MODE    = 0x3E;
static const uint8_t OFFSETS     = 0x55;   // 0x55..0x6A, writable in CONFIG only
static const uint8_t OFFSETS_LEN = 22;

static const uint8_t MODE_CONFIG  = 0x00;
static const uint8_t MODE_IMUPLUS = 0x08;
static const uint8_t MODE_NDOF    = 0x0C;
static const uint8_t ID_VALUE     = 0xA0;

static const uint8_t BURST_LEN = 0x28 - ACC_DATA;   // 32 bytes
static const int     MAX_ERRORS = 5;
}  // namespace bno

// Latest sample, in the chip's own units and axes:
//   quat  1/16384    gyro  1/16 deg/s    accel  1/100 m/s^2 (gravity included)
struct ImuSample {
  int16_t quat[4];    // w, x, y, z
  int16_t gyro[3];
  int16_t accel[3];
  uint8_t calib;      // CALIB_STAT: sys 7-6, gyro 5-4, accel 3-2, mag 1-0
};

static ImuSample imu_sample = {};
static uint8_t   imu_addr = 0;          // 0 = not found
static bool      imu_fresh = false;     // last imuRead() succeeded
static int       imu_errors = 0;        // consecutive failed reads
static uint32_t  imu_err_total = 0;
static uint32_t  imu_next_probe_ms = 0;
static bool      imu_offsets_saved = false;
static bool      imu_offsets_loaded = false;

static bool imuWrite(uint8_t reg, const uint8_t * data, size_t n) {
  Wire.beginTransmission(imu_addr);
  Wire.write(reg);
  Wire.write(data, n);
  return Wire.endTransmission() == 0;
}

static bool imuWrite8(uint8_t reg, uint8_t v) {
  return imuWrite(reg, &v, 1);
}

static bool imuReadRegs(uint8_t addr, uint8_t reg, uint8_t * out, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) out[i] = (uint8_t)Wire.read();
  return true;
}

static uint8_t imuFusionMode() {
  return IMU_USE_COMPASS ? bno::MODE_NDOF : bno::MODE_IMUPLUS;
}

static void imuSetMode(uint8_t mode) {
  imuWrite8(bno::OPR_MODE, mode);
  // Datasheet table 3-6: 19 ms into CONFIG, 7 ms out of it.
  delay(mode == bno::MODE_CONFIG ? 25 : 10);
}

// Called in CONFIG mode only. One register per transaction: burst writes to
// the offset block are reported to be dropped silently (flynneva/bno055,
// Adafruit_BNO055 both write it byte by byte), so the result is read back and
// only a verified restore counts.
static void imuRestoreOffsets() {
  Preferences prefs;
  if (!prefs.begin("bno055", true)) return;
  uint8_t ofs[bno::OFFSETS_LEN];
  const size_t n = prefs.getBytes("ofs", ofs, sizeof(ofs));
  prefs.end();
  if (n != sizeof(ofs)) return;

  for (uint8_t i = 0; i < bno::OFFSETS_LEN; i++)
    imuWrite8(bno::OFFSETS + i, ofs[i]);

  uint8_t back[bno::OFFSETS_LEN];
  if (imuReadRegs(imu_addr, bno::OFFSETS, back, sizeof(back)) &&
      memcmp(back, ofs, sizeof(ofs)) == 0) {
    imu_offsets_loaded = true;
    Serial.println("imu    BNO055 calibration restored from NVS (verified)");
  } else {
    Serial.println("imu    BNO055 calibration in NVS did NOT stick (read-back "
                   "differs); calibrate again");
  }
}

// Leaves fusion for ~35 ms (offsets are only readable in CONFIG mode), and
// re-entering a fusion mode restarts the fusion: in IMUPLUS the yaw jumps
// back to 0. So it runs once per boot, after a full calibration, and only
// while the robot stands still (imuRead's robot_still).
static void imuSaveOffsets() {
  uint8_t ofs[bno::OFFSETS_LEN];
  imuSetMode(bno::MODE_CONFIG);
  const bool ok = imuReadRegs(imu_addr, bno::OFFSETS, ofs, sizeof(ofs));
  imuSetMode(imuFusionMode());
  imu_offsets_saved = true;   // one attempt per boot either way
  if (!ok) return;
  Preferences prefs;
  if (!prefs.begin("bno055", false)) return;
  prefs.putBytes("ofs", ofs, sizeof(ofs));
  prefs.end();
  Serial.println("imu    BNO055 fully calibrated, offsets saved to NVS");
}

static bool imuProbe() {
  static const uint8_t ADDRS[] = {0x28, 0x29};
  for (uint8_t a : ADDRS) {
    uint8_t id = 0;
    if (imuReadRegs(a, bno::CHIP_ID, &id, 1) && id == bno::ID_VALUE) {
      imu_addr = a;
      break;
    }
  }
  if (!imu_addr) return false;

  // No soft reset: it costs 650 ms, and CONFIG mode is enough to take over a
  // chip left running by the previous boot of this sketch.
  imuSetMode(bno::MODE_CONFIG);
  imuWrite8(bno::PAGE_ID, 0);
  imuWrite8(bno::PWR_MODE, 0x00);   // normal
  imuWrite8(bno::UNIT_SEL, 0x00);   // m/s^2, deg/s, degrees, Celsius
  imuRestoreOffsets();
  imuSetMode(imuFusionMode());

  imu_errors = 0;
  Serial.printf("imu    BNO055 at 0x%02X, %s fusion\n", imu_addr,
                IMU_USE_COMPASS ? "NDOF (compass)" : "IMUPLUS (no compass)");
  return true;
}

static void imuBegin() {
  if (!IMU_ENABLED) return;
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_HZ);
  Wire.setTimeOut(10);   // ms; the BNO055 stretches the clock, never this long
  // The chip ignores I2C for ~650 ms after power-on.
  imu_next_probe_ms = millis() + 800;
  Serial.printf("imu    BNO055 on I2C SDA=GPIO%d SCL=GPIO%d @ %lu Hz\n",
                IMU_SDA_PIN, IMU_SCL_PIN, (unsigned long)IMU_I2C_HZ);
}

// Once per HBT1 datagram. Never blocks for long: a missing chip costs one
// failed address probe every IMU_RETRY_MS. robot_still gates the one-time
// calibration save, which resets the fused heading.
static void imuRead(bool robot_still) {
  imu_fresh = false;
  if (!IMU_ENABLED) return;

  const uint32_t now = millis();
  if (!imu_addr) {
    if ((int32_t)(now - imu_next_probe_ms) < 0) return;
    imu_next_probe_ms = now + IMU_RETRY_MS;
    if (!imuProbe()) return;
  }

  uint8_t b[bno::BURST_LEN];
  uint8_t calib = 0;
  if (!imuReadRegs(imu_addr, bno::ACC_DATA, b, sizeof(b)) ||
      !imuReadRegs(imu_addr, bno::CALIB_STAT, &calib, 1)) {
    imu_err_total++;
    if (++imu_errors >= bno::MAX_ERRORS) {
      Serial.printf("imu    BNO055 at 0x%02X stopped answering, probing again\n",
                    imu_addr);
      imu_addr = 0;
      imu_next_probe_ms = now + IMU_RETRY_MS;
    }
    return;
  }
  imu_errors = 0;

  auto s16 = [&](uint8_t reg) {
    const uint8_t i = reg - bno::ACC_DATA;
    return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8));
  };
  for (int i = 0; i < 3; i++) imu_sample.accel[i] = s16(0x08 + 2 * i);
  for (int i = 0; i < 3; i++) imu_sample.gyro[i]  = s16(0x14 + 2 * i);
  for (int i = 0; i < 4; i++) imu_sample.quat[i]  = s16(0x20 + 2 * i);
  imu_sample.calib = calib;

  // All-zero quaternion = fusion not started yet (first ~10 ms of a mode).
  imu_fresh = (imu_sample.quat[0] | imu_sample.quat[1] |
               imu_sample.quat[2] | imu_sample.quat[3]) != 0;

  const uint8_t gyr = (calib >> 4) & 3, acc = (calib >> 2) & 3, mag = calib & 3;
  if (!imu_offsets_saved && robot_still &&
      gyr == 3 && acc == 3 && (!IMU_USE_COMPASS || mag == 3))
    imuSaveOffsets();
}

// HBT1 imu_status: bit0 chip found, bit1 sample fresh, bit10 compass (NDOF),
// bits 2-9 = CALIB_STAT's four 2-bit fields: sys 2-3, gyro 4-5, accel 6-7,
// mag 8-9.
static uint16_t imuStatus() {
  if (!imu_addr) return 0;
  const uint8_t c = imu_sample.calib;
  uint16_t s = 0x0001;
  if (imu_fresh)       s |= 0x0002;
  if (IMU_USE_COMPASS) s |= 0x0400;
  s |= (uint16_t)((c >> 6) & 3) << 2;   // sys
  s |= (uint16_t)((c >> 4) & 3) << 4;   // gyro
  s |= (uint16_t)((c >> 2) & 3) << 6;   // accel
  s |= (uint16_t)(c & 3) << 8;          // mag
  return s;
}

static void imuLog() {
  if (!IMU_ENABLED) return;
  if (!imu_addr) {
    Serial.printf("imu    no BNO055 at 0x28/0x29 on SDA=GPIO%d SCL=GPIO%d "
                  "(check 3V3, GND, SDA/SCL swapped)\n", IMU_SDA_PIN, IMU_SCL_PIN);
    return;
  }
  const ImuSample & m = imu_sample;
  const float w = m.quat[0] / 16384.0f, x = m.quat[1] / 16384.0f,
              y = m.quat[2] / 16384.0f, z = m.quat[3] / 16384.0f;
  const float yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  const uint8_t c = m.calib;
  Serial.printf("imu    yaw %+6.1f deg  gyro z %+6.1f deg/s  acc z %+5.2f m/s2  "
                "calib sys/gyr/acc/mag %u/%u/%u/%u%s  %lu errors\n",
                yaw * 57.29578f, m.gyro[2] / 16.0f, m.accel[2] / 100.0f,
                (c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3,
                imu_offsets_loaded ? " (restored)" : "",
                (unsigned long)imu_err_total);
}

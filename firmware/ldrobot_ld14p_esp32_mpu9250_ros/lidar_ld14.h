// lidar_ld14.h - LDROBOT LD14/LD14P decoder and the "LDS1" UDP scan sender.
//
// Sends one UDP "LDS1" datagram per revolution to robot_ws's
// udp_lidar_receiver_node, which publishes sensor_msgs/LaserScan on /scan
// (frame_id "base_scan").
//
// Wire format, identical to the kaiaai firmware's scan_udp.h so both LiDARs
// talk to the same receiver:
//
//   offset  size          field
//        0     4          "LDS1"
//        4     2          bin count           uint16 little-endian
//        6     4          scan id             uint32 little-endian
//       10     4          reserved, zero      (the receiver skips it)
//       14  2*bins        ranges, millimetres uint16 little-endian each
//
// 0 mm means "no return"; the receiver publishes +inf for that bin.
//
// *** Bin i is the measurement i*(360/bins) degrees CLOCKWISE from the
// LiDAR's own zero mark. *** That is the raw convention the LDROBOT hardware
// reports in, and it is what the receiver expects: it does the ROS-side
// mirroring with a negative angle_increment (flip_lr, default true) and
// rotates the whole scan with angle0_deg (default 180). Fix a mounting offset
// with those two node parameters, never by shifting bins here - the ranges
// array is never reordered on either side, or the two corrections fight.
//
// The SDK's ranging-centre correction is still applied here, per point,
// because it depends on the measured distance and cannot be undone downstream.
//
// The packet layout, CRC8 table, angle interpolation and LD14/LD14P
// ranging-centre correction are ported from LDROBOT's official SDK (MIT
// licence), vendored at robot_ws/src/ldlidar_ros2_ws/src/ldlidar_ros2/sdk.
//
// Wiring:
//   LiDAR TX  -> GPIO16               3.3 V logic, no level shifter needed
//   LiDAR RX  -> unwired              this sketch never transmits (GPIO17 is
//                                     the left HC-SR04 TRIG now)
//   LiDAR GND -> GND                  must share ground with the ESP32
//   LiDAR VCC -> 5V                   the LD14P spins up on its own
//
// The reader runs as its own FreeRTOS task on core 0 (lidarTask); the sender
// (sendPendingScan) runs from loop() on core 1, because a WiFi send can stall
// for milliseconds. The two exchange one finished revolution at a time through
// a mutex-protected snapshot.

#pragma once

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"

HardwareSerial LidarSerial(1);

// ------------------------------------------------------------- on-wire packets
static const uint8_t PKG_HEADER        = 0x54;
static const uint8_t DATA_PKG_INFO     = 0x2C;
static const uint8_t HEALTH_PKG_INFO   = 0xE0;
static const uint8_t MANUFACT_PKG_INFO = 0x0F;
static const int     POINT_PER_PACK    = 12;

#pragma pack(push, 1)
struct LidarPoint {
  uint16_t distance;      // mm, 0 = no return
  uint8_t  intensity;     // 0..255
};

struct MeasurePacket {    // 47 bytes
  uint8_t    header;      // 0x54
  uint8_t    ver_len;     // 0x2C = 12 points per packet
  uint16_t   speed;       // deg/s
  uint16_t   start_angle; // 0.01 deg
  LidarPoint point[POINT_PER_PACK];
  uint16_t   end_angle;   // 0.01 deg
  uint16_t   timestamp;   // ms, wraps
  uint8_t    crc8;
};

struct HealthPacket {     // 4 bytes
  uint8_t header;
  uint8_t information;    // 0xE0
  uint8_t error_code;
  uint8_t crc8;
};

struct ManufacturePacket {  // 23 bytes
  uint8_t  header;
  uint8_t  information;     // 0x0F
  uint16_t speed;
  uint16_t product_version;
  uint32_t sn_high;
  uint32_t sn_low;
  uint32_t hardware_version;
  uint32_t firmware_version;
  uint8_t  crc8;
};
#pragma pack(pop)

static_assert(sizeof(MeasurePacket) == 47, "MeasurePacket must not be padded");
static_assert(sizeof(HealthPacket) == 4, "HealthPacket must not be padded");
static_assert(sizeof(ManufacturePacket) == 23, "ManufacturePacket must not be padded");

// ------------------------------------------------------------------------ CRC8
static const uint8_t kCrcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
  0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
  0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
  0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
  0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
  0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
  0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
  0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
  0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
  0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
  0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
  0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
  0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
  0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
  0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
  0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
  0x7f, 0x32, 0xe5, 0xa8
};

static uint8_t calcCRC8(const uint8_t * data, uint16_t len) {
  uint8_t crc = 0;
  while (len--)
    crc = kCrcTable[(crc ^ *data++) & 0xff];
  return crc;
}

// ----------------------------------------------------------------- the driver
class LD14P {
public:
  // A measurement point. angle_deg is the ranging-centre-corrected angle,
  // CLOCKWISE from the LiDAR's zero mark - the convention the "LDS1" datagram
  // carries. scan_completed marks the first point of a new revolution.
  typedef void (*ScanPointCallback)(float angle_deg, uint16_t distance_mm,
                                    uint8_t intensity, bool scan_completed);
  typedef void (*HealthCallback)(uint8_t error_code);
  typedef void (*DeviceInfoCallback)(const ManufacturePacket & info);

  void setScanPointCallback(ScanPointCallback cb)   { scan_point_cb_ = cb; }
  void setHealthCallback(HealthCallback cb)         { health_cb_ = cb; }
  void setDeviceInfoCallback(DeviceInfoCallback cb) { device_info_cb_ = cb; }

  // Feed the driver everything the UART has. Call this often.
  void poll(Stream & in) {
    while (in.available() > 0) {
      int b = in.read();
      if (b < 0)
        break;
      raw_byte_count_++;
      if (sample_len_ < SAMPLE_LEN)
        sample_[sample_len_++] = (uint8_t)b;
      feed((uint8_t)b);
    }
  }

  // Throw away everything learned so far, e.g. after changing the baud rate.
  void resetStats() {
    reset();
    raw_byte_count_ = 0;
    sample_len_ = 0;
    packet_count_ = 0;
    crc_error_count_ = 0;
  }

  float    getScanFreqHz() const    { return speed_ / 360.0f; }  // revolutions/s
  uint16_t getSpeedDegPerS() const  { return speed_; }
  uint32_t getPacketCount() const   { return packet_count_; }
  uint32_t getCrcErrorCount() const { return crc_error_count_; }
  uint32_t getLastPacketMs() const  { return last_packet_ms_; }
  uint32_t getRawByteCount() const  { return raw_byte_count_; }
  const uint8_t * getSample() const { return sample_; }
  uint8_t  getSampleLen() const     { return sample_len_; }

private:
  // Ranging-centre offsets for LD14/LD14P, in mm (sl_transform.cpp).
  static constexpr float OFFSET_X = 5.9f;
  static constexpr float OFFSET_Y = -18.975571f;

  enum State { HEADER, VER_LEN, DATA, DATA_HEALTH, DATA_MANUFACTURE };

  void feed(uint8_t byte) {
    switch (state_) {
      case HEADER:
        if (byte == PKG_HEADER) {
          buf_[count_++] = byte;
          state_ = VER_LEN;
        }
        break;

      case VER_LEN:
        buf_[count_++] = byte;
        if (byte == DATA_PKG_INFO)
          state_ = DATA;
        else if (byte == HEALTH_PKG_INFO)
          state_ = DATA_HEALTH;
        else if (byte == MANUFACT_PKG_INFO)
          state_ = DATA_MANUFACTURE;
        else
          reset();  // that 0x54 was payload, not a header
        break;

      case DATA:
        buf_[count_++] = byte;
        if (count_ >= sizeof(MeasurePacket)) {
          if (checkCRC(sizeof(MeasurePacket)))
            handleMeasurePacket(*(const MeasurePacket *)buf_);
          reset();
        }
        break;

      case DATA_HEALTH:
        buf_[count_++] = byte;
        if (count_ >= sizeof(HealthPacket)) {
          if (checkCRC(sizeof(HealthPacket)) && health_cb_ != nullptr)
            health_cb_(((const HealthPacket *)buf_)->error_code);
          reset();
        }
        break;

      case DATA_MANUFACTURE:
        buf_[count_++] = byte;
        if (count_ >= sizeof(ManufacturePacket)) {
          if (checkCRC(sizeof(ManufacturePacket)) && device_info_cb_ != nullptr)
            device_info_cb_(*(const ManufacturePacket *)buf_);
          reset();
        }
        break;
    }
  }

  bool checkCRC(size_t len) {
    bool ok = calcCRC8(buf_, len - 1) == buf_[len - 1];
    if (!ok)
      crc_error_count_++;
    return ok;
  }

  void reset() {
    state_ = HEADER;
    count_ = 0;
  }

  void handleMeasurePacket(const MeasurePacket & pkt) {
    packet_count_++;
    last_packet_ms_ = millis();
    speed_ = pkt.speed;
    timestamp_ = pkt.timestamp;

    // Angle swept by this packet, handling the 360 -> 0 wrap.
    uint32_t sweep = ((uint32_t)pkt.end_angle + 36000u - (uint32_t)pkt.start_angle) % 36000u;

    // Sanity check from the SDK: at the reported rotation speed, 12 points
    // cannot span much more than speed * 12 / measure_freq degrees. A wider
    // sweep means we resynced onto a corrupt packet whose CRC happened to pass.
    if (speed_ > 0) {
      float max_sweep_deg =
        (float)speed_ * POINT_PER_PACK / LIDAR_MEASURE_FREQ * 1.5f;
      if (sweep / 100.0f > max_sweep_deg)
        return;
    }

    float step  = (sweep / 100.0f) / (POINT_PER_PACK - 1);
    float start = pkt.start_angle / 100.0f;

    for (int i = 0; i < POINT_PER_PACK; i++) {
      float raw_angle = start + i * step;
      if (raw_angle >= 360.0f)
        raw_angle -= 360.0f;

      bool scan_completed = (raw_angle < 20.0f) && (last_raw_angle_ > 340.0f);
      last_raw_angle_ = raw_angle;

      uint16_t distance = pkt.point[i].distance;
      uint8_t intensity = (distance == 0) ? 0 : pkt.point[i].intensity;

      if (scan_point_cb_ != nullptr)
        scan_point_cb_(correctAngle(raw_angle, distance), distance, intensity,
                       scan_completed);
    }
  }

  // Move the origin from the sensor's ranging centre to the centre of the
  // LiDAR circle (sl_transform.cpp). The SDK spells this as
  // (360 - raw) + shift, i.e. in the right-hand frame it flips into at the
  // same time; the datagram wants the clockwise angle, which is the same
  // correction with the flip left out: raw - shift.
  float correctAngle(float raw_angle, uint16_t distance) {
    if (distance > 0) {
      float x = distance + OFFSET_X;
      float y = distance * 0.11923f + OFFSET_Y;
      last_shift_deg_ = atanf(y / x) * 180.0f / (float)M_PI;
    }
    // A zero-distance point reuses the previous point's correction.
    float angle = raw_angle - last_shift_deg_;
    if (angle >= 360.0f)
      angle -= 360.0f;
    if (angle < 0.0f)
      angle += 360.0f;
    return angle;
  }

  State    state_ = HEADER;
  uint16_t count_ = 0;
  uint8_t  buf_[sizeof(MeasurePacket)];

  uint16_t speed_ = 0;
  uint16_t timestamp_ = 0;
  float    last_raw_angle_ = 0.0f;
  float    last_shift_deg_ = 0.0f;

  uint32_t packet_count_ = 0;
  uint32_t crc_error_count_ = 0;
  uint32_t last_packet_ms_ = 0;

  // Raw-byte diagnostics: they answer "is anything reaching the RX pin at all?"
  static const uint8_t SAMPLE_LEN = 32;
  uint32_t raw_byte_count_ = 0;
  uint8_t  sample_[SAMPLE_LEN];
  uint8_t  sample_len_ = 0;

  ScanPointCallback  scan_point_cb_ = nullptr;
  HealthCallback     health_cb_ = nullptr;
  DeviceInfoCallback device_info_cb_ = nullptr;
};

LD14P lidar;

// ------------------------------------------------- scan handoff between tasks
// RANGE_MIN_MM, RANGE_MAX_MM and SCAN_MAX_AGE_MS are in config.h.

struct ScanSnapshot {
  uint16_t range_mm[SCAN_BINS];   // 0 = no valid return in this bin
  float    freq_hz;
  uint32_t start_ms;              // millis() when this revolution began
  uint32_t end_ms;
  uint16_t points;
  uint16_t valid_points;
};

// Written by the LiDAR task, read by the sender task, both under scan_mutex.
static ScanSnapshot      shared_scan;
static volatile bool     shared_scan_ready = false;
static SemaphoreHandle_t scan_mutex = nullptr;

// Owned by the LiDAR task alone.
static ScanSnapshot acc;

static void resetAccumulator(uint32_t now_ms) {
  memset(acc.range_mm, 0, sizeof(acc.range_mm));
  acc.points = 0;
  acc.valid_points = 0;
  acc.start_ms = now_ms;
}

// Hand the finished revolution to the sender. A snapshot that was never picked
// up is simply overwritten - a stale scan is worse than none.
static void publishAccumulator(uint32_t now_ms) {
  acc.freq_hz = lidar.getScanFreqHz();
  acc.end_ms  = now_ms;
  if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    memcpy(&shared_scan, &acc, sizeof(ScanSnapshot));
    shared_scan_ready = true;
    xSemaphoreGive(scan_mutex);
  }
  resetAccumulator(now_ms);
}

// Called from the LiDAR task for every point the driver decodes. The datagram
// carries ranges only, so the intensity byte is decoded and checked but not
// forwarded.
void lidar_scan_point_callback(float angle_deg, uint16_t distance_mm,
                               uint8_t intensity, bool scan_completed) {
  (void)intensity;
  uint32_t now = millis();

  if (scan_completed && acc.points > 0)
    publishAccumulator(now);

  acc.points++;
  if (distance_mm < RANGE_MIN_MM || distance_mm > RANGE_MAX_MM)
    return;  // out of range, or 0 = no return

  float a = fmodf(angle_deg, 360.0f);
  if (a < 0.0f)
    a += 360.0f;

  int bin = (int)(a * (float)SCAN_BINS / 360.0f + 0.5f);
  if (bin >= (int)SCAN_BINS)
    bin -= SCAN_BINS;
  if (bin < 0)
    return;

  // Several points can land in one bin. Keep the nearest: a range that reads
  // slightly short is a much cheaper mistake for the costmap than an obstacle
  // that disappears behind a farther return.
  if (acc.range_mm[bin] == 0 || distance_mm < acc.range_mm[bin])
    acc.range_mm[bin] = distance_mm;
  acc.valid_points++;
}

void lidar_health_callback(uint8_t error_code) {
  Serial.printf("LiDAR health: error code 0x%02X%s\n", error_code,
                error_code == 0 ? " (ok)" : "");
}

void lidar_device_info_callback(const ManufacturePacket & info) {
  Serial.printf("LiDAR SN %08lX%08lX  hw %lu  fw %lu  product ver %u\n",
                (unsigned long)info.sn_high, (unsigned long)info.sn_low,
                (unsigned long)info.hardware_version,
                (unsigned long)info.firmware_version,
                (unsigned)info.product_version);
}

// ------------------------------------------------------------- the LiDAR task
// CANDIDATE_BAUD (the rates tried when the LiDAR stays silent) is in config.h.
static const int CANDIDATE_BAUD_COUNT =
  sizeof(CANDIDATE_BAUD) / sizeof(CANDIDATE_BAUD[0]);
static int baud_index = 0;

static void diagnoseSilentLidar() {
  if (lidar.getRawByteCount() == 0) {
    // Not one byte has reached the RX pin, so the baud rate is not the issue.
    Serial.printf("nothing at all on GPIO%d - not a single byte received\n",
                  LIDAR_RX_PIN);
    Serial.println("  1. is the LiDAR motor actually spinning? if not it is a "
                   "power problem: the LD14P wants 5V and ~200 mA");
    Serial.println("  2. LiDAR TX must go to GPIO16, not the LiDAR RX pin - "
                   "try swapping the two signal wires");
    Serial.println("  3. LiDAR GND and ESP32 GND must be connected");
    Serial.println("  4. if this board is an ESP32-WROVER or any module with "
                   "PSRAM, GPIO16/17 are wired to the PSRAM and cannot be "
                   "used - move LIDAR_RX_PIN/LIDAR_TX_PIN to free pins such "
                   "as 25/26");
    return;
  }

  // Bytes are arriving but nothing frames up, which usually means the baud
  // rate is wrong. Show what came in, then try the next candidate.
  Serial.printf("%lu bytes at %lu baud, but no valid packet (%lu CRC errors)."
                " first bytes:", (unsigned long)lidar.getRawByteCount(),
                (unsigned long)CANDIDATE_BAUD[baud_index],
                (unsigned long)lidar.getCrcErrorCount());
  const uint8_t * sample = lidar.getSample();
  for (uint8_t i = 0; i < lidar.getSampleLen(); i++)
    Serial.printf(" %02X", sample[i]);
  Serial.println();

  baud_index = (baud_index + 1) % CANDIDATE_BAUD_COUNT;
  Serial.printf("trying %lu baud...\n",
                (unsigned long)CANDIDATE_BAUD[baud_index]);
  LidarSerial.updateBaudRate(CANDIDATE_BAUD[baud_index]);
  lidar.resetStats();
}

static void lidarTask(void * arg) {
  (void)arg;
  uint32_t last_report_ms = 0;

  for (;;) {
    lidar.poll(LidarSerial);

    uint32_t now = millis();

    // A revolution whose wrap-around point never arrived still goes out.
    if (acc.points > 0 && now - acc.start_ms > SCAN_MAX_AGE_MS)
      publishAccumulator(now);

    if (now - lidar.getLastPacketMs() >= 1000 && now - last_report_ms >= 3000) {
      last_report_ms = now;
      diagnoseSilentLidar();
    }

    vTaskDelay(1);  // one tick; the 2 kB RX buffer holds ~180 ms of LD14 data
  }
}

// --------------------------------------------------------------- the datagram
static const uint16_t UDP_HEADER_LEN = 14;
static const uint16_t UDP_BINS_MAX   = 720;   // 14 + 720*2 fits one MTU

static_assert(SCAN_BINS >= 1 && SCAN_BINS <= UDP_BINS_MAX,
  "SCAN_BINS must fit one UDP datagram: 14 + 2*SCAN_BINS bytes, and the "
  "receiver reads at most 4096 bytes per recvfrom()");

static WiFiUDP   udp;
static uint8_t   udp_buf[UDP_HEADER_LEN + UDP_BINS_MAX * 2];
static uint32_t  scan_id = 0;

static uint32_t sent_count = 0;
static uint32_t sent_prev = 0;
static uint32_t fail_count = 0;

// Take the newest finished revolution, if there is one, and send it.
static void sendPendingScan() {
  if (!shared_scan_ready)
    return;
  if (!dest_valid || WiFi.status() != WL_CONNECTED)
    return;
  if (xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
    return;

  float    freq         = shared_scan.freq_hz;
  uint16_t points       = shared_scan.points;
  uint16_t valid_points = shared_scan.valid_points;

  udp_buf[0] = 'L';
  udp_buf[1] = 'D';
  udp_buf[2] = 'S';
  udp_buf[3] = '1';
  udp_buf[4] = (uint8_t)(SCAN_BINS);
  udp_buf[5] = (uint8_t)(SCAN_BINS >> 8);
  udp_buf[6] = (uint8_t)(scan_id);
  udp_buf[7] = (uint8_t)(scan_id >> 8);
  udp_buf[8] = (uint8_t)(scan_id >> 16);
  udp_buf[9] = (uint8_t)(scan_id >> 24);
  udp_buf[10] = 0;    // reserved, the receiver skips bytes 10..13
  udp_buf[11] = 0;
  udp_buf[12] = 0;
  udp_buf[13] = 0;

  for (uint16_t i = 0; i < SCAN_BINS; i++) {
    // An empty bin means "nothing measured here"; the receiver spells that
    // +inf. Bins go out in the sensor's own clockwise order, never reversed.
    uint16_t mm = shared_scan.range_mm[i];
    udp_buf[UDP_HEADER_LEN + i * 2]     = (uint8_t)(mm);
    udp_buf[UDP_HEADER_LEN + i * 2 + 1] = (uint8_t)(mm >> 8);
  }
  shared_scan_ready = false;
  xSemaphoreGive(scan_mutex);

  const size_t len = UDP_HEADER_LEN + (size_t)SCAN_BINS * 2;
  bool ok = false;
  if (udp.beginPacket(dest_ip, (uint16_t)DEST_PORT) == 1) {
    udp.write(udp_buf, len);
    ok = udp.endPacket() == 1;
  }

  scan_id++;
  if (ok) {
    sent_count++;
  } else {
    fail_count++;
    // UDP is fire-and-forget, so a failure here is local: no route, or the
    // WiFi stack is out of buffers. Say so instead of going quiet - a silently
    // dropped scan looks exactly like a dead LiDAR from the ROS side.
    static uint32_t last_fail_ms = 0;
    if (millis() - last_fail_ms >= 2000) {
      last_fail_ms = millis();
      Serial.printf("UDP send to %s:%d failed (%lu total)\n",
                    DEST_IP, DEST_PORT, (unsigned long)fail_count);
    }
  }

  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms >= 1000) {
    uint32_t scans = sent_count - sent_prev;
    sent_prev = sent_count;
    last_log_ms = now;
    Serial.printf("scan %5.2f Hz  %3u pts (%3u valid)  %lu pkt  %lu crc err  "
                  "%lu scan/s sent  %lu err  rssi %d\n",
                  freq, (unsigned)points, (unsigned)valid_points,
                  (unsigned long)lidar.getPacketCount(),
                  (unsigned long)lidar.getCrcErrorCount(),
                  (unsigned long)scans, (unsigned long)fail_count,
                  (int)WiFi.RSSI());
  }
}

// ------------------------------------------------------------ lifecycle hooks
static void lidarBegin() {
  LidarSerial.setRxBufferSize(2048);  // must be called before begin()
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setHealthCallback(lidar_health_callback);
  lidar.setDeviceInfoCallback(lidar_device_info_callback);

  scan_mutex = xSemaphoreCreateMutex();
  resetAccumulator(millis());

  Serial.printf("LiDAR  UART1 @ %lu baud, RX=GPIO%d TX=%d (-1 = unwired)\n",
                (unsigned long)LIDAR_BAUD, LIDAR_RX_PIN, LIDAR_TX_PIN);
}

static void lidarOnWifiUp() {
  Serial.printf("LiDAR: sending %u-bin scans (%u B) to %s:%d\n",
                (unsigned)SCAN_BINS, (unsigned)(UDP_HEADER_LEN + SCAN_BINS * 2),
                DEST_IP, DEST_PORT);
  udp.begin(0);   // any local port; the receiver never replies
}

static void lidarOnWifiDown() {
  shared_scan_ready = false;  // never send a stale scan on reconnect
}

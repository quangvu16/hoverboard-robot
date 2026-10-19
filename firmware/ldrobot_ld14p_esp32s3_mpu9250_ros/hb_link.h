// hb_link.h - the hoverboard over WiFi: "HBT1" telemetry out, "HBC1" in.
//
// The ROS 2 end is hoverboard_udp_bridge_node in robot_ws/src/esp_tcp_server,
// started by `ros2 launch robot_bringup robot_bringup.launch.py
// base:=hoverboard`. It owns all robot geometry (wheel radius, track, ticks
// per rev); this side knows RPM and ticks only, so tuning never needs a
// reflash. Keep the two in step if either format changes.
//
// Everything here runs in hoverboardTask on core 0: the mainboard UART, the
// sonar, the MPU9250 and this UDP socket. Nothing is shared with loop() except the
// read-only dest_ip.
//
// Little-endian both ways.
//
//   ESP32 -> PC, "HBT1", 78 bytes, every HB_TELEM_PERIOD_MS, to HB_TELEM_PORT
//     0   4  "HBT1"            14  4  ticks_left   int32, running total
//     4   2  seq       uint16  18  4  ticks_right  int32, running total
//     6   4  millis    uint32  22  2  rpm_left     int16, measured, + = fwd
//    10   2  flags     uint16  24  2  rpm_right    int16, measured, + = fwd
//    12   2  fb_bytes  uint16  26  2  battery      int16, centi-volts
//          0 = probing, 18/22/26 28  2  board_temp   int16, deci-Celsius
//                              30  2  curr_left    int16, centi-amps
//    flags bit0 mainboard up   32  2  curr_right   int16
//          bit1 cmd timed out  34  4  frame_count  uint32
//          bit2 current valid  38  4  checksum_err uint32
//          bit3 sonar obstacle 42  2  sonar_mm     uint16, 0 = no echo
//          bit4 sonar backing off               (FRONT sonar)
//          bit5 sonar stale    (front sonar)
//          bit6 ticks synthesised from RPM (18-byte upstream firmware)
//          bit7 buzzer on
//          bit8 left sonar obstacle    bit10 sonar avoidance turn
//          bit9 right sonar obstacle   bit11 left sonar stale
//                                      bit12 right sonar stale
//          bit13 left LED lit   bit14 right LED lit   bit15 back LED lit
//          (lights.h; the indicators toggle with their blink)
//          (bits 3-5 and sonar_mm keep their meaning for the front sensor)
//                              44  2  sent_steer   int16, what went to the mainboard
//                              46  2  sent_speed   int16
//                              48  2  sonar_left_mm  uint16, 0 = no echo
//                              50  2  sonar_right_mm uint16, 0 = no echo
//                              52  2  imu_status   uint16, 0 = no IMU
//                              54  8  quat w,x,y,z int16 x4, 1/16384
//                              62  6  gyro x,y,z   int16 x3, 1/16 deg/s
//                              68  6  accel x,y,z  int16 x3, 1/100 m/s^2,
//                                     gravity included
//                              74  1  bat_state    uint8, 0 unknown 1 ok
//                                     2 low (speed limited) 3 critical
//                                     (wheels stopped), battery.h
//                              75  1  bat_cells    uint8
//                              76  2  bat_filtered uint16, centi-volts
//    imu_status bit0 IMU found   bit1 sample fresh   bit10 compass (never)
//               bits 2-3 / 4-5 / 6-7 / 8-9 calibration 0..3 of
//               sys / gyro / accel / mag (MPU9250: see imuStatus())
//    Same units and layout as the BNO055 firmware, so the bridge is
//    unchanged. The IMU fields are in the chip's own axes; the URDF's
//    imu_joint maps them onto the robot (mpu9250.h).
//    (the mainboard's own echo of what it received is in the serial log)
//
//   PC -> ESP32, "HBC1", 12 bytes, to HB_CMD_PORT
//     0   4  "HBC1"             6  2  rpm_left   int16, + = forward
//     4   2  seq       uint16   8  2  rpm_right  int16, + = forward
//                              10  2  flags      uint16, bit0 = enable,
//                                                bit1 = buzzer
//
// Ticks are running totals, never deltas: a dropped datagram costs nothing
// because the next one still carries the truth. The bridge re-baselines when
// millis goes backwards, i.e. when this board reboots.
//
// bit0 clear in HBC1 is an explicit stop, not the same as silence: it still
// refreshes the command timeout.

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "hoverboard.h"
#include "sonar.h"
#include "lights.h"
#include "mpu9250.h"

HardwareSerial HbSerial(2);
Hoverboard     hoverboard;

#include "battery.h"     // after hoverboard.h: low-battery protection
#include "ota.h"         // ota_active / ota_robot_still
#include "web_drive.h"   // after hoverboard.h: the drive page (web_drive.h)

static const uint16_t HB_TELEM_LEN = 78;
static const uint16_t HB_CMD_LEN   = 12;

static const uint16_t HBT_MAINBOARD_UP  = 0x0001;
static const uint16_t HBT_CMD_TIMEOUT   = 0x0002;
static const uint16_t HBT_CURRENT_VALID = 0x0004;
static const uint16_t HBT_SONAR_NEAR    = 0x0008;
static const uint16_t HBT_SONAR_BACKOFF = 0x0010;
static const uint16_t HBT_SONAR_STALE   = 0x0020;
static const uint16_t HBT_TICKS_SYNTH   = 0x0040;
static const uint16_t HBT_BUZZER        = 0x0080;
static const uint16_t HBT_SONAR_L_NEAR  = 0x0100;
static const uint16_t HBT_SONAR_R_NEAR  = 0x0200;
static const uint16_t HBT_SONAR_TURNING = 0x0400;
static const uint16_t HBT_SONAR_L_STALE = 0x0800;
static const uint16_t HBT_SONAR_R_STALE = 0x1000;
static const uint16_t HBT_LIGHT_LEFT    = 0x2000;
static const uint16_t HBT_LIGHT_RIGHT   = 0x4000;
static const uint16_t HBT_LIGHT_BACK    = 0x8000;
static const uint16_t HBC_ENABLE        = 0x0001;
static const uint16_t HBC_BUZZER        = 0x0002;

static bool hb_buzzer = false;       // what the pin is doing
static bool hb_buzzer_ros = false;   // what HBC1 last asked for

static void hbSetBuzzer(bool on) {
  if (on == hb_buzzer) return;
  hb_buzzer = on;
  digitalWrite(BUZZER_PIN, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

static WiFiUDP  hb_udp;
static bool     hb_udp_open = false;
static uint8_t  hb_tx_buf[HB_TELEM_LEN];
static uint8_t  hb_rx_buf[64];
static uint16_t hb_telem_seq = 0;
static uint32_t hb_cmd_count = 0;
static uint32_t hb_cmd_bad = 0;

static inline void put16(uint8_t * b, uint16_t off, uint16_t v) {
  b[off]     = (uint8_t)v;
  b[off + 1] = (uint8_t)(v >> 8);
}

static inline void put32(uint8_t * b, uint16_t off, uint32_t v) {
  b[off]     = (uint8_t)v;
  b[off + 1] = (uint8_t)(v >> 8);
  b[off + 2] = (uint8_t)(v >> 16);
  b[off + 3] = (uint8_t)(v >> 24);
}

static inline uint16_t get16(const uint8_t * b, uint16_t off) {
  return (uint16_t)b[off] | ((uint16_t)b[off + 1] << 8);
}

static inline uint16_t sat16(float v) {
  return (uint16_t)(int16_t)constrain(lroundf(v), -32768L, 32767L);
}

static void hbSendTelemetry() {
  if (!hb_udp_open || !dest_valid)
    return;

  uint16_t flags = 0;
  if (hoverboard.isConnected())        flags |= HBT_MAINBOARD_UP;
  if (hoverboard.isTimedOut())         flags |= HBT_CMD_TIMEOUT;
  if (hoverboard.hasCurrentFeedback()) flags |= HBT_CURRENT_VALID;
  if (sonar.obstacleNear())            flags |= HBT_SONAR_NEAR;
  if (sonarBackingOff())               flags |= HBT_SONAR_BACKOFF;
  if (sonarLeft.obstacleNear())        flags |= HBT_SONAR_L_NEAR;
  if (sonarRight.obstacleNear())       flags |= HBT_SONAR_R_NEAR;
  if (sonarTurning())                  flags |= HBT_SONAR_TURNING;
  if (sonarLeft.isStale())             flags |= HBT_SONAR_L_STALE;
  if (sonarRight.isStale())            flags |= HBT_SONAR_R_STALE;
  if (light_left_on)                   flags |= HBT_LIGHT_LEFT;
  if (light_right_on)                  flags |= HBT_LIGHT_RIGHT;
  if (light_back_on)                   flags |= HBT_LIGHT_BACK;
  if (sonar.isStale())                 flags |= HBT_SONAR_STALE;
  if (hoverboard.getFeedbackBytes() == HB_FB_LEN_UPSTREAM)
    flags |= HBT_TICKS_SYNTH;
  if (hb_buzzer)                       flags |= HBT_BUZZER;

  hb_tx_buf[0] = 'H'; hb_tx_buf[1] = 'B'; hb_tx_buf[2] = 'T'; hb_tx_buf[3] = '1';
  put16(hb_tx_buf, 4,  hb_telem_seq++);
  put32(hb_tx_buf, 6,  millis());
  put16(hb_tx_buf, 10, flags);
  put16(hb_tx_buf, 12, hoverboard.getFeedbackBytes());
  put32(hb_tx_buf, 14, (uint32_t)hoverboard.getWheelTicks(HB_LEFT));
  put32(hb_tx_buf, 18, (uint32_t)hoverboard.getWheelTicks(HB_RIGHT));
  put16(hb_tx_buf, 22, sat16(hoverboard.getWheelRPM(HB_LEFT)));
  put16(hb_tx_buf, 24, sat16(hoverboard.getWheelRPM(HB_RIGHT)));
  put16(hb_tx_buf, 26, sat16(hoverboard.getBatteryVolts() * 100.0f));
  put16(hb_tx_buf, 28, sat16(hoverboard.getBoardTempC() * 10.0f));
  put16(hb_tx_buf, 30, sat16(hoverboard.getCurrentAmps(HB_LEFT) * 100.0f));
  put16(hb_tx_buf, 32, sat16(hoverboard.getCurrentAmps(HB_RIGHT) * 100.0f));
  put32(hb_tx_buf, 34, hoverboard.getFrameCount());
  put32(hb_tx_buf, 38, hoverboard.getChecksumErrorCount());
  put16(hb_tx_buf, 42, sonar.distanceMm());
  put16(hb_tx_buf, 44, (uint16_t)hoverboard.getCmdSteer());
  put16(hb_tx_buf, 46, (uint16_t)hoverboard.getCmdSpeed());
  put16(hb_tx_buf, 48, sonarLeft.distanceMm());
  put16(hb_tx_buf, 50, sonarRight.distanceMm());

  put16(hb_tx_buf, 52, imuStatus());
  for (int i = 0; i < 4; i++) put16(hb_tx_buf, 54 + 2 * i, (uint16_t)imu_sample.quat[i]);
  for (int i = 0; i < 3; i++) put16(hb_tx_buf, 62 + 2 * i, (uint16_t)imu_sample.gyro[i]);
  for (int i = 0; i < 3; i++) put16(hb_tx_buf, 68 + 2 * i, (uint16_t)imu_sample.accel[i]);

  hb_tx_buf[74] = (uint8_t)bat_state;
  hb_tx_buf[75] = (uint8_t)BAT_CELLS;
  put16(hb_tx_buf, 76, (uint16_t)constrain(lroundf(bat_vf * 100.0f), 0L, 65535L));

  if (hb_udp.beginPacket(dest_ip, (uint16_t)HB_TELEM_PORT) == 1) {
    hb_udp.write(hb_tx_buf, HB_TELEM_LEN);
    hb_udp.endPacket();
  }
}

static void hbReceiveCommands() {
  if (!hb_udp_open)
    return;

  // Drain the queue and act on the newest valid datagram only.
  bool     got = false;
  int16_t  rpm_l = 0, rpm_r = 0;
  uint16_t flags = 0;

  while (hb_udp.parsePacket() > 0) {
    const int n = hb_udp.read(hb_rx_buf, sizeof(hb_rx_buf));
    if (n < (int)HB_CMD_LEN || memcmp(hb_rx_buf, "HBC1", 4) != 0) {
      hb_cmd_bad++;
      continue;
    }
    rpm_l = (int16_t)get16(hb_rx_buf, 6);
    rpm_r = (int16_t)get16(hb_rx_buf, 8);
    flags = get16(hb_rx_buf, 10);
    got = true;
    hb_cmd_count++;
  }

  if (!got)
    return;
  ros_last_ms = millis();

  float want_l = 0.0f, want_r = 0.0f;
  if (flags & HBC_ENABLE) {
    want_l = (float)rpm_l;
    want_r = (float)rpm_r;
  }

  // The web stick (web_drive.h) wins while it is held.
  if (!webDriveOwns()) hoverboard.setWheelRPM(want_l, want_r);
  hb_buzzer_ros = (flags & HBC_BUZZER) != 0;
}

// The obstacle reflex sits between the network and the wheels, so it
// still works when WiFi drops or the ROS side crashes. It is recomputed from
// the ROS request every cycle rather than written back into it: an obstacle
// never waits for the next datagram, and a back-off stops the moment the
// sonar clears instead of lingering as the new setpoint.
//
// Only while ROS (or the web stick) is asking for motion. With no request (boot, ROS gone, or an
// explicit stop) the wheels stay still: backing off is blind, and a robot
// that reverses by itself at power-on because something is near the sonar is
// worse than one that simply refuses to go forward.
static void hbApplyReflex() {
  if (ota_active) {               // flashing: nothing moves until the reboot
    hoverboard.setOutputRPM(0.0f, 0.0f);
    sonarReflexIdle();
    return;
  }
  float l = hoverboard.getTargetRPM(HB_LEFT);
  float r = hoverboard.getTargetRPM(HB_RIGHT);
  if (l == 0.0f && r == 0.0f) {
    hoverboard.setOutputRPM(0.0f, 0.0f);
    sonarReflexIdle();
    return;
  }
  sonarGate(&l, &r);
  // Battery last, so it also limits the reflex: a critical pack stops even a
  // back-off (battery.h).
  const float k = batterySpeedScale();
  hoverboard.setOutputRPM(l * k, r * k);
}

static void hbLog() {
  if (!hoverboard.isConnected()) {
    Serial.printf("hoverboard: no feedback on GPIO%d (%lu frames, %lu bad). "
                  "check TX/RX, common ground, %lu baud, and that the "
                  "mainboard was built with VARIANT_USART on the cable you "
                  "wired (USART2 = left)\n",
                  HB_RX_PIN, (unsigned long)hoverboard.getFrameCount(),
                  (unsigned long)hoverboard.getChecksumErrorCount(),
                  (unsigned long)HB_BAUD);
  } else {
    Serial.printf("hoverboard: %u B every %lu ms  %.1f V  %.0f C  ticks %ld/%ld  "
                  "rpm %.0f/%.0f  sent spd %d str %d  board got spd %d str %d  "
                  "%lu cmd  %lu bad%s\n",
                  (unsigned)hoverboard.getFeedbackBytes(),
                  (unsigned long)hoverboard.getFeedbackPeriodMs(),
                  hoverboard.getBatteryVolts(), hoverboard.getBoardTempC(),
                  (long)hoverboard.getWheelTicks(HB_LEFT),
                  (long)hoverboard.getWheelTicks(HB_RIGHT),
                  hoverboard.getWheelRPM(HB_LEFT),
                  hoverboard.getWheelRPM(HB_RIGHT),
                  (int)hoverboard.getCmdSpeed(), (int)hoverboard.getCmdSteer(),
                  (int)hoverboard.getBoardCmd2(), (int)hoverboard.getBoardCmd1(),
                  (unsigned long)hb_cmd_count, (unsigned long)hb_cmd_bad,
                  hoverboard.isTimedOut() ? "  [CMD TIMEOUT - stopped]" : "");
  }

  if (HB_SPEED_LOOP && hoverboard.isConnected()) {
    static const char * F[] = {"", " STALLED", " WRONG-WAY"};
    Serial.printf("speed loop: L tgt %.0f meas %.0f out %.0f%s | "
                  "R tgt %.0f meas %.0f out %.0f%s\n",
                  hoverboard.getOutputRPM(HB_LEFT), hoverboard.getWheelRPM(HB_LEFT),
                  hoverboard.getLoopOutput(HB_LEFT), F[hoverboard.getLoopFault(HB_LEFT)],
                  hoverboard.getOutputRPM(HB_RIGHT), hoverboard.getWheelRPM(HB_RIGHT),
                  hoverboard.getLoopOutput(HB_RIGHT), F[hoverboard.getLoopFault(HB_RIGHT)]);
  }

  // Printed even without a mainboard: wiring the sonars is usually done first.
  const HCSR04 * const S[] = {&sonar, &sonarLeft, &sonarRight};
  const char * const NAME[] = {"front", "left", "right"};
  for (int i = 0; i < 3; i++) {
    Serial.printf("sonar %-5s: %u mm  %+.2f m/s  %s  %lu noise\n", NAME[i],
                  (unsigned)S[i]->distanceMm(), S[i]->speed(),
                  S[i]->isStale() ? "NO SENSOR - forward blocked" :
                    (S[i]->obstacleNear() ? "OBSTACLE" : "clear"),
                  (unsigned long)S[i]->noiseCount());
  }
  if (sonar_reflex != REFLEX_NONE)
    Serial.printf("sonar reflex: %s\n", sonarReflexName());
  batteryLog();
  imuLog();
}

static void hoverboardTask(void * arg) {
  (void)arg;
  uint32_t last_telem_ms = 0;
  uint32_t last_imu_ms = 0;
  uint32_t last_log_ms = 0;
  uint32_t last_web_ms = 0;

  for (;;) {
    hoverboard.poll();    // drains the UART, enforces the command timeout
    sonarLoop();          // fires the next ping when due, collects echoes

    // Same rate as HBT1, but read with or without WiFi, so the boot log and
    // the IMU filter do not wait for the network.
    if (millis() - last_imu_ms >= HB_TELEM_PERIOD_MS) {
      last_imu_ms = millis();
      // Still = nothing commanded (after the reflex) and nothing turning.
      const bool still =
          hoverboard.getOutputRPM(HB_LEFT) == 0.0f &&
          hoverboard.getOutputRPM(HB_RIGHT) == 0.0f &&
          fabsf(hoverboard.getWheelRPM(HB_LEFT)) < 1.0f &&
          fabsf(hoverboard.getWheelRPM(HB_RIGHT)) < 1.0f;
      imuRead(still);
      ota_robot_still = still;    // OTA only answers while this holds
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (!hb_udp_open) {
        hb_udp.begin((uint16_t)HB_CMD_PORT);
        hb_udp_open = true;
        Serial.printf("hoverboard: commands on UDP %d, telemetry to %s:%d\n",
                      HB_CMD_PORT, DEST_IP, HB_TELEM_PORT);
      }
      hbReceiveCommands();

      const uint32_t now = millis();
      if (now - last_telem_ms >= HB_TELEM_PERIOD_MS) {
        last_telem_ms = now;
        hbSendTelemetry();
      }
    } else if (hb_udp_open) {
      // Hoverboard::poll()'s command timeout stops the wheels by itself once
      // datagrams stop; this only releases the socket.
      hb_udp.stop();
      hb_udp_open = false;
      Serial.println("hoverboard: WiFi down, command socket closed");
    }

    webDriveApply(hoverboard);   // web stick command, or hand back to ROS
    batteryUpdate(hoverboard);
    hbApplyReflex();
    lightsUpdate(hoverboard);
    if (millis() - last_web_ms >= WEB_STATUS_PERIOD_MS) {
      last_web_ms = millis();
      webDrivePublish(hoverboard, sonar.obstacleNear(), sonarReflexName(),
                      sonar.distanceMm(), sonarLeft.distanceMm(), sonarRight.distanceMm());
    }
    // ROS's buzzer follows the wheels' rule (no commands, no noise); the
    // battery's chirps sound regardless.
    hbSetBuzzer(!ota_active && ((hb_buzzer_ros && !hoverboard.isTimedOut()) || batteryBeep()));
    hoverboard.transmit();

    const uint32_t now = millis();
    if (now - last_log_ms >= HB_LOG_PERIOD_MS) {
      last_log_ms = now;
      hbLog();
    }

    vTaskDelay(1);
  }
}

static void hbBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? LOW : HIGH);   // off

  sonarBegin();
  Serial.printf("sonar  front TRIG=GPIO%d ECHO=GPIO%d, stop<%u mm, clear>%u mm "
                "-> reverse %.0f rpm for max %lu ms\n",
                SONAR_TRIG_PIN, SONAR_ECHO_PIN, SONAR_STOP_MM, SONAR_CLEAR_MM,
                SONAR_BACKOFF_RPM, (unsigned long)SONAR_BACKOFF_MAX_MS);
  Serial.printf("sonar  left  TRIG=GPIO%d ECHO=GPIO%d, right TRIG=GPIO%d "
                "ECHO=GPIO%d, stop<%u mm, clear>%u mm -> turn away %.0f rpm "
                "for max %lu ms\n",
                SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_RIGHT_TRIG_PIN,
                SONAR_RIGHT_ECHO_PIN, SONAR_SIDE_STOP_MM, SONAR_SIDE_CLEAR_MM,
                SONAR_TURN_RPM, (unsigned long)SONAR_TURN_MAX_MS);

  lightsBegin();
  Serial.printf("lights LED left=GPIO%d right=GPIO%d back=GPIO%d\n",
                LED_LEFT_PIN, LED_RIGHT_PIN, LED_BACK_PIN);

  imuBegin();     // ~50 ms power-on wait; before the tasks

  Serial.printf("battery %d cells: low <%.1f V (speed x%.1f), critical <%.1f V (stop), "
                "recover >%.1f V%s\n", (int)BAT_CELLS, BAT_LOW_V, BAT_LOW_SPEED_SCALE,
                BAT_CRIT_V, BAT_RECOVER_V, BAT_PROTECT_ENABLED ? "" : "  [DISABLED]");

  hoverboard.begin(&HbSerial);
  Serial.printf("hover  UART2 @ %lu baud, RX=GPIO%d TX=GPIO%d, "
                "%ld ticks/rev, N_MOT_MAX %.0f rpm\n",
                (unsigned long)HB_BAUD, HB_RX_PIN, HB_TX_PIN,
                (long)HB_TICKS_PER_REV, HB_MB_N_MOT_MAX);
}

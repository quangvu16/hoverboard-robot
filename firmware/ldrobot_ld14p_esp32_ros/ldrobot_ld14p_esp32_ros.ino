// ESP32-WROOM on a hoverboard robot: LD14/LD14P LiDAR, hoverboard mainboard
// and an HC-SR04, all bridged to ROS 2 over WiFi/UDP.
//
//   LD14 --UART1--> ESP32 --"LDS1" UDP 9900--> udp_lidar_receiver_node  /scan
//   mainboard <--UART2--> ESP32 <--"HBT1" 9902 / "HBC1" 9901--> hoverboard_udp_bridge_node
//                                                     /odom, TF, /cmd_vel, /ultrasonic/*
//   BNO055 --I2C--> ESP32 (inside HBT1) ------------------------------->  /imu/data
//
// ROS 2 side (robot_ws):
//
//   ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard
//
// The mainboard runs hoverboard-robotics/hoverboard-firmware-hack-FOC built
// with VARIANT_USART; this board takes the place of the PC serial port that
// hoverboard-robotics/hoverboard-driver would otherwise use. See HOVERBOARD.md
// for the mainboard build, wiring and first-drive procedure, README.md for
// the LiDAR.
//
//   config.h       ALL set-up data: pin map, WiFi, ports,   <- the only file
//                  LiDAR model, mainboard mirror, speed        you edit
//                  loop, sonars, lights, buzzer
//   hoverboard.h   mainboard USART driver
//   sonar.h        three HC-SR04s and the avoidance reflex
//   lights.h       turn signals and stop light
//   lidar_ld14.h   LD14 decoder, scan binning, LDS1 sender
//   bno055.h       BNO055 IMU (I2C), read into every HBT1
//   hb_link.h      HBT1/HBC1 datagrams and the hoverboard task
//
// Task layout. Core 0: lidarTask (LiDAR UART) and hoverboardTask (mainboard
// UART, sonar, IMU, hoverboard socket), the latter at higher priority - a late
// scan costs one revolution, a late command frame lets the mainboard hit its
// own serial timeout. Core 1: loop(), which owns the WiFi association and the
// scan send, since a WiFi send can stall for milliseconds.
//
// Third-party libraries: none; only the ESP32 core (WiFi, Wire, Preferences).

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "lidar_ld14.h"
#include "hb_link.h"

// ----------------------------------------------------------------------- WiFi
// Nothing here blocks: WiFi.begin() is fired once and the status is polled, so
// a wrong SSID or a router still booting never wedges the sketch. Both tasks
// keep running throughout; datagrams are simply not sent until the link is up,
// and the hoverboard stops on its own command timeout meanwhile.
static bool     wifi_was_connected = false;
static uint32_t wifi_retry_ms = 0;

static void wifiBegin() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // power save adds ~100 ms of latency to every packet
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifi_retry_ms = millis();
}

static void wifiLoop() {
  const bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wifi_was_connected) {
    Serial.printf("WiFi up as %s\n", WiFi.localIP().toString().c_str());
    lidarOnWifiUp();
  } else if (!connected && wifi_was_connected) {
    Serial.println("WiFi lost, reconnecting");
    lidarOnWifiDown();
  }
  wifi_was_connected = connected;

  if (!connected && millis() - wifi_retry_ms >= WIFI_RETRY_MS) {
    Serial.printf("WiFi \"%s\" still down, retrying\n", WIFI_SSID);
    wifiBegin();
  }
}

// ---------------------------------------------------------------------- setup
void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  delay(200);
  Serial.println("\nLD14 + hoverboard (FOC, VARIANT_USART) on ESP32 -> UDP -> ROS 2");

  dest_valid = dest_ip.fromString(DEST_IP);
  if (!dest_valid)
    Serial.println("DEST_IP is not a valid IPv4 address; nothing will be sent");

  lidarBegin();
  hbBegin();      // starts sending zero commands at once, so the mainboard arms

  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(hoverboardTask, "hover", 4096, NULL, 3, NULL, 0);

  Serial.printf("WiFi \"%s\" -> ROS 2 at %s (scan %d, telemetry %d, commands %d)\n",
                WIFI_SSID, DEST_IP, DEST_PORT, HB_TELEM_PORT, HB_CMD_PORT);
  wifiBegin();
}

// ----------------------------------------------------------------------- loop
void loop() {
  wifiLoop();
  sendPendingScan();
  delay(2);
}

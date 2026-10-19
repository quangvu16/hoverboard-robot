// ota.h - firmware updates over Wi-Fi (ArduinoOTA, part of the ESP32 core).
//
//   tools/ota_upload.sh <sketch-dir> hoverboard.local     (or the board's IP)
//
// Three rules, because this board drives a hoverboard:
//   - A password is required (OTA_PASSWORD in secrets.h). Without one, OTA
//     stays off: anyone on the Wi-Fi could otherwise flash the robot.
//   - Requests are only answered while the robot stands still (nothing
//     commanded, wheels not turning). While it moves, espota times out with
//     "No response from the ESP"; stop the robot and try again.
//   - From the first byte to the reboot, the wheels are held at zero and
//     every command (ROS, web stick, sonar reflex) is ignored.
//
// The upload runs inside loop() on core 1 and blocks it for 10-30 s: no LiDAR
// scans go out meanwhile. hoverboardTask on core 0 keeps feeding the
// mainboard zero commands. On success the board reboots into the new image;
// the bootloader falls back to the old one if the new one does not boot.

#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>

#include "config.h"

// Written by hoverboardTask (core 0), read here (core 1). Single bools, so no
// lock: a stale read costs one 2 ms loop() pass.
static volatile bool ota_robot_still = true;
// Set when an upload starts; hoverboardTask then holds the wheels at zero.
static volatile bool ota_active = false;

static bool ota_ready = false;      // begin() ran with a password

static bool otaEnabled() {
#if OTA_ENABLED
  return strlen(OTA_PASSWORD) > 0;
#else
  return false;
#endif
}

// Once, from loop(), the first time WiFi is up. mDNS is handled by the caller
// (one hostname for the drive page and OTA).
static void otaBegin() {
#if OTA_ENABLED
  if (ota_ready) return;
  if (!otaEnabled()) {
    Serial.println("ota: OFF - add #define OTA_PASSWORD \"...\" to secrets.h to enable it");
    return;
  }
  ArduinoOTA.setHostname(ROBOT_HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setMdnsEnabled(false);

  ArduinoOTA.onStart([]() {
    ota_active = true;            // hoverboardTask zeroes the wheels from now on
    Serial.printf("ota: receiving %s - wheels held at zero\n",
                  ArduinoOTA.getCommand() == U_FLASH ? "firmware" : "filesystem");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int last = -1;
    const int pct = total ? (int)(100ULL * done / total) : 0;
    if (pct / 10 != last) { last = pct / 10; Serial.printf("ota: %d%%\n", pct); }
  });
  ArduinoOTA.onEnd([]() { Serial.println("ota: done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) {
    static const char * const NAME[] = {"auth failed (wrong password)", "begin failed (image too big?)",
                                        "connect failed", "receive failed", "end failed (bad image?)"};
    Serial.printf("ota: error: %s - old firmware keeps running\n", (unsigned)e < 5 ? NAME[e] : "unknown");
    ota_active = false;
  });
  ArduinoOTA.begin();
  ota_ready = true;
  Serial.printf("ota: ready on %s.local:%u (password set; answers only while the robot stands still)\n",
                ROBOT_HOSTNAME, (unsigned)OTA_PORT);
#endif
}

// Every loop() pass.
static void otaLoop() {
#if OTA_ENABLED
  if (ota_ready && ota_robot_still) ArduinoOTA.handle();
#endif
}

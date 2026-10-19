// battery.h - low-battery protection, on the ESP32 so it works without ROS.
//
// The voltage is the mainboard's own reading (its feedback frame), filtered
// here because a 36 V pack sags by a volt or more under acceleration:
//
//   OK        normal
//   LOW       below BAT_LOW_V_PER_CELL: wheel targets scaled by
//             BAT_LOW_SPEED_SCALE, a short chirp every BAT_LOW_BEEP_MS
//   CRITICAL  below BAT_CRIT_V_PER_CELL: wheels held at zero whatever ROS or
//             the web stick asks, a double chirp every BAT_CRIT_BEEP_MS
//
// Going down needs the filtered voltage past the threshold for BAT_CONFIRM_MS,
// so one hard launch does not trip it. Coming back needs BAT_RECOVER_V_PER_CELL,
// well above both: a stopped pack recovers a volt or so by itself, and a robot
// that stops, recovers, drives, sags and stops again forever is worse than one
// that waits for the charger (or a reboot).
//
// Both thresholds sit above the mainboard's own BAT_DEAD (3.37 V/cell in
// EmanuelFeru's Inc/config.h), which powers the board off without warning -
// this way the robot stops on purpose first. Until the mainboard reports a
// voltage the state is UNKNOWN and nothing is limited.
//
// Runs in hoverboardTask (core 0), like everything that touches the wheels.

#pragma once

#include <Arduino.h>

#include "config.h"
#include "hoverboard.h"

enum BatState : uint8_t { BAT_UNKNOWN = 0, BAT_OK = 1, BAT_LOW = 2, BAT_CRITICAL = 3 };

static BatState bat_state = BAT_UNKNOWN;
static float    bat_vf = 0.0f;              // filtered volts
static uint32_t bat_last_ms = 0;
static uint32_t bat_state_since_ms = 0;     // for the beep pattern
static BatState bat_pending = BAT_UNKNOWN;  // where the voltage says we should go
static uint32_t bat_pending_since_ms = 0;

static const float BAT_LOW_V     = BAT_LOW_V_PER_CELL * BAT_CELLS;
static const float BAT_CRIT_V    = BAT_CRIT_V_PER_CELL * BAT_CELLS;
static const float BAT_RECOVER_V = BAT_RECOVER_V_PER_CELL * BAT_CELLS;

static const char * batteryStateName(BatState s = bat_state) {
  switch (s) {
    case BAT_OK:       return "ok";
    case BAT_LOW:      return "low";
    case BAT_CRITICAL: return "critical";
    default:           return "unknown";
  }
}

static void batterySet(BatState s) {
  if (s == bat_state) return;
  Serial.printf("battery: %s -> %s at %.2f V (%.2f V/cell)%s\n",
                batteryStateName(bat_state), batteryStateName(s), bat_vf, bat_vf / BAT_CELLS,
                s == BAT_CRITICAL ? " - WHEELS STOPPED, charge the battery" :
                s == BAT_LOW ? " - speed limited" : "");
  bat_state = s;
  bat_state_since_ms = millis();
}

// Where the filtered voltage points, given where we are. Down one or two
// steps freely; up only past BAT_RECOVER_V, and then straight to OK.
static BatState batteryTarget(float v) {
  if (v < BAT_CRIT_V) return BAT_CRITICAL;
  if (bat_state == BAT_CRITICAL) return v > BAT_RECOVER_V ? BAT_OK : BAT_CRITICAL;
  if (v < BAT_LOW_V) return BAT_LOW;
  if (bat_state == BAT_LOW) return v > BAT_RECOVER_V ? BAT_OK : BAT_LOW;
  return BAT_OK;
}

// Call every hoverboardTask cycle.
static void batteryUpdate(const Hoverboard & hb) {
#if BAT_PROTECT_ENABLED
  const uint32_t now = millis();
  if (now - bat_last_ms < BAT_SAMPLE_MS) return;
  const float dt = (now - bat_last_ms) / 1000.0f;
  bat_last_ms = now;

  const float v = hb.getBatteryVolts();
  // No mainboard, or a frame without a believable voltage: keep the last state.
  if (!hb.isConnected() || v < BAT_CELLS * 2.5f || v > BAT_CELLS * 4.5f) return;

  if (bat_state == BAT_UNKNOWN) {   // first reading: start from it, no confirm wait
    bat_vf = v;
    const BatState s = v < BAT_CRIT_V ? BAT_CRITICAL : (v < BAT_LOW_V ? BAT_LOW : BAT_OK);
    batterySet(s);
    bat_pending = s;
    return;
  }

  bat_vf += (v - bat_vf) * min(1.0f, dt / BAT_FILTER_S);

  const BatState want = batteryTarget(bat_vf);
  if (want == bat_state) { bat_pending = bat_state; return; }
  if (want != bat_pending) { bat_pending = want; bat_pending_since_ms = now; return; }
  if (now - bat_pending_since_ms >= BAT_CONFIRM_MS) batterySet(want);
#else
  (void)hb;
#endif
}

// Factor for the wheel targets: 1 = untouched, 0 = stopped.
static float batterySpeedScale() {
  switch (bat_state) {
    case BAT_LOW:      return BAT_LOW_SPEED_SCALE;
    case BAT_CRITICAL: return 0.0f;
    default:           return 1.0f;
  }
}

// Buzzer request from the battery: a 120 ms chirp every BAT_LOW_BEEP_MS when
// low, two of them every BAT_CRIT_BEEP_MS when critical. Starts at once on
// entering the state.
static bool batteryBeep() {
#if BAT_BEEP_ENABLED
  const uint32_t t = millis() - bat_state_since_ms;
  if (bat_state == BAT_LOW) return t % BAT_LOW_BEEP_MS < 120;
  if (bat_state == BAT_CRITICAL) {
    const uint32_t p = t % BAT_CRIT_BEEP_MS;
    return p < 120 || (p >= 240 && p < 360);
  }
#endif
  return false;
}

static void batteryLog() {
  if (bat_state == BAT_UNKNOWN) return;
  Serial.printf("battery: %s, %.2f V filtered (%.2f V/cell; low <%.1f V, critical <%.1f V, "
                "recover >%.1f V)\n", batteryStateName(), bat_vf, bat_vf / BAT_CELLS,
                BAT_LOW_V, BAT_CRIT_V, BAT_RECOVER_V);
}

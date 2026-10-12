// lights.h - turn signals and the stop light
//
//   back LED   ON while the robot is stopped, OFF while it moves
//   left LED   BLINKS while the robot turns left  (front-left indicator)
//   right LED  BLINKS while the robot turns right (front-right indicator)
//
// Driven from what the wheels are actually doing, not from what ROS asked
// for: the output setpoint AFTER the sonar reflex, plus the measured wheel
// RPM while the mainboard is reporting. So a sonar avoidance turn blinks the
// right indicator, a refused forward command shows the stop light, and a robot
// still coasting to a halt is not yet "stopped".
//
// "Turning" is a left/right speed difference of more than LIGHT_TURN_DIFF_RPM,
// which covers both spinning in place and a curve while driving: right wheel
// faster than the left = turning left.
//
// --------------------------------------------------------------------- wiring
//
// Each LED: GPIO -> resistor (220-330 ohm for a plain 3.3 V LED) -> LED anode,
// cathode -> GND. A GPIO sources at most ~20-40 mA, so anything brighter (LED
// strips, 12 V lamps) needs a transistor or MOSFET module, and then the
// polarity is set by LIGHT_ACTIVE_HIGH. Pins are in config.h.

#pragma once

#include <Arduino.h>

#include "config.h"
#include "hoverboard.h"

// What each LED is showing right now, blink phase included - reported to ROS
// in the HBT1 flags (hb_link.h).
static bool light_left_on = false, light_right_on = false, light_back_on = true;

static void lightWrite(int pin, bool on) {
  digitalWrite(pin, (on == LIGHT_ACTIVE_HIGH) ? HIGH : LOW);
}

static void lightsBegin() {
  pinMode(LED_LEFT_PIN, OUTPUT);
  pinMode(LED_RIGHT_PIN, OUTPUT);
  pinMode(LED_BACK_PIN, OUTPUT);
  lightWrite(LED_LEFT_PIN, false);
  lightWrite(LED_RIGHT_PIN, false);
  lightWrite(LED_BACK_PIN, true);     // powered up = standing still
}

// Call every cycle, after the reflex has set the output setpoint.
static void lightsUpdate(const Hoverboard & hb) {
  const float out_l = hb.getOutputRPM(HB_LEFT);
  const float out_r = hb.getOutputRPM(HB_RIGHT);

  // Measured speed counts only while the mainboard is actually reporting.
  float meas_l = 0.0f, meas_r = 0.0f;
  if (hb.isConnected()) {
    meas_l = hb.getWheelRPM(HB_LEFT);
    meas_r = hb.getWheelRPM(HB_RIGHT);
  }

  const bool commanded = fabsf(out_l) >= LIGHT_STOPPED_RPM ||
                         fabsf(out_r) >= LIGHT_STOPPED_RPM;
  const bool rolling   = fabsf(meas_l) >= LIGHT_STOPPED_RPM ||
                         fabsf(meas_r) >= LIGHT_STOPPED_RPM;
  const bool stopped   = !commanded && !rolling;

  // Turning by the command while one is given, by the wheels while coasting.
  const float diff = commanded ? (out_r - out_l) : (meas_r - meas_l);
  const bool turning_left  = !stopped && diff >  LIGHT_TURN_DIFF_RPM;
  const bool turning_right = !stopped && diff < -LIGHT_TURN_DIFF_RPM;

  const bool blink_on = (millis() / LIGHT_BLINK_MS) % 2 == 0;

  light_back_on  = stopped;
  light_left_on  = turning_left && blink_on;
  light_right_on = turning_right && blink_on;

  lightWrite(LED_BACK_PIN, light_back_on);
  lightWrite(LED_LEFT_PIN, light_left_on);
  lightWrite(LED_RIGHT_PIN, light_right_on);
}

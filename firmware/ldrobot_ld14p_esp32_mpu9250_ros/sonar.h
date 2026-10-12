// sonar.h - three HC-SR04 obstacle sensors (front, left, right) and the
// avoidance reflex
//
// API-compatible with mataruzz/libHCSR04 (HCSR04(trigger, echo), distance(),
// speed(), both in metres) so code written against that library reads the same
// here. The implementation is NOT a port of it: that library is built on
// wiringPi for the Raspberry Pi and cannot compile for the ESP32, and two of
// its behaviours are unsafe on a board that drives motors:
//
//   * distance() busy-waits on `while(digitalRead(echo) == LOW)` with no
//     timeout. A disconnected sensor, or a ping that never echoes - which is
//     the normal result for an open room - spins forever. Here that would
//     stall the hoverboard command loop and trip the task watchdog while the
//     wheels are still turning.
//   * speed() sleeps a full second between its two samples.
//
// So this version measures each echo on a pin-change interrupt, never blocks,
// and treats a missing echo as "nothing within range" after ECHO_TIMEOUT_US.
//
// ------------------------------------------------------------ ping schedule
//
// Three HC-SR04s fired together hear each other's bursts and report phantom
// obstacles. sonarLoop() therefore fires ONE sensor per SONAR_SLOT_MS, in the
// order front, left, front, right: the front sensor - the one that matters at
// driving speed - keeps its 60 ms cycle, each side sensor gets 120 ms.
//
// ------------------------------------------------------------- the reflex
//
// sonarGate() takes the wheel RPM the ROS bridge asked for and returns what
// should actually be sent to the mainboard. First matching row wins:
//
//   front latched            -> reverse at BACKOFF_RPM
//   left AND right latched   -> reverse at BACKOFF_RPM (too tight to turn)
//   left latched             -> spin right in place at TURN_RPM
//   right latched            -> spin left in place at TURN_RPM
//   clear                    -> the command, unchanged
//
// Each reaction is bounded - BACKOFF_MAX_MS / TURN_MAX_MS - and after that the
// robot stops and waits for the obstacle to move or for a human. A sensor that
// has gone stale (no real echo for SONAR_STALE_MS) refuses forward motion on
// top of all that, when it is required (SONAR_REQUIRED, SONAR_SIDES_REQUIRED).
//
// Hysteresis matches the interlock the kaiaai path already uses on this robot:
// NEAR_SAMPLES consecutive close readings to latch, FAR_SAMPLES consecutive
// clear ones to release. A single spurious short reading - which the HC-SR04
// produces regularly off soft or angled surfaces - therefore cannot stop the
// robot, and a single spurious long one cannot release it.
//
// *** THE REVERSE IS BLIND. *** There is no rear sensor, so backing up is only
// safe because it is bounded: BACKOFF_MAX_MS of reverse, then the robot stops.
// Raising that limit trades a front collision for a rear one. Spinning in
// place swings the chassis corners, so TURN_MAX_MS is bounded the same way.
//
// --------------------------------------------------------------------- wiring
//
//   HC-SR04     ESP32                      note
//   ---------   ------------------------   -------------------------------
//   VCC         5 V rail                   needs 5 V; 3.3 V gives short readings
//   TRIG        SONAR_*TRIG_PIN (config.h) 3.3 V is above its input threshold
//   ECHO        SONAR_*ECHO_PIN (config.h) *** VIA A DIVIDER - SEE BELOW ***
//   GND         GND                        must be common
//
// *** ECHO idles at 5 V and the ESP32 is NOT 5 V tolerant. Wiring it straight
// to a GPIO will damage the pin. Use a divider on EACH sensor - ECHO -> 1k ->
// GPIO, and GPIO -> 2k -> GND - which lands 5 V at 3.3 V. ***

#pragma once

#include <Arduino.h>

#include "config.h"

// Pins, distances and timings live in config.h.

// ------------------------------------------------------------ echo capture
// One block per sensor, written by its ISR. attachInterruptArg hands the ISR
// a pointer to its own block, so the three sensors share one handler.

struct SonarEcho {
  int               pin;
  volatile uint32_t rise_us;
  volatile uint32_t width_us;
  volatile uint32_t seq;
};

static void IRAM_ATTR sonarEchoIsr(void * arg) {
  SonarEcho * e = static_cast<SonarEcho *>(arg);
  if (digitalRead(e->pin)) {
    e->rise_us = micros();
  } else if (e->rise_us != 0) {
    e->width_us = micros() - e->rise_us;
    e->rise_us = 0;
    e->seq++;
  }
}

class HCSR04 {
public:
  HCSR04(int trigger, int echo, uint16_t stop_mm = SONAR_STOP_MM,
         uint16_t clear_mm = SONAR_CLEAR_MM, bool required = SONAR_REQUIRED)
    : trigger_(trigger), stop_mm_(stop_mm), clear_mm_(clear_mm),
      required_(required) {
    echo_.pin = echo;
    echo_.rise_us = 0;
    echo_.width_us = 0;
    echo_.seq = 0;
  }

  void begin() {
    pinMode(trigger_, OUTPUT);
    digitalWrite(trigger_, LOW);
    // Pulled down, so a disconnected ECHO wire sits at a defined low instead
    // of floating and generating phantom edges. The internal pulldown is ~45k
    // and does not disturb the 1k/2k divider a real sensor needs.
    pinMode(echo_.pin, INPUT_PULLDOWN);
    attachInterruptArg(digitalPinToInterrupt(echo_.pin), sonarEchoIsr, &echo_, CHANGE);
  }

  // Fire one ping. The schedule in sonarLoop() decides when.
  void ping(uint32_t now) {
    last_ping_ms_ = now;
    pending_ = true;
    digitalWrite(trigger_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_, HIGH);
    delayMicroseconds(10);         // the datasheet's 10 us trigger
    digitalWrite(trigger_, LOW);
  }

  // Call often. Collects the echo of the last ping, or times it out. Never
  // blocks.
  void poll(uint32_t now) {
    // Collect whatever the ISR captured since the last visit.
    const uint32_t seq = echo_.seq;
    if (seq != last_seq_) {
      last_seq_ = seq;
      const uint32_t w = echo_.width_us;
      if (w >= SONAR_ECHO_MIN_US && w < SONAR_ECHO_TIMEOUT_US) {
        // 343 m/s, there and back.
        distance_m_ = (w / 1000000.0f) * 343.0f / 2.0f;
        last_echo_ms_ = now;      // a REAL echo - the only thing that proves
        have_reading_ = true;     // the sensor is alive
        pending_ = false;
        updateLatch(distanceMm(), now);
      }
      else {
        noise_++;   // too short to be real; leave pending_ set so the timeout
      }             // below still runs and ages the sensor towards stale
    }

    if (pending_ && (now - last_ping_ms_) > (SONAR_ECHO_TIMEOUT_US / 1000 + 5)) {
      // Nothing came back. That is either an open room or a sensor that is
      // not there, and they look identical - so this does NOT refresh
      // last_echo_ms_. Sustained silence therefore ages into isStale().
      pending_ = false;
      distance_m_ = 0.0f;          // 0 = no return, same convention as the LiDAR
      updateLatch(0, now);         // 0 is treated as "far" by updateLatch
    }
  }

  // Metres, matching libHCSR04. 0.0 means no return within range.
  double distance() const { return distance_m_; }

  // Millimetres. 0 means no return - callers must not read that as "touching".
  uint16_t distanceMm() const {
    const float mm = distance_m_ * 1000.0f;
    if (mm <= 0.0f) return 0;
    return (mm > 65535.0f) ? 65535 : (uint16_t)lroundf(mm);
  }

  // Closing speed in m/s, from consecutive samples over their real interval.
  // Negative means the obstacle is getting closer. libHCSR04 sleeps a second
  // to get this; here it falls out of the sampling that is happening anyway.
  double speed() const { return speed_m_s_; }

  uint32_t noiseCount() const { return noise_; }
  bool obstacleNear() const { return latched_; }
  // Latched AND alive: a stale sensor's old latch must not steer the robot.
  bool active() const { return latched_ && !isStale(); }
  // True when no genuine echo has arrived recently, i.e. the sensor is
  // probably unplugged, miswired or dead. Never true for an optional sensor.
  bool isStale() const {
    if (!required_) return false;
    return !have_reading_ || (millis() - last_echo_ms_) > SONAR_STALE_MS;
  }

private:
  // 0 mm is "no return", i.e. nothing close, so it counts as far.
  void updateLatch(uint16_t mm, uint32_t now) {
    if (last_mm_ != 0 && mm != 0 && last_sample_ms_ != 0 && now > last_sample_ms_) {
      const float dt = (now - last_sample_ms_) / 1000.0f;
      speed_m_s_ = ((mm - last_mm_) / 1000.0f) / dt;
    }
    last_mm_ = mm;
    last_sample_ms_ = now;

    const bool near = (mm != 0) && (mm < stop_mm_);
    const bool far  = (mm == 0) || (mm > clear_mm_);

    if (near) {
      far_count_ = 0;
      if (near_count_ < 255) near_count_++;
      if (near_count_ >= SONAR_NEAR_SAMPLES) latched_ = true;
    } else if (far) {
      near_count_ = 0;
      if (far_count_ < 255) far_count_++;
      if (far_count_ >= SONAR_FAR_SAMPLES) latched_ = false;
    }
    // Between stop_mm_ and clear_mm_ neither counter moves: the latch holds
    // whatever state it already had. That band is the hysteresis.
  }

  int       trigger_;
  SonarEcho echo_;
  uint16_t  stop_mm_, clear_mm_;
  bool      required_;

  float    distance_m_ = 0.0f;
  float    speed_m_s_  = 0.0f;
  uint16_t last_mm_ = 0;
  uint32_t last_sample_ms_ = 0;

  uint32_t last_ping_ms_ = 0;
  uint32_t last_echo_ms_ = 0;   // last REAL echo, not a timeout
  uint32_t last_seq_ = 0;
  bool     pending_ = false;
  bool     have_reading_ = false;

  uint32_t noise_ = 0;
  uint8_t  near_count_ = 0;
  uint8_t  far_count_  = 0;
  bool     latched_ = false;
};

HCSR04 sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN);             // front
HCSR04 sonarLeft(SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN,
                 SONAR_SIDE_STOP_MM, SONAR_SIDE_CLEAR_MM, SONAR_SIDES_REQUIRED);
HCSR04 sonarRight(SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN,
                  SONAR_SIDE_STOP_MM, SONAR_SIDE_CLEAR_MM, SONAR_SIDES_REQUIRED);

// ------------------------------------------------------------ the schedule

static uint32_t sonar_slot_ms = 0;
static uint8_t  sonar_slot = 0;

static void sonarBegin() {
  sonar.begin();
  sonarLeft.begin();
  sonarRight.begin();
  sonar_slot_ms = millis();
}

// Call often. Collects echoes and fires the next sensor in the rotation.
static void sonarLoop() {
  const uint32_t now = millis();
  sonar.poll(now);
  sonarLeft.poll(now);
  sonarRight.poll(now);

  if (now - sonar_slot_ms >= SONAR_SLOT_MS) {
    sonar_slot_ms = now;
    static HCSR04 * const ORDER[] = {&sonar, &sonarLeft, &sonar, &sonarRight};
    ORDER[sonar_slot]->ping(now);
    sonar_slot = (sonar_slot + 1) % 4;
  }
}

static bool sonarAnyStale() {
  return sonar.isStale() || sonarLeft.isStale() || sonarRight.isStale();
}

// ------------------------------------------------------------- the reflex

enum SonarReflex : uint8_t { REFLEX_NONE, REFLEX_BACK, REFLEX_TURN_LEFT, REFLEX_TURN_RIGHT };

static SonarReflex sonar_reflex = REFLEX_NONE;
static uint32_t    sonar_reflex_start_ms = 0;

static const char * sonarReflexName() {
  switch (sonar_reflex) {
    case REFLEX_BACK:       return "BACKING OFF";
    case REFLEX_TURN_LEFT:  return "TURNING LEFT";
    case REFLEX_TURN_RIGHT: return "TURNING RIGHT";
    default:                return "";
  }
}

static bool sonarBackingOff() { return sonar_reflex == REFLEX_BACK; }
static bool sonarTurning() {
  return sonar_reflex == REFLEX_TURN_LEFT || sonar_reflex == REFLEX_TURN_RIGHT;
}

// No motion requested: no reflex either (see hbApplyReflex in hb_link.h).
static void sonarReflexIdle() { sonar_reflex = REFLEX_NONE; }

// The reflex. Takes what the ROS bridge asked for, returns what the mainboard
// should actually be told.
static void sonarGate(float * rpm_l, float * rpm_r) {
  const uint32_t now = millis();

  SonarReflex want = REFLEX_NONE;
  if (sonar.active())                                 want = REFLEX_BACK;
  else if (sonarLeft.active() && sonarRight.active()) want = REFLEX_BACK;
  else if (sonarLeft.active())                        want = REFLEX_TURN_RIGHT;
  else if (sonarRight.active())                       want = REFLEX_TURN_LEFT;

  // Start the time window on every change of reaction.
  if (want != sonar_reflex) {
    sonar_reflex = want;
    sonar_reflex_start_ms = now;
  }

  const uint32_t limit = (want == REFLEX_BACK) ? SONAR_BACKOFF_MAX_MS : SONAR_TURN_MAX_MS;
  // Long enough: something is not moving out of the way, so stop rather than
  // keep moving blindly.
  const bool expired = (now - sonar_reflex_start_ms) >= limit;

  switch (want) {
    case REFLEX_BACK:
      *rpm_l = expired ? 0.0f : -SONAR_BACKOFF_RPM;
      *rpm_r = expired ? 0.0f : -SONAR_BACKOFF_RPM;
      break;
    case REFLEX_TURN_RIGHT:   // obstacle on the left: clockwise, away from it
      *rpm_l = expired ? 0.0f :  SONAR_TURN_RPM;
      *rpm_r = expired ? 0.0f : -SONAR_TURN_RPM;
      break;
    case REFLEX_TURN_LEFT:    // obstacle on the right: counter-clockwise
      *rpm_l = expired ? 0.0f : -SONAR_TURN_RPM;
      *rpm_r = expired ? 0.0f :  SONAR_TURN_RPM;
      break;
    default:
      break;
  }

  // A dead sensor must not read as a clear path. Refuse forward motion, but
  // never reverse on no information - reversing blind is worse.
  if (sonarAnyStale() && *rpm_l + *rpm_r > 0.0f) {
    *rpm_l = 0.0f;
    *rpm_r = 0.0f;
  }
}

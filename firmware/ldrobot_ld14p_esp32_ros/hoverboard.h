// hoverboard.h - hoverboard mainboard driver for the ESP32.
//
// Speaks the USART protocol of a hoverboard mainboard flashed with
// hoverboard-robotics/hoverboard-firmware-hack-FOC (VARIANT_USART), the same
// protocol hoverboard-robotics/hoverboard-driver speaks from a PC serial port.
// The ESP32 stands where that driver's serial port would be: it sends wheel
// setpoints, reads the feedback, and turns the wheel counters into running
// tick totals for hb_link.h to ship over WiFi.
//
//   ESP32 -> board  SerialCommand   8 bytes  0xABCD  every HB_CMD_PERIOD_MS
//   board -> ESP32  SerialFeedback 18/22/26  0xABCD  every 10 ms
//   board -> ESP32  SerialImu      18 bytes  0xACDC  gen2.x boards only
//
// Every frame is little-endian 16-bit words whose last word is the XOR of all
// the others.
//
// ---------------------------------------------------- what the mainboard does
// Facts from the FOC fork's source that this driver depends on. If you build
// the mainboard differently, the settings in config.h have to follow.
//
// * Command. {start, steer, speed, checksum} - steer comes FIRST on the wire.
//   The mainboard mixes them (main.c, mixerFcn):
//       cmdL = speed * SPEED_COEFFICIENT + steer * STEER_COEFFICIENT
//       cmdR = speed * SPEED_COEFFICIENT - steer * STEER_COEFFICIENT
//   each clamped to +/-1000. VARIANT_USART leaves the defaults, 1.0 and 0.5,
//   so steer = cmdL - cmdR. hoverboard-driver computes exactly that as
//   "(left - speed) * 2"; this driver solves the general form so non-default
//   coefficients still come out right.
//
// * Units. The fork builds CTRL_MOD_REQ = SPD_MODE, in which cmd +/-1000 is
//   +/-N_MOT_MAX wheel RPM (1000 by default), i.e. one unit is one RPM. In
//   EmanuelFeru's upstream the default is VLT_MODE, where the same number is a
//   voltage and the wheel speed depends on load - odometry still works, but
//   the RPM you ask for is not the RPM you get.
//
// * Motor direction. The right motor is mirrored in software:
//       pwmr = -cmdR   (unless INVERT_R_DIRECTION)
//       pwml =  cmdL   (unless INVERT_L_DIRECTION)
//   so while the robot drives forward speedL_meas is POSITIVE and speedR_meas
//   is NEGATIVE. hoverboard-driver copes by taking abs() and borrowing the sign
//   of its own command, which is wrong whenever the wheel is being pushed or is
//   still braking from the other direction. This driver undoes the mirror
//   instead and keeps the measured sign.
//
// * Wheel counters (bldc.c). odom_l counts hall transitions up, odom_r counts
//   them down - the same mirror - so both INCREASE going forward, and both wrap
//   modulo 9000 (100 wheel turns at 90 ticks/turn: 15 pole pairs x 6 hall
//   states). Only the hoverboard-robotics fork has them; see feedback sizes.
//
// * Timeouts. The mainboard zeroes its command after SERIAL_TIMEOUT = 160 main
//   loops of 5 ms = 0.8 s without a valid frame, and beeps 3 times. It also
//   refuses to ARM the motors until it has seen a command within +/-50 of zero
//   (two short beeps). Both are why this driver sends a frame every
//   HB_CMD_PERIOD_MS, zeros included, from the moment it boots.
//
// ------------------------------------------------------- the feedback sizes
// Which firmware is on the board decides the frame size, and guessing wrong
// makes every checksum fail so the link looks dead. The driver probes all
// three and locks onto whichever validates twice in a row:
//
//   18 B  EmanuelFeru upstream. No wheel counters: ticks are synthesised by
//         integrating the measured RPM, which drifts. Flash the fork.
//   22 B  hoverboard-robotics/hoverboard-firmware-hack-FOC master. The one
//         this sketch is built around.
//   26 B  what hoverboard-driver's humble protocol.hpp declares: two DC-link
//         currents (centi-amps) between wheelL_cnt and batVoltage.
//
// ------------------------------------------------------------------- wiring
//   mainboard GND  <-> ESP32 GND       required, the shared reference
//   mainboard TX   -->  ESP32 HB_RX_PIN   (config.h, GPIO26)
//   mainboard RX   <--  ESP32 HB_TX_PIN   (config.h, GPIO27)
//   mainboard VCC  ---  DO NOT CONNECT (its 3V3 rail cannot supply an ESP32;
//                        on some boards that pin is 15 V)
//
// The fork's VARIANT_USART enables USART2, the LEFT sensor-board cable (PA2/
// PA3). hoverboard-driver's README says USART3 because it assumes a different
// build. USART2 is NOT 5 V tolerant; the ESP32 is 3.3 V so that is fine, but
// never put a 5 V USB-serial adapter on it.

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

#include "config.h"

// Every setting - mainboard mirror, pins, timeouts, the ESP32 speed loop -
// lives in config.h.

// ------------------------------------------------------------ wire protocol
static const uint16_t HB_START_FRAME     = 0xABCD;
static const uint16_t HB_START_FRAME_IMU = 0xACDC;

static const uint8_t HB_FB_LEN_NONE     = 0;
static const uint8_t HB_FB_LEN_UPSTREAM = 18;   // no counters
static const uint8_t HB_FB_LEN_FORK     = 22;   // counters
static const uint8_t HB_FB_LEN_DRIVER   = 26;   // counters + DC current
static const uint8_t HB_IMU_LEN         = 18;
static const uint8_t HB_FB_LEN_MAX      = HB_FB_LEN_DRIVER;

static const int32_t HB_CMD_LIMIT = 1000;       // INPUT_MIN / INPUT_MAX

#pragma pack(push, 1)
typedef struct {
  uint16_t start;
  int16_t  steer;       // steer precedes speed on the wire
  int16_t  speed;
  uint16_t checksum;
} HoverCommand;
#pragma pack(pop)
static_assert(sizeof(HoverCommand) == 8, "HoverCommand must be 8 bytes");

enum hb_wheel_t { HB_LEFT = 0, HB_RIGHT = 1 };

// ---------------------------------------------------------------- the driver
class Hoverboard {
public:
  void begin(HardwareSerial * port) {
    serial_ = port;
    // 512 bytes is ~23 frames, enough to ride out a scheduling hiccup. The
    // stock 128-byte buffer holds five.
    serial_->setRxBufferSize(512);
    serial_->begin(HB_BAUD, SERIAL_8N1, HB_RX_PIN, HB_TX_PIN);
    serial_->setTimeout(0);
    sendCommand();   // zeros, so the mainboard can arm as soon as it boots
  }

  // The ROS request: wheel RPM, positive = forward for BOTH wheels, in the
  // robot's left/right. Refreshes the command timeout.
  void setWheelRPM(float rpm_left, float rpm_right) {
    target_rpm_[HB_LEFT]  = rpm_left;
    target_rpm_[HB_RIGHT] = rpm_right;
    last_cmd_ms_ = millis();
    have_cmd_ = true;
    timed_out_ = false;
  }

  // What actually goes to the mainboard: the request after local reflexes
  // (the sonar gate). Recomputed from the request every cycle, so a reflex
  // can never outlive its cause, and never refreshes the command timeout.
  void setOutputRPM(float rpm_left, float rpm_right) {
    out_rpm_[HB_LEFT]  = rpm_left;
    out_rpm_[HB_RIGHT] = rpm_right;
  }

  // Drain the UART and enforce the command timeout. Call often.
  void poll() {
    readSerial();

    if (have_cmd_ && millis() - last_cmd_ms_ > HB_CMD_TIMEOUT_MS) {
      if (!timed_out_ &&
          (target_rpm_[HB_LEFT] != 0.0f || target_rpm_[HB_RIGHT] != 0.0f))
        Serial.println("hoverboard: command timeout, stopping wheels");
      timed_out_ = true;
    }
    if (timed_out_) {
      target_rpm_[HB_LEFT]  = 0.0f;
      target_rpm_[HB_RIGHT] = 0.0f;
    }
  }

  // Send the output setpoint when due - zeros included, always.
  void transmit() {
    if (millis() - last_cmd_sent_ms_ >= HB_CMD_PERIOD_MS)
      sendCommand();
  }

  // --- robot-frame accessors: left/right and forward-positive ---
  int32_t  getWheelTicks(uint8_t w) const  { return ticks_[w]; }
  float    getWheelRPM(uint8_t w) const    { return rpm_[w]; }
  float    getTargetRPM(uint8_t w) const   { return target_rpm_[w]; }
  float    getOutputRPM(uint8_t w) const   { return out_rpm_[w]; }
  float    getCurrentAmps(uint8_t w) const { return current_[w]; }
  float    getBatteryVolts() const         { return battery_volts_; }
  float    getBoardTempC() const           { return board_temp_c_; }
  // What the mainboard is actually executing, after its own clamp.
  int16_t  getCmdSteer() const             { return sent_steer_; }
  int16_t  getCmdSpeed() const             { return sent_speed_; }
  // What the mainboard says it received (feedback cmd1 = steer, cmd2 = speed).
  int16_t  getBoardCmd1() const            { return mb_cmd1_; }
  int16_t  getBoardCmd2() const            { return mb_cmd2_; }
  // Speed loop state, robot frame. Fault: 0 none, 1 stalled, 2 wrong way.
  float    getLoopOutput(uint8_t w) const  { return u_[w]; }
  uint8_t  getLoopFault(uint8_t w) const   { return fault_[w]; }

  uint8_t  getFeedbackBytes() const        { return layout_; }
  bool     hasCurrentFeedback() const      { return layout_ == HB_FB_LEN_DRIVER; }
  bool     hasWheelCounters() const        { return layout_ == HB_FB_LEN_FORK ||
                                                    layout_ == HB_FB_LEN_DRIVER; }
  uint32_t getFrameCount() const           { return frame_count_; }
  uint32_t getChecksumErrorCount() const   { return checksum_errors_; }
  uint32_t getImuFrameCount() const        { return imu_frames_; }
  bool     isTimedOut() const              { return timed_out_; }

  // True while feedback keeps arriving: mainboard powered, UART wired right,
  // matching baud and frame layout.
  // Smoothed interval between feedback frames, as the mainboard really sends.
  uint32_t getFeedbackPeriodMs() const     { return (uint32_t)lroundf(fb_period_ms_); }

  bool isConnected() const {
    const uint32_t limit = max(HB_FEEDBACK_TIMEOUT_MS,
                               (uint32_t)(4.0f * fb_period_ms_));
    return frame_count_ > 0 && millis() - last_frame_ms_ < limit;
  }

private:
  // Per-wheel sign that turns a mainboard reading into "forward = positive",
  // derived from main.c / bldc.c as described at the top of this file.
  static float rpmSign(uint8_t board_side) {
    if (board_side == HB_LEFT) return HB_MB_INVERT_L_DIRECTION ? -1.0f : 1.0f;
    return HB_MB_INVERT_R_DIRECTION ? 1.0f : -1.0f;       // pwmr = -cmdR
  }
  static int32_t tickSign(uint8_t board_side) {
    // bldc.c already negates odom_r, so both counters share the cmd sign.
    if (board_side == HB_LEFT) return HB_MB_INVERT_L_DIRECTION ? -1 : 1;
    return HB_MB_INVERT_R_DIRECTION ? -1 : 1;
  }
  // Robot wheel <-> mainboard side, and the robot-side inversion.
  static uint8_t boardSide(uint8_t robot_wheel) {
    return HB_SWAP_WHEELS ? (uint8_t)(1 - robot_wheel) : robot_wheel;
  }
  static float robotSign(uint8_t robot_wheel) {
    const bool inv = (robot_wheel == HB_LEFT) ? HB_INVERT_LEFT : HB_INVERT_RIGHT;
    return inv ? -1.0f : 1.0f;
  }

  // ------------------------------------------------------------- receive
  void readSerial() {
    while (serial_->available() > 0)
      feedByte((uint8_t)serial_->read());

    // A frame that stalls half-received would block the parser until the
    // missing bytes happen to arrive. Give up and hunt again.
    if (rx_len_ > 0 && millis() - rx_start_ms_ > RX_FRAME_TIMEOUT_MS)
      rx_len_ = 0;
  }

  void feedByte(uint8_t b) {
    if (rx_len_ == 0) {
      // Hunting: slide a 2-byte window for a start word (little-endian).
      const uint16_t start = ((uint16_t)b << 8) | rx_prev_;
      if (start == HB_START_FRAME || start == HB_START_FRAME_IMU) {
        rx_buf_[0] = rx_prev_;
        rx_buf_[1] = b;
        rx_len_ = 2;
        rx_is_imu_ = (start == HB_START_FRAME_IMU);
        rx_start_ms_ = millis();
      }
      rx_prev_ = b;
      return;
    }

    rx_buf_[rx_len_++] = b;
    rx_prev_ = b;

    if (rx_is_imu_) {
      // Not used here, but it shares the stream and must be consumed whole so
      // it cannot be mistaken for the start of something else.
      if (rx_len_ >= HB_IMU_LEN) {
        if (verify(HB_IMU_LEN)) imu_frames_++;
        rx_len_ = 0;
      }
      return;
    }

    if (frameDone())
      rx_len_ = 0;
  }

  uint16_t word(uint8_t i) const {
    return (uint16_t)rx_buf_[i * 2] | ((uint16_t)rx_buf_[i * 2 + 1] << 8);
  }

  bool verify(uint8_t len) const {
    const uint8_t words = len / 2;
    uint16_t x = 0;
    for (uint8_t i = 0; i + 1 < words; i++)
      x ^= word(i);
    return x == word(words - 1);
  }

  // True once the buffered bytes have been consumed, decoded or discarded;
  // false means "keep filling".
  bool frameDone() {
    if (layout_ != HB_FB_LEN_NONE) {
      if (rx_len_ < layout_)
        return false;
      if (verify(layout_))
        handleFeedback(layout_);
      else
        checksum_errors_++;
      return true;
    }

    // Probing. A frame validates at its own length; that a longer frame also
    // validates at a shorter one is a 1-in-65536 accident, and frames seen
    // while probing are never acted on, so the accident costs two frames.
    if ((rx_len_ == HB_FB_LEN_UPSTREAM || rx_len_ == HB_FB_LEN_FORK ||
         rx_len_ == HB_FB_LEN_DRIVER) && verify(rx_len_)) {
      noteCandidate(rx_len_);
      return true;
    }
    if (rx_len_ >= HB_FB_LEN_MAX) {
      checksum_errors_++;
      return true;
    }
    return false;
  }

  void noteCandidate(uint8_t len) {
    if (candidate_ != len) {
      candidate_ = len;
      candidate_run_ = 1;
      return;
    }
    if (++candidate_run_ < 2)
      return;

    layout_ = len;
    const char * what =
      len == HB_FB_LEN_FORK   ? "hoverboard-robotics FOC fork, wheel counters" :
      len == HB_FB_LEN_DRIVER ? "wheel counters + DC current" :
                                "UPSTREAM FOC, NO wheel counters - odometry "
                                "integrates RPM and will drift; flash the "
                                "hoverboard-robotics fork";
    Serial.printf("hoverboard: locked onto %u-byte feedback (%s)\n",
                  (unsigned)len, what);
  }

  void handleFeedback(uint8_t len) {
    const uint32_t now = millis();

    // Words 0..4 are common: start, cmd1, cmd2, speedR_meas, speedL_meas.
    int16_t speed_b[2], cnt_b[2] = {0, 0}, curr_b[2] = {0, 0};
    int16_t bat, temp;
    speed_b[HB_RIGHT] = (int16_t)word(3);
    speed_b[HB_LEFT]  = (int16_t)word(4);
    if (len == HB_FB_LEN_UPSTREAM) {
      bat  = (int16_t)word(5);
      temp = (int16_t)word(6);
    } else {
      cnt_b[HB_RIGHT] = (int16_t)word(5);
      cnt_b[HB_LEFT]  = (int16_t)word(6);
      if (len == HB_FB_LEN_DRIVER) {
        curr_b[HB_LEFT]  = (int16_t)word(7);
        curr_b[HB_RIGHT] = (int16_t)word(8);
        bat  = (int16_t)word(9);
        temp = (int16_t)word(10);
      } else {
        bat  = (int16_t)word(7);
        temp = (int16_t)word(8);
      }
    }

    // cmd1/cmd2 are the mainboard's own input1/input2 after its filtering -
    // for VARIANT_USART that is the steer/speed it actually received. The
    // one direct proof that commands are getting through.
    mb_cmd1_ = (int16_t)word(1);
    mb_cmd2_ = (int16_t)word(2);
    battery_volts_ = bat * 0.01f;
    board_temp_c_  = temp * 0.1f;

    const uint32_t dt = now - last_frame_ms_;
    const uint32_t gap_limit = max(HB_RESTART_GAP_MS,
                                   (uint32_t)(3.0f * fb_period_ms_));
    // The first frames only teach the period: on a slow board every one of
    // them would otherwise look like a gap, and the filter would never learn.
    const bool learning = frame_count_ < FB_LEARN_FRAMES;
    const bool gap = !learning && dt > gap_limit;
    if (gap) {
      gap_count_++;
      if (now - last_gap_log_ms_ >= 5000) {   // never once per frame
        last_gap_log_ms_ = now;
        Serial.printf("hoverboard: %lu ms feedback gap (usual %lu ms), "
                      "re-baselined counters; %lu gaps so far\n",
                      (unsigned long)dt, (unsigned long)getFeedbackPeriodMs(),
                      (unsigned long)gap_count_);
      }
    } else if (frame_count_ > 0) {
      // Learn the real period from ordinary frames only; a gap must not
      // teach the filter that gaps are normal. Seeded fast, then smoothed.
      fb_period_ms_ = learning ? (float)dt : 0.9f * fb_period_ms_ + 0.1f * dt;
    }

    for (uint8_t w = 0; w < 2; w++) {
      const uint8_t side = boardSide(w);
      const float   rs   = robotSign(w);

      rpm_[w]     = rs * rpmSign(side) * (float)speed_b[side];
      current_[w] = curr_b[side] * 0.01f;

      if (len == HB_FB_LEN_UPSTREAM) {
        // No counters on the wire. Integrate RPM into fractional ticks so the
        // ROS side still sees a running total - it will drift.
        if (frame_count_ > 0 && !gap) {
          tick_frac_[w] += rpm_[w] / 60.0f * HB_TICKS_PER_REV *
                           ((now - last_frame_ms_) / 1000.0f);
          const int32_t whole = (int32_t)tick_frac_[w];
          ticks_[w] += whole;
          tick_frac_[w] -= whole;
        }
        continue;
      }

      const int32_t cnt = cnt_b[side];
      if (frame_count_ == 0 || gap) {
        last_cnt_[w] = cnt;        // baseline only, nothing to integrate
        continue;
      }
      // Shortest way round the 0..8999 ring. At the 1000 RPM ceiling a wheel
      // moves 15 ticks per 10 ms frame, nowhere near half the ring.
      int32_t d = cnt - last_cnt_[w];
      if (d >  HB_ENCODER_WRAP / 2) d -= HB_ENCODER_WRAP;
      if (d < -HB_ENCODER_WRAP / 2) d += HB_ENCODER_WRAP;
      last_cnt_[w] = cnt;
      ticks_[w] += (int32_t)rs * tickSign(side) * d;
    }

    if (HB_SPEED_LOOP && frame_count_ > 0)
      speedLoop(gap ? 0.0f : fminf(dt / 1000.0f, 0.5f));

    last_frame_ms_ = now;
    frame_count_++;
  }

  // ------------------------------------------------------------ transmit
  // One step of the per-wheel loop, run on every feedback frame so it acts on
  // fresh measurements. dt = 0 after a gap: no integration across it.
  void speedLoop(float dt) {
    for (uint8_t w = 0; w < 2; w++) {
      const float t = out_rpm_[w];
      const float m = rpm_[w];

      if (fabsf(t) < HB_SL_TARGET_MIN) {
        // Stop request: output off, and the only thing that clears a fault.
        u_[w] = 0.0f; integ_[w] = 0.0f;
        stall_ms_[w] = wrong_ms_[w] = 0.0f;
        fault_[w] = 0;
        last_sign_[w] = 0;
        was_moving_[w] = false;
        continue;
      }
      const int8_t sg = t > 0.0f ? 1 : -1;
      if (sg != last_sign_[w]) {            // new direction: start clean
        integ_[w] = 0.0f;
        stall_ms_[w] = wrong_ms_[w] = 0.0f;
        last_sign_[w] = sg;
        was_moving_[w] = false;
      }
      if (fault_[w]) { u_[w] = 0.0f; continue; }

      const float e  = t - m;
      const float ff = HB_SL_STATIC * sg + HB_SL_KFF * t;
      const float trial = ff + HB_SL_KP * e + integ_[w];
      // Anti-windup: stop integrating while saturated, unless the error
      // would pull the output back off the limit.
      if (fabsf(trial) < HB_SL_U_MAX || (e > 0.0f) != (trial > 0.0f)) {
        integ_[w] += HB_SL_KI * e * dt;
        integ_[w] = constrain(integ_[w], -HB_SL_I_MAX, HB_SL_I_MAX);
      }
      // Stiction: the push that breaks a wheel free is far more than the push
      // that keeps it rolling, so the integral built while stuck throws it
      // past the target the moment it moves (measured on the floor: 30 rpm
      // asked, ~49 reached). Unwind it fast once the wheel runs over.
      if (m * sg > fabsf(t)) integ_[w] *= HB_SL_OVER_DECAY;
      // And at the moment it breaks free, drop most of what was built up
      // while it was stuck: that part only existed to beat stiction.
      // Once per command (re-armed by a stop or a direction change), or a
      // slow wheel hovering at the threshold would lose it over and over.
      if (!was_moving_[w] && m * sg > HB_SL_STALL_RPM) {
        integ_[w] *= HB_SL_BREAKAWAY_KEEP;
        was_moving_[w] = true;
      }
      float u = ff + HB_SL_KP * e + integ_[w];
      u = constrain(u, -HB_SL_U_MAX, HB_SL_U_MAX);
      if (u * sg < 0.0f) u = 0.0f;          // never drive against the request
      u_[w] = u;

      const float ms = dt * 1000.0f;
      // Pushing as hard as the loop is allowed (integral maxed or output
      // capped) and the wheel still not turning.
      const bool maxed = fabsf(integ_[w]) >= 0.99f * HB_SL_I_MAX ||
                         fabsf(u) >= 0.99f * HB_SL_U_MAX;
      if (maxed && fabsf(m) < HB_SL_STALL_RPM)
        stall_ms_[w] += ms;
      else
        stall_ms_[w] = 0.0f;
      if (m * sg < -HB_SL_WRONGWAY_RPM)
        wrong_ms_[w] += ms;
      else
        wrong_ms_[w] = 0.0f;

      if (stall_ms_[w] > HB_SL_STALL_MS || wrong_ms_[w] > HB_SL_WRONGWAY_MS) {
        fault_[w] = stall_ms_[w] > HB_SL_STALL_MS ? 1 : 2;
        u_[w] = 0.0f; integ_[w] = 0.0f;
        Serial.printf("hoverboard: %s wheel %s - output cut until the command "
                      "returns to zero (target %.0f rpm, measured %.0f)\n",
                      w == HB_LEFT ? "LEFT" : "RIGHT",
                      fault_[w] == 1 ? "STALLED (pushing at the limit, not turning)"
                                     : "turning AGAINST the command (sign error?)",
                      t, m);
      }
    }
  }

  void sendCommand() {
    // Robot wheels -> mainboard cmdL/cmdR, in the mainboard's own units. Both
    // are forward-positive: the motor mirror (pwmr = -cmdR) and any
    // INVERT_*_DIRECTION are applied AFTER the mixer, so they change the sign
    // of what comes back (rpmSign/tickSign) but never of what goes out.
    float cmd_b[2];
    for (uint8_t w = 0; w < 2; w++)
      cmd_b[boardSide(w)] = robotSign(w) *
        (HB_SPEED_LOOP ? (isConnected() ? u_[w] : 0.0f)
                       : out_rpm_[w] * (1000.0f / HB_MB_N_MOT_MAX));

    // The mainboard clamps cmdL/cmdR, and before that speed and steer, to
    // +/-1000 each, independently - which changes the turn radius. Scale the
    // pair together at both stages instead, so saturation only ever slows the
    // robot down along the path it was asked to follow.
    float big = fmaxf(fabsf(cmd_b[HB_LEFT]), fabsf(cmd_b[HB_RIGHT]));
    if (big > (float)HB_CMD_LIMIT) {
      cmd_b[HB_LEFT]  *= HB_CMD_LIMIT / big;
      cmd_b[HB_RIGHT] *= HB_CMD_LIMIT / big;
    }

    // Invert the mixer: cmdL = a*speed + b*steer, cmdR = a*speed - b*steer.
    float speed = (cmd_b[HB_LEFT] + cmd_b[HB_RIGHT]) / (2.0f * HB_MB_SPEED_COEFF);
    float steer = (cmd_b[HB_LEFT] - cmd_b[HB_RIGHT]) / (2.0f * HB_MB_STEER_COEFF);

    // With the default 0.5 steer coefficient, steer = cmdL - cmdR can reach
    // 2000, so a spin in place tops out at +/-500 RPM per wheel.
    big = fmaxf(fabsf(speed), fabsf(steer));
    if (big > (float)HB_CMD_LIMIT) {
      speed *= HB_CMD_LIMIT / big;
      steer *= HB_CMD_LIMIT / big;
    }

    HoverCommand c;
    c.start    = HB_START_FRAME;
    c.steer    = (int16_t)lroundf(steer);
    c.speed    = (int16_t)lroundf(speed);
    c.checksum = (uint16_t)(c.start ^ (uint16_t)c.steer ^ (uint16_t)c.speed);
    serial_->write((const uint8_t *)&c, sizeof(c));

    sent_steer_ = c.steer;
    sent_speed_ = c.speed;
    last_cmd_sent_ms_ = millis();
  }

  static const uint32_t FB_LEARN_FRAMES = 4;
  static const uint32_t RX_FRAME_TIMEOUT_MS = 20;   // a 26-byte frame is 2.3 ms

  HardwareSerial * serial_ = nullptr;

  uint8_t  rx_buf_[HB_FB_LEN_MAX] = {0};
  uint8_t  rx_len_ = 0;
  uint8_t  rx_prev_ = 0;
  bool     rx_is_imu_ = false;
  uint32_t rx_start_ms_ = 0;

  uint8_t  layout_ = HB_FB_LEN_NONE;
  uint8_t  candidate_ = HB_FB_LEN_NONE;
  uint8_t  candidate_run_ = 0;

  float    target_rpm_[2] = {0, 0};   // ROS request
  float    out_rpm_[2] = {0, 0};      // after reflexes, what is sent
  float    rpm_[2] = {0, 0};
  float    current_[2] = {0, 0};
  int32_t  ticks_[2] = {0, 0};
  int32_t  last_cnt_[2] = {0, 0};
  float    tick_frac_[2] = {0, 0};

  float    battery_volts_ = 0.0f;
  float    board_temp_c_ = 0.0f;
  int16_t  sent_steer_ = 0;
  int16_t  sent_speed_ = 0;
  int16_t  mb_cmd1_ = 0;
  int16_t  mb_cmd2_ = 0;

  uint32_t frame_count_ = 0;
  uint32_t checksum_errors_ = 0;
  uint32_t imu_frames_ = 0;
  uint32_t last_frame_ms_ = 0;
  float    fb_period_ms_ = 10.0f;     // FOC fork nominal until measured
  uint32_t gap_count_ = 0;
  uint32_t last_gap_log_ms_ = 0;
  uint32_t last_cmd_sent_ms_ = 0;
  uint32_t last_cmd_ms_ = 0;
  bool     have_cmd_ = false;
  bool     timed_out_ = false;

  float    u_[2] = {0, 0};          // speed loop output, cmd units, robot frame
  float    integ_[2] = {0, 0};
  float    stall_ms_[2] = {0, 0};
  float    wrong_ms_[2] = {0, 0};
  int8_t   last_sign_[2] = {0, 0};
  uint8_t  fault_[2] = {0, 0};
  bool     was_moving_[2] = {false, false};
};

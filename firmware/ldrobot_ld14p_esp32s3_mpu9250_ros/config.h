// config.h - ALL set-up data for this board, in one place: the pin map, WiFi
// and ports, the LiDAR model, the hoverboard mainboard mirror and speed loop,
// the HC-SR04 gate and the buzzer. The other files hold drivers and wire
// protocols only; nothing in them needs editing to set up a robot.
//
// The ROS 2 side owns the robot geometry (wheel radius, track, speed limits)
// in robot_ws, so those are not here and never need a reflash.

#pragma once

#include <Arduino.h>

// -------------------------------------------------------------------- pin map
// Every GPIO this sketch touches. ESP32-S3-DevKitC-1 v1.1 numbering (the GPIO
// number printed next to each header pin), checked against Espressif's user
// guide for that board revision.
//
//   GPIO  header  direction  wired to                     constant
//   ----  ------  ---------  ---------------------------  ------------------
//     16  J1-9    in         LiDAR TX                     LIDAR_RX_PIN
//                            (LiDAR RX unwired: the LD14 free-runs; LIDAR_TX_PIN -1)
//     21  J3-18   out        buzzer                       BUZZER_PIN
//      6  J1-6    in         mainboard TX (USART2, left)  HB_RX_PIN
//      7  J1-7    out        mainboard RX (USART2, left)  HB_TX_PIN
//      5  J1-5    out        back LED (stop light)        LED_BACK_PIN
//     13  J1-19   out        right LED (front-right)      LED_RIGHT_PIN
//     14  J1-20   out        left LED (front-left)        LED_LEFT_PIN
//     17  J1-10   out        left HC-SR04 TRIG            SONAR_LEFT_TRIG_PIN
//     10  J1-16   in         left HC-SR04 ECHO, 1k/2k     SONAR_LEFT_ECHO_PIN
//     18  J1-11   out        right HC-SR04 TRIG           SONAR_RIGHT_TRIG_PIN
//     11  J1-17   in         right HC-SR04 ECHO, 1k/2k    SONAR_RIGHT_ECHO_PIN
//      1  J3-4    out        front HC-SR04 TRIG           SONAR_TRIG_PIN
//      2  J3-5    in         front HC-SR04 ECHO, 1k/2k    SONAR_ECHO_PIN
//      8  J1-12   i/o        MPU9250 SDA (I2C)            IMU_SDA_PIN
//      9  J1-15   out        MPU9250 SCL (I2C)            IMU_SCL_PIN
//                            (free: 4, 12, 15, 39-42, 47)
//
// Moved from the ESP32-WROOM build: the S3 has no GPIO 22-25, and 26-32 are
// its flash. Same pins as before: LiDAR 16, LEDs 5/13/14, side TRIGs 17/18.
//
// Grounds: LiDAR, mainboard, HC-SR04s and LEDs must all join ESP32 GND. Power
// the LiDAR (5 V, ~200 mA) and the HC-SR04s (5 V) from the robot's 5 V rail, not
// from the DevKit's 5V pin while it runs on USB. Never connect the mainboard's
// VCC pin. Every S3 GPIO is 3.3 V only, so the ECHO dividers stay.
//
// Pins NOT to wire anything to - the checks below refuse them at compile time,
// but they cannot see a stray wire, so check the harness too:
//
//   26-32  the module's SPI flash (not on the headers).
//   33-37  the octal PSRAM on N16R8 / N8R8 / N32R16V boards (35-37 are on
//          J3 but taken; 33/34 are not broken out).
//
// The S3 chip has 45 GPIOs (0-21, 26-48), but on an N16R8 module 26-37 are
// its flash and PSRAM, which leaves 33; the boot, USB, UART0 and RGB LED pins
// below leave 24 safe ones, of which this sketch uses 15.
//   0, 3, 45, 46
//          strapping pins: 0 LOW at power-on = bootloader, 3 picks the JTAG
//          source, 45 sets the flash voltage (HIGH = 1.8 V, boot-loops),
//          46 must be LOW to flash.
//   19, 20 the native USB port (USB D-/D+).
//   43, 44 UART0 = the USB-to-UART port: Serial Monitor and flashing.
//   38     the on-board RGB LED on v1.1 (it was 48 on v1.0, so 48 is kept
//          clear too).
//   22-25  do not exist on the S3.
//
// Unlike the plain ESP32 there are no input-only pins, and every GPIO has an
// internal pull-down, which the HC-SR04 ECHO inputs use.
//
// UARTs: UART0 Serial Monitor, UART1 LiDAR, UART2 mainboard.
static const int LIDAR_RX_PIN   = 16;   // LiDAR TX -> here
static const int LIDAR_TX_PIN   = -1;   // not routed (core 2.0.x: -1 = no TX pin)
static const int BUZZER_PIN     = 21;
static const int HB_RX_PIN      = 6;    // mainboard TX -> this pin
static const int HB_TX_PIN      = 7;    // mainboard RX <- this pin
static const int SONAR_TRIG_PIN = 1;
static const int SONAR_ECHO_PIN = 2;    // through the divider, see sonar.h
static const int SONAR_LEFT_TRIG_PIN  = 17;
static const int SONAR_LEFT_ECHO_PIN  = 10;   // through its own divider
static const int SONAR_RIGHT_TRIG_PIN = 18;
static const int SONAR_RIGHT_ECHO_PIN = 11;   // through its own divider
static const int LED_LEFT_PIN  = 14;   // front-left indicator
static const int LED_RIGHT_PIN = 13;   // front-right indicator
static const int LED_BACK_PIN  = 5;    // stop light
static const int IMU_SDA_PIN   = 8;    // MPU9250, see mpu9250.h
static const int IMU_SCL_PIN   = 9;

#if !CONFIG_IDF_TARGET_ESP32S3
#error "This sketch is for the ESP32-S3: build with --fqbn esp32:esp32:esp32s3 (the ESP32-WROOM version is ldrobot_ld14p_esp32_mpu9250_ros)"
#endif

// Compile-time pin checks (C++11 constexpr, so recursion instead of loops).
namespace pincheck {
constexpr bool usable(int p) {
  return ((p >= 1 && p <= 21) || (p >= 38 && p <= 48)) &&   // 22-37 flash/PSRAM or absent
         p != 3 && p != 45 && p != 46 &&                    // strapping (0 excluded above)
         p != 19 && p != 20 &&                              // native USB
         p != 43 && p != 44 &&                              // UART0
         p != 38 && p != 48;                                // RGB LED (v1.1 / v1.0)
}
constexpr int kAll[] = {LIDAR_RX_PIN, BUZZER_PIN,
                        HB_RX_PIN, HB_TX_PIN, SONAR_TRIG_PIN, SONAR_ECHO_PIN,
                        SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN,
                        SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN,
                        LED_LEFT_PIN, LED_RIGHT_PIN, LED_BACK_PIN,
                        IMU_SDA_PIN, IMU_SCL_PIN};
constexpr int kN = sizeof(kAll) / sizeof(kAll[0]);
constexpr bool unique_from(int i, int j) {
  return i >= kN ? true
       : j >= kN ? unique_from(i + 1, i + 2)
       : kAll[i] != kAll[j] && unique_from(i, j + 1);
}
constexpr bool all_usable(int i) {
  return i >= kN ? true : usable(kAll[i]) && all_usable(i + 1);
}
}  // namespace pincheck
static_assert(pincheck::unique_from(0, 1), "two functions share one GPIO");
static_assert(pincheck::all_usable(0),
              "pin is flash/PSRAM (22-37), strapping (0, 3, 45, 46), USB (19, 20), "
              "UART0 (43, 44), the RGB LED (38, 48) or does not exist");
static_assert(LIDAR_TX_PIN == -1,
              "LIDAR_TX_PIN: add the pin to kAll");

// ------------------------------------------------------------- Serial Monitor
static const uint32_t SERIAL_MONITOR_BAUD = 115200;

// ----------------------------------------------------------------------- WiFi
// WIFI_SSID and WIFI_PASS live in secrets.h, which git ignores.
// Copy secrets.h.example to secrets.h and fill in your network.
#include "secrets.h"
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""   // older secrets.h without it: OTA stays off
#endif

// mDNS name for the drive page and OTA: http://hoverboard.local/,
// tools/ota_upload.sh ... hoverboard.local
#define ROBOT_HOSTNAME "hoverboard"

// The machine running robot_bringup - the ROS 2 dev box, not the ESP32. Both
// receivers bind INADDR_ANY, so only the ports have to match.
#define DEST_IP     "192.168.1.12"

// udp_lidar_receiver_node: "LDS1" scans, ESP32 -> PC.
#define DEST_PORT   9900

// hoverboard_udp_bridge_node: "HBT1" telemetry ESP32 -> PC on HB_TELEM_PORT,
// "HBC1" wheel commands PC -> ESP32 on HB_CMD_PORT. Telemetry is sent from the
// command socket, so the bridge learns this board's address from it and never
// needs it configured.
#define HB_TELEM_PORT 9902
#define HB_CMD_PORT   9901

// While the network is down, re-issue WiFi.begin() this often.
static const uint32_t WIFI_RETRY_MS = 10000;

// ---------------------------------------------------------------------- LiDAR
// UART1 on LIDAR_RX_PIN / LIDAR_TX_PIN (pin map above).
//
// Model profile. This unit answers at 115200, which is the LD14 profile; the
// LD14P uses 230400 and measures 4000 points/s. Change both lines together.
static const uint32_t LIDAR_BAUD         = 115200;  // LD14: 115200, LD14P: 230400
static const uint32_t LIDAR_MEASURE_FREQ = 2300;    // LD14: 2300,   LD14P: 4000

// One bin per SCAN_BINS-th of a turn. Bins with no return go out as 0 mm and
// the receiver publishes +inf, so the bin count wants to match what the sensor
// actually delivers: too many bins manufactures phantom gaps, too few throws
// away measurements.
//
// Points per revolution is NOT a property of the model, it is
// LIDAR_MEASURE_FREQ / rotation_speed. Two runs of this exact unit:
//
//   11.96 Hz -> 197 pts/rev        4.83 Hz -> 487 pts/rev
//
//           197 pts/rev        487 pts/rev
//   180     1.09 pts/bin       2.71 pts/bin  (63% of points discarded)
//   360     0.55 pts/bin       1.35 pts/bin  (45% empty vs 26% discarded)
//
// 360 is the default because a 1 deg grid is what ROS tooling expects, and
// discarding measurements is worse than leaving honest gaps. If the motor
// settles at a stable >8 Hz, 180 becomes the better fit. The ceiling is 720:
// 14 + 2*bins bytes have to fit one datagram.
static const uint16_t SCAN_BINS = 360;

// Valid LD14/LD14P range is 0.02 .. 8.0 m (ld14p.launch.py). The receiver
// advertises range_min 0.02 / range_max 20.0 for every LiDAR it serves, so
// clipping to the real sensor limits has to happen here.
static const uint16_t RANGE_MIN_MM = 20;
static const uint16_t RANGE_MAX_MM = 8000;

// A revolution that never completes - the LiDAR dropping packets looks exactly
// like this - is flushed anyway once it is this old, matching scan_udp.h.
static const uint32_t SCAN_MAX_AGE_MS = 500;

// If the LiDAR stays silent, walk these baud rates looking for one that frames
// up. 115200 is the LD14 rate; 230400 is the LD14P/LD06/LD19 rate. The first
// entry must match LIDAR_BAUD, since that is what the port is opened with.
static const uint32_t CANDIDATE_BAUD[] = {115200, 230400, 460800, 921600};

// ----------------------------------------------------------------- hoverboard
// Driver: hoverboard.h. Wire protocol constants stay there; everything below is
// set-up data.

// ============================================== must match the MAINBOARD build
// Copy these from the Inc/config.h the mainboard was flashed with.

// SPEED_COEFFICIENT / STEER_COEFFICIENT as plain factors (16384 = 1.0).
static const float HB_MB_SPEED_COEFF = 1.0f;
static const float HB_MB_STEER_COEFF = 0.5f;

// N_MOT_MAX: the RPM that cmd = 1000 means in SPD_MODE.
static const float HB_MB_N_MOT_MAX = 1000.0f;

// true if that config.h defines INVERT_L_DIRECTION / INVERT_R_DIRECTION.
// These only tell the driver how to read the feedback back: the mainboard
// applies them after its mixer, so cmd > 0 is forward either way.
static const bool HB_MB_INVERT_L_DIRECTION = false;
static const bool HB_MB_INVERT_R_DIRECTION = false;

// USART2_BAUD / USART3_BAUD.
static const uint32_t HB_BAUD = 115200;

// Counter wrap and resolution (bldc.c: modulo 9000; 15 pole pairs x 6 hall
// states). hoverboard-driver's config.hpp has the same ENCODER_MAX and
// TICKS_PER_ROTATION. The ROS side's encoder_tpr parameter must equal
// HB_TICKS_PER_REV.
static const int32_t HB_ENCODER_WRAP  = 9000;
static const int32_t HB_TICKS_PER_REV = 90;

// ======================================================== this robot's build
// (its UART pins are in the pin map at the top of this file)

// How the mainboard sits in the chassis. Change ONE at a time, robot on
// blocks, and only after the mainboard-side flags above are right. These
// apply to the command, the measured RPM and the ticks alike, so odometry
// always agrees with the direction the wheels were told to turn.
static const bool HB_SWAP_WHEELS  = false;  // mainboard's left is the robot's right
static const bool HB_INVERT_LEFT  = false;  // robot's left wheel runs backwards
static const bool HB_INVERT_RIGHT = false;  // robot's right wheel runs backwards

// Resend the setpoint this often. Well inside the mainboard's 0.8 s timeout.
static const uint32_t HB_CMD_PERIOD_MS = 50;

// Stop the wheels if no command has been set for this long, i.e. the ROS 2
// side went away (WiFi drop, VM suspended, node crashed). The frames keep
// flowing, as zeros, so the mainboard stays armed.
static const uint32_t HB_CMD_TIMEOUT_MS = 1000;

// Report the mainboard "disconnected" after this much feedback silence, or
// after 4 of its own measured feedback periods if that is longer. The FOC
// fork sends every 10 ms, but boards have been seen here sending every
// ~500 ms, and a fixed timeout would flap on those.
static const uint32_t HB_FEEDBACK_TIMEOUT_MS = 500;

// Treat a feedback gap longer than this as a possible mainboard reboot (its
// counters restart at 0) and re-baseline instead of integrating the jump.
// Costs at most the distance travelled during the gap. Like the timeout
// above, it stretches to 3 measured feedback periods on a slow board.
static const uint32_t HB_RESTART_GAP_MS = 200;

// ------------------------------------------------------ ESP32 speed loop
// For a mainboard WITHOUT its own speed loop - EmanuelFeru's upstream
// firmware in its default VLT_MODE, which is what this robot has (18-byte
// feedback). There the command is a motor VOLTAGE (1000 = full battery), so a
// fixed "RPM" number means nothing under load: on the floor 50 barely creeps
// one wheel and 40 does nothing. This closes the loop here instead, per
// wheel, on the RPM the mainboard measures and reports:
//
//   u = STATIC*sign(target) + KFF*target + KP*error + integral
//
// STATIC/KFF get the wheel moving at once; the integral finds whatever the
// load really needs. u never opposes the requested direction, and is capped
// at U_MAX. Two cutouts latch a wheel to zero until the request returns to
// zero (the ROS side sends that whenever /cmd_vel stops):
//   stall     - pushing at the limit, wheel not turning, for STALL_MS:
//               it is blocked
//   wrong way - wheel turning against the request for WRONGWAY_MS: a sign
//               error somewhere, and a PI loop on a flipped sign runs away
//
// Set HB_SPEED_LOOP false with the hoverboard-robotics fork in SPD_MODE: that
// board closes its own loop and the command is already RPM.
#ifndef HB_SPEED_LOOP_ON
#define HB_SPEED_LOOP_ON 1                         // 0 = off (fork in SPD_MODE)
#endif
static const bool     HB_SPEED_LOOP     = HB_SPEED_LOOP_ON;
static const float    HB_SL_STATIC      = 60.0f;   // breakaway, cmd units
static const float    HB_SL_KFF         = 1.0f;    // cmd units per target rpm
static const float    HB_SL_KP          = 2.0f;    // cmd units per rpm error
static const float    HB_SL_KI          = 8.0f;    // cmd units per rpm*s
static const float    HB_SL_I_MAX       = 300.0f;
static const float    HB_SL_OVER_DECAY  = 0.85f;   // per frame while over target
static const float    HB_SL_BREAKAWAY_KEEP = 0.3f; // integral kept when a stuck wheel starts
static const float    HB_SL_U_MAX       = 500.0f;  // 50% of battery voltage
static const float    HB_SL_TARGET_MIN  = 0.5f;    // |rpm| below this is "stop"
static const uint32_t HB_SL_STALL_MS    = 1500;
static const float    HB_SL_STALL_RPM   = 3.0f;
static const uint32_t HB_SL_WRONGWAY_MS = 300;
static const float    HB_SL_WRONGWAY_RPM = 8.0f;

// HBT1 telemetry to ROS (hb_link.h), and how often the link summary is logged
// to the Serial Monitor.
static const uint32_t HB_TELEM_PERIOD_MS = 20;   // 50 Hz
static const uint32_t HB_LOG_PERIOD_MS   = 5000;

// -------------------------------------------------------------- HC-SR04 sonar
// Three sensors: front, left, right. Driver and the reasoning behind each
// value: sonar.h. Pins are in the pin map.

// One sensor fires per slot, in the order front, left, front, right, so they
// never hear each other. An HC-SR04 needs ~60 ms between its own pings or the
// previous burst is still echoing around the room; at 30 ms per slot the front
// sensor pings every 60 ms and each side sensor every 120 ms.
static const uint32_t SONAR_SLOT_MS     = 30;
static const uint32_t SONAR_ECHO_TIMEOUT_US = 25000;   // ~4.3 m, past its range
static_assert(SONAR_SLOT_MS >= SONAR_ECHO_TIMEOUT_US / 1000 + 5,
              "a slot must outlast one echo timeout");

// The HC-SR04 cannot see closer than ~20 mm, which is a 116 us round trip.
// Anything shorter is not a measurement: with the sensor unplugged the ECHO
// pin floats and couples in noise, and every one of those glitches decodes as
// an obstacle a few millimetres away - which would have the robot reversing
// blindly on interference. Below this width the pulse is discarded outright.
static const uint32_t SONAR_ECHO_MIN_US = 100;

// FRONT sensor: reverse below STOP_MM, resume only above CLEAR_MM. The gap
// between them is what stops the robot oscillating at the threshold. Kept
// well above the side threshold on purpose: at 0.2 m/s the robot covers 5 cm
// in 0.25 s, less than the two samples it takes to latch, so a 50 mm front
// threshold would only trigger after contact.
static const uint16_t SONAR_STOP_MM  = 350;
static const uint16_t SONAR_CLEAR_MM = 500;

// SIDE sensors: turn away below SIDE_STOP_MM (left obstacle -> turn right,
// right obstacle -> turn left, both -> reverse), resume above SIDE_CLEAR_MM.
// The HC-SR04 reads down to ~20 mm, so 50 mm is within its range.
static const uint16_t SONAR_SIDE_STOP_MM  = 50;
static const uint16_t SONAR_SIDE_CLEAR_MM = 100;

static const uint8_t SONAR_NEAR_SAMPLES = 2;   // to latch
static const uint8_t SONAR_FAR_SAMPLES  = 3;   // to release

// Reverse speed and how long reversing may continue before giving up. At
// 25 rpm on a 6.5" wheel this is ~0.2 m/s, and 1500 ms of it is ~0.3 m.
static const float    SONAR_BACKOFF_RPM    = 25.0f;
static const uint32_t SONAR_BACKOFF_MAX_MS = 1500;

// Spin-in-place speed (each wheel, opposite directions) when a side sensor
// latches, and how long the turn may continue before the robot stops.
static const float    SONAR_TURN_RPM    = 20.0f;
static const uint32_t SONAR_TURN_MAX_MS = 1500;

// How long without a REAL echo before the sensor is presumed broken.
//
// This has to be measured against genuine echoes, not against the synthetic
// "nothing came back" result: an unplugged sensor produces the latter forever,
// and if that counted as healthy then a dead sensor would read as a clear road
// - the precise failure this is meant to catch.
//
// The cost is that a robot with truly nothing within the sensor's ~4 m range
// for this long is also judged broken and refuses to drive forward. Indoors
// that is rare and failing safe is the right side to err on; in a large open
// space, raise this or set SONAR_REQUIRED to false.
static const uint32_t SONAR_STALE_MS = 5000;

// false = a silent sensor is treated as a clear path rather than blocking
// forward motion. Only set this if you accept that a broken forward sensor
// then provides no protection at all.
static const bool SONAR_REQUIRED = true;

// The same for the two side sensors. Along an open wall-less space a side
// sensor can see nothing for SONAR_STALE_MS; set this false if that blocks
// driving, at the cost of no warning when a side sensor dies.
static const bool SONAR_SIDES_REQUIRED = true;

// --------------------------------------------------------------------- buzzer
// Buzzer, switched by bit1 of the HBC1 command flags (the ROS side's /Buzzer
// topic).
// An ACTIVE buzzer module (sounds on DC) on a 3.3 V pin, or a transistor
// driving a 5 V one. It switches off by itself when commands stop arriving.
static const bool BUZZER_ACTIVE_HIGH = true;

// --------------------------------------------------------------------- lights
// Driver: lights.h. Back LED on while stopped; left/right LED blink while
// turning that way. Pins are in the pin map.
static const bool     LIGHT_ACTIVE_HIGH   = true;   // false for low-side drivers
static const uint32_t LIGHT_BLINK_MS      = 300;    // on 300 ms, off 300 ms
// Both wheels below this |rpm| (command and measurement) = stopped.
static const float    LIGHT_STOPPED_RPM   = 1.0f;
// Left/right wheel difference above this = turning. Small enough to catch a
// gentle curve, large enough that straight driving does not flicker it.
static const float    LIGHT_TURN_DIFF_RPM = 4.0f;

// ------------------------------------------------------------------ MPU9250
// 9-axis IMU on I2C; driver and wiring in mpu9250.h. Raw gyro + accel are
// fused on the ESP32 (the clone chips' DMP is dead; yaw relative to
// power-on); quaternion, gyro and accel go out in HBT1 bytes 52-73, and
// hoverboard_udp_bridge_node publishes them as /imu/data. The compass is not
// used: the hub motors' magnets make it useless anywhere near the wheels.
// Missing or dead chip = the robot runs as before, imu_status 0 in telemetry.
#define IMU_ENABLED 1
static const uint32_t IMU_I2C_HZ = 400000;     // 100000 on long or noisy wires
// Probe again this often when the chip is missing or was dropped after read
// errors.
static const uint32_t IMU_RETRY_MS = 2000;

// ------------------------------------------------------------------- battery
// Low-battery protection (battery.h), from the voltage the mainboard reports.
// Per-cell thresholds for the stock 10S (36 V) hoverboard pack. Keep CRIT
// above the mainboard's BAT_DEAD (3.37 V/cell upstream), which powers the
// board off without warning. If the mainboard's voltage reading is off, fix
// its BAT_CALIB_* first: these trust it.
#ifndef BAT_PROTECT_ENABLED
#define BAT_PROTECT_ENABLED 1
#endif
#define BAT_BEEP_ENABLED 1
static constexpr float BAT_CELLS              = 10;     // must match the mainboard's BAT_CELLS
static constexpr float BAT_LOW_V_PER_CELL     = 3.55f;  // 35.5 V: limit speed, chirp
static constexpr float BAT_CRIT_V_PER_CELL    = 3.45f;  // 34.5 V: stop the wheels
static constexpr float BAT_RECOVER_V_PER_CELL = 3.70f;  // 37.0 V: back to normal (charged)
static constexpr float BAT_LOW_SPEED_SCALE    = 0.5f;   // wheel targets x this while LOW
static constexpr float BAT_FILTER_S           = 5.0f;   // low-pass time constant (sag)
static const uint32_t BAT_CONFIRM_MS         = 10000;  // past a threshold this long
static const uint32_t BAT_SAMPLE_MS          = 100;
static const uint32_t BAT_LOW_BEEP_MS        = 30000;  // one chirp per period while LOW
static const uint32_t BAT_CRIT_BEEP_MS       = 5000;   // two chirps per period while CRITICAL
static_assert(BAT_CRIT_V_PER_CELL < BAT_LOW_V_PER_CELL && BAT_LOW_V_PER_CELL < BAT_RECOVER_V_PER_CELL,
              "battery thresholds must be CRIT < LOW < RECOVER");
static_assert(BAT_CRIT_V_PER_CELL > 3.37f, "CRIT must stay above the mainboard's BAT_DEAD (3.37 V/cell)");

// ------------------------------------------------------------- web drive
// Drive page + WebSocket served by this board (web_drive.h): open
// http://<board-ip>/ or http://ROBOT_HOSTNAME.local/ on a phone on the same
// Wi-Fi and drive with the stick, no PC needed. The commands go through the
// same path as HBC1, so the sonar reflex and the command timeout still apply.
// Anyone on the Wi-Fi can open the page: 0 turns it off.
#ifndef WEB_DRIVE_ENABLED
#define WEB_DRIVE_ENABLED 1
#endif
static const uint16_t WEB_DRIVE_PORT = 80;
// Hard cap per wheel for the page, whatever it asks. 60 rpm on a 6.5" wheel
// is ~0.6 m/s; the page's sliders stop here too.
static const float    WEB_DRIVE_MAX_RPM = 60.0f;
// The stick owns the wheels this long after its last message (it sends 10 a
// second while held); then the wheels are zeroed once and ROS (HBC1) gets
// them back. Shorter than HB_CMD_TIMEOUT_MS.
static const uint32_t WEB_DRIVE_TIMEOUT_MS = 500;
static const uint32_t WEB_STATUS_PERIOD_MS = 200;   // status to the page, 5 Hz
static_assert(WEB_DRIVE_TIMEOUT_MS < HB_CMD_TIMEOUT_MS, "web hand-back must beat the mainboard timeout");

// ------------------------------------------------------------------------ OTA
// Firmware updates over Wi-Fi (ota.h): tools/ota_upload.sh. Needs
// OTA_PASSWORD in secrets.h; only answers while the robot stands still, and
// holds the wheels at zero during the upload. The image must fit one OTA slot
// of the default partition scheme (1.25 MB; this sketch is ~0.85 MB).
#ifndef OTA_ENABLED
#define OTA_ENABLED 1
#endif
static const uint16_t OTA_PORT = 3232;   // ArduinoOTA / espota default

// ------------------------------------------------- resolved at boot, not edited
// DEST_IP parsed once in setup(). Shared by the LiDAR sender (core 1) and the
// hoverboard link (core 0); written before either task starts, read-only after.
static IPAddress dest_ip;
static bool      dest_valid = false;

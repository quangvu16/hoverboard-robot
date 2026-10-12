# Hoverboard drive over WiFi

The ESP32 that reads the LiDAR also drives a hoverboard mainboard over UART.
It stands where [hoverboard-driver]'s PC serial port would be: the mainboard
runs [hoverboard-firmware-hack-FOC] (the hoverboard-robotics fork) unchanged,
and the ESP32 relays wheel commands and feedback to ROS 2 over UDP.

```
mainboard (FOC fork, VARIANT_USART) <--UART2 115200--> ESP32 <--WiFi/UDP--> ROS 2
  closes its own wheel speed loop       ticks, RPM, sonar gate   hoverboard_udp_bridge_node
                                                                 -> /odom, TF, /wheel_joint_states
                                                                 <- /cmd_vel, /cmd_vel_manual
```

[hoverboard-firmware-hack-FOC]: https://github.com/hoverboard-robotics/hoverboard-firmware-hack-FOC
[hoverboard-driver]: https://github.com/hoverboard-robotics/hoverboard-driver

## Where the numbers live

| Lives on | What | Why |
|---|---|---|
| **Mainboard** `Inc/config.h` | control mode, `N_MOT_MAX`, mixer coefficients, motor inverts | The FOC firmware's own build |
| **ESP32** `config.h` | a mirror of those values, pins, wheel swap/invert, timeouts, speed loop | Must agree with the mainboard; rarely changes |
| **ROS 2** `robot_bringup.launch.py` | wheel radius, track, ticks/rev, speed limits | Robot geometry. Tuned without reflashing. |

The wire carries **wheel RPM** down and **accumulated ticks** up. The ESP32
knows nothing about metres. Ticks are a running total, not a delta, so a
dropped UDP datagram costs nothing.

---

## 1. Build the mainboard firmware

Flash `hoverboard-robotics/hoverboard-firmware-hack-FOC` (master) with
`VARIANT_USART` selected. The settings below are the fork's defaults; the
ESP32 side assumes them, and each has a matching constant in the hoverboard
section of `config.h` that must change with it.

| Mainboard `Inc/config.h` | Default | ESP32 `config.h` |
|---|---|---|
| `CTRL_MOD_REQ` | `SPD_MODE` | — (in `VLT_MODE` the command is a voltage, not RPM) |
| `N_MOT_MAX` | `1000` | `HB_MB_N_MOT_MAX` |
| `SPEED_COEFFICIENT` | `16384` (1.0) | `HB_MB_SPEED_COEFF` |
| `STEER_COEFFICIENT` | `8192` (0.5) | `HB_MB_STEER_COEFF` |
| `INVERT_L_DIRECTION` / `INVERT_R_DIRECTION` | not defined | `HB_MB_INVERT_L/R_DIRECTION` |
| `USART2_BAUD` | `115200` | `HB_BAUD` |
| `CONTROL_SERIAL_USART2` + `FEEDBACK_SERIAL_USART2` | enabled | — (which cable to wire) |

### Which firmware, which feedback frame

The feedback frame size depends on the firmware, and guessing wrong makes
every checksum fail. The ESP32 probes and locks onto whichever validates twice
in a row, and says so on the console:

| Size | Firmware | Odometry |
|---|---|---|
| **22 B** | hoverboard-robotics fork — **use this** | wheel hall counters, exact |
| 26 B | what hoverboard-driver's `humble` `protocol.hpp` declares (adds DC current) | wheel hall counters, exact |
| 18 B | EmanuelFeru upstream | **no counters** — ticks integrated from RPM, drifts. Telemetry flag bit6 is set. |

### Things the FOC firmware does that matter here

- **The right motor is mirrored in software** (`pwmr = -cmdR`), so the board
  reports `speedR_meas` *negative* while driving forward. The ESP32 undoes that
  and sends forward-positive RPM for both wheels. (hoverboard-driver instead
  takes `abs()` and borrows its command's sign, which is wrong while braking.)
- **Wheel counters** count 0..8999 and wrap (100 turns at 90 ticks/turn);
  `bldc.c` already negates the right one so both rise going forward.
- **It will not arm** until it sees a command within ±50 of zero — two short
  beeps. The ESP32 sends zeros from boot, so this happens on its own.
- **Serial timeout is 0.8 s** (`SERIAL_TIMEOUT` 160 × 5 ms), then three beeps
  and zero command. The ESP32 sends a frame every 50 ms, zeros included, so you
  should only hear this if the UART is broken.
- It clamps `speed`, `steer`, `cmdL` and `cmdR` to ±1000 each. The ESP32
  scales both wheels together before that can happen, so saturation slows the
  robot along its path instead of changing the turn. With the default 0.5
  steer coefficient a spin in place tops out at 500 RPM per wheel.

### Running the stock (upstream) firmware: the ESP32 speed loop

EmanuelFeru's upstream firmware defaults to `VLT_MODE`: the command is a motor
**voltage**, not RPM, and nothing on the board holds a speed. On this robot, on
the floor, 0.4 m/s worth of command did nothing and 0.5 m/s crept one wheel.
So `hoverboard.h` closes the loop on the ESP32 (`HB_SPEED_LOOP` in `config.h`,
on by default), per wheel, on the RPM the mainboard reports every 10 ms:
feedforward (`HB_SL_STATIC` breakaway + `HB_SL_KFF`) plus PI, with the
integral unwound at breakaway and whenever the wheel overshoots, so stiction
does not throw it past the target.

Measured on the floor (2026-09-26): 30 RPM reached in ~0.3 s, held at ±3 RPM,
no overshoot, full stop in ~0.3 s.

Two cutouts zero a wheel until the command returns to zero, logged on the
console: **stall** (pushing at the limit, not turning, 1.5 s) and **wrong
way** (turning against the command for 0.3 s — a sign error, which would
otherwise make the loop run away). `HB_SL_U_MAX` caps the output at 50 %
battery voltage.

With the hoverboard-robotics fork in `SPD_MODE`, set `HB_SPEED_LOOP_ON` to 0:
that board closes its own loop and takes RPM directly.

---

## 2. Wiring

**Three wires only. Do not connect the mainboard's VCC pin.**

| Mainboard (left sensor cable, USART2) | ESP32 | |
|---|---|---|
| GND | GND | Required |
| TX (PA2) | **GPIO26** | `HB_RX_PIN` |
| RX (PA3) | **GPIO27** | `HB_TX_PIN` |
| VCC | **leave open** | Cannot supply an ESP32; 15 V on some boards |

Both sides are 3.3 V logic. `VARIANT_USART` in this fork is the **left**
cable (USART2), not USART3 as hoverboard-driver's README says. USART2 is not
5 V tolerant — never hang a 5 V USB-serial adapter on it.

GPIO16 is the LiDAR (its RX is left unwired), 32/33 the front HC-SR04, 17/19
and 21/22 the left and right HC-SR04s (TRIG/ECHO, each ECHO through its own
1k/2k divider), 14/13/5 the left/right/back lights (`lights.h`: back on while
stopped, left or right blinking while turning that way), and
**GPIO25 is the buzzer**. Every pin is in the pin map at the top of
`config.h`, which also lists the GPIOs that must stay unwired (6-11 flash,
12 flash-voltage strap) and refuses them at compile time. The buzzer is an
active buzzer module, switched from ROS by `/Buzzer` (std_msgs/Bool) through bit1 of the HBC1 flags.
It switches itself off when commands stop arriving, like the wheels.

A **BNO055 IMU** (`bno055.h`) is on I2C: **SDA GPIO21, SCL GPIO22**, VIN to
3V3, GND, ADR and PS0/PS1 open (0x28; 0x29 is probed too). The ST7789 status
display it replaced was dead hardware and has been removed; GPIO 2, 4 and 15
are free. The chip runs in IMUPLUS mode
(gyro + accel fusion, no compass: the hub motors' magnets ruin it) and its
quaternion, gyro and accel ride in HBT1 bytes 52-73, published by the bridge
as `/imu/data`. Without the chip the robot runs as before. Calibration
offsets are saved to NVS once gyro and accel both reach 3 (rest the board in
~6 orientations, each axis up and down, before mounting) and restored at
every boot.

---

## 3. Flash the ESP32

No third-party libraries are needed (WiFi, Wire and Preferences ship with
the ESP32 core). Open `ldrobot_ld14p_esp32_ros.ino` in Arduino IDE
(**ESP32 Dev Module**), check `config.h` (all set-up data lives there), upload, Serial Monitor at 115200. Or from a shell:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 ~/Arduino/ldrobot_ld14p_esp32_ros
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 ~/Arduino/ldrobot_ld14p_esp32_ros
```

A good boot:

```
LD14 + hoverboard (FOC, VARIANT_USART) on ESP32 -> UDP -> ROS 2
LiDAR  UART1 @ 115200 baud, RX=GPIO16 TX=-1 (-1 = unwired)
sonar  HC-SR04 TRIG=GPIO32 ECHO=GPIO33, stop<350 mm, clear>500 mm, reverse 25 rpm for max 1500 ms
imu    BNO055 on I2C SDA=GPIO21 SCL=GPIO22 @ 400000 Hz
hover  UART2 @ 115200 baud, RX=GPIO26 TX=GPIO27, 90 ticks/rev, N_MOT_MAX 1000 rpm
hoverboard: locked onto 22-byte feedback (hoverboard-robotics FOC fork, wheel counters)
WiFi up as 192.168.1.6
LiDAR: sending 360-bin scans (734 B) to 192.168.1.12:9900
hoverboard: commands on UDP 9901, telemetry to 192.168.1.12:9902
imu    BNO055 at 0x28, IMUPLUS (no compass) fusion
hoverboard: 22 B  36.5 V  28 C  ticks 0/0  rpm 0/0  sent spd 0 str 0  0 cmd  0 bad
```

`hoverboard: no feedback on GPIO26` means the mainboard is not talking: wrong
cable, TX/RX swapped, no common ground, wrong baud, or the board is off (it
needs its power button).

---

## 4. ROS 2 side

The bridge is `hoverboard_udp_bridge_node` in `robot_ws/src/esp_tcp_server`,
already built with the workspace:

```bash
cd ~/robot_ws && colcon build --packages-select esp_tcp_server
source install/local_setup.bash
ros2 launch robot_bringup robot_bringup.launch.py base:=hoverboard
```

It binds UDP 9902 and replies to whoever sent the telemetry, so the ESP32's
address is never configured. Geometry (`wheel_radius_m`,
`wheel_separation_m`, `encoder_tpr`) is set in `robot_bringup.launch.py`;
`encoder_tpr` must equal `HB_TICKS_PER_REV`.

```bash
ros2 topic echo /hoverboard/link_status   # esp:up mainboard:up fb=22B ...
ros2 topic hz /odom                       # ~50 Hz
```

The old Python node in `ros2/hoverboard_udp_bridge/` speaks the same wire
format but is superseded by the C++ one; do not run both.

On a VM the network must be **bridged**, and open the ports:

```bash
sudo ufw allow 9901/udp && sudo ufw allow 9902/udp
```

---

## 5. First drive — on blocks

**Wheels off the ground.** A hoverboard has the torque to throw a chassis.
Lower the bridge's `max_wheel_speed_mps` first.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'   # both forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'  # left back, right forward
```

| What you see | Fix in `config.h` |
|---|---|
| Left and right swapped | `HB_SWAP_WHEELS` |
| One wheel runs backwards | `HB_INVERT_LEFT` / `HB_INVERT_RIGHT` |
| Both run backwards | both `HB_INVERT_*` (a board mounted facing backwards usually also needs `HB_SWAP_WHEELS`) |
| Wheel turns the right way but `/odom` says the opposite | `HB_MB_INVERT_*_DIRECTION` does not match the mainboard's `config.h` |

Change one at a time. The robot-side flags apply to the command, the measured
RPM and the ticks together, so once the wheels are right, odometry is too. Do
not also flip `invert_left`/`invert_right` on the ROS side.

Then drive a measured 1.0 m and compare `/odom`. Distance wrong →
`wheel_radius_m`. Rotation wrong → `wheel_separation_m` (contact patch to
contact patch).

---

## 6. Safety layers

1. **Mainboard** zeroes its command after 0.8 s of UART silence.
2. **ESP32** zeroes the ROS request after 1 s without an HBC1 datagram
   (`HB_CMD_TIMEOUT_MS`) — WiFi drop, VM suspended, node dead — and keeps
   sending zeros so the mainboard stays armed.
3. **ESP32 sonar gate** (`sonar.h`) backs off for at most 1.5 s from an
   obstacle within 350 mm of the front sensor, and turns away for at most
   1.5 s from one within 50 mm of a side sensor (left -> turn right, right ->
   turn left, both -> back off). It is recomputed from the ROS request every
   cycle, so it works with no network and never lingers as a setpoint.
4. **Bridge** sends an explicit stop after `cmd_timeout_s` of `/cmd_vel`
   silence and keeps sending at `cmd_rate_hz` regardless.

---

## Protocol reference

The HBT1/HBC1 layout is documented in `hb_link.h` and in
`robot_ws/src/esp_tcp_server/src/hoverboard_udp_bridge.cpp`; keep the two in
step. The mainboard protocol is in `hoverboard.h`:

- Command, 8 B: `{0xABCD, steer, speed, checksum}` — **steer first**.
  `speed = (L + R) / (2 · SPEED_COEFF)`, `steer = (L − R) / (2 · STEER_COEFF)`,
  which with the defaults is hoverboard-driver's `steer = (L − speed) · 2`.
- Feedback, 18/22/26 B, `0xABCD`, every 10 ms.
- IMU, 18 B, `0xACDC` — gen2.x boards only; consumed and counted, not used.

Every frame is 16-bit little-endian words ending in the XOR of all the others.

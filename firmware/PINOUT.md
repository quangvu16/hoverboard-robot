# Pinout: ESP32-WROOM vs ESP32-S3

The same robot wired to either board. The firmware is identical except for the pin map
in each sketch's `config.h`:

| Board | Sketch |
|---|---|
| ESP32-WROOM DevKit | [`ldrobot_ld14p_esp32_mpu9250_ros`](ldrobot_ld14p_esp32_mpu9250_ros) |
| ESP32-S3-DevKitC-1 v1.1 | [`ldrobot_ld14p_esp32s3_mpu9250_ros`](ldrobot_ld14p_esp32s3_mpu9250_ros) |

Go by the **GPIO number printed next to each pin**. The header positions below are for
the 30-pin ESP32 DevKit V1 and Espressif's
[ESP32-S3-DevKitC-1 v1.1 user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html).
Other ESP32 DevKits (for example the 38-pin DevKitC) put the same GPIOs in different places.

## Every connection

| Device | Device pin | ESP32 GPIO | ESP32 header | S3 GPIO | S3 header | Rewire? |
|---|---|---|---|---|---|---|
| LD14P LiDAR | TX | 16 | R10 (RX2) | 16 | J1-9 | no |
| LD14P LiDAR | RX | not wired | | not wired | | |
| Mainboard (left sensor cable, USART2) | TX | 26 | L9 | 6 | J1-6 | **yes** |
| Mainboard (left sensor cable, USART2) | RX | 27 | L10 | 7 | J1-7 | **yes** |
| MPU9250 | SDA | 21 | R5 | 8 | J1-12 | **yes** |
| MPU9250 | SCL | 22 | R2 | 9 | J1-15 | **yes** |
| Front HC-SR04 | TRIG | 32 | L6 | 1 | J3-4 | **yes** |
| Front HC-SR04 | ECHO (via 1k/2k) | 33 | L7 | 2 | J3-5 | **yes** |
| Left HC-SR04 | TRIG | 17 | R9 (TX2) | 17 | J1-10 | no |
| Left HC-SR04 | ECHO (via 1k/2k) | 19 | R6 | 10 | J1-16 | **yes** |
| Right HC-SR04 | TRIG | 18 | R7 | 18 | J1-11 | no |
| Right HC-SR04 | ECHO (via 1k/2k) | 23 | R1 | 11 | J1-17 | **yes** |
| Left LED (front-left) | + (via resistor) | 14 | L11 | 14 | J1-20 | no |
| Right LED (front-right) | + (via resistor) | 13 | L13 | 13 | J1-19 | no |
| Back LED (stop light) | + (via resistor) | 5 | R8 | 5 | J1-5 | no |
| Buzzer | + / signal | 25 | L8 | 21 | J3-18 | **yes** |
| Everything | GND | GND | L14, R14 | G | J1-22, J3-1/21/22 | |
| MPU9250 | VCC (3.3 V) | 3V3 | R15 | 3V3 | J1-1/2 | |

"ECHO via 1k/2k": HC-SR04 ECHO → 1 kΩ → GPIO, and GPIO → 2 kΩ → GND. Both chips are
3.3 V only, so every echo line needs its own divider on either board.

Power the LiDAR and the HC-SR04s from the robot's 5 V rail, not from the DevKit. Join all
grounds. **Never connect the mainboard's VCC pin**: it can carry 15 V.

## ESP32 DevKit V1 (30-pin)

USB at the bottom, antenna at the top. `►` / `◄` mark a robot connection.

```
                            ┌────────antenna────────┐
                         L1 │ EN                 23 │ R1   ◄ right HC-SR04 ECHO
                         L2 │ VP/36              22 │ R2   ◄ MPU9250 SCL
                         L3 │ VN/39           TX0/1 │ R3
                         L4 │ 34              RX0/3 │ R4
                         L5 │ 35                 21 │ R5   ◄ MPU9250 SDA
  front HC-SR04 TRIG ►   L6 │ 32                 19 │ R6   ◄ left HC-SR04 ECHO
  front HC-SR04 ECHO ►   L7 │ 33                 18 │ R7   ◄ right HC-SR04 TRIG
              buzzer ►   L8 │ 25                  5 │ R8   ◄ back LED
        mainboard TX ►   L9 │ 26             TX2/17 │ R9   ◄ left HC-SR04 TRIG
        mainboard RX ►  L10 │ 27             RX2/16 │ R10  ◄ LiDAR TX
            left LED ►  L11 │ 14                  4 │ R11
                        L12 │ 12 (keep free)      2 │ R12
           right LED ►  L13 │ 13                 15 │ R13
                 GND ►  L14 │ GND               GND │ R14
                        L15 │ VIN               3V3 │ R15  ◄ MPU9250 VCC
                            └─────────[USB]─────────┘
```

Keep GPIO 12 free: held high at power-on, it switches the flash to 1.8 V and the board
boot-loops.

## ESP32-S3-DevKitC-1 v1.1

Antenna at the top, the two USB ports at the bottom. Use the **UART** port for flashing
and the Serial Monitor. Pins marked (PSRAM), (strap), (BOOT), (USB) and (RGB LED) must stay free.

```
                              ┌─────────antenna─────────┐
                         J1-1 │ 3V3                   G │ J3-1
         MPU9250 VCC ►   J1-2 │ 3V3               TX/43 │ J3-2
                         J1-3 │ RST               RX/44 │ J3-3
                         J1-4 │ 4                     1 │ J3-4   ◄ front HC-SR04 TRIG
            back LED ►   J1-5 │ 5                     2 │ J3-5   ◄ front HC-SR04 ECHO
        mainboard TX ►   J1-6 │ 6                    42 │ J3-6
        mainboard RX ►   J1-7 │ 7                    41 │ J3-7
                         J1-8 │ 15                   40 │ J3-8
            LiDAR TX ►   J1-9 │ 16                   39 │ J3-9
   left HC-SR04 TRIG ►  J1-10 │ 17         38 (RGB LED) │ J3-10
  right HC-SR04 TRIG ►  J1-11 │ 18           37 (PSRAM) │ J3-11
         MPU9250 SDA ►  J1-12 │ 8            36 (PSRAM) │ J3-12
                        J1-13 │ 3 (strap)    35 (PSRAM) │ J3-13
                        J1-14 │ 46 (strap)     0 (BOOT) │ J3-14
         MPU9250 SCL ►  J1-15 │ 9            45 (strap) │ J3-15
   left HC-SR04 ECHO ►  J1-16 │ 10                   48 │ J3-16
  right HC-SR04 ECHO ►  J1-17 │ 11                   47 │ J3-17
                        J1-18 │ 12                   21 │ J3-18  ◄ buzzer
           right LED ►  J1-19 │ 13          20 (USB D+) │ J3-19
            left LED ►  J1-20 │ 14          19 (USB D-) │ J3-20
                        J1-21 │ 5V                    G │ J3-21
                 GND ►  J1-22 │ G                     G │ J3-22
                              └────[UART USB]──[USB]────┘
```

Spare pins for later: 4, 12, 15, 39, 40, 41, 42, 47.

## Pins to avoid

| | ESP32-WROOM | ESP32-S3-DevKitC-1 v1.1 |
|---|---|---|
| Internal flash / PSRAM | 6-11 | 26-32 (flash), 33-37 (octal PSRAM on N8R8 / N32R16V) |
| Boot (strapping) pins | 0, 2, 12, 15 (12 is the dangerous one) | 0, 3, 45, 46 |
| Serial Monitor / flashing | 1, 3 | 43, 44 |
| Native USB | none | 19, 20 |
| On-board LED | 2 | 38 (RGB; it was 48 on v1.0) |
| Input only, no pull-down | 34-39 | none |
| Not on the chip | 20, 24, 28-31 | 22-25 |

The S3 has no GPIO 22-25 at all, and 26-37 are taken by its memory. That is why the
mainboard, buzzer, IMU, front sonar and two ECHO lines had to move. The S3 sketch's
`config.h` refuses any pin from this table at compile time.

## Moving the robot from the ESP32 to the S3

1. Flash the S3 sketch first, with the board unplugged from the robot
   (see the [README](../README.md#esp32-s3-devkitc-1-v11)).
2. Move the wires marked **yes** in the first table. The LiDAR, LEDs and the two side TRIG
   wires stay on the same GPIO numbers but are at different places on the header.
3. Keep the three ECHO dividers.
4. With the wheels off the ground, power up and check the Serial Monitor. It prints every
   pin at boot (`LiDAR ... RX=GPIO16`, `hover UART2 ... RX=GPIO6 TX=GPIO7`, `imu ... SDA=GPIO8`).
5. On the PC, `ros2 topic echo /hoverboard/link_status` and `ros2 topic hz /scan`.
   Nothing on the ROS side changes.

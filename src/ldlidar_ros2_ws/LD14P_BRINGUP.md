# LD14P over WiFi: bring-up guide

End-to-end path for putting an **LDROBOT LD14P** on `/scan`, with the LiDAR
wired to an **ESP32** and the vendor driver (`ldlidar_ros2`) running on this
machine.

```
LD14P ──UART2──▶ ESP32 (ld14p_tcp_bridge) ──TCP 8889──▶ ldlidar_ros2_node ──▶ /scan ──▶ rviz2
        230400 baud        static IP .50            192.168.1.12      frame base_scan
```

The ESP32 does **no decoding**: it copies UART bytes onto a TCP socket. All
protocol work (framing, CRC8, angle interpolation, ranging-centre correction)
happens in the LDROBOT SDK on the ROS 2 side.

---

## 0. Which board does what — read this first

The bridge sketch turns the ESP32 into a **LiDAR-only** device. It does not
drive motors and does not speak the `esp_tcp_server` telemetry protocol.

| you have | do this |
| --- | --- |
| A spare ESP32 just for the LiDAR | Flash `ld14p_tcp_bridge`, follow this guide. |
| One ESP32 that also drives the motors | Do **not** flash it with this sketch — you would lose `/odom` and `/cmd_vel`. Use the `LDS1` path instead (`~/Arduino/ld14p_udp_scan`, received by `udp_lidar_receiver_node` in `robot_bringup`), or merge the bridge into the kaiaai firmware. |

Wiring, either way:

| LD14P | ESP32 | note |
| --- | --- | --- |
| TX | GPIO16 (RX2) | 3.3 V logic, no level shifter |
| RX | GPIO17 (TX2) | optional, the bridge never sends |
| GND | GND | must be common |
| VCC | 5V | ~200 mA, the unit spins up on its own |

On an ESP32-WROVER (or any module with PSRAM) GPIO16/17 are taken by the
PSRAM — move `LIDAR_RX_PIN`/`LIDAR_TX_PIN` to free pins such as 25/26.

---

## 1. Check the network and pick an address

The ROS node dials the ESP32, so the board needs an address that does not move.

```bash
# this PC
ip -4 -o addr show scope global | awk '{print $2, $4}'    # ens33 192.168.1.12/24
ip route | awk '/default/{print "gateway", $3}'           # gateway 192.168.1.1

# is the address you want free?
ping -c1 -W1 192.168.1.12 ; ip neigh | grep 192.168.1.12
#   "100% packet loss" + "FAILED"  -> free
#   a reply                        -> pick another
```

Pick an address **outside the router's DHCP pool** (check the router's admin
page). On this LAN `.1`–`.10` are in use and `.12` is this PC; `.50` is free.

Write down: **PC = 192.168.1.12**, **ESP32 = 192.168.1.50**, **port 8889**.

---

## 2. Configure the sketch

Edit `~/Arduino/ld14p_tcp_bridge/ld14p_tcp_bridge.ino`:

| setting | value | note |
| --- | --- | --- |
| `WIFI_SSID` / `WIFI_PASS` | your AP | 2.4 GHz only — ESP32 has no 5 GHz radio |
| `STATIC_IP` | `192.168.1.50` | from step 1; must equal `bridge_ip` later |
| `GATEWAY` / `SUBNET` | `192.168.1.1` / `255.255.255.0` | from step 1 |
| `BRIDGE_PORT` | `8889` | must equal `bridge_port` later |
| `LIDAR_BAUD` | **230400** for LD14P, **115200** for LD14 | must match the unit |

`USE_STATIC_IP 0` falls back to DHCP; then read the address off the serial
console after every boot and pass that as `bridge_ip`.

---

## 3. Connect the board and compile

This machine is a VMware VM, so the USB adapter must be handed to the guest:
**VMware ▸ Removable Devices ▸ (CP210x / CH340) ▸ Connect**.

```bash
ls /dev/ttyUSB*                       # the adapter should appear
~/bin/arduino-cli board list          # shows the port and detected board

~/bin/arduino-cli compile --fqbn esp32:esp32:esp32 ~/Arduino/ld14p_tcp_bridge
~/bin/arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 ~/Arduino/ld14p_tcp_bridge
```

Installed here: arduino-cli 1.5.2, core `esp32:esp32` 2.0.17, FQBN
`esp32:esp32:esp32` (plain ESP32 dev board).

If upload says *permission denied* on `/dev/ttyUSB0`: the `vu` account is in
`dialout`, but a session started before that is not — log out and back in.

Some boards need the **BOOT** button held while "Connecting…" is printed.

---

## 4. Watch the board come up

```bash
~/bin/arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Expected:

```
LD14/LD14P UART <-> TCP bridge for ldlidar_ros2
UART2 @ 230400 baud, RX=GPIO16 TX=GPIO17
WiFi up as 192.168.1.50, listening on TCP 8889
  ros2 launch ldlidar_ros2 ld14p_network.launch.py bridge_ip:=192.168.1.50 bridge_port:=8889
waiting  0 B/s to host  0 B total  0 B to lidar  0 drops  rssi -55
```

The board prints the exact launch line to use. The LED blinks while waiting
for a client and goes solid once the ROS node attaches.

Then confirm from the PC:

```bash
ping -c3 192.168.1.50
timeout 2 bash -c 'echo > /dev/tcp/192.168.1.50/8889' && echo "port 8889 open"
```

---

## 5. Run the ROS 2 node

`~/.bashrc` already sources `~/ldlidar_ros2_ws/install/local_setup.bash`, so a
fresh terminal is enough.

```bash
ros2 launch ldlidar_ros2 ld14p_network.launch.py \
     bridge_ip:=192.168.1.50 bridge_port:=8889
```

Success looks like:

```
ldlidar tcp_client: linking to 192.168.1.50:8889 ...
ldlidar network connect is success
ldlidar communication is normal.
ldlidar driver start is success.
start normal, pub lidar data
```

and the ESP32 console flips to `attached  ~23000 B/s to host`.

Launch arguments:

| arg | default | meaning |
| --- | --- | --- |
| `bridge_ip` | `192.168.1.50` | the ESP32 (client mode) — never this PC |
| `bridge_port` | `8889` | TCP port of the bridge |
| `product_name` | `LDLiDAR_LD14P` | parser profile; `LDLiDAR_LD14` pairs with 115200 |
| `frame_id` | `base_scan` | matches `robot_esp.urdf` |
| `topic_name` | `scan` | remap if something else owns `/scan` |
| `comm_mode` | `tcp_client` | also `serial`, `tcp_server`, `udp_client`, `udp_server` |
| `publish_static_tf` | `false` | `true` only when `robot_state_publisher` is **not** running |
| `auto_reconnect` | `true` | retry forever, including the first link |

With the rest of the robot, let `robot_bringup` start it instead — it brings up
the ESP32 control link and TF at the same time, and leaves `udp_lidar_receiver_node`
out so nothing else claims `/scan`:

```bash
ros2 launch robot_bringup robot_bringup.launch.py \
     lidar:=ldlidar bridge_ip:=192.168.1.50 bridge_port:=8889
```

Serial-attached LiDAR instead? The stock launch files are unchanged:
`ros2 launch ldlidar_ros2 ld14p.launch.py` (uses `/dev/ttyUSB0`).

---

## 6. Check the data and view it

```bash
ros2 topic hz /scan                    # ~10 Hz for a healthy LD14P
ros2 topic echo /scan --once | head -20
rviz2 -d ~/robot_ws/src/robot_description/rviz/scan.rviz
```

`scan.rviz` uses Fixed Frame `base_footprint` (always available from the URDF)
and colours the scan flat green. Switch the Fixed Frame to `odom` once the
robot link is up and you want a map-relative view.

Standalone (no `robot_bringup`), there is no TF for `base_scan`, so add it:

```bash
ros2 launch ldlidar_ros2 ld14p_network.launch.py \
     bridge_ip:=192.168.1.50 publish_static_tf:=true
```

---

## 7. Orientation

Do not reorder ranges to fix a mounting offset — set it in one place:

* **Sweep direction:** `laser_scan_dir` (`True` = counter-clockwise).
* **Blind sector** (e.g. a mast in view): `enable_angle_crop_func:=True` plus
  `angle_crop_min` / `angle_crop_max`, in degrees.
* **Rotation** (the LiDAR's zero mark does not face forward): rotate the
  `base_link → base_scan` joint in `robot_esp.urdf`, or the yaw argument of the
  static transform. `ldlidar_ros2` has no angle-offset parameter.

---

## 8. Only one publisher on `/scan`

| publisher | fed by | active when |
| --- | --- | --- |
| `udp_lidar_receiver_node` (`robot_bringup`) | `LDS1` datagrams, UDP 9900 | the ESP32 runs the kaiaai/`LDS1` firmware |
| `ldlidar_publisher_ld14p` | raw LD14P bytes, TCP 8889 | the ESP32 runs `ld14p_tcp_bridge` |
| micro-ROS sketch | micro-ROS agent, UDP 8888 | only with `micro_ros_agent` running |

`robot_bringup` picks one with its `lidar` argument — `udp` (the default),
`ldlidar`, or `none` — so the two never start together. A board can only run one
firmware either, but leftover nodes from an earlier launch do publish. Check
before debugging a doubled map:

```bash
ros2 topic info /scan --verbose | grep "Node name"
```

---

## Troubleshooting

| symptom | cause | fix |
| --- | --- | --- |
| `TCP,connect server error. No route to host` | nothing at `bridge_ip` | board off, not on WiFi, or wrong IP. `ping` it; check the serial console |
| `TCP,connect server error. Connection refused` | host is up, nothing listening on that port | wrong `bridge_port`, or the sketch is not running. UDP 9900 is **not** a TCP bridge |
| `connect is success` then `communication is abnormal` forever | `bridge_ip` is **this PC** (192.168.1.12) and `bridge_port` 9000, so the node linked to `esp_tcp_server_node` instead of the bridge | point `bridge_ip` at the ESP32 (`192.168.1.50:8889`). Port 9000 is the motor link — dialling it also locks the real board out of its own port |
| `ldlidar communication is abnormal` | link is up, bytes do not frame | `LIDAR_BAUD` vs `product_name` mismatch, or the LiDAR is not spinning |
| ESP32 says `attached` but `0 B/s to host` | nothing arriving on GPIO16 | LiDAR unpowered, TX/RX swapped, no common ground, or PSRAM pins |
| `ros2 topic hz /scan` prints nothing | no data | check the ESP32 console first — `B/s to host` is the ground truth |
| rviz2 empty but `hz` shows data | Fixed Frame has no TF | use `base_footprint`, or add `publish_static_tf:=true` |
| `/dev/ttyUSB0` missing | USB not passed to the VM | VMware ▸ Removable Devices ▸ Connect |
| upload: permission denied | session predates `dialout` membership | log out and back in |
| node exits instead of retrying | `auto_reconnect:=false` | drop the argument |

Logs worth keeping when something is odd: the ESP32 serial console and the
node's own `[LOG]` lines, which name the SDK file and line that failed.

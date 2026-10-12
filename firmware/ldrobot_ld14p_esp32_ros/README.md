# LD14P -> ESP32 -> ROS 2 `/scan`

Runbook for bringing this LiDAR up from cold. Three stages: confirm the
addresses, flash the ESP32, start the ROS 2 receiver.

| | |
|---|---|
| Board | ESP32 Dev Module on **COM8** |
| LiDAR | LDROBOT LD14/LD14P, UART1 @ 115200, RX=GPIO16 (LiDAR RX unwired) |
| WiFi | SSID and password in `secrets.h` (copy `secrets.h.example`) |
| Sends to | **192.168.1.12:9900** — `DEST_IP` / `DEST_PORT` in `config.h` |
| Wire format | UDP `"LDS1"` datagram, 360 bins, **734 bytes** per revolution |
| ROS 2 output | `/scan`, frame `base_scan`, from `udp_lidar_receiver_node` |

There is no micro-ROS agent in this path any more. The ESP32 sends plain UDP
and the receiver node does the ROS side.

The same board also drives the hoverboard motors — see
**[HOVERBOARD.md](HOVERBOARD.md)** for wiring, the `/cmd_vel` and odometry
path, and the first-drive procedure.

---

## 1. Check the IP addresses

Two addresses matter, and only one of them lives in the firmware.

### The ROS 2 machine — this is the one the firmware needs

On the Ubuntu box:

```bash
hostname -I
```

Whatever comes back must match `DEST_IP` in `config.h` (currently
`192.168.1.12`). This is the most common thing to break: the firmware has the
address hard-coded, so when DHCP hands the Ubuntu box a different lease the
ESP32 keeps transmitting into the void with no error on either side. UDP is
fire-and-forget — nobody reports the loss.

If it has moved, either update `DEST_IP` and reflash, or better, give the
machine a static DHCP reservation on the router so it stops moving.

### The ESP32 — useful for diagnosis, not needed by anything

The sketch prints it on boot:

```
WiFi up as 192.168.1.6, sending 360-bin scans (734 B) to 192.168.1.12:9900
```

From the Ubuntu box, confirm the two can actually see each other:

```bash
ping -c 3 192.168.1.6
```

Both must be on the same subnet. If the router has AP isolation or a guest
network enabled, they will associate fine and still never exchange a packet.

### Let the datagrams through

```bash
sudo ufw allow 9900/udp
```

### Confirm packets are really arriving

This is the highest-value check in the guide — it proves the network half
independently of ROS:

```bash
sudo tcpdump -i any -n udp port 9900
```

Expect one line per revolution, `length 734`, at roughly 5–12 per second.

- **Lines appear** — the network is fine, anything left is ROS-side.
- **Nothing** — the ESP32 is not sending, is sending to the wrong address, or
  a firewall is eating it. Go back to the serial console.

---

## 2. Compile and flash the firmware

**No third-party libraries are needed.** `WiFi.h` and `WiFiUdp.h` both ship
with the ESP32 board package. The `micro_ros_arduino` library that an earlier
version of this sketch depended on is no longer used.

1. Open `ldrobot_ld14p_esp32_ros.ino` in Arduino IDE 2.x.
2. Copy `secrets.h.example` to `secrets.h` and set `WIFI_SSID` / `WIFI_PASS`.
   Then check `config.h` — `DEST_IP`, `DEST_PORT` and the LiDAR model lines.
3. **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
4. **Tools → Port → COM8**
5. Upload (`Ctrl+U`). Hold the BOOT button if it will not sync.
6. **Tools → Serial Monitor**, set to **115200 baud**.

### What a good boot looks like

```
LD14 + hoverboard (FOC, VARIANT_USART) on ESP32 -> UDP -> ROS 2
LiDAR  UART1 @ 115200 baud, RX=GPIO16 TX=-1 (-1 = unwired)
...
WiFi up as 192.168.1.6
LiDAR: sending 360-bin scans (734 B) to 192.168.1.12:9900
scan 11.96 Hz  197 pts (132 valid)  7326 pkt  2 crc err  12 scan/s sent  0 err  rssi -58
```

Reading that status line:

| Field | Means | Healthy |
|---|---|---|
| `11.96 Hz` | Motor rotation speed | **8–12 Hz**; see troubleshooting if lower |
| `197 pts` | Points per revolution | `2300 / Hz` — rises as the motor slows |
| `(132 valid)` | Points inside 0.02–8.0 m | Rest are no-return; varies with the room |
| `7326 pkt` | LiDAR packets decoded | Climbing steadily |
| `2 crc err` | Corrupt packets | A couple at startup, then **frozen** |
| `12 scan/s sent` | Datagrams sent per second | Should equal the Hz figure |
| `0 err` | Local send failures | Stays 0 |
| `rssi -58` | WiFi signal | Better than -70 |

WiFi state and the IP address are in the Serial Monitor (`WiFi up as ...`).
Nothing blocks on the network — if WiFi never comes up the LiDAR
still decodes and the console still reports, scans are just dropped.

---

## 3. Run the ROS 2 side

The receiver is part of the normal bringup. Adjust the workspace path to
whichever one holds the `esp_tcp_server` package:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py
```

Or run just the receiver, which is the better choice while debugging:

```bash
ros2 run esp_tcp_server udp_lidar_receiver_node
```

It binds `0.0.0.0:9900`, so it does not care what the ESP32 address is — only
the port has to match.

### Verify

```bash
ros2 topic hz /scan
```

Should report the same rate as the `Hz` field on the serial console.

```bash
ros2 topic echo /scan --once
```

Check `header.frame_id` is `base_scan` and that `ranges` is not all `inf`.

### View it

```bash
ros2 run rviz2 rviz2
```

Set **Fixed Frame** to `base_scan`, then **Add → LaserScan** and set its topic
to `/scan`. If the topic is listed but nothing draws, change the display's
**Reliability** setting — a QoS mismatch looks exactly like this.

### If the scan is mirrored or rotated

Fix it with the receiver node parameters, **never** by reordering bins in the
firmware. The ranges array is sent in the sensor's own clockwise order and the
node does the ROS-side correction with `flip_lr` (default true) and
`angle0_deg` (default 180). Change both sides and the two corrections fight.

```bash
ros2 param list                     # find the receiver node name
ros2 param get <node> angle0_deg
ros2 param set <node> angle0_deg 180
```

---

## 4. Troubleshooting

| Symptom | Cause |
|---|---|
| LED blinking, `WiFi "<your SSID>" still down, retrying` | Wrong SSID/password, or out of range. Retries every 10 s by itself. |
| `nothing at all on GPIO16 - not a single byte received` | Power or wiring. LiDAR TX must reach GPIO16, grounds must be common, motor must spin. The sketch prints a full checklist. |
| Bytes arrive but no valid packet | Wrong baud. The sketch walks 115200 / 230400 / 460800 / 921600 on its own and reports what it tried. |
| `UDP send ... failed` | Local fault — no route, or the WiFi stack is out of buffers. Not a receiver problem. |
| Serial healthy but `/scan` silent | Receiver not running, firewall, or stale `DEST_IP`. Run the `tcpdump` check to tell network from ROS. |
| **Scan rate below ~8 Hz** | Motor problem, not software. Sampling is fixed at ~2350 pts/s, so `pts` rising while `Hz` falls means the motor slowed. Usual causes: hair or dust round the rotor, or a sagging 5 V rail. This unit has been seen at both 11.96 Hz and 4.83 Hz. |
| Scan drawn mirrored or rotated | `flip_lr` / `angle0_deg` on the receiver node. |

### A note on `SCAN_BINS`

Points per revolution is `2300 / rotation_speed`, not a fixed property of the
model — so the right bin count depends on how fast the motor actually turns:

| Motor | pts/rev | 360 bins | 180 bins |
|---|---|---|---|
| 11.96 Hz | 197 | 45% of bins empty | 1.09 pts/bin — good |
| 4.83 Hz | 487 | 1.35 pts/bin — good | 63% of points discarded |

360 is the default because a 1-degree grid is what ROS tooling expects. Once
the motor holds a stable rate above 8 Hz, 180 becomes the better fit. The
ceiling is 720 bins — `14 + 2*bins` has to fit one datagram.

---

## 5. Daily startup

1. Power the LiDAR, wait for the LED to go solid.
2. `ros2 launch robot_bringup robot_bringup.launch.py`
3. `ros2 topic hz /scan` — confirm it matches the serial `Hz` field.

If step 3 is silent, run `sudo tcpdump -i any -n udp port 9900`. That one
command tells you which half of the system to look at.

# Robot Nav: Android app

A native Android app (Kotlin, no AndroidX) for driving and navigating the hoverboard robot. It speaks
the rosbridge websocket protocol directly, using the same topics as the
`robot_web_nav` web page, so the robot side is the same:

```bash
# on the robot PC (bringup already running)
ros2 launch robot_web_nav web_nav.launch.py stack:=cartographer map:=$HOME/m_map.pbstream
#   or stack:=amcl map:=$HOME/m_map.yaml
#   or stack:=none to only drive with the stick (no map, no Nav2)
```

In the app, tap **Robot address** and enter `ws://<robot-pc-ip>:9090`. The address is
remembered, and the app reconnects by itself every 2 s.

| Screen part | What it does |
|---|---|
| **Drive** stick | Drag the blue knob to drive, and let go to stop. Up is forward, right turns right. Publishes `/cmd_vel_manual` 10 times a second while held, then a few zeros. Grabbing it during a goal cancels the goal. |
| Max speed / Max turn | Caps for full stick: 0.05-0.5 m/s (default 0.2) and 0.2-2.0 rad/s (default 0.8). Remembered. |
| **Stop** | Centres the stick and sends zeros. Leaving the app (home button, screen off) does the same. |
| `sonar` pill | `/ultrasonic/obstacle_near`: `obstacle ahead` when the ESP32's front sonar reflex is blocking forward driving. |
| Sonar line | Front / left / right distances in metres from `/range/{front,left,right}` (`clear` = no echo within 2 m). |
| **Hold still** / **Release** | The `twist_mux` lock, `/cmd_vel_lock`. Hold still blocks every input, Nav2 and the sticks included, and greys the stick. The app re-sends it every 2 s while holding. Release is sent three times. The label shows what the topic last carried, from any client. Needs `ros-humble-twist-mux` on the robot PC: without it, the lock does nothing. |
| `battery` pill | `/hoverboard/battery_state` and `/hoverboard/battery_volts`: the voltage, orange at `battery low` (the ESP32 halves the speed), red at `battery critical` (the ESP32 holds the wheels at zero until the pack is charged). |
| Map | `/map` with the robot (`/web/robot_pose`), the planned path (`/plan`) and the goal. Pinch to zoom; two fingers pan. **Fit** and **Robot** re-centre. |
| **Goal** mode | Tap where to go and drag to set the heading. With a plain tap, the robot faces the way it drove. Publishes `/web/goal`. |
| **Set pose** mode | Tap where the robot really is and drag the way it faces. Publishes `/initialpose`. |
| **Pan** mode | One finger moves the map. |
| Status | Driving / Arrived / Failed / Cancelled, with distance left, ETA, driving time and recoveries (`/web/nav_status`). |
| Cancel goal | `/web/cancel` |
| Saved spots | Named positions stored on the phone. **Go** sends the robot there. |
| Pills | websocket, stack, Nav2 ready, map size |

The screen stays on while the app is open.

## Build

Builds on this PC without Android Studio. The toolchain lives in `~/Android`:
JDK 17 is extracted from Ubuntu's openjdk-17 packages, alongside the SDK and Gradle 8.7.

```bash
cd ~/robot_ws/android_app
export JAVA_HOME=~/Android/jdk ANDROID_HOME=~/Android/Sdk
./gradlew --no-daemon assembleRelease
# -> app/build/outputs/apk/release/app-release.apk  (signed with the debug key)
```

Network quirks on this machine:
- Java needs `-Djava.net.preferIPv6Addresses=true` (already in `gradle.properties`),
  because IPv4 has no route.
- GitHub is unreachable, so the wrapper downloads Gradle from the Tencent mirror and
  checks it against the official SHA-256 from gradle.org.

## Install on the phone

- **USB:** enable Developer options → USB debugging, then
  `~/Android/Sdk/platform-tools/adb install -r app/build/outputs/apk/release/app-release.apk`
- **Without a cable:** copy the APK to the phone, open it, and allow "install unknown apps".

Android 8.0 (API 26) or newer. The phone must be on the same Wi-Fi as the robot PC.
If ufw is enabled on the PC, port 9090/tcp must be open.

## Code

- `RosBridge.kt`: the rosbridge v2 client (OkHttp websocket). It re-subscribes after
  reconnecting and advertises topics before their first publish.
- `JoystickView.kt`: the drive stick (dead zone, unit-circle clamp, one finger).
- `Drive.kt`: stick to `Twist`, the 10 Hz send / zero-burst rule and the speed caps.
- `MapView.kt`: draws the grid, robot, plan and goal, and handles touch for picking,
  pan and pinch.
- `MainActivity.kt`: topics, status panel, robot address and saved spots.

#!/usr/bin/env python3
"""Live view of the HC-SR04 as the firmware sees it.

Reads the "HBT1" telemetry the ESP32 broadcasts at 50 Hz and prints the sonar
range and the three reflex flags. Needs no ROS and no mainboard - the sonar
runs as soon as the board is powered, so this works while you are still wiring.

    python3 ~/Arduino/ldrobot_ld14p_esp32_ros/tools/sonar_monitor.py

If robot_bringup is running it already owns UDP 9902, and only one socket gets
each datagram - so stop the bringup first, or the readings will look like they
are dropping out.
"""
import socket
import struct
import sys
import time

PORT = 9902
HBT_MIN = 44          # 42 bytes of hoverboard frame + the appended sonar range


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", PORT))
    s.settimeout(3.0)

    print("listening for HBT1 on UDP %d - ctrl-c to stop\n" % PORT)
    print("%-9s %-8s %-10s %s" % ("range", "closing", "state", "note"))

    last = None
    while True:
        try:
            d, addr = s.recvfrom(128)
        except socket.timeout:
            print("no telemetry - is the board powered and on the network?")
            continue
        except KeyboardInterrupt:
            return 0

        if d[:4] != b"HBT1":
            continue
        if len(d) < HBT_MIN:
            print("board is running firmware without the sonar (%d-byte frame)" % len(d))
            time.sleep(1.0)
            continue

        flags = struct.unpack_from("<H", d, 10)[0]
        mm = struct.unpack_from("<H", d, 42)[0]
        near, backing, stale = (flags >> 3) & 1, (flags >> 4) & 1, (flags >> 5) & 1

        if stale:
            state, note = "NO SENSOR", "forward blocked; check ECHO wiring"
        elif backing:
            state, note = "BACKING", "reversing away from the obstacle"
        elif near:
            state, note = "OBSTACLE", "forward refused"
        else:
            state, note = "clear", ""

        rng = "no return" if mm == 0 else "%d mm" % mm
        closing = ""
        now = time.time()
        if last is not None and mm and last[0]:
            dt = now - last[1]
            if dt > 0:
                closing = "%+.2f m/s" % (((mm - last[0]) / 1000.0) / dt)
        last = (mm, now)

        print("%-9s %-8s %-10s %s" % (rng, closing, state, note))
        time.sleep(0.2)


if __name__ == "__main__":
    sys.exit(main())

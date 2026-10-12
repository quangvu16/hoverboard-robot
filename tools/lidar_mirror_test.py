#!/usr/bin/env python3
"""LiDAR mount check: is /scan mirrored or rotated relative to the robot?

Turns the robot ~60 deg in place (odometry-timed) and cross-correlates the scans
taken before and after. For a correctly configured scan the room shifts by
+turn in the laser frame; a mirrored scan shifts by -turn; 0 means the LiDAR
did not turn with the wheels (not mounted on the chassis).

Needs: robot_bringup running (base:=hoverboard), ~0.4 m clear around the robot.
Fix a mirrored scan with udp_lidar_receiver_node's flip_lr, a rotated one with
angle0_deg - never by reordering bins in the firmware.

    python3 ~/robot_ws/tools/lidar_mirror_test.py
"""
import math, time, rclpy, numpy as np
import sys
# Only compare returns this far out. Anything the robot carries (posts, the
# sonar, cables, a shroud) turns WITH the LiDAR and matches perfectly at 0 deg,
# which fooled the first version of this test into reporting "no rotation"
# whenever the robot's own structure filled most of the view.
MIN_R = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
rclpy.init(); n=Node('mirror_test'); st={'o':None,'scans':[]}
n.create_subscription(Odometry,'/odom',lambda m: st.__setitem__('o',m),qos_profile_sensor_data)
n.create_subscription(LaserScan,'/scan',lambda m: st['scans'].append(m),qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',10)
def spin(t):
    e=time.time()+t
    while time.time()<e: rclpy.spin_once(n,timeout_sec=0.01)
def grab(k=4):
    st['scans'].clear(); spin(0.3)
    while len(st['scans'])<k: spin(0.05)
    m=st['scans'][-1]
    # resample to a 1-degree grid of LASER-FRAME angle (handles negative increment)
    grid=np.full(360,np.nan)
    for s in st['scans'][-k:]:
        for i,r in enumerate(s.ranges):
            if math.isfinite(r) and MIN_R<r<8.0:
                a=(s.angle_min+i*s.angle_increment)
                d=int(round(math.degrees(a)))%360
                grid[d]=r if np.isnan(grid[d]) else min(grid[d],r)
    return grid, m
def best_shift(b,a):
    best=None
    for s in range(-180,180):
        x=np.roll(b,-s)   # x[d] = b[d+s]
        ok=~np.isnan(x)&~np.isnan(a)
        if ok.sum()<40: continue
        # fraction of bins that disagree by more than 10 cm: robust to a few
        # changed returns, and not dominated by a static majority
        err=np.mean(np.abs(x[ok]-a[ok]) > 0.10)
        if best is None or err<best[1]: best=(s,err,ok.sum())
    return best
spin(1.5)
before,m=grab(); o0=yaw(st['o'].pose.pose.orientation)
print(f"scan: angle_min {m.angle_min:.3f} angle_increment {m.angle_increment:.5f} ({'NEGATIVE' if m.angle_increment<0 else 'positive'})")
t0=time.time(); osum=0; prev=o0
while osum<math.radians(60) and time.time()-t0<8:
    tw=Twist(); tw.angular.z=0.5; pub.publish(tw); spin(0.05)
    o=yaw(st['o'].pose.pose.orientation); osum+=wrap(o-prev); prev=o
for _ in range(20): pub.publish(Twist()); spin(0.05)
spin(1.0); after,_=grab()
o=yaw(st['o'].pose.pose.orientation); osum+=wrap(o-prev)
s,err,cnt=best_shift(before,after)
print(f"odometry turned {math.degrees(osum):+.0f} deg (CCW positive)")
print(f"scan content shifted by {s:+d} deg  ({100*err:.0f}% of {cnt} bins beyond {MIN_R} m disagree at that shift)")
print("expected for a CORRECT scan: shift ~= +turn ;  MIRRORED scan: shift ~= -turn")

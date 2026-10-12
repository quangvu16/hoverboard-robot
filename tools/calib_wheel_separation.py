#!/usr/bin/env python3
"""Wheel-separation calibration: odometry yaw vs Cartographer (LiDAR) yaw.

Spins the robot one full turn in place by odometry, then compares with the
rotation Cartographer measured by scan matching. The effective track is
    wheel_separation_m * odom_angle / cartographer_angle
Set that as wheel_separation_m in robot_bringup.launch.py (and the wheel joint
y offsets in robot_description/urdf/robot_esp.urdf).

Needs: robot_bringup (base:=hoverboard) AND
       ros2 launch robot_cartographer cartographer_nav.launch.py nav:=false
       running, LiDAR MOUNTED on the chassis (check lidar_mirror_test.py first),
       ~0.4 m clear around the robot.
SEP_CFG below must equal the wheel_separation_m the bridge is running with.

    python3 ~/robot_ws/tools/calib_wheel_separation.py
"""
import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
WZ, TARGET, TIMEOUT, SEP_CFG = 0.6, 2*math.pi, 30.0, 0.18
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
rclpy.init(); n=Node('calib_spin'); st={'o':None}
n.create_subscription(Odometry,'/odom',lambda m: st.__setitem__('o',m),qos_profile_sensor_data)
buf=Buffer(); TransformListener(buf,n)
pub=n.create_publisher(Twist,'/cmd_vel',10)
def spin(t):
    e=time.time()+t
    while time.time()<e: rclpy.spin_once(n,timeout_sec=0.01)
def carto_yaw(): return yaw(buf.lookup_transform('map','base_footprint',rclpy.time.Time()).transform.rotation)
def cmd(w):
    m=Twist(); m.angular.z=float(w); pub.publish(m)
spin(2.0)
o_prev=yaw(st['o'].pose.pose.orientation); c_prev=carto_yaw(); o_sum=c_sum=0.0; t0=time.time(); last=0
try:
    while o_sum < TARGET and time.time()-t0 < TIMEOUT:
        cmd(WZ); spin(0.05)
        o=yaw(st['o'].pose.pose.orientation); c=carto_yaw()
        o_sum+=wrap(o-o_prev); c_sum+=wrap(c-c_prev); o_prev, c_prev = o, c
        if time.time()-last>2: last=time.time(); print(f"  {time.time()-t0:4.1f}s odom {math.degrees(o_sum):6.1f} deg  cartographer {math.degrees(c_sum):6.1f} deg",flush=True)
finally:
    for _ in range(20): cmd(0.0); spin(0.05)
spin(2.0)   # let cartographer catch up with the last scans
o=yaw(st['o'].pose.pose.orientation); c=carto_yaw(); o_sum+=wrap(o-o_prev); c_sum+=wrap(c-c_prev)
print(f"FINAL odom {math.degrees(o_sum):.1f} deg   cartographer {math.degrees(c_sum):.1f} deg   ratio odom/carto {o_sum/c_sum:.3f}")
print(f"=> effective wheel_separation = {SEP_CFG*o_sum/c_sum:.3f} m  (configured {SEP_CFG})")

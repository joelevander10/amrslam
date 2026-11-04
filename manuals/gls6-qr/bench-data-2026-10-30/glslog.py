#!/usr/bin/env python3
"""Read-only: log every /gls/reading to CSV and print a live line at ~4 Hz.
Usage: glslog.py out.csv   (Ctrl-C to stop)"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
from amr_interfaces.msg import GlsReading
from nav_msgs.msg import Odometry

out = open(sys.argv[1], "a", buffering=1)
if out.tell() == 0:
    out.write("t_wall,stamp,valid,label_id,x_counts,y_counts,angle_deg,spread_counts,n_codes,n_outliers,fps,odom_x,odom_y,odom_yaw_deg\n")
last = [0.0]
od = [float("nan")] * 3


def ocb(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    od[:] = [p.x, p.y, math.degrees(2 * math.atan2(q.z, q.w))]


def cb(m):
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    out.write(f"{time.time():.3f},{st:.3f},{int(m.valid)},{m.label_id},{m.x_counts:.1f},{m.y_counts:.1f},"
              f"{m.angle_deg:.3f},{m.spread_counts:.2f},{m.n_codes},{m.n_outliers},{m.fps:.1f},{od[0]:.4f},{od[1]:.4f},{od[2]:.3f}\n")
    if time.time() - last[0] >= 0.25:
        last[0] = time.time()
        if m.valid:
            print(f"{time.strftime('%H:%M:%S')}  VALID {m.label_id}  x={m.x_counts:7.1f}  y={m.y_counts:7.1f}  "
                  f"ang={m.angle_deg:7.2f}  n={m.n_codes}  spread={m.spread_counts:4.1f}", flush=True)
        else:
            print(f"{time.strftime('%H:%M:%S')}  -- no label --", flush=True)


rclpy.init()
n = Node("glslog_ro")
n.create_subscription(Odometry, "/odom_raw", ocb, qos_profile_sensor_data)
n.create_subscription(GlsReading, "/gls/reading", cb, 100)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass

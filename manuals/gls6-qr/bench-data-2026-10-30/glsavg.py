#!/usr/bin/env python3
"""Read-only: average /gls/reading over N seconds. Usage: glsavg.py [secs] [tag]"""
import math, sys, time, statistics as st
import rclpy
from rclpy.node import Node
from amr_interfaces.msg import GlsReading

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
tag = sys.argv[2] if len(sys.argv) > 2 else ""
rows, total = [], [0]


def cb(m):
    total[0] += 1
    if m.valid:
        rows.append((m.x_counts, m.y_counts, m.angle_deg, m.spread_counts, m.n_codes, m.label_id))


rclpy.init()
n = Node("glsavg_ro")
n.create_subscription(GlsReading, "/gls/reading", cb, 50)
t0 = time.time()
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()

if not rows:
    print(f"{tag}: NO VALID READINGS ({total[0]} msgs)"); sys.exit(1)
xs, ys, a, sp, nc, ids = zip(*rows)
s = sum(math.sin(math.radians(v)) for v in a); c = sum(math.cos(math.radians(v)) for v in a)
am = math.degrees(math.atan2(s, c)) % 360
ad = [((v - am + 180) % 360) - 180 for v in a]
sd = lambda v: st.pstdev(v) if len(v) > 1 else 0.0
print(f"{tag}: n={len(rows)}/{total[0]} valid  label={sorted(set(ids))}\n"
      f"  x={st.mean(xs):8.2f} (σ{sd(xs):.2f})  y={st.mean(ys):8.2f} (σ{sd(ys):.2f})  "
      f"ang={am:8.3f}° (σ{sd(ad):.3f})\n"
      f"  spread mean {st.mean(sp):.2f} [{min(sp):.1f}..{max(sp):.1f}]  "
      f"n_codes mean {st.mean(nc):.2f} [{min(nc)}..{max(nc)}]")

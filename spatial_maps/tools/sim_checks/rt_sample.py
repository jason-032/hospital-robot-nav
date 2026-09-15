#!/usr/bin/env python3
"""Real-time factor over a wall-clock window: delta /clock over delta wall time."""
import sys, time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

window = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
label = sys.argv[2] if len(sys.argv) > 2 else 'rt'
rclpy.init()
n = Node('rt_sampler')
samples = []
n.create_subscription(Clock, '/clock', lambda m: samples.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
t0 = time.monotonic()
while time.monotonic() - t0 < window + 30 and (not samples or samples[-1][0] - samples[0][0] < window):
    rclpy.spin_once(n, timeout_sec=0.2)
if len(samples) < 2:
    print(f'[{label}] RT: no /clock messages')
    sys.exit(2)
(w0, s0), (w1, s1) = samples[0], samples[-1]
print(f'[{label}] RT factor {(s1 - s0) / (w1 - w0):.3f} over {w1 - w0:.0f} s wall ({s1 - s0:.1f} s sim)')

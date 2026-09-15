#!/usr/bin/env python3
"""Drive the robot straight at the wall ahead of spawn and compare Gazebo ground
truth with wheel odometry.  A physical wall stops ground truth while odometry,
which integrates wheel speed, keeps advancing.

Along y = 38 from spawn (21, 38) facing +x:
  first PGM occupied cell  x = 24.99  (a PGM box in the boxes world)
  first mesh wall face     x = 28.20  (IfcWall in the mesh world)
"""
import re, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

label = sys.argv[1] if len(sys.argv) > 1 else 'contact'
SPEED = 0.3
DRIVE_SIM_S = float(sys.argv[2]) if len(sys.argv) > 2 else 35.0
PGM_OBSTACLE_X, MESH_WALL_X = 24.99, 28.20


def ground_truth():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/1f/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=30).stdout
    i = out.find('name: "my_robot"')
    if i < 0:
        return None
    block = out[i:out.find('orientation', i)]
    x = re.search(r'\bx: ([-\d.e+]+)', block)
    y = re.search(r'\by: ([-\d.e+]+)', block)
    return (float(x.group(1)) if x else 0.0, float(y.group(1)) if y else 0.0)


class Driver(Node):
    def __init__(self):
        super().__init__('contact_test', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    n = Driver()
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 120:
        rclpy.spin_once(n, timeout_sec=0.2)
    gt0 = ground_truth()
    raw = subprocess.run(['gz', 'topic', '-e', '-t', '/world/1f/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=30).stdout
    i = raw.find('name: "my_robot"')
    print(f'[{label}] raw pose block: ' + ' '.join(raw[max(0, i - 40):i + 260].split()))
    if n.odom is None or gt0 is None:
        print(f'[{label}] contact: no odom or ground truth')
        sys.exit(2)
    o0 = n.odom.pose.pose.position.x
    cmd = Twist(); cmd.linear.x = SPEED
    start, trace, last_gt = n.now(), [], 0.0
    while n.now() - start < DRIVE_SIM_S:
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last_gt > 3.0:
            g = ground_truth(); last_gt = time.time()
            if g:
                trace.append((n.now() - start, g[0]))
    for _ in range(10):
        n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    gt1 = ground_truth()
    o1 = n.odom.pose.pose.position.x
    def front_range():
        for _ in range(40):
            rclpy.spin_once(n, timeout_sec=0.1)
        m = n.scan
        k = int(round((0.0 - m.angle_min) / m.angle_increment))
        return min(r for r in m.ranges[max(k - 2, 0):k + 3])
    fr = front_range()
    print(f'[{label}] lidar front range {fr:.3f} m -> lidar x ~ {MESH_WALL_X - fr:.3f} if that return is the mesh wall at x={MESH_WALL_X}')
    late = [x for t, x in trace if t > DRIVE_SIM_S - 10.0]
    stalled = len(late) >= 2 and max(late) - min(late) < 0.03
    print(f'[{label}] contact: ground truth x {gt0[0]:.3f} -> {gt1[0]:.3f} (moved {gt1[0] - gt0[0]:.3f} m, y {gt1[1]:.3f}); '
          f'odometry moved {o1 - o0:.3f} m; commanded {SPEED * DRIVE_SIM_S:.1f} m; '
          f'stalled in last 10 s: {stalled}; gap to PGM obstacle {PGM_OBSTACLE_X - gt1[0]:+.3f} m, '
          f'gap to mesh wall {MESH_WALL_X - gt1[0]:+.3f} m')
    print(f'[{label}] contact trace (sim s, gt x): ' + ' '.join(f'{t:.0f}:{x:.2f}' for t, x in trace))
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

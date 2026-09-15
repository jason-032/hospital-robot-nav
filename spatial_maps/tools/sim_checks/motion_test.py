#!/usr/bin/env python3
"""Open-loop motion check: spin in place, then drive straight, comparing Gazebo
ground truth with wheel odometry and the measured wheel joint velocities.

Separates three failure modes:
  wheels turn at the commanded rate but the body does not follow  -> traction / contact
  wheels do not reach the commanded rate                          -> joint motor / effort
  body follows the wheels                                         -> motion is fine

Run in a world without interior collision (1f_nocoll*) so walls cannot interfere.
Usage: motion_test.py <label> [spin_rad_s] [linear_m_s] [phase_sim_s]
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gt_logger import GroundTruth, yaw_of

label = sys.argv[1] if len(sys.argv) > 1 else 'motion'
SPIN = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
LIN = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
PHASE = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
WHEEL_R, WHEEL_SEP = 0.1, 0.45          # DiffDrive plugin parameters
P = f'[{label}]'
PHASE_SKIP = 20                         # drop the first 1/20 of wheel samples (spin-up)


class Tester(Node):
    def __init__(self):
        super().__init__('motion_test', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.wheels = []                 # (left_vel, right_vel) samples during a phase
        self.collect = False
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(JointState, '/joint_states_gz', self.on_joints, 10)

    def on_joints(self, m):
        if not self.collect:
            return
        v = dict(zip(m.name, m.velocity))
        if 'base_left_wheel_joint' in v and 'base_right_wheel_joint' in v:
            self.wheels.append((v['base_left_wheel_joint'], v['base_right_wheel_joint']))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def odom_pose(n):
    p = n.odom.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)


def run_phase(n, gt, name, lin, ang):
    cmd = Twist(); cmd.linear.x = lin; cmd.angular.z = ang
    g0, o0 = gt.get(), odom_pose(n)
    gyaw = oyaw = 0.0
    pg, po = g0[3], o0[2]
    n.wheels, n.collect = [], True
    start = n.now()
    while n.now() - start < PHASE:
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.02)
        g, o = gt.get(), odom_pose(n)
        gyaw += math.remainder(g[3] - pg, 2 * math.pi); pg = g[3]
        oyaw += math.remainder(o[2] - po, 2 * math.pi); po = o[2]
    n.collect = False
    stop(n, 3.0)
    g1, o1 = gt.get(), odom_pose(n)
    gyaw += math.remainder(g1[3] - pg, 2 * math.pi)
    oyaw += math.remainder(o1[2] - po, 2 * math.pi)
    # steady-state wheel speed: skip the first second of samples
    ws = n.wheels[len(n.wheels) // PHASE_SKIP:] if n.wheels else []
    wl = sum(w[0] for w in ws) / len(ws) if ws else float('nan')
    wr = sum(w[1] for w in ws) / len(ws) if ws else float('nan')
    cl = (lin - ang * WHEEL_SEP / 2) / WHEEL_R
    cr = (lin + ang * WHEEL_SEP / 2) / WHEEL_R
    gd = math.hypot(g1[1] - g0[1], g1[2] - g0[2])
    od = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    print(f'{P} {name}: commanded lin {lin:.2f} m/s ang {ang:.2f} rad/s for {PHASE:.0f} s sim '
          f'-> {lin * PHASE:.2f} m, {ang * PHASE:.2f} rad')
    print(f'{P} {name}: odometry moved {od:.3f} m, turned {oyaw:.3f} rad | ground truth moved {gd:.3f} m, '
          f'turned {gyaw:.3f} rad')
    print(f'{P} {name}: wheel joint velocity mean L {wl:.3f} R {wr:.3f} rad/s vs commanded L {cl:.3f} R {cr:.3f} '
          f'({len(ws)} samples)')
    if ang:
        print(f'{P} {name}: true/commanded rotation {gyaw / (ang * PHASE):.3f}; true/odometry rotation '
              f'{gyaw / oyaw if oyaw else float("nan"):.3f}')
    else:
        print(f'{P} {name}: true/commanded distance {gd / (lin * PHASE):.3f}; true/odometry distance '
              f'{gd / od if od else float("nan"):.3f}')


def stop(n, secs):
    t = n.now()
    while n.now() - t < secs:
        n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)


def main():
    gt = GroundTruth(); gt.start()
    rclpy.init()
    n = Tester()
    t0 = time.time()
    while (n.odom is None or gt.get() is None) and time.time() - t0 < 120:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.odom is None or gt.get() is None:
        print(f'{P} motion: no odometry or ground truth'); sys.exit(2)
    stop(n, 3.0)
    g = gt.get()
    print(f'{P} rest pose: x {g[1]:.3f} y {g[2]:.3f} z {g[5]:.4f} roll {math.degrees(g[6]):.2f} deg '
          f'pitch {math.degrees(g[7]):.2f} deg yaw {math.degrees(g[3]):.2f} deg')
    run_phase(n, gt, 'spin', 0.0, SPIN)
    run_phase(n, gt, 'straight', LIN, 0.0)
    g = gt.get()
    print(f'{P} end pose: z {g[5]:.4f} roll {math.degrees(g[6]):.2f} deg pitch {math.degrees(g[7]):.2f} deg')
    n.destroy_node(); rclpy.shutdown()
    if gt.proc:
        gt.proc.terminate()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Passive ground-truth logger for 1F sweeps.

Records, without sending anything to the simulation:
  <label>.gt_traj.csv    every 0.5 s sim: Gazebo ground-truth pose of my_robot and
                         the pose Nav2 believes (TF map -> base_footprint)
  <label>.gt_events.csv  every NavigateToPose status change; terminal states carry
                         both poses at that instant and the end of the last /plan

Ground truth comes from `gz topic -e -t /world/1f/dynamic_pose/info` (the source
contact_test.py uses). Analyse afterwards with gt_analyse.py.

Usage: gt_logger.py <label> [out_dir]
"""
import csv, math, os, subprocess, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Path
import tf2_ros

label = sys.argv[1] if len(sys.argv) > 1 else 'gt'
out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/sim_trials')
ROBOT = 'my_robot'
TERMINAL = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class GroundTruth(threading.Thread):
    """Streams Pose_V text from the gz CLI and keeps the latest my_robot pose.
    Protobuf text omits zero-valued fields, so missing fields default to 0."""

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.latest = None          # (stamp_s, x, y, yaw, wall_time, z, roll, pitch)
        self.proc = None

    def run(self):
        while True:
            self.proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', '/world/1f/dynamic_pose/info'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
            sec = nsec = 0
            in_header = in_robot = False
            section = None
            vals = {}
            for line in self.proc.stdout:
                s = line.strip()
                if line.startswith('header {'):
                    in_header, sec, nsec = True, 0, 0
                    continue
                if in_header:
                    if s.startswith('sec:'):
                        sec = int(s.split()[1])
                    elif s.startswith('nsec:'):
                        nsec = int(s.split()[1])
                    elif line.startswith('}'):
                        in_header = False
                    continue
                if s == f'name: "{ROBOT}"':
                    in_robot, section, vals = True, None, {}
                    continue
                if not in_robot:
                    continue
                if s in ('position {', 'orientation {'):
                    section = s[0]
                elif s == '}' and line.startswith('  }'):
                    section = None
                elif line.startswith('}'):
                    in_robot = False
                    q = [vals.get(('o', k), 0.0) for k in 'xyzw']
                    if not any(q):
                        q[3] = 1.0
                    qx, qy, qz, qw = q
                    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
                    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
                    with self.lock:
                        self.latest = (sec + nsec * 1e-9, vals.get(('p', 'x'), 0.0),
                                       vals.get(('p', 'y'), 0.0), yaw_of(*q), time.time(),
                                       vals.get(('p', 'z'), 0.0), roll, pitch)
                elif section and ':' in s:
                    k, v = s.split(':', 1)
                    vals[('p' if section == 'p' else 'o', k)] = float(v)
            time.sleep(2.0)         # gz CLI exited (sim not up yet); retry

    def get(self):
        with self.lock:
            return self.latest


class Logger(Node):
    def __init__(self, gt):
        super().__init__('gt_logger', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.gt = gt
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.status = {}            # goal uuid hex -> last status
        self.plan_end = {}          # goal uuid hex -> (x, y)
        self.executing = None
        self.goal_seq = {}          # goal uuid hex -> acceptance order
        os.makedirs(out_dir, exist_ok=True)
        self.traj_f = open(os.path.join(out_dir, f'{label}.gt_traj.csv'), 'w', newline='')
        self.ev_f = open(os.path.join(out_dir, f'{label}.gt_events.csv'), 'w', newline='')
        self.traj = csv.writer(self.traj_f)
        self.ev = csv.writer(self.ev_f)
        self.traj.writerow(['sim_t', 'wall_t', 'goal_seq', 'gt_stamp', 'gt_x', 'gt_y', 'gt_yaw',
                            'tf_x', 'tf_y', 'tf_yaw'])
        self.ev.writerow(['sim_t', 'wall_t', 'goal_seq', 'goal_id', 'status', 'status_name',
                          'gt_stamp', 'gt_x', 'gt_y', 'gt_yaw', 'tf_x', 'tf_y', 'tf_yaw',
                          'plan_end_x', 'plan_end_y'])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, qos)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_timer(0.5, self.sample)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tf_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return ('', '', '')
        r = t.transform.rotation
        return (f'{t.transform.translation.x:.4f}', f'{t.transform.translation.y:.4f}',
                f'{yaw_of(r.x, r.y, r.z, r.w):.4f}')

    def gt_fields(self):
        g = self.gt.get()
        if g is None or time.time() - g[4] > 2.0:
            return ('', '', '', '')
        return (f'{g[0]:.3f}', f'{g[1]:.4f}', f'{g[2]:.4f}', f'{g[3]:.4f}')

    def sample(self):
        seq = self.goal_seq.get(self.executing, '') if self.executing else ''
        self.traj.writerow([f'{self.now():.3f}', f'{time.time():.3f}', seq, *self.gt_fields(), *self.tf_pose()])
        self.traj_f.flush()

    def on_plan(self, msg):
        if self.executing and msg.poses:
            p = msg.poses[-1].pose.position
            self.plan_end[self.executing] = (p.x, p.y)

    def on_status(self, msg):
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()
            if self.status.get(gid) == st.status:
                continue
            if gid not in self.goal_seq:
                self.goal_seq[gid] = len(self.goal_seq) + 1
            self.status[gid] = st.status
            if st.status == 2:
                self.executing = gid
            elif st.status in TERMINAL and self.executing == gid:
                self.executing = None
            pe = self.plan_end.get(gid, ('', ''))
            self.ev.writerow([f'{self.now():.3f}', f'{time.time():.3f}', self.goal_seq[gid], gid[:8],
                              st.status, TERMINAL.get(st.status, {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING'}.get(st.status, '')),
                              *self.gt_fields(), *self.tf_pose(),
                              *(f'{v:.3f}' if v != '' else '' for v in pe)])
            self.ev_f.flush()


def main():
    gt = GroundTruth()
    gt.start()
    rclpy.init()
    n = Logger(gt)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.traj_f.close(); n.ev_f.close()
        if gt.proc:
            gt.proc.terminate()


if __name__ == '__main__':
    main()

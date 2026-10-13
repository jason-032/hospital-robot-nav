#!/usr/bin/env python3
"""Localisation preflight for a robot running Nav2 + AMCL on a prebuilt map.

Read-only. It subscribes and reads parameters; it never publishes, sets a pose or
changes a parameter. Each check comes from a fault found on hospital 1F in
September 2026 (see README.md beside this file):

  map    is the latched map being republished (AMCL reinitialises on every copy)
  tf     who publishes map -> odom, how often, and how stale it is
  amcl   the AMCL parameters that mattered on 1F, and the pose covariance
  scan   how well the LiDAR scan fits the map at the believed pose, and the
         pose correction (dx, dy, dyaw) that would fit it best

Usage (robot or simulator running, ROS environment sourced):
  python3 localisation_preflight.py                       # real robot
  python3 localisation_preflight.py --sim-time            # Gazebo
  python3 localisation_preflight.py --scan-topic /ouster/scan            # SCS Lab Go2
  ros2 bag play <bag> & python3 localisation_preflight.py ...           # from a recording

Run it on the robot or on a machine whose clock is synchronised with it: the
transform ages compare message stamps with the local clock.
Written and tested on Jazzy. Nothing Jazzy-specific is used, but Humble is
untested. Needs rclpy, tf2_ros, NumPy and SciPy.
"""
import argparse
import math
import sys
import time

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

# AMCL parameters worth reading before a run, with the 1F finding for each.
AMCL_PARAMS = [
    ('tf_broadcast', 'must be true unless something else owns map -> odom'),
    ('set_initial_pose', 'false means AMCL waits for /initialpose'),
    ('initial_pose.x', ''), ('initial_pose.y', ''), ('initial_pose.yaw', ''),
    ('alpha1', '1F: defaults (0.2) diverged 13 m in 25 s where the map and the '
               'world disagreed; 0.02 held. Only lower them if odometry is good'),
    ('alpha2', ''), ('alpha3', ''), ('alpha4', ''), ('alpha5', ''),
    ('max_beams', '1F: 60 was too few, 180 held'),
    ('laser_max_range', 'should match the real sensor, not the default'),
    ('laser_min_range', ''),
    ('min_particles', '1F: 1000'), ('max_particles', '1F: 5000'),
    ('update_min_d', ''), ('update_min_a', ''),
    ('laser_model_type', ''), ('robot_model_type', 'omni model for a quadruped that strafes'),
    ('base_frame_id', ''), ('odom_frame_id', ''), ('global_frame_id', ''),
    ('scan_topic', ''), ('transform_tolerance', ''),
]


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def param_value(v):
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
            4: v.string_value}.get(v.type, None)


class MapField:
    """Distance from any map point to the nearest occupied cell."""

    def __init__(self, msg, occupied=65):
        info = msg.info
        self.res = info.resolution
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        self.w, self.h = info.width, info.height
        grid = np.asarray(msg.data, dtype=np.int16).reshape(self.h, self.w)
        self.n_occupied = int((grid >= occupied).sum())
        self.dist = ndimage.distance_transform_edt(grid < occupied) * self.res

    def lookup(self, p):
        c = np.floor((p[:, 0] - self.ox) / self.res).astype(int)
        r = np.floor((p[:, 1] - self.oy) / self.res).astype(int)
        ok = (r >= 0) & (r < self.h) & (c >= 0) & (c < self.w)
        return self.dist[r[ok], c[ok]]


def best_correction(field, pts, centre, tol, max_shift=1.0, max_yaw_deg=30.0):
    """Rigid correction of the scan, about the sensor, that puts most hits on walls.

    Returns ((share within tol, dx, dy, dyaw_deg), (spread m, spread deg)). A robot
    whose believed pose is right scores best at or next to (0, 0, 0). The spread is
    how far apart the corrections within three points of the best score lie: a
    large spread means the scan fits the map in several places (a plain corridor)
    and the correction should not be trusted.

    Two passes: 0.10 m and 2 deg over the whole range, then 0.05 m and 1 deg
    around the best coarse fit and around zero."""
    if len(pts) > 1500:
        pts = pts[np.linspace(0, len(pts) - 1, 1500).astype(int)]
    rel = pts - centre
    found = {}

    def scan(xs, ys, yaws):
        for yaw_deg in yaws:
            a = math.radians(yaw_deg)
            rot = rel @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]) + centre
            for dx in xs:
                for dy in ys:
                    key = (round(float(dx), 2), round(float(dy), 2), round(float(yaw_deg), 1))
                    if key not in found:
                        d = field.lookup(rot + [dx, dy])
                        found[key] = float(np.mean(d <= tol)) * len(d) / len(rot) if len(d) else 0.0

    coarse = np.arange(-max_shift, max_shift + 1e-9, 0.10)
    scan(coarse, coarse, np.arange(-max_yaw_deg, max_yaw_deg + 1e-9, 2.0))
    cx, cy, cyaw = max(found, key=found.get)
    fine = np.arange(-0.10, 0.10 + 1e-9, 0.05)
    for ox, oy, oyaw in ((cx, cy, cyaw), (0.0, 0.0, 0.0)):
        scan(ox + fine, oy + fine, oyaw + np.arange(-2.0, 2.0 + 1e-9, 1.0))
    # Of the corrections within one point of the best score, take the smallest, so
    # a pose that is already right reports (0, 0, 0) and not a neighbouring cell.
    top = max(found.values())
    k = min((k for k, v in found.items() if v >= top - 0.01),
            key=lambda k: math.hypot(k[0], k[1]) + abs(k[2]) * 0.02)
    near = np.array([k for k, v in found.items() if v >= top - 0.03])
    spread = (float(np.hypot(*(near[:, :2].max(0) - near[:, :2].min(0)))),
              float(near[:, 2].max() - near[:, 2].min()))
    return (found[k], k[0] + 0.0, k[1] + 0.0, k[2] + 0.0), spread   # + 0.0 turns -0.0 into 0.0


def correct_pose(pose, centre, dx, dy, dyaw_deg):
    """Apply a correction found about `centre` (the sensor) to another pose."""
    a = math.radians(dyaw_deg)
    rx, ry = pose[0] - centre[0], pose[1] - centre[1]
    return (centre[0] + rx * math.cos(a) - ry * math.sin(a) + dx,
            centre[1] + rx * math.sin(a) + ry * math.cos(a) + dy,
            math.atan2(math.sin(pose[2] + a), math.cos(pose[2] + a)))


class Preflight(Node):
    def __init__(self, a):
        overrides = [Parameter('use_sim_time', Parameter.Type.BOOL, True)] if a.sim_time else []
        super().__init__('localisation_preflight', parameter_overrides=overrides)
        self.a = a
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msgs = []          # wall time of each map message
        self.map = None
        self.tf_dyn, self.tf_static = [], []   # (wall time, age s) of map -> odom
        self.scans = []
        self.amcl_pose = None
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(OccupancyGrid, a.map_topic, self.on_map, latched)
        self.create_subscription(TFMessage, '/tf', lambda m: self.on_tf(m, self.tf_dyn), 100)
        # Every static broadcaster latches its own message, so keep them all.
        static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static',
                                 lambda m: self.on_tf(m, self.tf_static), static)
        self.create_subscription(LaserScan, a.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, a.amcl_pose_topic,
                                 self.on_pose, latched)

    def on_map(self, msg):
        self.map_msgs.append(time.time())
        self.map = msg

    def on_tf(self, msg, store):
        for t in msg.transforms:
            if (t.header.frame_id.lstrip('/') == self.a.global_frame
                    and t.child_frame_id.lstrip('/') == self.a.odom_frame):
                now = self.get_clock().now().nanoseconds * 1e-9
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                store.append((time.time(), now - stamp))

    def on_pose(self, msg):
        self.amcl_pose = msg

    def on_scan(self, msg):
        if len(self.scans) >= self.a.scans:
            return
        try:
            tf = self.buf.lookup_transform(self.a.global_frame, msg.header.frame_id,
                                           rclpy.time.Time())
        except Exception:
            return
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max * 0.98)
        t = tf.transform.translation
        yaw = quat_yaw(tf.transform.rotation)
        pts = np.stack([t.x + r[ok] * np.cos(ang[ok] + yaw),
                        t.y + r[ok] * np.sin(ang[ok] + yaw)], 1)
        base = None
        try:
            b = self.buf.lookup_transform(self.a.global_frame, self.a.base_frame,
                                          rclpy.time.Time()).transform
            base = (b.translation.x, b.translation.y, quat_yaw(b.rotation))
        except Exception:
            pass
        self.scans.append((pts, (t.x, t.y, yaw), int(ok.sum()), len(r), base))

    def get_params(self, node_name, names):
        cli = self.create_client(GetParameters, f'/{node_name.lstrip("/")}/get_parameters')
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = cli.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return [param_value(v) for v in fut.result().values] if fut.result() else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--duration', type=float, default=20.0, help='listening time, wall seconds')
    ap.add_argument('--map-topic', default='/map')
    ap.add_argument('--scan-topic', default='/scan')
    ap.add_argument('--amcl-pose-topic', default='/amcl_pose')
    ap.add_argument('--amcl-node', default='amcl')
    ap.add_argument('--lifecycle-node', default='lifecycle_manager_nav',
                    help='lifecycle manager to read bond_timeout from')
    ap.add_argument('--global-frame', default='map')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--base-frame', default='base_footprint',
                    help='robot frame the pose correction is reported for')
    ap.add_argument('--scans', type=int, default=5)
    ap.add_argument('--tol', type=float, default=0.10,
                    help='a scan hit within this distance of an occupied cell counts as on the map')
    ap.add_argument('--max-shift', type=float, default=1.0,
                    help='pose correction search range, m each way')
    ap.add_argument('--max-yaw', type=float, default=30.0,
                    help='pose correction search range, deg each way')
    ap.add_argument('--sim-time', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = Preflight(a)
    warn = []
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < a.duration:
        rclpy.spin_once(n, timeout_sec=0.2)
    span = time.time() - t0

    # ── map ──────────────────────────────────────────────────────────────────
    if n.map is None:
        print(f'[map]  no message on {a.map_topic} in {span:.0f} s')
        warn.append('no map received')
    else:
        f = MapField(n.map)
        print(f'[map]  {f.w} x {f.h} cells at {f.res:.3f} m, origin ({f.ox:.2f}, {f.oy:.2f}), '
              f'frame {n.map.header.frame_id}, {f.n_occupied} occupied cells')
        print(f'[map]  {len(n.map_msgs)} message(s) in {span:.0f} s')
        if len(n.map_msgs) > 1:
            gaps = np.diff(n.map_msgs)
            warn.append(f'map republished every {np.median(gaps):.1f} s: AMCL reinitialises its '
                        'filter on each copy. Publish it once, latched (transient local)')

    # ── tf ───────────────────────────────────────────────────────────────────
    name = f'{a.global_frame} -> {a.odom_frame}'
    if n.tf_dyn:
        ages = np.array([x[1] for x in n.tf_dyn])
        p95 = float(np.percentile(ages, 95))
        print(f'[tf]   {name} on /tf: {len(n.tf_dyn) / span:.1f} Hz, stamp age '
              f'median {np.median(ages):+.2f} s, 95th percentile {p95:+.2f} s, '
              f'worst {ages.max():+.2f} s (negative = post-dated by transform_tolerance)')
        # One late sample is start-up noise; a late 95th percentile is a fault.
        if p95 > 1.0:
            warn.append(f'{name} was over {p95:.1f} s old in 5% of samples: the localiser '
                        'is not updating steadily (or this clock is not synchronised)')
    else:
        print(f'[tf]   {name} not seen on /tf')
    if n.tf_static:
        print(f'[tf]   {name} also on /tf_static ({len(n.tf_static)} message(s))')
    if n.tf_dyn and n.tf_static:
        warn.append(f'{name} has a static and a dynamic publisher: they fight. Remove the static one')
    elif n.tf_static:
        warn.append(f'{name} is static only: nothing corrects odometry drift')
    elif not n.tf_dyn:
        warn.append(f'nobody publishes {name}')

    # ── amcl ─────────────────────────────────────────────────────────────────
    vals = n.get_params(a.amcl_node, [p for p, _ in AMCL_PARAMS])
    if vals is None:
        print(f'[amcl] node /{a.amcl_node} did not answer get_parameters')
    else:
        got = dict(zip([p for p, _ in AMCL_PARAMS], vals))
        for p, note in AMCL_PARAMS:
            if got[p] is not None:
                print(f'[amcl] {p:22s} {got[p]!s:12s} {note}')
        if got.get('tf_broadcast') is False and not n.tf_static:
            warn.append('AMCL tf_broadcast is false and there is no static transform either')
    bt = n.get_params(a.lifecycle_node, ['bond_timeout'])
    if bt and bt[0] is not None:
        print(f'[amcl] lifecycle bond_timeout {bt[0]} s (1F: the 4 s default crash-looped AMCL '
              'on an 8 M cell map; 30 s held)')
    if n.amcl_pose is not None:
        c = n.amcl_pose.pose.covariance
        p = n.amcl_pose.pose.pose
        print(f'[amcl] pose ({p.position.x:.2f}, {p.position.y:.2f}, '
              f'{math.degrees(quat_yaw(p.orientation)):.1f} deg), sigma x {math.sqrt(max(c[0], 0)):.3f} m, '
              f'y {math.sqrt(max(c[7], 0)):.3f} m, yaw {math.degrees(math.sqrt(max(c[35], 0))):.1f} deg')
    else:
        print(f'[amcl] nothing on {a.amcl_pose_topic} (AMCL publishes it after the first update)')

    # ── scan against map ─────────────────────────────────────────────────────
    if n.map is None or not n.scans:
        print(f'[scan] no scan on {a.scan_topic} with a transform to {a.global_frame}')
        warn.append('scan could not be placed in the map frame')
    else:
        pts = np.concatenate([s[0] for s in n.scans])
        x, y, yaw = n.scans[0][1]
        print(f'[scan] sensor at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg) in {a.global_frame}, '
              f'{n.scans[0][2]}/{n.scans[0][3]} valid returns per scan, {len(n.scans)} scans')
        d = f.lookup(pts)
        if len(d) == 0:
            warn.append('every scan hit falls outside the map: the believed pose is far off')
        else:
            on = float(np.mean(d <= a.tol)) * len(d) / len(pts)
            print(f'[scan] {on * 100:.1f}% of hits within {a.tol:.2f} m of an occupied cell '
                  f'(median {np.median(d):.3f} m, p90 {np.percentile(d, 90):.3f} m, '
                  f'{(1 - len(d) / len(pts)) * 100:.1f}% outside the map)')
            (s, dx, dy, dyaw), spread = best_correction(f, pts, np.array([x, y]), a.tol,
                                                        a.max_shift, a.max_yaw)
            print(f'[scan] best fit: move the sensor by dx {dx:+.2f} m, dy {dy:+.2f} m, '
                  f'yaw {dyaw:+.0f} deg -> {s * 100:.1f}% on the map')
            ambiguous = spread[0] > 0.30 or spread[1] > 5.0
            print(f'[scan] fits within 3 points of the best span {spread[0]:.2f} m and '
                  f'{spread[1]:.0f} deg' + (': AMBIGUOUS here, the scan fits in several '
                                            'places, do not trust the correction' if ambiguous else ''))
            base = n.scans[0][4]
            if base is None:
                print(f'[scan] no transform {a.global_frame} -> {a.base_frame}: the correction '
                      'above is for the sensor, not the robot base (see --base-frame)')
            elif (dx, dy, dyaw) != (0.0, 0.0, 0.0) and not ambiguous:
                nx, ny, nyaw = correct_pose(base, (x, y), dx, dy, dyaw)
                print(f'[scan] {a.base_frame} would move from ({base[0]:.2f}, {base[1]:.2f}, '
                      f'{math.degrees(base[2]):.1f} deg) to ({nx:.2f}, {ny:.2f}, '
                      f'{math.degrees(nyaw):.1f} deg). Not sent. To try it:')
                print(f"[scan]   ros2 topic pub --once /initialpose "
                      f"geometry_msgs/msg/PoseWithCovarianceStamped \"{{header: {{frame_id: "
                      f"{a.global_frame}}}, pose: {{pose: {{position: {{x: {nx:.3f}, y: {ny:.3f}}}, "
                      f"orientation: {{z: {math.sin(nyaw / 2):.5f}, w: {math.cos(nyaw / 2):.5f}}}}}}}}}\"")
            if math.hypot(dx, dy) > 0.15 or abs(dyaw) > 3:
                if s - on > 0.10:
                    warn.append(f'the scan fits the map {(s - on) * 100:.0f} points better at '
                                f'dx {dx:+.2f}, dy {dy:+.2f}, yaw {dyaw:+.0f} deg: the pose is off')
            if s < 0.5:
                warn.append(f'even the best fit puts only {s * 100:.0f}% of hits on the map: the map '
                            f'lacks what the LiDAR sees (clutter, furniture) or the pose is over '
                            f'{a.max_shift:.1f} m or {a.max_yaw:.0f} deg off')

    print()
    for w in warn:
        print(f'WARN  {w}')
    print(f'{len(warn)} warning(s)')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if warn else 0)


if __name__ == '__main__':
    main()

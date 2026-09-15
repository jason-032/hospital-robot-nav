#!/usr/bin/env python3
"""Grab LiDAR scans in the running 1F sim, project them into the map frame, and
measure how far each hit lies from the nearest occupied PGM cell."""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import tf2_ros
from PIL import Image
from scipy import ndimage

RES, OX, OY = 0.02, 0.9106622527818615, -2.0
pgm = np.array(Image.open('/home/jason/ros2_ws/src/spatial_maps/maps/1F.pgm'))
H, W = pgm.shape
occ = (255.0 - pgm) / 255.0 > 0.65
dist = ndimage.distance_transform_edt(~occ) * RES
label = sys.argv[1] if len(sys.argv) > 1 else 'scan'
N_SCANS = 5


def quat_yaw(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Checker(Node):
    def __init__(self):
        super().__init__('scan_alignment_check',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.pts = []
        self.count = 0
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.count >= N_SCANS:
            return
        try:
            tf = self.buf.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        r = np.array(msg.ranges)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max * 0.98)
        t = tf.transform.translation
        yaw = quat_yaw(tf.transform.rotation)
        x = t.x + r[ok] * np.cos(a[ok] + yaw)
        y = t.y + r[ok] * np.sin(a[ok] + yaw)
        self.pts.append(np.stack([x, y], 1))
        self.count += 1
        if self.count == 1:
            self.get_logger().info(f'sensor pose in map: ({t.x:.2f}, {t.y:.2f}, yaw {np.degrees(yaw):.1f} deg), '
                                   f'{ok.sum()}/{len(r)} valid returns, frame {msg.header.frame_id}')


def score(p):
    r = (H - 1 - np.floor((p[:, 1] - OY) / RES)).astype(int)
    c = np.floor((p[:, 0] - OX) / RES).astype(int)
    ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
    return dist[r[ok], c[ok]]


def main():
    rclpy.init()
    n = Checker()
    t0 = time.time()
    while rclpy.ok() and n.count < N_SCANS and time.time() - t0 < 240:
        rclpy.spin_once(n, timeout_sec=0.5)
    if not n.pts:
        print(f'[{label}] no scans with TF received within 240 s')
        sys.exit(2)
    p = np.concatenate(n.pts)
    d = score(p)
    print(f'[{label}] {len(n.pts)} scans, {len(d)} hits: median {np.median(d):.3f} m, '
          f'p90 {np.percentile(d, 90):.3f} m, within 0.06 m {np.mean(d <= 0.06) * 100:.1f}%, '
          f'within 0.10 m {np.mean(d <= 0.10) * 100:.1f}%, beyond 0.30 m {np.mean(d > 0.30) * 100:.1f}%')
    best = max(((np.mean(score(p + [dx, dy]) <= 0.06), dx, dy)
                for dx in np.arange(-1.2, 1.21, 0.04) for dy in np.arange(-1.2, 1.21, 0.04)))
    print(f'[{label}] best shift of scan onto map: dx {best[1]:+.2f} dy {best[2]:+.2f} '
          f'-> within 0.06 m {best[0] * 100:.1f}%')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Simple occupancy grid publisher — no nav2 required.

Reads a PGM + YAML map file and publishes nav_msgs/OccupancyGrid
on /map at 1 Hz with TRANSIENT_LOCAL durability (latched).
"""

import os
import struct
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration


def _read_pgm(path: str):
    """Return (width, height, data_bytes) from a binary or ASCII PGM file."""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        # Skip comments
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = map(int, line.split())
        maxval = int(f.readline().strip())
        raw = f.read()

    if magic == b'P5':  # binary PGM
        pixels = list(raw[:width * height])
    elif magic == b'P2':  # ASCII PGM
        pixels = list(map(int, raw.split()))
    else:
        raise ValueError(f'Unsupported PGM format: {magic}')

    return width, height, pixels, maxval


class MapPublisherNode(Node):

    def __init__(self):
        super().__init__('map_publisher')

        self.declare_parameter('yaml_filename', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)

        yaml_file = self.get_parameter('yaml_filename').value
        frame_id  = self.get_parameter('frame_id').value
        rate_hz   = self.get_parameter('publish_rate_hz').value

        if not yaml_file or not os.path.isfile(yaml_file):
            self.get_logger().error(f'Map YAML not found: {yaml_file}')
            return

        # Load YAML metadata
        with open(yaml_file) as f:
            meta = yaml.safe_load(f)

        pgm_path = os.path.join(os.path.dirname(yaml_file), meta['image'])
        resolution   = float(meta['resolution'])
        origin       = meta['origin']          # [x, y, yaw]
        negate       = int(meta.get('negate', 0))
        occ_thresh   = float(meta.get('occupied_thresh', 0.65))
        free_thresh  = float(meta.get('free_thresh', 0.196))

        width, height, pixels, maxval = _read_pgm(pgm_path)
        self.get_logger().info(
            f'Loaded map {pgm_path} ({width}x{height} px, {resolution} m/px)')

        # Convert pixel values → ROS occupancy values (-1, 0–100)
        # nav2_map_server standard: occ = 1 - pixel/maxval
        #   white (255) → occ=0.0 → free
        #   black (0)   → occ=1.0 → occupied
        # negate=1 inverts that convention.
        data = []
        for p in pixels:
            occ = 1.0 - (p / maxval)
            if negate:
                occ = 1.0 - occ
            if occ > occ_thresh:
                data.append(100)      # occupied
            elif occ < free_thresh:
                data.append(0)        # free
            else:
                data.append(-1)       # unknown

        # ROS OccupancyGrid is stored bottom-row first (flip vertically)
        rows = [data[r * width:(r + 1) * width] for r in range(height)]
        rows_flipped = list(reversed(rows))
        data_flipped = [cell for row in rows_flipped for cell in row]

        # Build message once
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = frame_id
        self.msg.info.resolution = resolution
        self.msg.info.width  = width
        self.msg.info.height = height
        self.msg.info.origin.position.x = float(origin[0])
        self.msg.info.origin.position.y = float(origin[1])
        self.msg.info.origin.position.z = 0.0
        self.msg.info.origin.orientation.w = 1.0
        self.msg.data = data_flipped

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info('Map publisher ready, publishing on /map')

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

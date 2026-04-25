#!/usr/bin/env python3
"""
POI Click Node

Listens to RViz's /clicked_point topic (published by the
"Publish Point" tool — press P in RViz) and navigates to
the nearest POI to wherever you clicked on the map.

Usage in RViz:
  1. Select the "Publish Point" tool (toolbar, or press P)
  2. Click anywhere near a room sphere on the floor plan
  3. This node finds the closest room centroid and publishes
     its name to /goal_poi — the robot navigates there.
"""

import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class POIClickNode(Node):

    def __init__(self):
        super().__init__('poi_click_node')

        self.declare_parameter('semantic_json',
            '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json')
        self.declare_parameter('floor', '1F')
        self.declare_parameter('max_click_distance', 5.0)  # metres

        semantic_json     = self.get_parameter('semantic_json').value
        self.floor        = self.get_parameter('floor').value
        self.max_dist     = self.get_parameter('max_click_distance').value

        self.goal_pub = self.create_publisher(String, '/goal_poi', 10)
        self.create_subscription(PointStamped, '/clicked_point',
                                 self._on_click, 10)

        self.pois = self._load_pois(semantic_json)
        self.get_logger().info(
            f'POI Click Node ready — {len(self.pois)} rooms loaded. '
            f'Select "Publish Point" tool in RViz (P) and click a room sphere.'
        )

    # ------------------------------------------------------------------

    def _load_pois(self, path: str) -> list:
        try:
            with open(path) as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f'semantic.json not found: {path}')
            return []

        pois = []
        for entity in data.get('entities', {}).values():
            if entity.get('ifc_type') != 'IfcSpace':
                continue
            if entity.get('storey_name') != self.floor:
                continue
            if not entity.get('has_geometry'):
                continue
            c = entity['geometry']['centroid']
            pois.append({
                'id': entity.get('name', '?'),
                'x':  float(c[0]),
                'y':  float(c[1]),
            })
        return pois

    # ------------------------------------------------------------------

    def _on_click(self, msg: PointStamped):
        cx, cy = msg.point.x, msg.point.y

        if not self.pois:
            self.get_logger().warn('No POIs loaded.')
            return

        # Find nearest POI in XY (ignore Z — all rooms are on same floor)
        nearest = min(self.pois,
                      key=lambda p: math.hypot(p['x'] - cx, p['y'] - cy))
        dist = math.hypot(nearest['x'] - cx, nearest['y'] - cy)

        if dist > self.max_dist:
            self.get_logger().warn(
                f'Click at ({cx:.1f}, {cy:.1f}) — nearest room '
                f'{nearest["id"]} is {dist:.1f}m away (> {self.max_dist}m limit). '
                f'Click closer to a sphere.'
            )
            return

        self.get_logger().info(
            f'Click at ({cx:.1f}, {cy:.1f}) → '
            f'{nearest["id"]} ({dist:.1f}m away) — navigating'
        )
        out = String()
        out.data = nearest['id']
        self.goal_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = POIClickNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
POI Publisher Node

Reads semantic.json from the Spatial_Map pipeline and publishes
room/space entities as a visualization_msgs/MarkerArray.

Each POI is shown as:
  - A sphere at the room centroid
  - A text label above it (room number + Korean name)

Markers are color-coded by robot_response behavior code:
  A-* : blue   (autonomous patrol)
  B-* : green  (passive monitoring)
  C-* : yellow (interactive / user-facing)
  H-* : orange (human-sensitive)
  default: grey
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


# Behavior-code colour map (R, G, B)
_BEHAVIOR_COLORS = {
    'A': (0.2, 0.5, 1.0),   # blue  – autonomous patrol
    'B': (0.2, 0.8, 0.3),   # green – passive monitoring
    'C': (1.0, 0.9, 0.1),   # yellow– interactive
    'H': (1.0, 0.55, 0.0),  # orange– human-sensitive
}
_DEFAULT_COLOR = (0.6, 0.6, 0.6)  # grey


def _behavior_color(robot_response: dict) -> tuple:
    codes = robot_response.get('always', [])
    for code in codes:
        prefix = code[0].upper() if code else ''
        if prefix in _BEHAVIOR_COLORS:
            return _BEHAVIOR_COLORS[prefix]
    return _DEFAULT_COLOR


class POIPublisherNode(Node):

    def __init__(self):
        super().__init__('poi_publisher')

        self.declare_parameter('semantic_json',
            '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json')
        self.declare_parameter('floor', '1F')
        self.declare_parameter('marker_frame', 'map')
        self.declare_parameter('sphere_scale', 0.6)
        self.declare_parameter('text_scale', 0.4)
        self.declare_parameter('publish_rate_hz', 1.0)

        semantic_json = self.get_parameter('semantic_json').value
        self.floor = self.get_parameter('floor').value
        self.frame = self.get_parameter('marker_frame').value
        self.sphere_scale = self.get_parameter('sphere_scale').value
        self.text_scale = self.get_parameter('text_scale').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        # Latched-style QoS so RViz sees markers on late connect
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(MarkerArray, '/poi_markers', latched_qos)

        self.markers = self._load_markers(semantic_json)
        self.get_logger().info(
            f'Loaded {len(self.markers) // 2} POIs for floor {self.floor}')

        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

    # ------------------------------------------------------------------

    def _load_markers(self, path: str) -> list:
        try:
            with open(path) as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f'semantic.json not found: {path}')
            return []

        entities = data.get('entities', {})
        markers = []
        marker_id = 0

        for entity in entities.values():
            if entity.get('ifc_type') != 'IfcSpace':
                continue
            if entity.get('storey_name') != self.floor:
                continue
            if not entity.get('has_geometry'):
                continue

            centroid = entity['geometry']['centroid']
            name_num = entity.get('name', '?')

            # Korean display name from Identity Data if available
            id_data = entity.get('properties', {}).get('Identity Data', {})
            name_kr = id_data.get('Name', '')
            label = f'{name_num}\n{name_kr}' if name_kr else name_num

            # Robot accessibility flag
            other = entity.get('properties', {}).get('Other', {})
            accessible = other.get('\ub85c\ubd07\uc811\uadfc\uac00\ub2a5 \uc5ec\ubd80', 'True')
            alpha = 1.0 if accessible == 'True' else 0.3

            robot_resp = entity.get('robot_response', {})
            r, g, b = _behavior_color(robot_resp)

            # --- Sphere marker ---
            sphere = Marker()
            sphere.header.frame_id = self.frame
            sphere.ns = 'poi_spheres'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = centroid[0]
            sphere.pose.position.y = centroid[1]
            sphere.pose.position.z = centroid[2]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.sphere_scale
            sphere.scale.y = self.sphere_scale
            sphere.scale.z = self.sphere_scale
            sphere.color = ColorRGBA(r=r, g=g, b=b, a=alpha)
            sphere.lifetime = Duration(sec=0)  # persist until deleted
            markers.append(sphere)
            marker_id += 1

            # --- Text marker ---
            text = Marker()
            text.header.frame_id = self.frame
            text.ns = 'poi_labels'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = centroid[0]
            text.pose.position.y = centroid[1]
            text.pose.position.z = centroid[2] + self.sphere_scale * 0.8
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_scale
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=alpha)
            text.text = label
            text.lifetime = Duration(sec=0)
            markers.append(text)
            marker_id += 1

        return markers

    def _publish(self):
        if not self.markers:
            return
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for m in self.markers:
            m.header.stamp = stamp
        msg.markers = self.markers
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = POIPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
POI Navigation Node

Subscribes to /goal_poi (std_msgs/String) and sends a NavigateToPose
action goal to nav2 for the matching room.

If a room centroid is inside an enclosed area (not navigable from the
robot's starting position), the goal is projected to the nearest
reachable corridor cell — so the robot always navigates to the doorstep.

Usage:
  ros2 topic pub --once /goal_poi std_msgs/String "data: 'S1369'"
  ros2 topic pub --once /goal_poi std_msgs/String "data: 'ELEV'"
"""

import json
import math
import os
import struct
import yaml
from collections import deque

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class POINavNode(Node):

    def __init__(self):
        super().__init__('poi_nav_node')

        self.declare_parameter('semantic_json',
            '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json')
        self.declare_parameter('floor', '1F')
        self.declare_parameter('goal_z', 0.0)
        self.declare_parameter('map_yaml', '')
        # Robot starting position — used to build the reachable set
        self.declare_parameter('robot_start_x', 21.0)
        self.declare_parameter('robot_start_y', 38.0)

        semantic_json   = self.get_parameter('semantic_json').value
        self.floor      = self.get_parameter('floor').value
        self.goal_z     = self.get_parameter('goal_z').value
        map_yaml        = self.get_parameter('map_yaml').value
        start_x         = self.get_parameter('robot_start_x').value
        start_y         = self.get_parameter('robot_start_y').value

        self.pois = self._load_pois(semantic_json)
        self.get_logger().info(
            f'Loaded {len(self.pois)} POIs for floor {self.floor}')

        # Build reachable set from PGM (optional — enables goal projection)
        self._reachable = None
        self._map_meta  = None
        if map_yaml and os.path.isfile(map_yaml):
            self._build_reachable(map_yaml, start_x, start_y)
        else:
            self.get_logger().warn(
                'map_yaml not provided — goal projection disabled. '
                'Goals inside enclosed rooms will fail.')

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._sub = self.create_subscription(
            String, '/goal_poi', self._on_goal_poi, 10)

        self.get_logger().info(
            'POI Nav Node ready. Publish room name to /goal_poi to navigate.')

    # ------------------------------------------------------------------
    # Map / reachability

    def _build_reachable(self, yaml_file: str, start_x: float, start_y: float):
        """BFS over the PGM to find every free cell reachable from (start_x, start_y)."""
        self.get_logger().info('Building reachable-set (BFS over map) …')

        with open(yaml_file) as f:
            meta = yaml.safe_load(f)

        pgm_path   = os.path.join(os.path.dirname(yaml_file), meta['image'])
        resolution = float(meta['resolution'])
        origin     = meta['origin']           # [x, y, yaw]
        free_thresh  = float(meta.get('free_thresh', 0.196))
        negate       = int(meta.get('negate', 0))

        with open(pgm_path, 'rb') as f:
            magic = f.readline().strip()
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            width, height = map(int, line.split())
            maxval = int(f.readline().strip())
            raw = f.read()

        if magic == b'P5':
            pixels = bytearray(raw[:width * height])
        elif magic == b'P2':
            pixels = bytearray(map(int, raw.split()))
        else:
            self.get_logger().error(f'Unsupported PGM format: {magic}')
            return

        origin_x, origin_y = float(origin[0]), float(origin[1])

        def world_to_pgm(wx, wy):
            col = int((wx - origin_x) / resolution)
            rfb = int((wy - origin_y) / resolution)
            return col, height - 1 - rfb

        def pgm_to_world(col, row):
            # Use cell centres (+0.5) to avoid round-trip truncation
            wx = origin_x + (col + 0.5) * resolution
            rfb = height - 1 - row
            wy = origin_y + (rfb + 0.5) * resolution
            return wx, wy

        def is_free(col, row):
            if not (0 <= col < width and 0 <= row < height):
                return False
            p   = pixels[row * width + col]
            occ = 1.0 - (p / maxval) if not negate else p / maxval
            return occ < free_thresh

        # Euclidean distance transform: for every pixel, exact distance in
        # pixels to the nearest occupied cell. Computed once; O(W*H).
        obstacle_mask = np.zeros((height, width), dtype=bool)
        for r in range(height):
            for c in range(width):
                if not is_free(c, r):
                    obstacle_mask[r, c] = True
        dist_px = distance_transform_edt(~obstacle_mask)   # float64 array

        def wall_clearance_px(col, row, _max_check=None):
            """Return exact Euclidean distance in pixels to the nearest wall."""
            return float(dist_px[row, col])

        # Minimum clearance: inflation_radius (0.55 m) + small buffer → 0.65 m
        # so the projected goal is safely outside the inflated lethal zone.
        self._min_clearance_px = math.ceil(0.65 / resolution)

        self._map_meta = {
            'origin_x': origin_x, 'origin_y': origin_y,
            'resolution': resolution, 'width': width, 'height': height,
        }
        self._pgm_is_free        = is_free
        self._pgm_to_world       = pgm_to_world
        self._world_to_pgm       = world_to_pgm
        self._wall_clearance_px  = wall_clearance_px

        sc, sr = world_to_pgm(start_x, start_y)
        if not is_free(sc, sr):
            self.get_logger().error(
                f'Robot start ({start_x},{start_y}) is not free in map — '
                'goal projection disabled.')
            return

        visited = set()
        q = deque([(sc, sr)])
        visited.add((sc, sr))
        while q:
            col, row = q.popleft()
            for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nc, nr = col + dc, row + dr
                if (nc, nr) not in visited and is_free(nc, nr):
                    visited.add((nc, nr))
                    q.append((nc, nr))

        self._reachable = visited
        self.get_logger().info(
            f'Reachable-set built: {len(visited):,} cells '
            f'(robot at {start_x},{start_y})')

    def _project_goal(self, wx: float, wy: float):
        """
        Return (wx, wy) projected to the nearest reachable corridor cell that
        has adequate clearance from walls (>= inflation_radius).

        If the centroid is already reachable AND has clearance, return unchanged.
        Otherwise BFS outward (first through free space, then through walls) to
        find the closest reachable cell with sufficient clearance.
        """
        if self._reachable is None or self._map_meta is None:
            return wx, wy

        res   = self._map_meta['resolution']
        min_c = self._min_clearance_px

        gc, gr = self._world_to_pgm(wx, wy)

        def good_cell(col, row):
            """Reachable AND adequately clear of walls."""
            return ((col, row) in self._reachable and
                    self._wall_clearance_px(col, row, min_c) >= min_c)

        if good_cell(gc, gr):
            return wx, wy   # centroid is already navigable

        max_r = int(10.0 / res)   # search up to 10 m

        # Pass 1: BFS through free space only
        seen  = {(gc, gr)}
        bfs_q = deque([(gc, gr)])
        while bfs_q:
            col, row = bfs_q.popleft()
            dist = abs(col - gc) + abs(row - gr)
            if dist > max_r:
                break
            if good_cell(col, row):
                px, py = self._pgm_to_world(col, row)
                self.get_logger().info(
                    f'Goal projected: ({wx:.2f},{wy:.2f}) → ({px:.2f},{py:.2f}) '
                    f'[{dist * res:.1f} m offset]')
                return px, py
            for dc, dr in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                nc, nr = col + dc, row + dr
                if (nc, nr) not in seen and self._pgm_is_free(nc, nr):
                    seen.add((nc, nr))
                    bfs_q.append((nc, nr))

        # Pass 2: allow crossing walls (for rooms with no PGM door)
        seen2  = {(gc, gr)}
        bfs_q2 = deque([(gc, gr)])
        while bfs_q2:
            col, row = bfs_q2.popleft()
            dist = abs(col - gc) + abs(row - gr)
            if dist > max_r:
                break
            if good_cell(col, row):
                px, py = self._pgm_to_world(col, row)
                self.get_logger().info(
                    f'Goal projected (wall-crossing): ({wx:.2f},{wy:.2f}) → '
                    f'({px:.2f},{py:.2f}) [{dist * res:.1f} m offset]')
                return px, py
            for dc, dr in ((-1,0),(1,0),(0,-1),(0,1)):
                nc, nr = col + dc, row + dr
                if (nc, nr) not in seen2:
                    seen2.add((nc, nr))
                    bfs_q2.append((nc, nr))

        self.get_logger().warn(
            f'Could not project goal ({wx:.2f},{wy:.2f}) to reachable '
            f'navigable space within 10 m.')
        return wx, wy

    # ------------------------------------------------------------------
    # POI loading

    def _load_pois(self, path: str) -> dict:
        """Return dict of {room_number: entity} for the target floor."""
        try:
            with open(path) as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f'semantic.json not found: {path}')
            return {}

        entities = data.get('entities', {})
        if isinstance(entities, list):
            entities = {e['guid']: e for e in entities}

        pois = {}
        for entity in entities.values():
            if entity.get('ifc_type') != 'IfcSpace':
                continue
            if entity.get('storey_name') != self.floor:
                continue
            if not entity.get('has_geometry'):
                continue
            name = entity.get('name') or entity.get('guid', '')[:8]
            pois[name] = entity

        return pois

    def _find_poi(self, query: str):
        """Match query against room number or name (case-insensitive)."""
        q = query.strip()

        if q in self.pois:
            return self.pois[q]

        q_lower = q.lower()
        for name, entity in self.pois.items():
            if str(name).lower() == q_lower:
                return entity

        for entity in self.pois.values():
            id_data = entity.get('properties', {}).get('Identity Data', {})
            display_name = id_data.get('Name', '')
            if q_lower in display_name.lower():
                return entity

        return None

    # ------------------------------------------------------------------
    # Navigation

    def _on_goal_poi(self, msg: String):
        query = msg.data.strip()
        self.get_logger().info(f'Received goal POI: "{query}"')

        entity = self._find_poi(query)
        if entity is None:
            self.get_logger().warn(
                f'No POI found for "{query}". '
                f'Available rooms (sample): {", ".join(list(self.pois.keys())[:10])} …')
            return

        centroid = entity['geometry']['centroid']
        name     = entity.get('name', query)
        id_data  = entity.get('properties', {}).get('Identity Data', {})
        label    = id_data.get('Name', name)

        raw_x, raw_y = centroid[0], centroid[1]
        goal_x, goal_y = self._project_goal(raw_x, raw_y)

        self.get_logger().info(
            f'Navigating to {name} ({label}) '
            f'@ ({goal_x:.2f}, {goal_y:.2f})'
            + (f' [projected from ({raw_x:.2f},{raw_y:.2f})]'
               if (goal_x != raw_x or goal_y != raw_y) else ''))

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateToPose action server not available. Is nav2 running?')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = self.goal_z
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal was rejected by nav2.')
            return
        self.get_logger().info('Goal accepted — robot is navigating.')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f'  Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded — arrived at POI.')
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = POINavNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

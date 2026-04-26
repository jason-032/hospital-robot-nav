#!/usr/bin/env python3
"""
Room Navigation Sweep Test

Iterates through every POI on the configured floor, sends a NavigateToPose
goal for each one, and records pass / fail / timeout to a timestamped CSV.

Prerequisites: launch the full stack first, then run this script in a
second terminal once nav2 reports "Managed nodes are active".

Usage:
  ros2 run spatial_maps sweep_test.py
  ros2 run spatial_maps sweep_test.py --ros-args -p timeout_sec:=90
  ros2 run spatial_maps sweep_test.py --ros-args -p floor:=2F

Output: ~/sweep_<floor>_<timestamp>.csv
"""

import csv
import json
import math
import os
import time
import yaml
from collections import deque
from datetime import datetime

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# ── ANSI colours for the terminal summary ─────────────────────────────────────
_GRN = '\033[92m'
_RED = '\033[91m'
_YLW = '\033[93m'
_RST = '\033[0m'


class SweepTest(Node):

    def __init__(self):
        super().__init__('sweep_test')

        self.declare_parameter('semantic_json',
            '/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json')
        self.declare_parameter('floor', '1F')
        self.declare_parameter('map_yaml',
            os.path.expanduser(
                '~/ros2_ws/install/spatial_maps/share/spatial_maps/maps/1F.yaml'))
        self.declare_parameter('robot_start_x', 21.0)
        self.declare_parameter('robot_start_y', 38.0)
        self.declare_parameter('timeout_sec', 120.0)

        semantic_json = self.get_parameter('semantic_json').value
        self.floor     = self.get_parameter('floor').value
        map_yaml       = self.get_parameter('map_yaml').value
        start_x        = self.get_parameter('robot_start_x').value
        start_y        = self.get_parameter('robot_start_y').value
        self.timeout   = self.get_parameter('timeout_sec').value

        self.pois = self._load_pois(semantic_json)
        self.get_logger().info(f'Loaded {len(self.pois)} POIs for floor {self.floor}')

        self._reachable = None
        self._map_meta  = None
        if os.path.isfile(map_yaml):
            self._build_reachable(map_yaml, start_x, start_y)
        else:
            self.get_logger().warn('map_yaml not found — goal projection disabled')

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # ── Map / reachability (mirrors poi_nav_node) ──────────────────────────────

    def _build_reachable(self, yaml_file, start_x, start_y):
        self.get_logger().info('Building reachable-set …')
        with open(yaml_file) as f:
            meta = yaml.safe_load(f)

        pgm_path    = os.path.join(os.path.dirname(yaml_file), meta['image'])
        resolution  = float(meta['resolution'])
        origin      = meta['origin']
        free_thresh = float(meta.get('free_thresh', 0.196))
        negate      = int(meta.get('negate', 0))

        with open(pgm_path, 'rb') as f:
            magic = f.readline().strip()
            line  = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            width, height = map(int, line.split())
            maxval = int(f.readline().strip())
            raw    = f.read()

        if magic == b'P5':
            pixels = bytearray(raw[:width * height])
        else:
            pixels = bytearray(map(int, raw.split()))

        origin_x, origin_y = float(origin[0]), float(origin[1])

        def world_to_pgm(wx, wy):
            col = int((wx - origin_x) / resolution)
            rfb = int((wy - origin_y) / resolution)
            return col, height - 1 - rfb

        def pgm_to_world(col, row):
            wx = origin_x + (col + 0.5) * resolution
            wy = origin_y + (height - 1 - row + 0.5) * resolution
            return wx, wy

        def is_free(col, row):
            if not (0 <= col < width and 0 <= row < height):
                return False
            p   = pixels[row * width + col]
            occ = 1.0 - p / maxval if not negate else p / maxval
            return occ < free_thresh

        obstacle_mask = np.zeros((height, width), dtype=bool)
        for r in range(height):
            for c in range(width):
                if not is_free(c, r):
                    obstacle_mask[r, c] = True
        dist_px = distance_transform_edt(~obstacle_mask)

        self._min_clearance_px = math.ceil(0.65 / resolution)
        self._map_meta    = {'origin_x': origin_x, 'origin_y': origin_y,
                             'resolution': resolution,
                             'width': width, 'height': height}
        self._is_free     = is_free
        self._pgm_to_world = pgm_to_world
        self._world_to_pgm = world_to_pgm
        self._clearance   = lambda c, r, _=None: float(dist_px[r, c])

        sc, sr = world_to_pgm(start_x, start_y)
        if not is_free(sc, sr):
            self.get_logger().error('Start cell not free — projection disabled')
            return

        visited = set()
        q = deque([(sc, sr)])
        visited.add((sc, sr))
        while q:
            col, row = q.popleft()
            for dc, dr in ((-1,0),(1,0),(0,-1),(0,1)):
                nc, nr = col + dc, row + dr
                if (nc, nr) not in visited and is_free(nc, nr):
                    visited.add((nc, nr))
                    q.append((nc, nr))

        self._reachable = visited
        self.get_logger().info(f'Reachable-set: {len(visited):,} cells')

    def _project_goal(self, wx, wy):
        if self._reachable is None:
            return wx, wy
        res   = self._map_meta['resolution']
        min_c = self._min_clearance_px
        gc, gr = self._world_to_pgm(wx, wy)

        def good(col, row):
            return ((col, row) in self._reachable and
                    self._clearance(col, row) >= min_c)

        if good(gc, gr):
            return wx, wy

        max_r = int(10.0 / res)

        # Pass 1: free space only
        seen  = {(gc, gr)}
        bfs_q = deque([(gc, gr)])
        while bfs_q:
            col, row = bfs_q.popleft()
            if abs(col - gc) + abs(row - gr) > max_r:
                break
            if good(col, row):
                return self._pgm_to_world(col, row)
            for dc, dr in ((-1,0),(1,0),(0,-1),(0,1),
                           (-1,-1),(-1,1),(1,-1),(1,1)):
                nc, nr = col + dc, row + dr
                if (nc, nr) not in seen and self._is_free(nc, nr):
                    seen.add((nc, nr))
                    bfs_q.append((nc, nr))

        # Pass 2: allow crossing walls
        seen2  = {(gc, gr)}
        bfs_q2 = deque([(gc, gr)])
        while bfs_q2:
            col, row = bfs_q2.popleft()
            if abs(col - gc) + abs(row - gr) > max_r:
                break
            if good(col, row):
                return self._pgm_to_world(col, row)
            for dc, dr in ((-1,0),(1,0),(0,-1),(0,1)):
                nc, nr = col + dc, row + dr
                if (nc, nr) not in seen2:
                    seen2.add((nc, nr))
                    bfs_q2.append((nc, nr))

        return wx, wy  # fallback: original centroid

    # ── POI loading ────────────────────────────────────────────────────────────

    def _load_pois(self, path):
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

    # ── Navigation helper ──────────────────────────────────────────────────────

    def _navigate_blocking(self, goal_x, goal_y):
        """Send one goal and block until success/failure/timeout.
        Returns (result_str, notes_str)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self._nav.send_goal_async(goal_msg)

        # Wait for acceptance (max 10 s)
        t0 = time.time()
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 10.0:
                return 'TIMEOUT', 'goal accept timed out'

        handle = send_future.result()
        if not handle.accepted:
            return 'REJECTED', 'nav2 rejected goal (likely occupied cell)'

        result_future = handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > self.timeout:
                handle.cancel_goal_async()
                # drain the cancel
                t1 = time.time()
                while not result_future.done() and time.time() - t1 < 3.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                return 'TIMEOUT', f'exceeded {self.timeout:.0f} s'

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCESS', ''
        elif status == GoalStatus.STATUS_CANCELED:
            return 'CANCELLED', 'goal was cancelled'
        else:
            return 'FAILED', f'nav2 status={status}'

    # ── Main sweep ─────────────────────────────────────────────────────────────

    def run(self):
        rooms = list(self.pois.items())
        total = len(rooms)

        if not self._nav.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'NavigateToPose action server not available — is nav2 running?')
            return

        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.expanduser(f'~/sweep_{self.floor}_{ts}.csv')
        self.get_logger().info(
            f'\n{"="*60}\nStarting sweep: {total} rooms  |  timeout {self.timeout:.0f}s/room'
            f'\nResults → {csv_path}\n{"="*60}')

        counts = {'SUCCESS': 0, 'FAILED': 0, 'REJECTED': 0,
                  'TIMEOUT': 0, 'CANCELLED': 0}

        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['#', 'room', 'display_name',
                        'goal_x', 'goal_y', 'projected',
                        'result', 'duration_s', 'notes'])

            for idx, (name, entity) in enumerate(rooms, 1):
                centroid = entity['geometry']['centroid']
                id_data  = entity.get('properties', {}).get('Identity Data', {})
                label    = id_data.get('Name', name)

                raw_x, raw_y   = centroid[0], centroid[1]
                goal_x, goal_y = self._project_goal(raw_x, raw_y)
                projected      = goal_x != raw_x or goal_y != raw_y

                print(f'[{idx:>3}/{total}] {name:<12} {label:<30} '
                      f'→ ({goal_x:.2f}, {goal_y:.2f})'
                      + (' [proj]' if projected else ''),
                      end='  ', flush=True)

                t_start          = time.time()
                result, notes    = self._navigate_blocking(goal_x, goal_y)
                duration         = time.time() - t_start

                counts[result] = counts.get(result, 0) + 1

                colour = _GRN if result == 'SUCCESS' else (
                         _YLW if result in ('TIMEOUT', 'CANCELLED') else _RED)
                print(f'{colour}{result}{_RST} ({duration:.1f}s)'
                      + (f'  {notes}' if notes else ''))

                w.writerow([idx, name, label,
                            f'{goal_x:.3f}', f'{goal_y:.3f}', projected,
                            result, f'{duration:.1f}', notes])
                f.flush()

        # ── Summary ────────────────────────────────────────────────────────────
        print(f'\n{"="*60}')
        print(f'Sweep complete  —  floor {self.floor}  —  {total} rooms')
        print(f'  {_GRN}SUCCESS  {counts["SUCCESS"]:>3}{_RST}')
        print(f'  {_RED}FAILED   {counts["FAILED"]:>3}{_RST}')
        print(f'  {_RED}REJECTED {counts.get("REJECTED",0):>3}{_RST}')
        print(f'  {_YLW}TIMEOUT  {counts["TIMEOUT"]:>3}{_RST}')
        print(f'  {_YLW}CANCELLED{counts.get("CANCELLED",0):>3}{_RST}')
        print(f'{"="*60}')
        print(f'CSV saved to: {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SweepTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

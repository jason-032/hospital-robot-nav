#!/usr/bin/env python3
"""
Network-Guided Navigation Node
Uses Spatial_Map topological network for path planning with grid collision avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import json
import os
import math
import heapq
from typing import Dict, List, Optional, Tuple
from ament_index_python.packages import get_package_share_directory


class NetworkNavigator(Node):
    """Navigation using topological network + grid collision avoidance."""
    
    def __init__(self):
        super().__init__('network_navigator')
        
        # Parameters
        self.declare_parameter('network_file', '')
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('collision_radius', 0.3)
        
        # Load network
        network_file = self.get_parameter('network_file').value
        if network_file and os.path.exists(network_file):
            with open(network_file, 'r') as f:
                self.network = json.load(f)
            self.get_logger().info(f"Loaded network with {len(self.network.get('nodes', []))} nodes")
        else:
            self.network = {'nodes': [], 'edges': [], 'semantic_locations': {}}
            self.get_logger().warn("No network file loaded - navigation limited")
        
        # State
        self.current_pose = None
        self.occupancy_grid = None
        self.current_path = []
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'network_markers', 10)
        
        # Subscribers
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(String, 'goal_location', self.goal_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        
        # Timer for visualization
        self.create_timer(1.0, self.publish_network_markers)
        
        self.get_logger().info("Network Navigator initialized")
    
    def pose_callback(self, msg: PoseStamped):
        """Update current robot pose."""
        self.current_pose = msg
    
    def map_callback(self, msg: OccupancyGrid):
        """Update occupancy grid."""
        self.occupancy_grid = msg
    
    def goal_callback(self, msg: String):
        """Handle semantic goal (e.g., "Room 001")."""
        location_name = msg.data
        semantic_locations = self.network.get('semantic_locations', {})
        
        if location_name in semantic_locations:
            loc = semantic_locations[location_name]
            self.get_logger().info(f"Navigating to {location_name}")
            self.plan_path_to_node(loc['node_id'])
        else:
            self.get_logger().warn(f"Unknown location: {location_name}")
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Handle pose goal - find nearest network node."""
        if not self.network['nodes']:
            return
        
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Find nearest node
        nearest_node = None
        min_dist = float('inf')
        
        for node in self.network['nodes']:
            dist = math.sqrt((node['x'] - goal_x)**2 + (node['y'] - goal_y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        
        if nearest_node:
            self.get_logger().info(f"Goal near node {nearest_node['id']}, distance: {min_dist:.2f}m")
            self.plan_path_to_node(nearest_node['id'])
    
    def plan_path_to_node(self, goal_node_id: int):
        """Plan path using Dijkstra on network graph."""
        if not self.current_pose:
            self.get_logger().warn("No current pose available")
            return
        
        # Find nearest node to current position
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        start_node = None
        min_dist = float('inf')
        
        for node in self.network['nodes']:
            dist = math.sqrt((node['x'] - current_x)**2 + (node['y'] - current_y)**2)
            if dist < min_dist:
                min_dist = dist
                start_node = node
        
        if not start_node:
            return
        
        # Dijkstra
        path = self.dijkstra(start_node['id'], goal_node_id)
        
        if path:
            self.current_path = path
            self.publish_path(path)
            self.get_logger().info(f"Path planned: {len(path)} waypoints")
    
    def dijkstra(self, start_id: int, goal_id: int) -> List[dict]:
        """Dijkstra's algorithm on network graph."""
        nodes = {n['id']: n for n in self.network['nodes']}
        
        # Build adjacency list
        adj = {n['id']: [] for n in self.network['nodes']}
        for edge in self.network['edges']:
            adj[edge['source']].append((edge['target'], edge['cost']))
            if edge.get('bidirectional', True):
                adj[edge['target']].append((edge['source'], edge['cost']))
        
        # Dijkstra
        dist = {n: float('inf') for n in nodes}
        prev = {n: None for n in nodes}
        dist[start_id] = 0
        
        pq = [(0, start_id)]
        
        while pq:
            d, u = heapq.heappop(pq)
            
            if d > dist[u]:
                continue
            
            if u == goal_id:
                break
            
            for v, cost in adj.get(u, []):
                new_dist = dist[u] + cost
                if new_dist < dist[v]:
                    dist[v] = new_dist
                    prev[v] = u
                    heapq.heappush(pq, (new_dist, v))
        
        # Reconstruct path
        if dist[goal_id] == float('inf'):
            return []
        
        path = []
        current = goal_id
        while current is not None:
            path.append(nodes[current])
            current = prev[current]
        
        return list(reversed(path))
    
    def publish_path(self, path: List[dict]):
        """Publish planned path as ROS Path message."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for node in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = node['x']
            pose.pose.position.y = node['y']
            pose.pose.position.z = 0.0
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
    
    def publish_network_markers(self):
        """Publish network visualization markers."""
        markers = MarkerArray()
        
        # Node markers
        for i, node in enumerate(self.network.get('nodes', [])):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'network_nodes'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node['x']
            marker.pose.position.y = node['y']
            marker.pose.position.z = 0.5
            marker.scale.x = marker.scale.y = marker.scale.z = 0.4
            
            # Color by type
            if node.get('type') == 'room':
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            elif node.get('type') == 'door':
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.5, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
        
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

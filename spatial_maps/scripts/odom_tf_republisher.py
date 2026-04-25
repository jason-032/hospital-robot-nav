#!/usr/bin/env python3
"""
Reads /odom and re-publishes odom → base_footprint TF
stamped with the current clock (not the stale Gz Sim timestamp).
This eliminates TF_OLD_DATA warnings caused by ros_gz_bridge latency.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFRepublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_republisher')
        self._br = TransformBroadcaster(self)
        self._last_stamp_ns = -1  # sentinel: publish first message unconditionally
        self._sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)
        self.get_logger().info('odom_tf_republisher ready')

    def _on_odom(self, msg: Odometry):
        now = self.get_clock().now()
        # Only publish when the clock has actually advanced — prevents TF_OLD_DATA
        # floods caused by multiple callbacks firing within the same sim clock tick.
        if now.nanoseconds <= self._last_stamp_ns:
            return
        self._last_stamp_ns = now.nanoseconds

        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = msg.header.frame_id   # odom
        t.child_frame_id  = msg.child_frame_id    # base_footprint
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation      = p.orientation
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdomTFRepublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

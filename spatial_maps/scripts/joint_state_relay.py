#!/usr/bin/env python3
"""
Joint State Relay

Receives /joint_states_gz from the Gz bridge (best_effort, Gz-timestamped)
and republishes to /joint_states (reliable, current-clock timestamp).

robot_state_publisher subscribes to /joint_states and uses its header stamp
to publish TF. By re-stamping with now(), the wheel and arm link TF stays
fresh — fixing the RViz "No transform" errors for those links without
causing TF_OLD_DATA floods.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState


class JointStateRelay(Node):

    def __init__(self):
        super().__init__('joint_state_relay')

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable    = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._pub = self.create_publisher(JointState, '/joint_states', reliable)
        self._last_stamp_ns = -1  # sentinel: publish first message unconditionally

        self.create_subscription(
            JointState, '/joint_states_gz', self._relay, best_effort)

        self.get_logger().info('joint_state_relay ready')

    def _relay(self, msg: JointState):
        now = self.get_clock().now()
        # Only forward when clock has advanced — prevents non-monotonic re-stamps
        # from causing TF_OLD_DATA floods in RSP's joint TF output.
        if now.nanoseconds <= self._last_stamp_ns:
            return
        self._last_stamp_ns = now.nanoseconds
        msg.header.stamp = now.to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

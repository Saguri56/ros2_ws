#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridgeNode(Node):
    def __init__(self):
        super().__init__('twist_bridge_node')

        # Publisher a TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/husky_velocity_controller/cmd_vel', 10)

        # Subscriber a Twist
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: Twist):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        self.publisher_.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.sub = self.create_subscription(
            Odometry,
            '/husky_velocity_controller/odom',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

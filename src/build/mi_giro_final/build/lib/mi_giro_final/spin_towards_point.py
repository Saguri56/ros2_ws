#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
import math

class SpinTowardsPoint(Node):
    def __init__(self):
        super().__init__('spin_towards_point')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.spin_client = ActionClient(self, Spin, 'spin')

        # Coordenadas del objeto a mirar (MODIFICA AQUÍ)
        self.target_x = 3.38
        self.target_y = 4.0

        self.timer = self.create_timer(1.0, self.run_once)

    def run_once(self):
        self.timer.cancel()

        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            q = trans.transform.rotation
            _, _, robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            dx = self.target_x - robot_x
            dy = self.target_y - robot_y
            desired_yaw = math.atan2(dy, dx)

            delta_yaw = desired_yaw - robot_yaw
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))  # normaliza

            self.get_logger().info(f"Rotando {delta_yaw:.2f} rad hacia el objetivo.")

            self.spin_client.wait_for_server()
            goal_msg = Spin.Goal()
            goal_msg.target_yaw = delta_yaw
            goal_msg.time_allowance.sec = 10

            self.spin_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

        except Exception as e:
            self.get_logger().error(f"Error obteniendo TF: {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('¡El objetivo de giro fue rechazado!')
            return

        self.get_logger().info('Giro iniciado...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('✅ ¡Giro completado!')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SpinTowardsPoint()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


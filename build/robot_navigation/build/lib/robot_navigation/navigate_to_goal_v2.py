#!/usr/bin/env python3
#Algoritmo paralelo
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from std_srvs.srv import Trigger
import math
import time
import threading
from rclpy.executors import SingleThreadedExecutor

class NavigateAndPick(Node):
    def __init__(self):
        super().__init__('navigate_and_pick')
        self.get_logger().info("Nodo Navigate iniciado.")

        # Inicializar Nav2
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 activo.")

        # Llamar al servicio del brazo en paralelo
        threading.Thread(target=self.call_arm_service).start()

        # Enviar objetivo de navegación
        self.send_navigation_goal(4.8,2.1,-10.0)

        # Esperar a que se alcance el objetivo
        while not self.navigator.isTaskComplete():
            self.get_logger().info("Navegando hacia el objetivo...")
            time.sleep(1.0)

        self.get_logger().info("Objetivo de navegación alcanzado.")

    def send_navigation_goal(self, x, y, theta_degrees):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        theta_radians = math.radians(theta_degrees)
        q = quaternion_from_euler(0, 0, theta_radians)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.get_logger().info(f"Enviando objetivo de navegación a ({x}, {y}, {theta_degrees}°)...")
        self.navigator.goToPose(goal_pose)

    def call_arm_service(self):
        client = self.create_client(Trigger, 'activate_arm')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando el servicio del brazo...')
        request = Trigger.Request()
        future = client.call_async(request)
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        if future.result() is not None and future.result().success:
            self.get_logger().info('El brazo se ha activado correctamente.')
        else:
            self.get_logger().error('Fallo al activar el brazo.')


def main(args=None):
    rclpy.init(args=args)
    node = NavigateAndPick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

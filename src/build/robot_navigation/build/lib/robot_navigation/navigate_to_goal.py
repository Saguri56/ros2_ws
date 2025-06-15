#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time

# Importaciones para MoveIt2
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
import moveit_commander

class NavigateAndPick(Node):
    def __init__(self):
        super().__init__('navigate_and_pick')
        self.get_logger().info("🚀 Nodo NavigateAndPick iniciado.")

        # Inicializar MoveIt2
        moveit_commander.roscpp_initialize([])
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = MoveGroupCommander(self.group_name)

        # Inicializar Nav2
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("🟢 Nav2 activo.")

        # Enviar objetivo de navegación
        self.send_navigation_goal(3.9, -1.1, -90)

        # Esperar a que se alcance el objetivo
        while not self.navigator.isTaskComplete():
            self.get_logger().info("⏳ Navegando hacia el objetivo...")
            time.sleep(1.0)

        self.get_logger().info("✅ Objetivo de navegación alcanzado.")

        # Mover el brazo a la pose 'pick'
        self.move_arm_to_pick()

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

        self.get_logger().info(f"🧭 Enviando objetivo de navegación a ({x}, {y}, {theta_degrees}°)...")
        self.navigator.goToPose(goal_pose)

    def move_arm_to_pick(self):
        self.get_logger().info("🤖 Moviendo el brazo a la pose 'pick'...")

        # Establecer la pose objetivo como 'pick'
        self.move_group.set_named_target("pick")

        # Planificar y ejecutar el movimiento
        plan = self.move_group.plan()
        if plan:
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.get_logger().info("✅ Movimiento del brazo completado.")
        else:
            self.get_logger().error("❌ No se pudo planificar el movimiento del brazo.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigateAndPick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

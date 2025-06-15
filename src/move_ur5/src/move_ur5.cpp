#include <memory>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_ur5",
    rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
        .append_parameter_override("use_sim_time", true)
  );
  auto const logger = rclcpp::get_logger("move_ur5");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = std::make_shared<MoveGroupInterface>(node, "ur5_arm");

  // Crear el servicio
  auto service = node->create_service<std_srvs::srv::Trigger>(
    "activate_arm",
    [move_group_interface, logger](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      RCLCPP_INFO(logger, "Servicio recibido: activando el brazo.");

      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = 0.707;
      target_pose.position.y = 0.0;
      target_pose.position.z = 0.4;

      tf2::Quaternion q;
      q.setRPY(0, M_PI, 0);
      target_pose.orientation = tf2::toMsg(q);

      move_group_interface->setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_interface->plan(plan));

      if (success)
      {
        move_group_interface->execute(plan);
        response->success = true;
        response->message = "Movimiento del brazo completado.";
        RCLCPP_INFO(logger, "Brazo movido correctamente.");
      }
      else
      {
        response->success = false;
        response->message = "Falló la planificación del brazo.";
        RCLCPP_ERROR(logger, "Falló el movimiento del brazo.");
      }
    }
  );

  RCLCPP_INFO(logger, "Servicio 'activate_arm' listo y esperando llamadas...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
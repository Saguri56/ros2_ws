#include <memory>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class AlgrtSecuencialNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  AlgrtSecuencialNode() : Node("algrt_secuencial")
  {
    this->declare_parameter("use_sim_time", true);
    this->get_logger().info("Nodo algrt_secuencial iniciado.");

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    while (!nav_action_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(this->get_logger(), "Esperando al servidor de acciones de navegación...");
    }

    send_navigation_goal();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

  void send_navigation_goal()
  {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose.position.x = 3.9;
    goal_msg.pose.pose.position.y = 1.1;

    tf2::Quaternion q;
    q.setRPY(0, 0, -125.0 * M_PI / 180.0);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "Enviando objetivo de navegación...");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&AlgrtSecuencialNode::nav_done_callback, this, _1);

    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void nav_done_callback(const GoalHandleNav::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navegación completada. Iniciando movimiento del brazo...");
      execute_arm_task();
    } else {
      RCLCPP_ERROR(this->get_logger(), "La navegación falló.");
    }
  }

  void execute_arm_task()
  {
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group_interface(shared_from_this(), "ur5_arm");

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.707;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.1;

    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    target_pose.orientation = tf2::toMsg(q);

    move_group_interface.setPoseTarget(target_pose);

    MoveGroupInterface::Plan plan;
    if (move_group_interface.plan(plan)) {
      move_group_interface.execute(plan);

      std::string ee_link = move_group_interface.getEndEffectorLink();
      tf2_ros::Buffer tf_buffer(this->get_clock());
      tf2_ros::TransformListener tf_listener(tf_buffer);

      geometry_msgs::msg::PointStamped relative_point;
      relative_point.header.frame_id = ee_link;
      relative_point.header.stamp = rclcpp::Time(0);
      relative_point.point.x = 0.0;
      relative_point.point.y = 0.0;
      relative_point.point.z = -0.1;

      geometry_msgs::msg::PointStamped world_point;
      try {
        world_point = tf_buffer.transform(relative_point, "world", tf2::durationFromSec(2.0));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Error al transformar: %s", ex.what());
        return;
      }

      double x = world_point.point.x;
      double y = world_point.point.y;
      double z = world_point.point.z;

      RCLCPP_INFO(this->get_logger(), "Brazo posicionado. Spawneando bloque en: x=%.3f, y=%.3f, z=%.3f", x, y, z);

      std::ostringstream cmd;
      cmd << "ros2 run gazebo_ros spawn_entity.py "
          << "-entity graspable_block "
          << "-database graspable_block "
          << "-x " << x << " -y " << y << " -z " << z;

      int ret = std::system(cmd.str().c_str());
      if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error al spawnear graspable_block");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Falló la planificación del brazo.");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlgrtSecuencialNode>());
  rclcpp::shutdown();
  return 0;
}


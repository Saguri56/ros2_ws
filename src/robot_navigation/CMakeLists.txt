find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_to_pick_node src/move_to_pick_node.cpp)
ament_target_dependencies(move_to_pick_node
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS
  move_to_pick_node
  DESTINATION lib/${PROJECT_NAME}
)

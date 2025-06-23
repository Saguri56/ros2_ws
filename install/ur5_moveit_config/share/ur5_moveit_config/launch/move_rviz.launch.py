import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz config file name"
        ),
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("ur5",package_name="ur5_moveit_config")
        .robot_description(file_path="config/husky_ur5_gazebo.urdf.xacro")
        .robot_description_semantic(file_path="config/husky_ur5_gazebo.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    use_sim_time = {"use_sim_time": True}

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("ur5_moveit_config"),
        "launch",
        LaunchConfiguration("rviz_config")
    ])

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), use_sim_time]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                use_sim_time,
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description, use_sim_time]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_world_to_base",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
        )
    ]

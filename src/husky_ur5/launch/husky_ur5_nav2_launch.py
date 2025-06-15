import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    husky_ur5_pkg = FindPackageShare("husky_ur5")

    # Parámetros
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    params_file = PathJoinSubstitution([husky_ur5_pkg, "config", "nav2_params.yaml"])
    map_file = PathJoinSubstitution([husky_ur5_pkg, "maps", "map.yaml"])

    # Lanzar Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([husky_ur5_pkg, "launch", "gazebo_launch.py"])
        ),
        launch_arguments={"world_path": "empty_world.sdf"}.items(),
    )

    # Lanzar Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "map": map_file,
        }.items(),
    )

    return LaunchDescription([gazebo_launch, nav2_launch])
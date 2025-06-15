from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import SetParameter
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("husky_ur5_gazebo", package_name="husky_ur5_moveit")
        .to_moveit_configs()
    )

    return LaunchDescription([
        # 👇 Esto fuerza a todos los nodos ROS 2 de este launch a usar tiempo simulado
        SetParameter(name="use_sim_time", value=True),
        generate_move_group_launch(moveit_config)
    ])
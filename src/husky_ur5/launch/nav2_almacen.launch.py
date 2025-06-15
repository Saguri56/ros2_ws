import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('husky_ur5'),
            'maps',
            'almacen_map.yaml'))

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('husky_ur5'),
            'config',
            'nav2_params.yaml'))

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to the map file'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the Nav2 parameter file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz automatically'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen')
    ])
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(
        get_package_share_directory('aws_robomaker_small_warehouse_world'),
        'worlds',
        'small_warehouse',
        'small_warehouse.world'
    )
    model_path = os.path.expanduser('~/.gazebo/models')

    return LaunchDescription([
        # Set Gazebo model path
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),

        # Launch gzserver with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),
    ])

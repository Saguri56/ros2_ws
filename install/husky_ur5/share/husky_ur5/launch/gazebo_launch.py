from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction 

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import launch_ros
import os

def launch_setup(context, *args, **kwargs):

    # Declaración de argumentos**
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    
    disable_shm = SetEnvironmentVariable(
    name="FASTRTPS_TRANSPORT_SHM_DISABLE",
    value="1"
    )   
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    # Configuraración del entorno
    gz_resource_path = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=[
        EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        "/usr/share/gazebo-11/models/:",
        str(Path(get_package_share_directory("husky_description")).parent.resolve()),
    ])
    almacen_world = PathJoinSubstitution([
        FindPackageShare("husky_ur5"),
        "worlds",
        "almacen.world"
    ])

    # Preparación del modelo del robot
    husky_ur5_package = FindPackageShare("husky_ur5").find("husky_ur5")
    controllers_file = PathJoinSubstitution(
        [husky_ur5_package, "config", "husky_ur5_controllers.yaml"]
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([husky_ur5_package, "urdf", "husky_ur5_gazebo.urdf.xacro"]),
            " ",
            "is_sim:=true",
            " ",
            "prefix:=husky_",
            " ",
            "ur_type:=", ur_type,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "sim_gazebo:=true",
            " ",
            "sim_ignition:=false",
            " ",
            "laser_enabled:=true",
            " ",
            "use_fake_hardware:=true", 
            " ",
            "sim_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodo para la publicación de las TF del robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Nodo ros2_control para gestionar los controladores
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_file,
            {'use_sim_time': True},
        ],
        output="screen",
    )

    #Lanzamiento de controladores
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen',
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["husky_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_ur5_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )

    spawn_ur5_controller_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    spawn_robotiq_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    velocity_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[spawn_husky_velocity_controller],
        )
    )

    ur5_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_husky_velocity_controller,
            on_exit=[spawn_ur5_controller, spawn_ur5_controller_stopped,spawn_robotiq_gripper_controller],
        )
    )

    # Lanzamiento de Gazebo
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
            almacen_world],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )


    # Spawn del robot en Gazebo
    spawn_robot = TimerAction(
        period=5.0,  
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_husky_ur5",
                arguments=[
                    "-entity", "husky_ur5",
                    "-topic", "robot_description",
                     "-z", "0.4",
                ],
                remappings=[
                    ("/base_laser_plugin/out", "/scan")
                ],
                output="screen",
            )
        ]   
    )   

    #Lanzamiento de teleop_base para control de Husky por teclado
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))


    # Lista de nodos a iniciar
    nodes_to_start = [
        disable_shm,
        gz_resource_path,
        gzserver,
        gzclient,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        velocity_controller_callback,
        ur5_controller_callback,
        spawn_robot,
        launch_husky_teleop_base,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e", choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
                              description="Type/series of used UR robot."),
        DeclareLaunchArgument("safety_limits", default_value="true",
                              description="Enables the safety limits controller if true."),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15",
                              description="The margin to lower and upper limits in the safety controller."),
        DeclareLaunchArgument("safety_k_position", default_value="20",
                              description="k-position factor in the safety controller."),
        DeclareLaunchArgument("description_package", default_value="husky_ur5",
                              description="Description package with robot URDF/XACRO files."),
        DeclareLaunchArgument("description_file", default_value="husky_ur5_gazebo.urdf.xacro",
                              description="URDF/XACRO description file with the robot."),
        DeclareLaunchArgument("prefix", default_value='""',
                              description="Prefix for the joint names."),
        DeclareLaunchArgument("start_joint_controller", default_value="true",
                              description="Start the joint trajectory controller."),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller",
                              description="Initial robot controller to start."),
        DeclareLaunchArgument("gazebo_gui", default_value="true",
                              description="Start Gazebo with GUI."),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


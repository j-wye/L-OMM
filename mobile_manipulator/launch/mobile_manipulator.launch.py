import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    TimerAction, ExecuteProcess, RegisterEventHandler,
    DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
)
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    port = LaunchConfiguration('port')
    port_realtime = LaunchConfiguration('port_realtime')
    session_inactivity_timeout_ms = LaunchConfiguration('session_inactivity_timeout_ms')
    connection_inactivity_timeout_ms = LaunchConfiguration('connection_inactivity_timeout_ms')
    use_internal_bus_gripper_comm = LaunchConfiguration('use_internal_bus_gripper_comm')
    gripper_joint_name = LaunchConfiguration('gripper_joint_name')
    gripper_max_velocity = LaunchConfiguration('gripper_max_velocity')
    gripper_max_force = LaunchConfiguration('gripper_max_force')
    gripper_com_port = LaunchConfiguration('gripper_com_port')
    initial_value = LaunchConfiguration('initial_value')
    
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(description_package), 'urdf', description_file]
            ), " ",
            "robot_ip:=", robot_ip, " ",
            "username:=", username, " ",
            "password:=", password, " ",
            "port:=", port, " ",
            "port_realtime:=", port_realtime, " ",
            "session_inactivity_timeout_ms:=", session_inactivity_timeout_ms, " ",
            "connection_inactivity_timeout_ms:=", connection_inactivity_timeout_ms, " ",
            "use_internal_bus_gripper_comm:=", use_internal_bus_gripper_comm, " ",
            "gripper_joint_name:=", gripper_joint_name, " ",
            "gripper_max_velocity:=", gripper_max_velocity, " ",
            "gripper_max_force:=", gripper_max_force, " ",
            "gripper_com_port:=", gripper_com_port, " ",
            "initial_value:=", initial_value, " ",
            "controllers_file:=", controllers_file, " ",
            "description_package:=", description_package, " ",
            "description_file:=", description_file, " ",
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(description_package), 'config', controllers_file
    ])

    tracer_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('tracer_base'),
            '/launch/tracer_mini_base.launch.py'
        ]),
        launch_arguments={
            'port_name': 'can0',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic_name': 'odom',
            'is_tracer_mini': 'true',
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('sllidar_ros2'),
            '/launch/sllidar_a2m8_launch.py'
        ]),
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        # output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        # output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'mobile_manipulator.rviz'])],
        condition=IfCondition(launch_rviz)
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    nodes_to_start = [
        tracer_base_launch,
        lidar_launch,
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        delayed_rviz,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # Immutable Variables
    declared_arguments.append(DeclareLaunchArgument("robot_ip", default_value="192.168.1.10"))
    declared_arguments.append(DeclareLaunchArgument("username", default_value="admin"))
    declared_arguments.append(DeclareLaunchArgument("password", default_value="admin"))
    declared_arguments.append(DeclareLaunchArgument("port", default_value="10000"))
    declared_arguments.append(DeclareLaunchArgument("port_realtime", default_value="10001"))
    declared_arguments.append(DeclareLaunchArgument("session_inactivity_timeout_ms", default_value="60000"))
    declared_arguments.append(DeclareLaunchArgument("connection_inactivity_timeout_ms", default_value="2000"))
    declared_arguments.append(DeclareLaunchArgument("use_internal_bus_gripper_comm", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("gripper_joint_name", default_value="right_finger_bottom_joint"))
    declared_arguments.append(DeclareLaunchArgument("gripper_max_velocity", default_value="100.0"))
    declared_arguments.append(DeclareLaunchArgument("gripper_max_force", default_value="100.0"))
    declared_arguments.append(DeclareLaunchArgument("gripper_com_port", default_value="/dev/ttyUSB0"))

    # Initial Value
    declared_arguments.append(DeclareLaunchArgument("initial_value", default_value="initial_value.yaml"))

    # Controller File
    declared_arguments.append(DeclareLaunchArgument("controllers_file", default_value="ros2_controller.yaml"))

    # Description Package
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mobile_manipulator",
        )
    )
    # Description File
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="integrate_execution.xacro",
        )
    )
    # Launch Rviz
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true"))
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
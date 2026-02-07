from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    model = LaunchDescription.declare_argument(
        'model',
        default_value='',
        description='Optional model name for further customization'
    )

    robot_description_cmd = [
        'xacro',
        '(find tracer_description)/urdf/tracer_v1.xacro',
        'robot_namespace:=$(arg model)'  # Optional namespace based on model argument
    ]

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_cmd}
            ]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',  # Use rviz2 for ROS 2 compatibility
            executable='rviz2',
            name='rviz',
            arguments=[
                '-d',  # Flag for RViz configuration file
                f'$(find tracer_description)/rviz/model_display.rviz'  # Path to RViz configuration file
            ]
        )
    ])

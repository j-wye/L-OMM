from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_namespace = LaunchDescription.declare_argument(
        'robot_namespace',
        default_value='/',
        description='Namespace for the robot'
    )

    urdf_extras = LaunchDescription.declare_argument(
        'urdf_extras',
        default_value="$(find tracer_description)/urdf/empty.urdf",
        description='Path to additional URDF elements'
    )

    robot_description_cmd = [
        'xacro',
        '(find tracer_description)/urdf/tracer_v1.xacro',
        'robot_namespace:=$(arg robot_namespace)',
        'urdf_extras:=$(arg urdf_extras)'
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
        )
    ])

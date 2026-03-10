from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('r2_decision_ros2'),
        'config',
        'params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='r2_decision_ros2',
            executable='launch_manager_node',
            name='launch_manager_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='r2_decision_ros2',
            executable='safety_guard_node',
            name='safety_guard_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='r2_decision_ros2',
            executable='mission_state_node',
            name='mission_state_node',
            output='screen',
            parameters=[params],
        ),
    ])

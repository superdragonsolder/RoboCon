from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    R2决策系统 - 仅路径规划测试
    使用场景: 测试梅林路径规划算法，不启动完整系统
    """
    
    params = PathJoinSubstitution([
        FindPackageShare('r2_decision_ros2'),
        'config',
        'params.yaml',
    ])

    return LaunchDescription([
        # 仅启动任务状态机节点（梅林模式）
        Node(
            package='r2_decision_ros2',
            executable='mission_state_node',
            name='mission_state_node',
            output='screen',
            parameters=[
                params,
                {
                    'launch_mode': 'test_path_planning',
                    'use_sim_hardware': True,
                    'skip_to_meilin': True,  # 直接跳到梅林阶段
                }
            ],
        ),
    ])

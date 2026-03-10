from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    R2决策系统 - 返回梅林模式
    使用场景: 梅林阶段KFS收集不足，需要返回梅林区域补充
    """
    
    params = PathJoinSubstitution([
        FindPackageShare('r2_decision_ros2'),
        'config',
        'params.yaml',
    ])

    return LaunchDescription([
        # 启动模式参数
        DeclareLaunchArgument(
            'use_sim_hardware',
            default_value='true',
            description='Use simulated hardware (true) or real hardware (false)'
        ),
        
        # 启动管理节点
        Node(
            package='r2_decision_ros2',
            executable='launch_manager_node',
            name='launch_manager_node',
            output='screen',
            parameters=[
                params,
                {
                    'cold_start': False,
                    'rebuild_weapon': False,
                    'recollect_mf_kfs': True,
                    'use_sim_hardware': LaunchConfiguration('use_sim_hardware'),
                }
            ],
        ),
        
        # 安全监护节点
        Node(
            package='r2_decision_ros2',
            executable='safety_guard_node',
            name='safety_guard_node',
            output='screen',
            parameters=[params],
        ),
        
        # 任务状态机节点
        Node(
            package='r2_decision_ros2',
            executable='mission_state_node',
            name='mission_state_node',
            output='screen',
            parameters=[
                params,
                {}
            ],
        ),
    ])

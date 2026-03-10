from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    R2决策系统 - 真实硬件模式
    使用场景: 实际比赛，连接真实硬件设备
    """
    
    params = PathJoinSubstitution([
        FindPackageShare('r2_decision_ros2'),
        'config',
        'params.yaml',
    ])

    return LaunchDescription([
        # 启动模式参数
        DeclareLaunchArgument(
            'cold_start',
            default_value='true',
            description='Cold start mode flag'
        ),
        DeclareLaunchArgument(
            'rebuild_weapon',
            default_value='false',
            description='Return-to-MC rebuild weapon flag'
        ),
        DeclareLaunchArgument(
            'recollect_mf_kfs',
            default_value='false',
            description='Return-to-MF recollect KFS flag'
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
                    'cold_start': LaunchConfiguration('cold_start'),
                    'rebuild_weapon': LaunchConfiguration('rebuild_weapon'),
                    'recollect_mf_kfs': LaunchConfiguration('recollect_mf_kfs'),
                    'use_sim_hardware': False,  # 强制使用真实硬件
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

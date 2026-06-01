#!/usr/bin/env python3
"""
R2 键盘遥控 Launch 文件

用法:
  ros2 launch r2_communication keyboard_control.launch.py

参数（可在命令行覆盖）:
  port         串口设备路径，默认 /dev/ttyACM0
  baudrate     波特率，默认 115200
  linear_step  线速度步长 (m/s)，默认 0.5
  angular_step 角速度步长 (rad/s)，默认 0.5

示例:
  ros2 launch r2_communication keyboard_control.launch.py port:=/dev/ttyUSB1 linear_step:=0.3

注意:
  键盘输入必须在 TTY 终端中运行。本 launch 使用 emulate_tty 为节点分配伪终端。
  如果 ros2 launch 本身不在交互终端中，请改用 ros2 run 直接启动节点。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="串口设备路径",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="波特率",
        ),
        DeclareLaunchArgument(
            "linear_step",
            default_value="0.5",
            description="线速度步长 (m/s)",
        ),
        DeclareLaunchArgument(
            "angular_step",
            default_value="0.5",
            description="角速度步长 (rad/s)",
        ),

        Node(
            package="r2_communication",
            executable="r2_keyboard_control",
            name="r2_keyboard_control",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "linear_step": LaunchConfiguration("linear_step"),
                "angular_step": LaunchConfiguration("angular_step"),
            }],
        ),
    ])

#!/usr/bin/env python3
"""
R2 行为树导航 Launch 文件

用法:
  ros2 launch r2_communication behavior_tree.launch.py

参数（可在命令行覆盖）:
  port                  串口设备路径，默认 /dev/ttyACM0
  baudrate              波特率，默认 115200
  rate_hz               行为树 tick 频率，默认 20.0
  watchdog_timeout_sec  看门狗超时 (秒)，默认 0.5
  gc_interval_sec       手动 GC 间隔 (秒)，默认 10.0

示例:
  ros2 launch r2_communication behavior_tree.launch.py port:=/dev/ttyUSB1 rate_hz:=30.0

注意:
  需要先 source install/setup.bash 以及确保 py_trees 已安装:
    pip install py_trees
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 行为树脚本路径（相对于 launch 文件自身位置：向上 4 级到达 connect/ 根目录）
    bt_script = os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..", "..",
        "r2_behavior_tree.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="/dev/ttyACM0",
            description="串口设备路径",
        ),
        DeclareLaunchArgument(
            "baudrate", default_value="115200",
            description="波特率",
        ),
        DeclareLaunchArgument(
            "rate_hz", default_value="20.0",
            description="行为树 tick 频率",
        ),
        DeclareLaunchArgument(
            "watchdog_timeout_sec", default_value="0.5",
            description="看门狗超时 (秒)",
        ),
        DeclareLaunchArgument(
            "gc_interval_sec", default_value="10.0",
            description="手动 GC 间隔 (秒)",
        ),
        ExecuteProcess(
            cmd=["python3", bt_script],
            output="screen",
            name="r2_behavior_tree",
            emulate_tty=True,
        ),
    ])

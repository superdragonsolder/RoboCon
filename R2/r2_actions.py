#!/usr/bin/env python3
"""
R2 共享动作库
=============
所有任务脚本 (mission_template_*.py) 共用此文件中的 ROS2 指令函数。

用法:
    from r2_actions import *

    # 直接调用
    急停(); 前进(1.2); 旋转(90); 夹爪(1); ...

    # 或使用规划器动作执行器
    for action in planner.plan_collect_2_r2():
        _execute_action(action)
"""

import os
import sys
import signal
import time
import math
import subprocess
from datetime import datetime
from typing import Optional

# ── 导出清单（供 Pylance 类型检查和 `from r2_actions import *` 使用）──
__all__ = [
    # 配置
    "ROS2_SETUP_BASH", "PUBLISH_TIMEOUT", "AVG_LINEAR_SPEED", "AVG_ANGULAR_SPEED", "STEP_GAP",
    # 工具
    "_ts", "_pub_raw", "_check_behavior_tree",
    # 朝向
    "_get_current_yaw", "旋转并验证", "_normalize_angle",
    # 动作
    "急停", "前进", "左移", "右移", "旋转", "组合移动",
    "机械臂", "攀爬", "夹爪", "下台阶", "吸附墙", "上台阶",
    # 等待
    "等待秒", "等待前进完成", "等待旋转完成", "等待吸附完成",
    # 执行器
    "_execute_action",
]


# ══════════════════════════════════════════════════════════════
# 配置常量
# ══════════════════════════════════════════════════════════════

ROS2_SETUP_BASH = os.path.expanduser("~/connect/R2_ws/install/setup.bash")
PUBLISH_TIMEOUT = 8.0
AVG_LINEAR_SPEED = 0.30
AVG_ANGULAR_SPEED = 0.45
STEP_GAP = 1.0

# ══════════════════════════════════════════════════════════════
# 内部工具
# ══════════════════════════════════════════════════════════════

_g_estop_sent = False


def _handle_sigint(sig, frame):
    global _g_estop_sent
    if not _g_estop_sent:
        _g_estop_sent = True
        print("\n\n⚠️  收到中断信号，正在急停...")
        _pub_raw("ros2 topic pub --once /r2/estop std_msgs/msg/Empty")
    sys.exit(0)


signal.signal(signal.SIGINT, _handle_sigint)


def _pub_raw(cmd: str) -> bool:
    if not _check_behavior_tree():
        print(f"\n[{_ts()}] ❌ 行为树已断开！请重启行为树后重试。")
        sys.exit(1)
    full_cmd = f"source {ROS2_SETUP_BASH} 2>/dev/null && {cmd}"
    try:
        result = subprocess.run(
            ["bash", "-c", full_cmd],
            capture_output=True, text=True, timeout=PUBLISH_TIMEOUT
        )
        if result.returncode != 0:
            err = result.stderr.strip() or result.stdout.strip()
            print(f"  ⚠ 话题发布失败: {cmd[:60]}...")
            if err:
                print(f"     {err[:200]}")
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"  ⚠ 话题发布超时: {cmd[:60]}...")
        return False
    except Exception as e:
        print(f"  ⚠ 话题发布异常: {e}")
        return False


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _check_behavior_tree() -> bool:
    full_cmd = f"source {ROS2_SETUP_BASH} 2>/dev/null && ros2 node list 2>/dev/null | grep -q r2_behavior_tree"
    try:
        result = subprocess.run(["bash", "-c", full_cmd], capture_output=True, timeout=3.0)
        return result.returncode == 0
    except Exception:
        return False


# ══════════════════════════════════════════════════════════════
# 朝向读取 & 旋转验证
# ══════════════════════════════════════════════════════════════

def _get_current_yaw() -> Optional[float]:
    """从 /odin1/odometry 读取当前航向角 (rad)，失败返回 None。"""
    cmd = (
        f"source {ROS2_SETUP_BASH} 2>/dev/null && "
        f"ros2 topic echo --once --field pose.pose.orientation /odin1/odometry 2>/dev/null"
    )
    try:
        result = subprocess.run(
            ["bash", "-c", cmd], capture_output=True, text=True, timeout=3.0
        )
        import re
        nums = re.findall(r'[-]?\d+\.?\d*(?:e[+-]?\d+)?', result.stdout)
        if len(nums) >= 4:
            x, y, z, w = map(float, nums[:4])
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            return yaw
    except Exception:
        pass
    return None


def _normalize_angle(rad: float) -> float:
    """将角度归一化到 [-π, π]。"""
    return math.atan2(math.sin(rad), math.cos(rad))


def 旋转并验证(目标角度度: float, 容差度: float = 3.0, 最大重试: int = 3):
    """旋转并验证角度是否达标，未达标则重试剩余角度。"""
    total_target = math.radians(目标角度度)
    yaw_start = _get_current_yaw()
    if yaw_start is None:
        print(f"  [{_ts()}] ⚠ 无法读取当前朝向，直接旋转 {目标角度度:.0f}°")
        旋转(目标角度度)
        等待旋转完成(目标角度度)
        return

    for attempt in range(最大重试):
        yaw_now = _get_current_yaw()
        if yaw_now is None:
            旋转(math.degrees(total_target))
            等待旋转完成(abs(math.degrees(total_target)))
            return

        # 计算还需旋转多少
        rotated = _normalize_angle(yaw_now - yaw_start)
        remaining = _normalize_angle(total_target - rotated)

        旋转(math.degrees(remaining))
        等待旋转完成(abs(math.degrees(remaining)))

        yaw_check = _get_current_yaw()
        if yaw_check is None:
            print(f"  [{_ts()}] ⚠ 无法验证角度")
            return

        actual = _normalize_angle(yaw_check - yaw_start)
        error_deg = abs(math.degrees(_normalize_angle(total_target - actual)))

        if error_deg <= 容差度:
            print(f"  [{_ts()}] ✓ 旋转达标 (误差 {error_deg:.1f}°)")
            return
        else:
            print(f"  [{_ts()}] ⚠ 旋转偏差 {error_deg:.1f}°，重试 {attempt+2}/{最大重试}")

    print(f"  [{_ts()}] ⚠ 旋转验证失败，已达最大重试次数")


# ══════════════════════════════════════════════════════════════
# 动作指令
# ══════════════════════════════════════════════════════════════

def 急停():
    print(f"[{_ts()}] 🛑 急停")
    _pub_raw("ros2 topic pub --once /r2/estop std_msgs/msg/Empty")


def 前进(距离米: float):
    direction = "前进" if 距离米 >= 0 else "后退"
    print(f"[{_ts()}] ⬆ {direction} {abs(距离米):.2f}m")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: {距离米}, y: 0.0, z: 0.0}}"')


def 左移(距离米: float):
    direction = "左移" if 距离米 >= 0 else "右移"
    print(f"[{_ts()}] ⬅ {direction} {abs(距离米):.2f}m")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: 0.0, y: {距离米}, z: 0.0}}"')


def 右移(距离米: float):
    左移(-距离米)


def 旋转(角度度: float):
    rad = math.radians(角度度)
    direction = "左转" if 角度度 >= 0 else "右转"
    print(f"[{_ts()}] 🔄 {direction} {abs(角度度):.0f}°")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: 0.0, y: 0.0, z: {rad}}}"')


def 组合移动(前向米: float = 0.0, 左移米: float = 0.0, 旋转度: float = 0.0):
    rad = math.radians(旋转度)
    print(f"[{_ts()}] 🔀 前进{前向米:.1f}m + 左移{左移米:.1f}m + 旋转{旋转度:.0f}°")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: {前向米}, y: {左移米}, z: {rad}}}"')


def 机械臂(动作: int):
    names = {0: "空闲", 1: "绘制KFS"}
    print(f"[{_ts()}] 🦾 机械臂 → {names.get(动作, f'未知({动作})')}")
    _pub_raw(f'ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{{data: {动作}}}"')


def 攀爬(指令: int):
    names = {0: "初始20cm", 1: "准备40cm", 2: "上爬20cm", 3: "上爬40cm", 4: "下降20cm"}
    print(f"[{_ts()}] 🪜 攀爬 → {names.get(指令, f'未知({指令})')}")
    _pub_raw(f'ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{{data: {指令}}}"')


def 夹爪(指令: int):
    names = {0: "夹持角90°", 1: "闭合", 2: "复位0°", 3: "释放"}
    print(f"[{_ts()}] ✋ 夹爪 → {names.get(指令, f'未知({指令})')}")
    _pub_raw(f'ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{{data: {指令}}}"')


def 下台阶(前距: float = 0.2, 后距: float = 0.3):
    """下台阶封装：先前进接近 → 攀爬下降 → 再前进离开。"""
    print(f"[{_ts()}] 🪜 下台阶: 前移{前距:.1f}m → 下降20cm → 后移{后距:.1f}m")
    if abs(前距) > 0.01:
        前进(前距)
        等待前进完成(前距)
    攀爬(4)
    等待秒(10.0)
    if abs(后距) > 0.01:
        前进(后距)
        等待前进完成(后距)


def 吸附墙():
    print(f"[{_ts()}] 🧱 吸附到最近墙方向")
    _pub_raw("ros2 topic pub --once /r2/snap_wall std_msgs/msg/Empty")


def 上台阶(前距: float = 0.3, 上爬指令: int = 2, 后距: float = 0.3):
    """上台阶封装：先前进接近 → 攀爬 → 再前进登顶。"""
    names = {2: "上爬20cm", 3: "上爬40cm"}
    print(f"[{_ts()}] 🪜 上台阶: 前移{前距:.1f}m → {names.get(上爬指令, str(上爬指令))} → 后移{后距:.1f}m")
    前进(前距)
    等待前进完成(前距)
    攀爬(上爬指令)
    等待秒(10.0)
    if abs(后距) > 0.01:
        前进(后距)
        等待前进完成(后距)


# ══════════════════════════════════════════════════════════════
# 等待函数
# ══════════════════════════════════════════════════════════════

def 等待秒(秒数: float):
    print(f"[{_ts()}] ⏳ 等待 {秒数:.1f}s ...")
    time.sleep(秒数)


def 等待前进完成(距离米: float, 额外秒: float = 2.0):
    t = abs(距离米) / AVG_LINEAR_SPEED + 额外秒 + STEP_GAP
    print(f"[{_ts()}] ⏳ 等待直线移动 ({abs(距离米):.2f}m 估算 {t:.1f}s) ...")
    time.sleep(t)


def 等待旋转完成(角度度: float, 额外秒: float = 1.0):
    t = min(5.0, abs(math.radians(角度度)) / AVG_ANGULAR_SPEED + 额外秒 + STEP_GAP)
    print(f"[{_ts()}] ⏳ 等待旋转 ({abs(角度度):.0f}° 估算 {t:.1f}s) ...")
    time.sleep(t)


def 等待吸附完成():
    print(f"[{_ts()}] ⏳ 等待吸附墙完成（5s）...")
    time.sleep(5.0)


# ══════════════════════════════════════════════════════════════
# 规划器动作 → ROS2 指令执行器
# ══════════════════════════════════════════════════════════════

def _execute_action(action: dict) -> None:
    """将 kfs_planner 输出的动作字典转为 ROS2 指令并执行。"""
    from kfs_planner import KFSPlanner

    atype = action["type"]
    params = action.get("params", {})

    if atype == "entry_grab":
        kfs = params["kfs"]
        dh = KFSPlanner.height_at(kfs[0], kfs[1]) - 0
        if dh in (200, 400):
            机械臂(1)
        等待秒(2.0)
        夹爪(0)
        等待秒(1.0)
        夹爪(1)
        等待秒(1.5)
        机械臂(0)
        等待秒(1.0)

    elif atype == "rotate":
        deg = params["degrees"]
        旋转并验证(deg)

    elif atype == "climb":
        cmd = params["cmd"]
        # 上坡: 前移0.3m接近 → 攀爬 → 后移0.3m到中心
        # 下坡: 直接攀爬 → 后移0.3m到中心
        if cmd == 2:
            上台阶(前距=0.3, 上爬指令=2, 后距=0.3)
        elif cmd == 4:
            下台阶(前距=0.0, 后距=0.3)
            攀爬(0)
            等待秒(2.0)

    elif atype == "forward":
        # 爬升/下降已完成后移0.3m到达格子中心，无需额外前进
        等待秒(0.3)

    elif atype == "grab":
        arm_dh = params["delta_mm"]
        # 伸臂：高度差 200/400 伸臂(1)，-200 也先伸臂(1) 再夹取
        if arm_dh in (200, 400, -200):
            机械臂(1)
        等待秒(2.0)
        夹爪(0)
        等待秒(1.0)
        夹爪(1)
        等待秒(1.5)
        机械臂(0)
        等待秒(1.0)

    else:
        print(f"  ⚠ 未知动作类型: {atype}，跳过")

#!/usr/bin/env python3
"""
R2 比赛任务脚本模板
====================
通过 ROS2 话题向行为树 (r2_behavior_tree.py) 发送指令，串行执行预设任务序列。

┌─────────────────────────────────────────────────────────────┐
│  使用方式:                                                   │
│  1. 先启动行为树:  python3 ~/connect/r2_behavior_tree.py      │
│  2. 再运行本脚本:  python3 ~/connect/mission_script.py        │
│                                                             │
│  紧急停止: 按 Ctrl+C 会发送急停指令                            │
└─────────────────────────────────────────────────────────────┘

修改指南:
  - 在下方 "══════ 任务序列 ══════" 区域编写你的任务步骤
  - 每个动作后跟一个 wait_* 等待动作完成
  - 等待时间建议 = (距离÷0.3) + 2s 余量（约 0.3m/s 平均速度）
  - 旋转等待: 90° 约 3-4s，45° 约 2-3s
"""

import os
import sys
import signal
import time
import math
import subprocess
from datetime import datetime


# ══════════════════════════════════════════════════════════════
# 配置区 —— 根据实际情况修改
# ══════════════════════════════════════════════════════════════

# ROS2 环境 source 命令（如果终端未 source，脚本会自动 source）
ROS2_SETUP_BASH = os.path.expanduser("~/connect/R2_ws/install/setup.bash")

# 话题发布超时 (秒)，单条指令的最长等待时间
PUBLISH_TIMEOUT = 8.0

# 全局速度参数（仅作参考，实际速度由行为树 PID 决定）
AVG_LINEAR_SPEED = 0.30   # 平均线速度 m/s（估算用）
AVG_ANGULAR_SPEED = 0.45  # 平均角速度 rad/s（估算用）

# 步间安全间隔 (秒)，每步之间额外等待的缓冲时间
STEP_GAP = 1.0


# ══════════════════════════════════════════════════════════════
# 底层工具函数 —— 无需修改
# ══════════════════════════════════════════════════════════════

_g_estop_sent = False


def _handle_sigint(sig, frame):
    """Ctrl+C 时自动急停。"""
    global _g_estop_sent
    if not _g_estop_sent:
        _g_estop_sent = True
        print("\n\n⚠️  收到中断信号，正在急停...")
        _pub_raw("ros2 topic pub --once /r2/estop std_msgs/msg/Empty")
    sys.exit(0)


signal.signal(signal.SIGINT, _handle_sigint)


def _pub_raw(cmd: str) -> bool:
    """执行一条 ros2 topic pub 命令，返回是否成功。"""
    # 先检查行为树是否存活
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
    """返回当前时间戳字符串。"""
    return datetime.now().strftime("%H:%M:%S")


def _check_behavior_tree() -> bool:
    """检查行为树节点是否存活。"""
    full_cmd = f"source {ROS2_SETUP_BASH} 2>/dev/null && ros2 node list 2>/dev/null | grep -q r2_behavior_tree"
    try:
        result = subprocess.run(
            ["bash", "-c", full_cmd],
            capture_output=True, timeout=3.0
        )
        return result.returncode == 0
    except Exception:
        return False


def _assert_behavior_tree():
    """断言行为树存活，否则退出。"""
    if not _check_behavior_tree():
        print(f"\n[{_ts()}] ❌ 行为树已断开！请重启行为树后重试。")
        sys.exit(1)


# ══════════════════════════════════════════════════════════════
# 动作函数 —— 每个函数对应一个 ROS2 话题
# ══════════════════════════════════════════════════════════════


def 急停():
    """
    立即制动，速度归零，清空移动目标。
    通常用于任务开始/结束或紧急情况。
    """
    print(f"[{_ts()}] 🛑 急停")
    _pub_raw("ros2 topic pub --once /r2/estop std_msgs/msg/Empty")


def 前进(距离米: float):
    """
    向前直线移动指定距离。
    参数: 距离米 — 正数前进，负数后退
    示例: 前进(2.0)   # 前进 2 米
         前进(-1.0)   # 后退 1 米
    """
    direction = "前进" if 距离米 >= 0 else "后退"
    print(f"[{_ts()}] ⬆ {direction} {abs(距离米):.2f}m")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: {距离米}, y: 0.0, z: 0.0}}"')


def 左移(距离米: float):
    """
    向左平移指定距离。
    参数: 距离米 — 正数左移，负数右移
    示例: 左移(0.5)    # 左移 0.5 米
         左移(-0.3)    # 右移 0.3 米
    """
    direction = "左移" if 距离米 >= 0 else "右移"
    print(f"[{_ts()}] ⬅ {direction} {abs(距离米):.2f}m")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: 0.0, y: {距离米}, z: 0.0}}"')


def 右移(距离米: float):
    """向右平移。等同 左移(-距离米)。"""
    左移(-距离米)


def 旋转(角度度: float):
    """
    原地旋转指定角度。
    参数: 角度度 — 正数左转(逆时针)，负数右转(顺时针)
    示例: 旋转(90)    # 左转 90°
         旋转(-45)   # 右转 45°
         旋转(180)   # 掉头
    """
    import math
    rad = math.radians(角度度) + 0.03575  # 左偏补偿 ~2.0°
    direction = "左转" if 角度度 >= 0 else "右转"
    print(f"[{_ts()}] 🔄 {direction} {abs(角度度):.0f}°")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: 0.0, y: 0.0, z: {rad}}}"')


def 组合移动(前向米: float = 0.0, 左移米: float = 0.0, 旋转度: float = 0.0):
    """
    组合移动：同时前进、平移、旋转。
    参数:
        前向米 — 前进距离 (m)
        左移米 — 左移距离 (m)
        旋转度 — 旋转角度 (°)
    示例: 组合移动(前向米=1.0, 旋转度=45)   # 前进1m同时左转45°
    """
    import math
    rad = math.radians(旋转度)
    print(f"[{_ts()}] 🔀 前进{前向米:.1f}m + 左移{左移米:.1f}m + 旋转{旋转度:.0f}°")
    _pub_raw(f'ros2 topic pub --once /move_relative geometry_msgs/msg/Point '
             f'"{{x: {前向米}, y: {左移米}, z: {rad}}}"')


def 机械臂(动作: int):
    """
    控制机械臂。
    参数: 动作 — 0=空闲, 1=绘制KFS
    示例: 机械臂(0)    # 机械臂空闲
         机械臂(1)    # 绘制KFS
    """
    names = {0: "空闲", 1: "绘制KFS"}
    print(f"[{_ts()}] 🦾 机械臂 → {names.get(动作, f'未知({动作})')}")
    _pub_raw(f'ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{{data: {动作}}}"')


def 攀爬(指令: int):
    """
    控制攀爬机构。
    参数: 指令 — 0=初始20cm, 1=准备40cm, 2=上爬20cm, 3=上爬40cm, 4=下降20cm
    示例: 攀爬(0)   # 初始姿态 20cm
         攀爬(4)   # 下降 20cm
    """
    names = {0: "初始20cm", 1: "准备40cm", 2: "上爬20cm", 3: "上爬40cm", 4: "下降20cm"}
    print(f"[{_ts()}] 🪜 攀爬 → {names.get(指令, f'未知({指令})')}")
    _pub_raw(f'ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{{data: {指令}}}"')


def 夹爪(指令: int):
    """
    控制夹爪机构。
    参数: 指令 — 0=夹持角90°, 1=闭合, 2=复位0°, 3=释放
    示例: 夹爪(0)   # 移至夹持角
         夹爪(1)   # 闭合夹爪
         夹爪(3)   # 释放
    """
    names = {0: "夹持角90°", 1: "闭合", 2: "复位0°", 3: "释放"}
    print(f"[{_ts()}] ✋ 夹爪 → {names.get(指令, f'未知({指令})')}")
    _pub_raw(f'ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{{data: {指令}}}"')


def 下台阶():
    """
    触发下台阶全自动流程。
    停车 → 启动深度相机 → 0.05m/s慢行 → 检测到楼梯 → 攀爬下降20cm。
    注意: 此动作耗时不定（取决于何时检测到楼梯，最长60s超时）。
    """
    print(f"[{_ts()}] 📷 下台阶自动流程")
    _pub_raw("ros2 topic pub --once /r2/stairs_down std_msgs/msg/Empty")


def 吸附墙():
    """
    将底盘旋转到最近的主方向（0°/±90°/180°，含 wall_yaw_offset 偏移），
    确保与墙壁平行或垂直。
    """
    print(f"[{_ts()}] 🧱 吸附到最近墙方向")
    _pub_raw("ros2 topic pub --once /r2/snap_wall std_msgs/msg/Empty")


# ══════════════════════════════════════════════════════════════
# 等待函数 —— 用于等待动作完成
# ══════════════════════════════════════════════════════════════


def 等待秒(秒数: float):
    """
    简单等待指定秒数。用于非移动动作之间的间隔。
    示例: 等待秒(5.0)   # 等待 5 秒
    """
    print(f"[{_ts()}] ⏳ 等待 {秒数:.1f}s ...")
    time.sleep(秒数)


def 等待前进完成(距离米: float, 额外秒: float = 2.0):
    """
    等待直线移动完成（估算时间）。
    参数:
        距离米 — 刚刚发出的前进距离
        额外秒 — 额外缓冲时间（默认 2s，确保到位）
    估算公式: 时间 = |距离| ÷ 0.30 + 额外秒 + 步间间隔
    """
    t = abs(距离米) / AVG_LINEAR_SPEED + 额外秒 + STEP_GAP
    print(f"[{_ts()}] ⏳ 等待直线移动 ({abs(距离米):.2f}m 估算 {t:.1f}s) ...")
    time.sleep(t)


def 等待旋转完成(角度度: float, 额外秒: float = 3.0):
    """
    等待旋转完成（估算时间）。
    参数:
        角度度 — 刚刚发出的旋转角度
        额外秒 — 额外缓冲时间（默认 3s）
    估算公式: 时间 = |角度(rad)| ÷ 0.45 + 额外秒 + 步间间隔
    """
    import math
    t = abs(math.radians(角度度)) / AVG_ANGULAR_SPEED + 额外秒 + STEP_GAP
    print(f"[{_ts()}] ⏳ 等待旋转 ({abs(角度度):.0f}° 估算 {t:.1f}s) ...")
    time.sleep(t)


def 等待下台阶完成(最大秒: float = 65.0):
    """
    等待下台阶流程完成。
    参数: 最大秒 — 最长等待时间（默认 65s，下台阶内部有 60s 超时）
    """
    print(f"[{_ts()}] ⏳ 等待下台阶完成（最长 {最大秒:.0f}s）...")
    time.sleep(最大秒)


def 等待吸附完成():
    """
    等待吸附墙完成。
    """
    print(f"[{_ts()}] ⏳ 等待吸附墙完成（5s）...")
    time.sleep(5.0)


# ══════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════
#                                                               #
#   ★★★  任务序列 —— 在这里编写你的比赛流程  ★★★                 #
#                                                               #
#   修改方法:                                                   #
#     1. 每行就是一个动作，按顺序执行                             #
#     2. 移动类动作后必须跟等待函数                               #
#     3. 等待时间不够就增大"额外秒"参数                           #
#     4. 如果实际跑偏了，调大等待时间                             #
#                                                               #
# ══════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════


def 执行任务序列():
    """
    ★ 比赛任务序列 —— 在这里编写你的任务步骤 ★

    动作函数速查:
    ┌──────────────┬──────────────────────────────────┐
    │ 函数          │ 说明                             │
    ├──────────────┼──────────────────────────────────┤
    │ 急停()        │ 立即制动                         │
    │ 前进(d)       │ 前进 d 米 (负数=后退)            │
    │ 左移(d)       │ 左移 d 米 (负数=右移)            │
    │ 旋转(deg)     │ 旋转 deg° (正=左转, 负=右转)     │
    │ 组合移动(f,l,r)│ 前进f + 左移l + 旋转r°           │
    │ 机械臂(n)     │ 0=空闲 1=绘制KFS                 │
    │ 攀爬(n)       │ 0=初始 1=准备 2=上20 3=上40 4=下20│
    │ 夹爪(n)       │ 0=夹持角 1=闭合 2=复位 3=释放    │
    │ 下台阶()      │ 全自动相机检测下台阶              │
    │ 吸附墙()      │ 旋转到最近的墙方向 (0/±90/180°)   │
    ├──────────────┼──────────────────────────────────┤
    │ 等待秒(s)     │ 等待 s 秒                        │
    │ 等待前进完成(d)│ 等待直线移动完成(+缓冲)          │
    │ 等待旋转完成(a)│ 等待旋转完成(+缓冲)              │
    │ 等待下台阶完成()│ 等待下台阶流程完成(最长65s)     │
    │ 等待吸附完成() │ 等待吸附墙完成（最长 ~7s）       │
    └──────────────┴──────────────────────────────────┘

    计时参考（含安全余量）:
      1m 前进  ≈ 5s
      2m 前进  ≈ 9s
      5m 前进  ≈ 20s
      45° 旋转 ≈ 4s
      90° 旋转 ≈ 6s
      180° 旋转≈ 11s
    """
    print("=" * 60)
    print("  R2 比赛任务序列 启动")
    print(f"  开始时间: {_ts()}")
    print("=" * 60)
  #  旋转(-90)
  #  等待旋转完成(90)
  #  前进(1.1)
 #   等待前进完成(1.1)
 #   旋转(90)
 #   等待旋转完成(90)
  #  前进(2.7)
 #   等待前进完成(2.7)
    旋转(90)
    等待旋转完成(90)
 #   前进(4.0)
  #  等待前进完成(4.0)
 #   等待旋转完成(-90)
   

    print("\n" + "=" * 60)
    print(f"  ✅ 任务序列完成! 结束时间: {_ts()}")
    print("=" * 60)


# ══════════════════════════════════════════════════════════════
# 入口
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║           R2 比赛任务脚本 (Behavior Tree 话题驱动)        ║
║                                                          ║
║  前提: 行为树已启动 (python3 r2_behavior_tree.py)         ║
║  紧急: 按 Ctrl+C 急停                                    ║
╚══════════════════════════════════════════════════════════╝
""")
    # 攀爬初始化（提前归位，确保任务开始前机构已就绪）
    print(f"[{_ts()}] 🔧 攀爬初始化...")
    攀爬(0)
    等待秒(3.0)
    # 等待用户确认
    input("按 Enter 开始执行任务序列...")
    执行任务序列()

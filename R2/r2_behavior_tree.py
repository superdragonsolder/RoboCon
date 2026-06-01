#!/usr/bin/env python3
"""
R2 行为树 —— 基于 py_trees 的机器人导航控制
==============================================
全中文节点名称，20Hz 控制频率，基于 MAVLink（R2_Protocol.py）串口通信。

组合逻辑：
  序列 (Sequence)：检查雷达重定位 → 设置相对目标 → 计算PID速度
  选择 (Selector) ：若定位丢失，则自动触发 底盘紧急制动。

所有运行时变量通过模块级 _shared 字典共享。
"""

import gc
import math
import os
import sys
import time
from typing import Tuple

import py_trees
import py_trees.display
import rclpy
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
from tf2_ros import Buffer, TransformListener

# ---- 复用现有通信层 ----
from R2Communication import R2Communication
from R2_Protocol import (
    ARM_ACTION_IDLE,
    ARM_DRAW_KFS_20cm,
    ARM_DRAW_KFS_40cm,
    ARM_DRAW_KFS_BELOW_20cm,
    CLIMBING_CMD_INIT_POSE,
    CLIMBING_CMD_PREPARE_40CM,
    CLIMBING_CMD_EXECUTE_UP_20CM,
    CLIMBING_CMD_EXECUTE_UP_40CM,
    CLIMBING_CMD_EXECUTE_DOWN_20CM,
    CLAMPING_CMD_MOVE_TO_PARALLEL,
    CLAMPING_CMD_GRAB,
    CLAMPING_CMD_MOVE_TO_RESET,
    CLAMPING_CMD_RELEASE,
)

# ---- 深度相机模块（下台阶检测）----
_HOWLONG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "howlong")
if _HOWLONG_DIR not in sys.path:
    sys.path.insert(0, _HOWLONG_DIR)
_CAMERA_AVAILABLE = False
try:
    from src.camera_manager import CameraManager
    from src.distance_measurer import DistanceMeasurer
    _CAMERA_AVAILABLE = True
except Exception as _e:
    print(f"[R2] 深度相机模块加载失败: {_e}")
    _CAMERA_AVAILABLE = False

# ---- 下台阶检测器（基于 ROI 比例检测）----
_STAIR_DETECTOR_AVAILABLE = False
try:
    from berxel.stair_down_detector import StairDownDetector
    _STAIR_DETECTOR_AVAILABLE = True
except Exception as _e:
    print(f"[R2] 下台阶检测器加载失败: {_e}")
    _STAIR_DETECTOR_AVAILABLE = False

# ---- 模块级共享字典（替代 py_trees blackboard，避免 namespace 问题）----
_shared = {
    "current_pose": (0.0, 0.0, 0.0),
    "current_map_pose": None,     # TF融合后的 map 帧位姿 (x, y, yaw)，None 表示 TF 不可用
    "map_frame_available": False, # map 帧 TF 是否可用
    "global_target": (0.0, 0.0),
    "target_yaw": None,            # 纯旋转目标角度 (rad)，非 None 时 计算PID速度 优先追踪
    "relative_command": (0.0, 0.0, 0.0),
    "odom_last_arrival": None,
    "pos_tolerance": 0.03,
    "kp_dist": 1.2,
    "kp_heading": 1.2,
    "max_linear": 0.32,
    "max_angular": 0.55,
    "slowdown_distance": 0.10,
    "wall_yaw_offset": 0.0,       # 墙方向偏移 (rad)，正=左偏，负=右偏
    "watchdog_timeout_sec": 0.5,
    "odom_timeout_sec": 0.5,
    "r2_comm": None,
    "serial_connected": True,
    # ── 下台阶检测 ──
    "stairs_down_triggered": False,
    "stairs_down_step_height_cm": 20.0,
    "stairs_down_tolerance_cm": 2.0,
    "stairs_down_confirm_frames": 3,
    "stairs_down_conversion_factor": 7900.0,
    "stairs_down_roi_size": 10,
    "stairs_down_slow_speed": 0.05,
    "stairs_down_camera_timeout_ms": 5,
    # ── 攀爬锁 ──
    "climbing_active": False,     # True 时 PID 强制零速，防止攀爬时底盘移动
}


# ======================== 工具函数 ========================

def norm_angle(a: float) -> float:
    """将角度归一化到 [-pi, pi)。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取 yaw 角。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ======================== 条件节点 ========================

class 检查雷达重定位(py_trees.behaviour.Behaviour):
    """条件节点：检查奥丁之眼（Odin1）定位话题是否在最近 0.5 秒内有更新。

    黑板键：
        odom_last_arrival  : float —— 最近一次里程计到达时间 (time.monotonic)
        odom_timeout_sec   : float —— 超时阈值，默认 0.5
    返回：
        SUCCESS —— 定位数据新鲜
        FAILURE —— 定位超时 / 丢失
    """

    def __init__(self, name: str = "检查雷达重定位"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        last_arrival = _shared["odom_last_arrival"]
        timeout = _shared["odom_timeout_sec"]

        if last_arrival is None:
            self.feedback_message = "尚未收到里程计数据"
            return py_trees.common.Status.FAILURE

        age = time.monotonic() - last_arrival
        if age > timeout:
            self.feedback_message = f"定位超时 {age:.3f}s > {timeout}s"
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"定位正常 (age={age:.3f}s)"
        return py_trees.common.Status.SUCCESS


class 是否到达目标(py_trees.behaviour.Behaviour):
    """条件节点：计算当前位姿与黑板中全局目标的欧式距离是否小于预设阈值。

    黑板键：
        current_pose  : (x, y, yaw)
        global_target : (target_x, target_y)
        pos_tolerance : float —— 到达阈值 (m)
    返回：
        SUCCESS —— 已到达
        FAILURE —— 尚未到达
    """

    def __init__(self, name: str = "是否到达目标"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        x, y, _ = _shared["current_pose"]
        tx, ty = _shared["global_target"]
        dist = math.hypot(tx - x, ty - y)
        tol = _shared["pos_tolerance"]

        if dist <= tol:
            self.feedback_message = f"已到达 dist={dist:.3f}m ≤ {tol:.3f}m"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"未到达 dist={dist:.3f}m > {tol:.3f}m"
        return py_trees.common.Status.FAILURE


class 看门狗检查(py_trees.behaviour.Behaviour):
    """条件节点：检查底盘是否在 watchdog_timeout_sec 内收到过速度指令。

    若超过阈值未发送任何指令（如串口断线、树卡死），
    则触发 FAILURE → 根序列中断 → 紧急制动。

    黑板键：
        r2_comm              : R2Communication
        watchdog_timeout_sec : float —— 看门狗超时阈值
    返回：
        SUCCESS —— 指令正常
        FAILURE —— 超时未发送指令，触发保护
    """

    def __init__(self, name: str = "看门狗检查"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        r2: R2Communication = _shared["r2_comm"]
        timeout = _shared["watchdog_timeout_sec"]

        # 初始状态：尚未发送过任何指令，无需触发看门狗
        if r2.last_send_time <= 0.0:
            self.feedback_message = "看门狗: 等待首次指令"
            return py_trees.common.Status.SUCCESS

        elapsed = time.monotonic() - r2.last_send_time

        if elapsed > timeout:
            self.feedback_message = (
                f"看门狗超时! 距上次指令 {elapsed:.3f}s > {timeout}s"
            )
            self.logger.error(self.feedback_message)
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"看门狗正常 (距上次指令 {elapsed:.3f}s)"
        return py_trees.common.Status.SUCCESS


# ======================== 动作节点 ========================

class 底盘紧急制动(py_trees.behaviour.Behaviour):
    """动作节点：立即向串口发送 Vx=0, Vy=0, Vω=0 的紧急制动指令。

    黑板键：
        r2_comm : R2Communication —— 通信对象引用
    返回：
        FAILURE —— 制动已执行，但返回 FAILURE 以向父节点传播异常状态，
                  确保上层 Sequence 感知到定位丢失并终止后续导航。
    """

    def __init__(self, name: str = "底盘紧急制动"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        r2: R2Communication = _shared["r2_comm"]
        r2.send_chassis_velocity(0.0, 0.0, 0.0)
        self.feedback_message = "紧急制动: Vx=0 Vy=0 Vω=0 已发送"
        self.logger.error(
            f"[{self.name}] 定位丢失，已紧急制动！请检查雷达/里程计系统。"
        )
        return py_trees.common.Status.FAILURE


class 设置相对目标(py_trees.behaviour.Behaviour):
    """动作节点：获取“相对坐标指令”(dx, dy, dθ)，结合当前绝对位姿，
    计算出新的“全局目标点”并写入黑板。

    黑板键（读）：
        relative_command : (dx, dy, dtheta)
        current_pose     : (x, y, yaw)
    黑板键（写）：
        global_target    : (target_x, target_y)
    返回：
        SUCCESS —— 目标已计算并写入黑板
    """

    def __init__(self, name: str = "设置相对目标"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        dx, dy, dtheta = _shared["relative_command"]
        x0, y0, yaw0 = _shared["current_pose"]

        # 无新指令：直接放行，不覆盖已有目标
        if abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dtheta) < 1e-6:
            self.feedback_message = "等待指令..."
            return py_trees.common.Status.SUCCESS

        # 纯旋转指令：直接按 dtheta 计算目标朝向
        if abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dtheta) > 1e-6:
            _shared["climbing_active"] = False  # 解锁攀爬
            target_yaw_val = norm_angle(yaw0 + dtheta)
            _shared["target_yaw"] = target_yaw_val
            _shared["global_target"] = (x0, y0)  # 位置不变
            _shared["relative_command"] = (0.0, 0.0, 0.0)
            self.feedback_message = (
                f"旋转目标: 当前={math.degrees(yaw0):.1f}° "
                f"→ 目标={math.degrees(target_yaw_val):.1f}° (Δ={math.degrees(dtheta):.1f}°)"
            )
            return py_trees.common.Status.SUCCESS

        # 位移指令：清除 yaw 目标、解锁攀爬，计算全局位置目标
        _shared["target_yaw"] = None
        _shared["climbing_active"] = False  # 新移动指令自动解锁攀爬
        target_x = x0 + dx * math.cos(yaw0) - dy * math.sin(yaw0)
        target_y = y0 + dx * math.sin(yaw0) + dy * math.cos(yaw0)

        _shared["global_target"] = (target_x, target_y)
        _shared["relative_command"] = (0.0, 0.0, 0.0)

        self.feedback_message = (
            f"新目标: dx={dx:.3f} dy={dy:.3f} → ({target_x:.3f},{target_y:.3f})"
        )
        return py_trees.common.Status.SUCCESS

class _相对移动(py_trees.behaviour.Behaviour):
    """内部基类：可参数化的相对移动动作节点。

    子类只需在 __init__ 中设置 self._dx, self._dy, self._dtheta 和 name。
    第一次 tick 时锁定当前位姿计算全局目标，后续 tick 执行 PID 追踪。

    返回：RUNNING（追踪中）→ SUCCESS（到达）
    """

    def __init__(self, name: str, dx: float, dy: float, dtheta: float):
        super().__init__(name)
        self._dx = dx
        self._dy = dy
        self._dtheta = dtheta
        self._target_locked = False
        self._target_x = 0.0
        self._target_y = 0.0
        self._distance = 0.0

    def _lock_target(self) -> None:
        x0, y0, yaw0 = _shared["current_pose"]
        self._target_x = x0 + self._dx * math.cos(yaw0) - self._dy * math.sin(yaw0)
        self._target_y = y0 + self._dx * math.sin(yaw0) + self._dy * math.cos(yaw0)
        self._distance = math.hypot(self._dx, self._dy)
        self._target_locked = True

    def update(self) -> py_trees.common.Status:
        if not self._target_locked:
            self._lock_target()

        x, y, yaw = _shared["current_pose"]
        r2: R2Communication = _shared["r2_comm"]
        tx, ty = self._target_x, self._target_y

        # 纯旋转节点
        if self._distance < 1e-6 and abs(self._dtheta) > 1e-6:
            target_yaw = norm_angle(yaw + self._dtheta)
            err = norm_angle(target_yaw - yaw)
            if abs(err) < 0.06:
                r2.send_chassis_velocity(0.0, 0.0, 0.0)
                self.feedback_message = f"{self.name}: 到位 err={math.degrees(err):.1f}°"
                return py_trees.common.Status.SUCCESS
            w = math.copysign(
                max(0.10, min(_shared["max_angular"],
                              _shared["kp_heading"] * abs(err))),
                err,
            )
            r2.send_chassis_velocity(0.0, 0.0, w)
            self.feedback_message = f"{self.name}: err={math.degrees(err):.1f}° w={w:.3f}"
            return py_trees.common.Status.RUNNING

        ex = tx - x
        ey = ty - y
        dist = math.hypot(ex, ey)

        if dist <= _shared["pos_tolerance"]:
            r2.send_chassis_velocity(0.0, 0.0, 0.0)
            self.feedback_message = f"{self.name}: 到达 dist={dist:.3f}m"
            return py_trees.common.Status.SUCCESS

        desired_yaw = math.atan2(ey, ex)
        heading_err = norm_angle(desired_yaw - yaw)

        v = min(_shared["max_linear"], _shared["kp_dist"] * dist)
        if dist < _shared["slowdown_distance"]:
            v *= max(0.2, dist / _shared["slowdown_distance"])
        if abs(heading_err) > 0.8:
            v *= 0.2

        w = max(-_shared["max_angular"],
                min(_shared["max_angular"],
                    _shared["kp_heading"] * heading_err))
        r2.send_chassis_velocity(v, 0.0, w)

        self.feedback_message = f"{self.name}: dist={dist:.3f}m v={v:.3f} w={w:.3f}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        r2: R2Communication = _shared["r2_comm"]
        r2.send_chassis_velocity(0.0, 0.0, 0.0)
        self.feedback_message = f"{self.name}: 完成 ({new_status})"


# ── 便捷移动节点 ──

class 前进(_相对移动):
    """向前移动指定距离（米）。用法: 前进(1.0)"""
    def __init__(self, distance: float = 1.0, name: str = "前进"):
        super().__init__(name, dx=distance, dy=0.0, dtheta=0.0)


class 后退(_相对移动):
    """向后移动指定距离（米）。用法: 后退(0.5)"""
    def __init__(self, distance: float = 1.0, name: str = "后退"):
        super().__init__(name, dx=-distance, dy=0.0, dtheta=0.0)


class 左移(_相对移动):
    """向左平移指定距离（米）。用法: 左移(0.3)"""
    def __init__(self, distance: float = 1.0, name: str = "左移"):
        super().__init__(name, dx=0.0, dy=distance, dtheta=0.0)


class 右移(_相对移动):
    """向右平移指定距离（米）。用法: 右移(0.3)"""
    def __init__(self, distance: float = 1.0, name: str = "右移"):
        super().__init__(name, dx=0.0, dy=-distance, dtheta=0.0)


class 左转90度(_相对移动):
    """原地左转 90°。"""
    def __init__(self, name: str = "左转90度"):
        super().__init__(name, dx=0.0, dy=0.0, dtheta=math.pi / 2.0)


class 右转90度(_相对移动):
    """原地右转 90°。"""
    def __init__(self, name: str = "右转90度"):
        super().__init__(name, dx=0.0, dy=0.0, dtheta=-math.pi / 2.0)


# ── 底盘急停 ──

class 急停(py_trees.behaviour.Behaviour):
    """动作节点：底盘紧急制动，速度归零。"""
    def __init__(self, name: str = "急停"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        r2: R2Communication = _shared["r2_comm"]
        r2.send_chassis_velocity(0.0, 0.0, 0.0)
        self.feedback_message = "底盘已急停 V=0"
        return py_trees.common.Status.SUCCESS


# ── 机械臂动作 ──

class 机械臂空闲(py_trees.behaviour.Behaviour):
    """动作节点：机械臂停止/空闲。"""
    def __init__(self, name: str = "机械臂空闲"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_arm_action(ARM_ACTION_IDLE)
        self.feedback_message = "机械臂 → 空闲"
        return py_trees.common.Status.SUCCESS


class 机械臂绘制KFS(py_trees.behaviour.Behaviour):
    """动作节点：机械臂绘制 KFS 20cm (V3)。"""
    def __init__(self, name: str = "机械臂绘制KFS"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_arm_action(ARM_DRAW_KFS_20cm)
        self.feedback_message = "机械臂 → 绘制 KFS 20cm"
        return py_trees.common.Status.SUCCESS


# ── 攀爬机构 ──

class 攀爬初始姿态(py_trees.behaviour.Behaviour):
    """动作节点：攀爬机构初始 20cm 准备姿态。"""
    def __init__(self, name: str = "攀爬初始姿态"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_climbing_cmd(CLIMBING_CMD_INIT_POSE)
        self.feedback_message = "攀爬 → 初始 20cm 姿态"
        return py_trees.common.Status.SUCCESS


class 攀爬准备40cm(py_trees.behaviour.Behaviour):
    """动作节点：攀爬机构升至 40cm 准备。"""
    def __init__(self, name: str = "攀爬准备40cm"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_climbing_cmd(CLIMBING_CMD_PREPARE_40CM)
        self.feedback_message = "攀爬 → 准备 40cm"
        return py_trees.common.Status.SUCCESS


class 攀爬上爬20cm(py_trees.behaviour.Behaviour):
    """动作节点：攀爬机构上爬 20cm。"""
    def __init__(self, name: str = "攀爬上爬20cm"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_climbing_cmd(CLIMBING_CMD_EXECUTE_UP_20CM)
        self.feedback_message = "攀爬 → 上爬 20cm"
        return py_trees.common.Status.SUCCESS


class 攀爬上爬40cm(py_trees.behaviour.Behaviour):
    """动作节点：攀爬机构上爬 40cm。"""
    def __init__(self, name: str = "攀爬上爬40cm"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_climbing_cmd(CLIMBING_CMD_EXECUTE_UP_40CM)
        self.feedback_message = "攀爬 → 上爬 40cm"
        return py_trees.common.Status.SUCCESS


class 攀爬下降20cm(py_trees.behaviour.Behaviour):
    """动作节点：攀爬机构下降 20cm。"""
    def __init__(self, name: str = "攀爬下降20cm"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_climbing_cmd(CLIMBING_CMD_EXECUTE_DOWN_20CM)
        self.feedback_message = "攀爬 → 下降 20cm"
        return py_trees.common.Status.SUCCESS


# ── 下台阶自动流程 ──

class 检查下台阶触发(py_trees.behaviour.Behaviour):
    """条件节点：检查是否收到 /r2/stairs_down 下台阶触发指令。

    黑板键：
        stairs_down_triggered : bool
    返回：
        SUCCESS —— 已触发，进入下台阶流程
        FAILURE —— 未触发，继续正常导航
    """

    def __init__(self, name: str = "检查下台阶触发"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        if _shared.get("stairs_down_triggered", False):
            self.feedback_message = "下台阶已触发"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "无下台阶指令"
        return py_trees.common.Status.FAILURE


class 下台阶自动流程(py_trees.behaviour.Behaviour):
    """动作节点：下台阶全自动流程（状态机驱动），使用 StairDownDetector。

    流程：
        1. STOPPING      —— 发送零速，确保底盘静止
        2. CAMERA_INIT   —— 启动深度相机 + StairDownDetector
        3. MOVING_SLOW   —— 以 0.05 m/s 慢速前进，轮询 StairDownDetector 检测楼梯
        4. STAIRS_DETECTED —— 检测到楼梯 → 锁底盘（同攀爬）→ 发送攀爬下降指令
        5. DONE          —— 完成，返回 SUCCESS

    若相机或检测器不可用，跳过检测直接执行攀爬下降。

    黑板键（读）：
        r2_comm, stairs_down_* 系列参数
    黑板键（写）：
        stairs_down_triggered, climbing_active (完成后清零)
    """

    # ── 状态枚举 ──
    (STOPPING, CAMERA_INIT, MOVING_SLOW, STAIRS_DETECTED, DONE, ERROR) = range(6)
    _STATE_NAMES = ["STOPPING", "CAMERA_INIT", "MOVING_SLOW", "STAIRS_DETECTED", "DONE", "ERROR"]

    def __init__(self, name: str = "下台阶自动流程"):
        super().__init__(name)
        self._state = self.STOPPING
        self._state_enter_time = 0.0
        self._detector = None
        self._step_command_sent = False

    def _enter_state(self, new_state: int) -> None:
        self._state = new_state
        self._state_enter_time = time.monotonic()
        self.feedback_message = f"状态 → {self._STATE_NAMES[new_state]}"

    def initialise(self) -> None:
        """首次 tick 前重置状态机。"""
        self._state = self.STOPPING
        self._state_enter_time = time.monotonic()
        if self._detector is not None:
            self._detector.reset_detection()
        self._step_command_sent = False
        self.feedback_message = "下台阶流程启动"

    def update(self) -> py_trees.common.Status:
        r2: R2Communication = _shared["r2_comm"]

        # ═══════════════════════════════════════════
        # 状态 0: STOPPING —— 确保底盘静止
        # ═══════════════════════════════════════════
        if self._state == self.STOPPING:
            r2.send_chassis_velocity(0.0, 0.0, 0.0)
            # 等待 0.5 秒让机器人真正停下
            if time.monotonic() - self._state_enter_time > 0.5:
                if _CAMERA_AVAILABLE and _STAIR_DETECTOR_AVAILABLE:
                    self._enter_state(self.CAMERA_INIT)
                else:
                    self.logger.warning(
                        f"[{self.name}] 深度相机/检测器不可用，跳过检测，直接执行攀爬下降"
                    )
                    self._enter_state(self.STAIRS_DETECTED)
            return py_trees.common.Status.RUNNING

        # ═══════════════════════════════════════════
        # 状态 1: CAMERA_INIT —— 初始化 StairDownDetector
        # ═══════════════════════════════════════════
        if self._state == self.CAMERA_INIT:
            try:
                self._detector = StairDownDetector()
                if not self._detector.initialize():
                    self.logger.error(f"[{self.name}] StairDownDetector 初始化失败，跳过检测")
                    self._detector = None
                    self._enter_state(self.STAIRS_DETECTED)
                    return py_trees.common.Status.RUNNING

                self.logger.info(f"[{self.name}] StairDownDetector 就绪，开始慢速前进检测楼梯")
                self._enter_state(self.MOVING_SLOW)
            except Exception as e:
                self.logger.error(f"[{self.name}] 检测器启动异常: {e}")
                self._enter_state(self.STAIRS_DETECTED)
            return py_trees.common.Status.RUNNING

        # ═══════════════════════════════════════════
        # 状态 2: MOVING_SLOW —— 慢速前进 + StairDownDetector 检测
        # ═══════════════════════════════════════════
        if self._state == self.MOVING_SLOW:
            slow_speed = _shared.get("stairs_down_slow_speed", 0.05)
            camera_timeout = _shared.get("stairs_down_camera_timeout_ms", 5)

            # 持续发送慢速前进指令
            r2.send_chassis_velocity(slow_speed, 0.0, 0.0)

            # 尝试读取相机帧，喂给 StairDownDetector
            detected = False
            if self._detector is not None:
                try:
                    depth_frame, pixel_type = self._detector.camera.get_depth_frame(timeout=camera_timeout)
                    if depth_frame is not None:
                        result = self._detector.check_stair_down(depth_frame, pixel_type)
                        if self._detector.is_ready():
                            detected = True
                        if result.get('mean_distance') is not None:
                            self.feedback_message = (
                                f"慢行中 v={slow_speed:.2f}m/s | "
                                f"距离={result['mean_distance']*100:.1f}cm | "
                                f"有效比={result['valid_ratio']*100:.1f}% | "
                                f"{'⚠楼梯' if self._detector.is_ready() else '正常'}"
                            )
                except Exception as e:
                    self.logger.debug(f"[{self.name}] 检测器读取异常: {e}")

            # 超时保护：若 60 秒未检测到楼梯，强制触发（防止无限慢行）
            elapsed = time.monotonic() - self._state_enter_time
            if elapsed > 60.0:
                self.logger.warning(
                    f"[{self.name}] 超时 {elapsed:.1f}s 未检测到楼梯，强制触发下降"
                )
                detected = True

            if detected:
                self._enter_state(self.STAIRS_DETECTED)

            return py_trees.common.Status.RUNNING

        # ═══════════════════════════════════════════
        # 状态 3: STAIRS_DETECTED —— 锁底盘（同上坡）→ 攀爬下降
        # ═══════════════════════════════════════════
        if self._state == self.STAIRS_DETECTED:
            if not self._step_command_sent:
                self.logger.info(f"[{self.name}] 检测到楼梯！锁底盘 + 攀爬下降20cm")

                # ── 同上坡逻辑：清空移动目标 + 上锁 ──
                _shared["relative_command"] = (0.0, 0.0, 0.0)
                _shared["global_target"] = _shared["current_pose"][:2]
                _shared["target_yaw"] = None
                _shared["climbing_active"] = True

                # ── 暴力刹车 + 直写串口发送攀爬下降指令 ──
                try:
                    mav = r2._mav
                    for _ in range(3):
                        mav.chassis_velocity_cmd_send(0.0, 0.0, 0.0)
                    mav.climbing_cmd_send(CLIMBING_CMD_EXECUTE_DOWN_20CM)
                    self.logger.info(f"[{self.name}] 攀爬下降20cm 已直写串口 ✓")
                except Exception as e:
                    self.logger.error(f"[{self.name}] 攀爬下降指令发送失败: {e}")

                self._step_command_sent = True

            # 等待 0.5 秒确保指令已发出
            if time.monotonic() - self._state_enter_time > 0.5:
                self._enter_state(self.DONE)

            return py_trees.common.Status.RUNNING

        # ═══════════════════════════════════════════
        # 状态 4: DONE —— 流程完成
        # ═══════════════════════════════════════════
        if self._state == self.DONE:
            _shared["stairs_down_triggered"] = False
            self.feedback_message = "下台阶流程完成 ✓"
            return py_trees.common.Status.SUCCESS

        # ═══════════════════════════════════════════
        # 状态 5: ERROR —— 异常退出
        # ═══════════════════════════════════════════
        _shared["stairs_down_triggered"] = False
        self.feedback_message = "下台阶流程异常"
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """无论节点如何结束，确保底盘停止并释放相机资源。"""
        r2: R2Communication = _shared["r2_comm"]
        r2.send_chassis_velocity(0.0, 0.0, 0.0)
        _shared["stairs_down_triggered"] = False

        if self._detector is not None:
            try:
                self._detector.close()
            except Exception:
                pass
            self._detector = None

        self.feedback_message = f"下台阶流程结束 ({new_status})"


# ── 夹爪机构 ──

class 夹爪夹持角(py_trees.behaviour.Behaviour):
    """动作节点：夹爪移至 90° 夹持角度。"""
    def __init__(self, name: str = "夹爪夹持角"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_clamping_cmd(CLAMPING_CMD_MOVE_TO_PARALLEL)
        self.feedback_message = "夹爪 → 移至 90° 夹持角"
        return py_trees.common.Status.SUCCESS


class 夹爪闭合(py_trees.behaviour.Behaviour):
    """动作节点：闭合夹爪。"""
    def __init__(self, name: str = "夹爪闭合"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_clamping_cmd(CLAMPING_CMD_GRAB)
        self.feedback_message = "夹爪 → 闭合"
        return py_trees.common.Status.SUCCESS


class 夹爪复位(py_trees.behaviour.Behaviour):
    """动作节点：夹爪复位至 0°。"""
    def __init__(self, name: str = "夹爪复位"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_clamping_cmd(CLAMPING_CMD_MOVE_TO_RESET)
        self.feedback_message = "夹爪 → 复位 0°"
        return py_trees.common.Status.SUCCESS


class 夹爪释放(py_trees.behaviour.Behaviour):
    """动作节点：释放夹爪。"""
    def __init__(self, name: str = "夹爪释放"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        _shared["r2_comm"].send_clamping_cmd(CLAMPING_CMD_RELEASE)
        self.feedback_message = "夹爪 → 释放"
        return py_trees.common.Status.SUCCESS


class 计算PID速度(py_trees.behaviour.Behaviour):
    """动作节点：读取 _shared 中的 global_target，PID 追踪直到到达。

    兼容原有的 /move_relative 话题驱动模式。
    返回：RUNNING（追踪中）→ SUCCESS（到达）
    """

    def __init__(self, name: str = "计算PID速度"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        x, y, yaw = _shared["current_pose"]
        tx, ty = _shared["global_target"]
        r2: R2Communication = _shared["r2_comm"]

        # ── 攀爬锁：攀爬期间强制零速（直写串口，绕过连接检查）──
        if _shared.get("climbing_active"):
            try:
                r2._mav.chassis_velocity_cmd_send(0.0, 0.0, 0.0)
            except Exception:
                pass
            self.feedback_message = "攀爬中，底盘锁定"
            self._was_locked = True
            return py_trees.common.Status.SUCCESS

        # ── 锁刚释放时打印日志 ──
        if getattr(self, "_was_locked", False):
            self._was_locked = False
            self.feedback_message = f"🔓 攀爬锁已解除，恢复导航 dist={math.hypot(tx - x, ty - y):.3f}m"

        # ── 如果有待执行的纯旋转目标 ──
        target_yaw = _shared.get("target_yaw")
        if target_yaw is not None:
            err = norm_angle(target_yaw - yaw)
            if abs(err) < 0.06:  # 3.4° 容差
                r2.send_chassis_velocity(0.0, 0.0, 0.0)
                _shared["target_yaw"] = None
                _shared["global_target"] = (x, y)  # 同步位置，防止 PID 回拉
                self.feedback_message = f"旋转到位 err={math.degrees(err):.1f}°"
                return py_trees.common.Status.SUCCESS
            w = _shared["kp_heading"] * err
            w = max(-_shared["max_angular"], min(_shared["max_angular"], w))
            w = math.copysign(max(abs(w), 0.08), err)  # 最小角速度 0.08 rad/s
            r2.send_chassis_velocity(0.0, 0.0, w)
            self.feedback_message = f"旋转中: err={math.degrees(err):.1f}° w={w:.3f}"
            return py_trees.common.Status.RUNNING

        # ── 位置 PID 追踪 ──
        dist = math.hypot(tx - x, ty - y)

        if dist <= _shared["pos_tolerance"]:
            r2.send_chassis_velocity(0.0, 0.0, 0.0)
            self.feedback_message = f"到达目标 dist={dist:.3f}m"
            return py_trees.common.Status.SUCCESS

        desired_yaw = math.atan2(ty - y, tx - x)
        heading_err = norm_angle(desired_yaw - yaw)
        v = min(_shared["max_linear"], _shared["kp_dist"] * dist)
        if dist < _shared["slowdown_distance"]:
            v *= max(0.2, dist / _shared["slowdown_distance"])
        if abs(heading_err) > 0.8:
            v *= 0.2
        w = max(-_shared["max_angular"],
                min(_shared["max_angular"],
                    _shared["kp_heading"] * heading_err))
        r2.send_chassis_velocity(v, 0.0, w)
        self.feedback_message = f"PID: dist={dist:.3f}m v={v:.3f} w={w:.3f}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        r2: R2Communication = _shared["r2_comm"]
        r2.send_chassis_velocity(0.0, 0.0, 0.0)
        self.feedback_message = f"PID 终止 ({new_status})"


# ======================== 更新节点（辅助） ========================
class 更新当前位姿(py_trees.behaviour.Behaviour):
    """辅助动作节点：每次 tick 检查 odometry 消息是否存活。
    位姿数据由 _on_odom 回调直接写入 _shared，不需要此节点做 TF 查询。"""

    def __init__(self, name: str = "更新当前位姿"):
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        x, y, yaw = _shared["current_pose"]
        self.feedback_message = f"pose=({x:.3f},{y:.3f})"
        return py_trees.common.Status.SUCCESS


# ======================== 行为树宿主节点 ========================

class R2BehaviorTreeHost(Node):
    """R2 行为树宿主 ROS2 节点。

    负责：
    - 生命周期管理（串口、TF、里程计）
    - 20Hz 主循环驱动行为树
    - 维护黑板公共变量
    """

    def __init__(self):
        super().__init__("r2_behavior_tree")

        # ---------- 参数声明 ----------
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "odin1_base_link")
        self.declare_parameter("odom_topic", "/odin1/odometry")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("pos_tolerance", 0.03)
        self.declare_parameter("kp_dist", 1.2)
        self.declare_parameter("kp_heading", 1.2)
        self.declare_parameter("max_linear", 0.32)
        self.declare_parameter("max_angular", 0.55)
        self.declare_parameter("slowdown_distance", 0.10)
        self.declare_parameter("wall_yaw_offset", 0.0)  # 墙方向偏移 (rad)

        # ====== 比赛级加固参数 ======
        self.declare_parameter("watchdog_timeout_sec", 0.5)
        self.declare_parameter("gc_interval_sec", 10.0)
        self.declare_parameter("cpu_affinity_cores", [4, 5])

        # ====== Python GC 控制：禁用自动 GC，改用手动定时 ======
        gc.disable()
        self.get_logger().info("已禁用 Python 自动 GC，切换为手动定时触发")

        # ---------- 串口通信 ----------
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        serial_baudrate = self.get_parameter("serial_baudrate").get_parameter_value().integer_value
        try:
            self._r2 = R2Communication(port=serial_port, baudrate=serial_baudrate)
            self.get_logger().info(f"串口 {serial_port} 已打开")
        except Exception as e:
            self.get_logger().fatal(f"串口打开失败: {e}")
            raise

        # ---------- TF2 ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- map 帧位姿发布 ----------
        self.map_pose_pub = self.create_publisher(Pose2D, "/r2/map_pose", 10)

        # ---------- 里程计订阅 ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            self._on_odom,
            20,
        )

        # ---------- 相对指令订阅（用于动态下发目标） ----------
        self.move_relative_sub = self.create_subscription(
            Point, "/move_relative", self._on_move_relative, 10
        )

        # ---------- 机械臂指令订阅 ----------
        self.arm_sub = self.create_subscription(
            Int32, "/r2/arm_cmd", self._on_arm_cmd, 10
        )

        # ---------- 攀爬指令订阅 ----------
        self.climbing_sub = self.create_subscription(
            Int32, "/r2/climbing_cmd", self._on_climbing_cmd, 10
        )

        # ---------- 夹爪指令订阅 ----------
        self.clamping_sub = self.create_subscription(
            Int32, "/r2/clamping_cmd", self._on_clamping_cmd, 10
        )

        # ---------- 急停指令订阅 ----------
        self.estop_sub = self.create_subscription(
            Empty, "/r2/estop", self._on_estop, 10
        )

        # ---------- 下台阶指令订阅 ----------
        self.stairs_down_sub = self.create_subscription(
            Empty, "/r2/stairs_down", self._on_stairs_down, 10
        )

        # ---------- 吸附墙指令订阅 ----------
        self.snap_wall_sub = self.create_subscription(
            Empty, "/r2/snap_wall", self._on_snap_wall, 10
        )

        # ---------- 填入共享变量 ----------
        _shared["r2_comm"] = self._r2
        _shared["pos_tolerance"] = self.get_parameter("pos_tolerance").get_parameter_value().double_value
        _shared["odom_timeout_sec"] = self.get_parameter("odom_timeout_sec").get_parameter_value().double_value
        _shared["kp_dist"] = self.get_parameter("kp_dist").get_parameter_value().double_value
        _shared["kp_heading"] = self.get_parameter("kp_heading").get_parameter_value().double_value
        _shared["max_linear"] = self.get_parameter("max_linear").get_parameter_value().double_value
        _shared["max_angular"] = self.get_parameter("max_angular").get_parameter_value().double_value
        _shared["slowdown_distance"] = self.get_parameter("slowdown_distance").get_parameter_value().double_value
        _shared["wall_yaw_offset"] = self.get_parameter("wall_yaw_offset").get_parameter_value().double_value
        _shared["watchdog_timeout_sec"] = self.get_parameter("watchdog_timeout_sec").get_parameter_value().double_value


        # ---------- 构建行为树 ----------
        self.tree = self._build_tree()
        self.get_logger().info("行为树已构建")

        # ---------- 可视化：启动时打印树结构 ----------
        self._print_tree_structure()

        # ---------- 安装快照访问器（用于运行时状态可视化） ----------
        self._snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_visitor(self._snapshot_visitor)

        # ---------- 20Hz 定时器 ----------
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate_hz, self._tick_tree)
        self.get_logger().info(f"行为树以 {rate_hz}Hz 运行")

        # ---------- 可视化：低速状态打印定时器（每 2 秒一次） ----------
        self._status_timer = self.create_timer(2.0, self._print_tree_status)

        # ---------- 比赛加固：手动 GC 定时器 ----------
        gc_interval = self.get_parameter("gc_interval_sec").get_parameter_value().double_value
        self._gc_timer = self.create_timer(gc_interval, self._manual_gc)
        self.get_logger().info(f"手动 GC 每 {gc_interval:.1f}s 触发一次")

        # ---------- 比赛加固：串口状态检查定时器 ----------
        self._serial_check_timer = self.create_timer(1.0, self._check_serial_status)

    # ---- 里程计回调 ----
    def _on_odom(self, msg: Odometry) -> None:
        _shared["odom_last_arrival"] = time.monotonic()
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        pyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        _shared["current_pose"] = (px, py, pyaw)

    # ---- 相对指令回调 ----
    def _on_move_relative(self, msg: Point) -> None:
        dx, dy, dtheta = msg.x, msg.y, msg.z
        _shared["relative_command"] = (dx, dy, dtheta)
        self.get_logger().info(
            f"收到移动指令: dx={dx:.3f}, dy={dy:.3f}, dθ={math.degrees(dtheta):.1f}°"
        )

    # ---- 机械臂指令回调 ----
    def _on_arm_cmd(self, msg: Int32) -> None:
        action_id = msg.data
        name = "空闲" if action_id == 0 else "绘制KFS" if action_id == 1 else f"未知({action_id})"
        self._r2.send_arm_action(action_id)
        self.get_logger().info(f"收到机械臂指令: {name} (action_id={action_id})")

    # ---- 攀爬指令回调 ----
    def _on_climbing_cmd(self, msg: Int32) -> None:
        cmd_id = msg.data
        names = {0: "初始20cm", 1: "准备40cm", 2: "上爬20cm", 3: "上爬40cm", 4: "下降20cm"}
        name = names.get(cmd_id, f"未知({cmd_id})")

        # 清空移动目标、上锁
        _shared["relative_command"] = (0.0, 0.0, 0.0)
        _shared["global_target"] = _shared["current_pose"][:2]
        _shared["target_yaw"] = None
        _shared["climbing_active"] = True

        # 暴力刹车 + 直接写串口发送攀爬指令（绕过 R2Communication 连接检查）
        try:
            mav = self._r2._mav
            for _ in range(3):
                mav.chassis_velocity_cmd_send(0.0, 0.0, 0.0)
            mav.climbing_cmd_send(cmd_id)
            self.get_logger().info(
                f"收到攀爬指令: {name} (cmd_id={cmd_id})，底盘已锁，直写串口完成"
            )
        except Exception as e:
            self.get_logger().error(f"攀爬指令发送失败: {e}")

    def _unlock_climbing(self):
        """攀爬完成后由下一个移动指令解锁。"""
        _shared["climbing_active"] = False

    # ---- 夹爪指令回调 ----
    def _on_clamping_cmd(self, msg: Int32) -> None:
        cmd_id = msg.data
        names = {0: "夹持角度90°", 1: "闭合夹爪", 2: "复位0°", 3: "释放夹爪"}
        name = names.get(cmd_id, f"未知({cmd_id})")
        self._r2.send_clamping_cmd(cmd_id)
        self.get_logger().info(f"收到夹爪指令: {name} (cmd_id={cmd_id})")

    # ---- 急停指令回调 ----
    def _on_estop(self, msg: Empty) -> None:
        self._r2.send_chassis_velocity(0.0, 0.0, 0.0)
        # 清空移动目标，防止恢复后继续移动
        _shared["relative_command"] = (0.0, 0.0, 0.0)
        _shared["global_target"] = _shared["current_pose"][:2]
        self.get_logger().error("⚠️ 收到 ROS2 急停指令！底盘已制动")

    # ---- 下台阶指令回调 ----
    def _on_stairs_down(self, msg: Empty) -> None:
        """收到 /r2/stairs_down 话题 → 触发下台阶自动流程。"""
        _shared["stairs_down_triggered"] = True
        # 清空当前移动目标，确保导航流让路给下台阶流程
        _shared["relative_command"] = (0.0, 0.0, 0.0)
        _shared["global_target"] = _shared["current_pose"][:2]
        self.get_logger().info("📷 收到下台阶指令！即将启动：停速→相机检测→攀爬下降")

    # ---- 吸附墙指令回调 ----
    def _on_snap_wall(self, msg: Empty) -> None:
        """收到 /r2/snap_wall 话题 → 吸附到最近墙方向（含 wall_yaw_offset 偏移）。"""
        _, _, yaw0 = _shared["current_pose"]
        offset = _shared.get("wall_yaw_offset", 0.0)
        # 四个主方向 + 偏移
        raw_cardinals = [-math.pi, -math.pi / 2, 0.0, math.pi / 2, math.pi]
        cardinals = [norm_angle(c + offset) for c in raw_cardinals]
        # 去重
        seen = set()
        cardinals_unique = []
        for c in cardinals:
            key = round(c, 4)
            if key not in seen:
                seen.add(key)
                cardinals_unique.append(c)
        cardinals_unique.sort()
        # 找最近的墙方向（最小绝对角度差）
        best = min(cardinals_unique, key=lambda c: abs(norm_angle(c - yaw0)))
        _shared["target_yaw"] = norm_angle(best)
        _shared["global_target"] = _shared["current_pose"][:2]
        _shared["relative_command"] = (0.0, 0.0, 0.0)
        self.get_logger().info(
            f"🧱 吸附墙: 当前={math.degrees(yaw0):.1f}° "
            f"→ 目标={math.degrees(best):.1f}° (偏移={math.degrees(offset):.1f}°)"
        )

    # ---- 构建行为树 ----
    def _build_tree(self) -> py_trees.trees.BehaviourTree:
        """构建完整的 R2 导航行为树（比赛加固版）。

        结构概览：
        ┌── 序列：R2导航根序列 (memory)
        │   ├── 更新当前位姿 (动作)              ← 每 tick 刷新位姿
        │   ├── 选择器：定位保护 (无记忆, 每 tick 重评估)
        │   │   ├── 检查雷达重定位 (条件)         ← SUCCESS → 选择器通过
        │   │   └── 底盘紧急制动 (动作)           ← FAILURE → 紧急制动
        │   ├── 看门狗检查 (条件)                 ← 串口断线/树卡死时 FAILURE
        │   └── 选择器：任务分流 (无记忆)
        │       ├── 序列：下台阶任务 (memory)
        │       │   ├── 检查下台阶触发 (条件)     ← /r2/stairs_down 触发
        │       │   └── 下台阶自动流程 (动作)     ← 停速→相机→慢行→检测→下降
        │       └── 序列：导航执行流 (memory)
        │           ├── 设置相对目标 (动作)       ← 计算全局目标
        │           ├── 计算PID速度 (动作)         ← RUNNING 直到到点
        │           └── 是否到达目标 (条件)        ← 到达校验

        关键语义：
        - 序列 (Sequence)：子节点依次执行，任一失败则整体失败。
        - 选择器 (Selector)：子节点依次尝试，任一成功则整体成功。
          此处 Selector 无记忆 → 每 tick 重新评估"检查雷达重定位"，
          确保定位恢复后能自动退出紧急制动状态。
        - 看门狗：串口连续 0.5s 无指令 → 自动中止导航。
        - 任务分流：下台阶触发时自动切换，完成后回到正常导航。
        """
        # ── 定位保护选择器（无记忆，每 tick 重新评估）──
        定位保护 = py_trees.composites.Selector(
            name="定位保护",
            memory=False,
            children=[
                检查雷达重定位(),
                底盘紧急制动(),
            ],
        )

        # ── 下台阶任务序列 ──
        下台阶任务 = py_trees.composites.Sequence(
            name="下台阶任务",
            memory=True,
            children=[
                检查下台阶触发(),
                下台阶自动流程(),
            ],
        )

        # ── 导航执行序列（无记忆）──
        导航执行流 = py_trees.composites.Sequence(
            name="导航执行流",
            memory=False,
            children=[
                设置相对目标(),
                计算PID速度(),
                是否到达目标(),
            ],
        )

        # ── 任务分流选择器（下台阶优先）──
        任务分流 = py_trees.composites.Selector(
            name="任务分流",
            memory=False,
            children=[
                下台阶任务,
                导航执行流,
            ],
        )

        # ── 根序列（无记忆，每 tick 从零评估）──
        root = py_trees.composites.Sequence(
            name="R2导航根序列",
            memory=False,
            children=[
                更新当前位姿(),
                定位保护,
                看门狗检查(),
                任务分流,
            ],
        )

        return py_trees.trees.BehaviourTree(root)

    # ---- 20Hz 主循环 ----
    def _tick_tree(self) -> None:
        _shared["serial_connected"] = self._r2.is_connected
        # 首次 tick：用当前位姿初始化目标，防止 PID 误追踪到 (0,0)
        if _shared["global_target"] == (0.0, 0.0):
            _shared["global_target"] = _shared["current_pose"][:2]
        # TF 融合：更新 map 帧位姿
        self._update_tf_pose()
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"行为树 tick 异常: {e}")
            try:
                self._r2.send_chassis_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass

    # ---- TF 融合：map 帧位姿更新 ----
    def _update_tf_pose(self) -> None:
        """查询 map -> odom 变换，将 odom 帧位姿转换到 map 帧。"""
        try:
            # 获取 map -> odom 变换
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            tyaw = yaw_from_quat(q.x, q.y, q.z, q.w)

            # 获取 odom 帧下当前位姿
            ox, oy, oyaw = _shared["current_pose"]

            # 变换：先旋转 odom 位姿，再平移
            cos_t, sin_t = math.cos(tyaw), math.sin(tyaw)
            mx = tx + ox * cos_t - oy * sin_t
            my = ty + ox * sin_t + oy * cos_t
            myaw = norm_angle(tyaw + oyaw)

            _shared["current_map_pose"] = (mx, my, myaw)
            _shared["map_frame_available"] = True

            # 发布 map 帧位姿到 /r2/map_pose
            msg = Pose2D()
            msg.x = mx
            msg.y = my
            msg.theta = myaw
            self.map_pose_pub.publish(msg)
        except Exception:
            _shared["current_map_pose"] = None
            _shared["map_frame_available"] = False

    # ---- 比赛加固：手动 GC ----
    def _manual_gc(self) -> None:
        """手动触发 Python 垃圾回收（在空闲时隙执行）。"""
        before = gc.get_count()
        collected = gc.collect()
        self.get_logger().debug(
            f"手动 GC: 回收 {collected} 对象, 计数 {before} → {gc.get_count()}"
        )

    # ---- 比赛加固：串口状态监控 ----
    def _check_serial_status(self) -> None:
        """每秒检查串口连接状态，断线时记录警告。"""
        if not self._r2.is_connected:
            self.get_logger().warn(
                "⚠️ 串口断线! 后台正在自动重连...",
                throttle_duration_sec=5.0,
            )
        elif self._r2.is_connection_lost:
            self.get_logger().warn(
                "串口已断线，等待重连...",
                throttle_duration_sec=2.0,
            )

    # ---- 生命周期 ----
    def destroy_node(self) -> None:
        """安全关闭：停止底盘、关闭串口、恢复 GC。"""
        self._r2.send_chassis_velocity(0.0, 0.0, 0.0)
        self._r2.close()
        gc.enable()  # 恢复 Python 自动 GC
        super().destroy_node()

    def stop_chassis(self) -> None:
        """便捷方法：紧急停止底盘。"""
        self._r2.send_chassis_velocity(0.0, 0.0, 0.0)

    # ==================== 可视化方法 ====================

    def _print_tree_structure(self) -> None:
        """启动时打印行为树静态结构（控制台 Unicode）。"""
        root = self.tree.root
        tree_str = py_trees.display.unicode_tree(
            root,
            show_status=False,
        )
        self.get_logger().info(
            f"\n========== R2 行为树结构 ==========\n{tree_str}\n====================================="
        )

    def _print_tree_status(self) -> None:
        """定期打印行为树运行时状态（含各节点 SUCCESS/FAILURE/RUNNING 标记）。"""
        root = self.tree.root
        # 从快照访问器获取各节点最近一次 tick 的状态
        visited = self._snapshot_visitor.visited
        previously_visited = self._snapshot_visitor.previously_visited
        tree_str = py_trees.display.unicode_tree(
            root,
            show_status=True,
            visited=visited,
            previously_visited=previously_visited,
        )
        self.get_logger().info(
            f"\n========== R2 行为树运行时状态 ==========\n{tree_str}\n============================================"
        )

    def save_tree_dot(self, target_dir: str | None = None) -> dict:
        """将行为树渲染为 Graphviz dot 图文件 (SVG/PNG)。

        Args:
            target_dir: 输出目录，默认为当前工作目录下的 bt_snapshot/
        Returns:
            dict: 生成的文件路径映射，如 {'svg': '/path/to/tree.svg', 'png': ...}
        """
        if target_dir is None:
            target_dir = os.path.join(os.getcwd(), "bt_snapshot")
        os.makedirs(target_dir, exist_ok=True)

        root = self.tree.root
        files = py_trees.display.render_dot_tree(
            root,
            target_directory=target_dir,
            with_blackboard_variables=True,
            with_qualified_names=False,
        )
        for fmt, path in files.items():
            self.get_logger().info(f"行为树 {fmt}: {path}")
        return files

    def print_blackboard(self) -> None:
        """打印当前黑板中所有运行时变量。"""
        from py_trees.display import unicode_blackboard
        board_str = unicode_blackboard()
        self.get_logger().info(
            f"\n========== 黑板变量 ==========\n{board_str}\n==============================="
        )

    def export_groot_xml(self, filepath: str = "r2_behavior_tree_groot.xml") -> str:
        """将当前行为树导出为 Groot2 (BehaviorTree.CPP v4) XML 文件。

        可以在 Groot2 中通过 File → Load Tree 加载查看树结构（静态拓扑）。

        Args:
            filepath: 输出 XML 路径
        Returns:
            str: 生成的 XML 字符串
        """
        import xml.dom.minidom as minidom
        import xml.etree.ElementTree as ET

        root = self.tree.root

        def _to_groot(node: py_trees.behaviour.Behaviour) -> ET.Element:
            if isinstance(node, py_trees.composites.Sequence):
                el = ET.Element("Sequence", {"name": node.name})
                for child in node.children:
                    el.append(_to_groot(child))
                return el
            elif isinstance(node, py_trees.composites.Selector):
                el = ET.Element("Fallback", {"name": node.name})
                for child in node.children:
                    el.append(_to_groot(child))
                return el
            elif isinstance(node, py_trees.composites.Parallel):
                el = ET.Element("Parallel", {"name": node.name})
                for child in node.children:
                    el.append(_to_groot(child))
                return el
            else:
                name = node.name
                if "检查" in name or "是否" in name:
                    return ET.Element("Condition", {"ID": name, "name": name})
                else:
                    return ET.Element("Action", {"ID": name, "name": name})

        # 收集叶子节点名
        leaf_names = []
        def _collect(node):
            if not hasattr(node, 'children') or not node.children:
                leaf_names.append(node.name)
            else:
                for c in node.children:
                    _collect(c)
        _collect(root)
        seen = set()
        leaf_names = [n for n in leaf_names if not (n in seen or seen.add(n))]

        root_el = ET.Element("root", {"BTCPP_format": "4"})
        bt_el = ET.SubElement(root_el, "BehaviorTree", {"ID": "R2Navigation"})
        bt_el.append(_to_groot(root))

        model_el = ET.SubElement(root_el, "TreeNodesModel")
        for name in leaf_names:
            if "检查" in name or "是否" in name:
                ET.SubElement(model_el, "Condition", {"ID": name, "editable": "true"})
            else:
                ET.SubElement(model_el, "Action", {"ID": name, "editable": "true"})

        raw = ET.tostring(root_el, encoding="unicode")
        dom = minidom.parseString(raw)
        xml_str = dom.toprettyxml(indent="  ")

        with open(filepath, "w", encoding="utf-8") as f:
            f.write(xml_str)
        self.get_logger().info(f"Groot2 XML 已导出: {filepath}")
        return xml_str


# ======================== 入口 ========================

def main():
    # ---- 比赛加固：CPU 亲和性绑定 ----
    try:
        cores = [4, 5]  # 绑定到 Orin Nano 第 4、5 核 (A78 大核)
        os.sched_setaffinity(0, set(cores))
        print(f"[R2] CPU 亲和性已绑定到核心: {cores}")
    except Exception as e:
        print(f"[R2] CPU 亲和性设置失败 (非关键): {e}")

    rclpy.init()
    node = R2BehaviorTreeHost()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，停止行为树")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

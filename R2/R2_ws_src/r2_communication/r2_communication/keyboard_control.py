#!/usr/bin/env python3
"""
R2 键盘遥控节点 —— 通过键盘直接控制 R2 机器人的各项功能。

按键映射:
  =========== 底盘速度 ===========
  w / s     前进 / 后退 (vx)
  a / d     左移 / 右移 (vy)
  q / e     左旋 / 右旋 (vw)
  空格       急停 (速度归零)

  =========== 机械臂动作 (V3) ===========
  1          ARM_ACTION_IDLE         (停止)
  2          ARM_DRAW_KFS_20cm       (绘制 KFS 20cm)
  3          ARM_DRAW_KFS_40cm       (绘制 KFS 40cm)
  4          ARM_DRAW_KFS_BELOW_20cm (从下方绘制 KFS 20cm)

  =========== 攀爬机构 ===========
  z          INIT_POSE         (初始 20cm 准备姿态)
  x          PREPARE_40CM      (升至 40cm 准备)
  c          EXECUTE_UP_20CM   (上爬 20cm)
  v          EXECUTE_UP_40CM   (上爬 40cm)
  b          EXECUTE_DOWN_20CM (下降 20cm)
  n          WEAPON_HEAD_CLAMP (武器头夹紧)
  m          WEAPON_ROD_DOCK   (武器杆对接)

  =========== 夹爪机构 ===========
  u          MOVE_TO_PARALLEL  (移至 90° 夹持角)
  i          GRAB              (闭合夹爪)
  o          MOVE_TO_RESET     (复位 0°)
  p          RELEASE           (释放夹爪)
  j          MOVE_TO_DOCK      (升至对接位)
  k          ADJUST            (底盘调整)
  l          START             (开始夹紧流程)

  =========== 全局 ===========
  h         显示帮助
  ESC / Ctrl+C  退出
"""

import os
import queue
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

from r2_communication.r2_communication import R2Communication
from r2_communication.R2_Protocol import (
    ARM_ACTION_IDLE,
    ARM_DRAW_KFS_20cm,
    ARM_DRAW_KFS_40cm,
    ARM_DRAW_KFS_BELOW_20cm,
    # 向后兼容
    ARM_DRAW_KFS,
    CLIMBING_CMD_INIT_POSE,
    CLIMBING_CMD_PREPARE_40CM,
    CLIMBING_CMD_EXECUTE_UP_20CM,
    CLIMBING_CMD_EXECUTE_UP_40CM,
    CLIMBING_CMD_EXECUTE_DOWN_20CM,
    CLIMBING_CMD_WEAPON_HEAD_CLAMP_START,
    CLIMBING_CMD_WEAPON_ROD_DOCK_START,
    CLAMPING_CMD_MOVE_TO_PARALLEL,
    CLAMPING_CMD_GRAB,
    CLAMPING_CMD_MOVE_TO_RESET,
    CLAMPING_CMD_RELEASE,
    CLAMPING_CMD_MOVE_TO_DOCK,
    CLAMPING_CMD_ADJUST,
    CLAMPING_CMD_START,
)

HELP_TEXT = """
╔══════════════════════════════════════════════════╗
║            R2 键盘遥控 — 帮助                      ║
╠══════════════════════════════════════════════════╣
║  底盘速度                                         ║
║    w/s : 前进/后退    a/d : 左移/右移               ║
║    q/e : 左旋/右旋    空格 : 急停                  ║
║                                                  ║
║  机械臂:  1:空闲  2:绘制KFS                        ║
║                                                  ║
║  攀爬:    z:初始20cm  x:准备40cm                   ║
║           c:上爬20cm  v:上爬40cm  b:下降20cm       ║
║           n:武器头夹紧  m:武器杆对接                ║
║                                                  ║
║  夹爪:    u:夹持角  i:闭合  o:复位  p:释放           ║
║           j:升至对接位  k:调整  l:开始夹紧           ║
║                                                  ║
║  全局:    h:帮助  ESC/Ctrl+C:退出                  ║
╚══════════════════════════════════════════════════╝
"""


class R2KeyboardControl(Node):
    """R2 键盘遥控 ROS2 节点（独立线程读取键盘，兼容 ros2 launch + emulate_tty）。"""

    def __init__(self):
        super().__init__("r2_keyboard_control")

        # ---- 参数声明 ----
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("linear_step", 0.5)
        self.declare_parameter("angular_step", 0.5)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._linear_step = self.get_parameter("linear_step").get_parameter_value().double_value
        self._angular_step = self.get_parameter("angular_step").get_parameter_value().double_value

        # ---- 初始化通信（必须在改终端之前，否则失败时终端无法恢复）----
        try:
            self._r2 = R2Communication(port=port, baudrate=baudrate)
            self.get_logger().info(f"串口 {port} 已打开，波特率 {baudrate}")
        except ConnectionError as e:
            self.get_logger().fatal(str(e))
            raise

        # ---- 当前速度状态 ----
        self._vx = 0.0
        self._vy = 0.0
        self._vw = 0.0

        # ---- 键盘输入：独立线程 → 队列 → ROS 回调消费 ----
        self._key_queue: queue.Queue = queue.Queue()
        self._tty_fd: int | None = None
        self._old_settings: list | None = None
        self._reading = True

        # 优先使用 sys.stdin（ros2 run / emulate_tty），不可用时回退到 /dev/tty
        if sys.stdin.isatty():
            self._kb_fd = sys.stdin.fileno()
            self._owns_fd = False
            self.get_logger().info("键盘输入: sys.stdin (TTY)")
        else:
            tty_path = "/dev/tty"
            if not os.path.exists(tty_path):
                self.get_logger().fatal(f"{tty_path} 不存在，无法读取键盘")
                raise RuntimeError(f"{tty_path} not found")
            self._kb_fd = os.open(tty_path, os.O_RDONLY)
            self._owns_fd = True
            self.get_logger().info(f"键盘输入: {tty_path}")

        self._tty_fd = self._kb_fd
        self._old_settings = termios.tcgetattr(self._kb_fd)
        tty.setcbreak(self._kb_fd)

        # 启动键盘读取线程
        self._kb_thread = threading.Thread(
            target=self._read_keys, name="R2-Kbd", daemon=True
        )
        self._kb_thread.start()

        # ---- 定时器：消费按键队列 + 持续发送速度（20Hz）----
        self._timer = self.create_timer(0.05, self._keyboard_callback)
        self._vel_timer = self.create_timer(0.05, self._send_velocity)

        self.get_logger().info("R2 键盘遥控已启动，按 h 查看帮助")

    # ---------- 键盘读取线程 ----------

    def _read_keys(self) -> None:
        """后台线程：阻塞读取 stdin/tty，字符放入队列。"""
        try:
            while self._reading:
                rlist, _, _ = select.select([self._kb_fd], [], [], 0.1)
                if not rlist:
                    continue
                data = os.read(self._kb_fd, 1)
                if not data:
                    break
                self._key_queue.put(data.decode("utf-8", errors="replace"))
        except (OSError, ValueError):
            pass  # fd 已关闭

    # ---------- ROS 定时回调 ----------

    def _keyboard_callback(self) -> None:
        """定时回调：从队列取出按键并执行对应操作。"""
        try:
            while True:
                key = self._key_queue.get_nowait()
                self._handle_key(key)
        except queue.Empty:
            return

    def _handle_key(self, key: str) -> None:
        """处理单个按键。"""
        # ---- 底盘速度控制 ----
        if key == "w":
            self._vx += self._linear_step
            self.get_logger().info(f"前进 vx={self._vx:.1f}")
        elif key == "s":
            self._vx -= self._linear_step
            self.get_logger().info(f"后退 vx={self._vx:.1f}")
        elif key == "a":
            self._vy -= self._linear_step
            self.get_logger().info(f"左移 vy={self._vy:.1f}")
        elif key == "d":
            self._vy += self._linear_step
            self.get_logger().info(f"右移 vy={self._vy:.1f}")
        elif key == "q":
            self._vw += self._angular_step
            self.get_logger().info(f"左转 vw={self._vw:.1f}")
        elif key == "e":
            self._vw -= self._angular_step
            self.get_logger().info(f"右转 vw={self._vw:.1f}")
        elif key == " ":
            self._vx = self._vy = self._vw = 0.0
            self.get_logger().info("急停！速度归零")

        # ---- 机械臂 (V3) ----
        elif key == "1":
            self._r2.send_arm_action(ARM_ACTION_IDLE)
            self.get_logger().info("机械臂 → 空闲/停止")
        elif key == "2":
            self._r2.send_arm_action(ARM_DRAW_KFS_20cm)
            self.get_logger().info("机械臂 → 绘制 KFS 20cm")
        elif key == "3":
            self._r2.send_arm_action(ARM_DRAW_KFS_40cm)
            self.get_logger().info("机械臂 → 绘制 KFS 40cm")
        elif key == "4":
            self._r2.send_arm_action(ARM_DRAW_KFS_BELOW_20cm)
            self.get_logger().info("机械臂 → 从下方绘制 KFS 20cm")

        # ---- 攀爬机构 ----
        elif key == "z":
            self._r2.send_climbing_cmd(CLIMBING_CMD_INIT_POSE)
            self.get_logger().info("攀爬 → 初始 20cm 姿态")
        elif key == "x":
            self._r2.send_climbing_cmd(CLIMBING_CMD_PREPARE_40CM)
            self.get_logger().info("攀爬 → 准备 40cm")
        elif key == "c":
            self._r2.send_climbing_cmd(CLIMBING_CMD_EXECUTE_UP_20CM)
            self.get_logger().info("攀爬 → 上爬 20cm")
        elif key == "v":
            self._r2.send_climbing_cmd(CLIMBING_CMD_EXECUTE_UP_40CM)
            self.get_logger().info("攀爬 → 上爬 40cm")
        elif key == "b":
            self._r2.send_climbing_cmd(CLIMBING_CMD_EXECUTE_DOWN_20CM)
            self.get_logger().info("攀爬 → 下降 20cm")
        elif key == "n":
            self._r2.send_climbing_cmd(CLIMBING_CMD_WEAPON_HEAD_CLAMP_START)
            self.get_logger().info("攀爬 → 武器头夹紧")
        elif key == "m":
            self._r2.send_climbing_cmd(CLIMBING_CMD_WEAPON_ROD_DOCK_START)
            self.get_logger().info("攀爬 → 武器杆对接")

        # ---- 夹爪机构 ----
        elif key == "u":
            self._r2.send_clamping_cmd(CLAMPING_CMD_MOVE_TO_PARALLEL)
            self.get_logger().info("夹爪 → 移至 90° 夹持角")
        elif key == "i":
            self._r2.send_clamping_cmd(CLAMPING_CMD_GRAB)
            self.get_logger().info("夹爪 → 闭合")
        elif key == "o":
            self._r2.send_clamping_cmd(CLAMPING_CMD_MOVE_TO_RESET)
            self.get_logger().info("夹爪 → 复位 0°")
        elif key == "p":
            self._r2.send_clamping_cmd(CLAMPING_CMD_RELEASE)
            self.get_logger().info("夹爪 → 释放")
        elif key == "j":
            self._r2.send_clamping_cmd(CLAMPING_CMD_MOVE_TO_DOCK)
            self.get_logger().info("夹爪 → 升至对接位")
        elif key == "k":
            self._r2.send_clamping_cmd(CLAMPING_CMD_ADJUST)
            self.get_logger().info("夹爪 → 底盘调整")
        elif key == "l":
            self._r2.send_clamping_cmd(CLAMPING_CMD_START)
            self.get_logger().info("夹爪 → 开始夹紧流程")

        # ---- 全局 ----
        elif key == "h":
            print(HELP_TEXT)
        elif key == "\x1b":  # ESC
            self.get_logger().info("ESC 按下，退出…")
            raise KeyboardInterrupt

    def _send_velocity(self) -> None:
        """发送当前累积速度（20Hz）。vw 取反对齐底盘实际方向。"""
        self._r2.send_chassis_velocity(self._vx, self._vy, -self._vw)

    # ---------- 资源清理 ----------

    def destroy_node(self) -> None:
        """销毁节点：停止键盘线程、恢复终端、关闭串口。"""
        self._reading = False
        if hasattr(self, "_tty_fd") and self._tty_fd is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
            if self._owns_fd:
                try:
                    os.close(self._tty_fd)
                except Exception:
                    pass
        self._r2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = R2KeyboardControl()
        print(HELP_TEXT)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"启动失败: {e}")
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

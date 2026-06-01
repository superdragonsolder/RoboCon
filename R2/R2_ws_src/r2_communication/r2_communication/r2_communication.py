"""
R2Communication - R2 机器人上下位机通信核心类
基于 MAVLink 协议 (R2_Protocol) 和 pyserial 实现串口通信。
"""

import threading
import time

import serial

from r2_communication.R2_Protocol import (
    # MAVLink 核心类
    MAVLink,
    MAVLink_message,
    # ARM_ACTION_CMD 枚举 (V3)
    ARM_ACTION_IDLE,
    ARM_DRAW_KFS_20cm,
    ARM_DRAW_KFS_40cm,
    ARM_DRAW_KFS_BELOW_20cm,
    # 向后兼容别名
    ARM_DRAW_KFS,
    # CLIMBING_CMD_TYPE 枚举
    CLIMBING_CMD_INIT_POSE,
    CLIMBING_CMD_PREPARE_40CM,
    CLIMBING_CMD_EXECUTE_UP_20CM,
    CLIMBING_CMD_EXECUTE_UP_40CM,
    CLIMBING_CMD_EXECUTE_DOWN_20CM,
    CLIMBING_CMD_WEAPON_HEAD_CLAMP_START,
    CLIMBING_CMD_WEAPON_ROD_DOCK_START,
    # CLAMPING_CMD_TYPE 枚举
    CLAMPING_CMD_MOVE_TO_PARALLEL,
    CLAMPING_CMD_GRAB,
    CLAMPING_CMD_MOVE_TO_RESET,
    CLAMPING_CMD_RELEASE,
    CLAMPING_CMD_MOVE_TO_DOCK,
    CLAMPING_CMD_ADJUST,
    CLAMPING_CMD_START,
)

# V3 向后兼容: ARM_DRAW_KFS 映射为 ARM_DRAW_KFS_20cm
ARM_DRAW_KFS = ARM_DRAW_KFS_20cm


class R2Communication:
    """R2 机器人上下位机通信管理类。

    封装了基于 MAVLink 协议的串口收发功能，线程安全。
    支持：底盘速度 / 机械臂 / 攀爬 / 夹爪 四类指令。
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200,
                 srcSystem: int = 1, srcComponent: int = 1):
        """初始化串口连接和 MAVLink 协议实例，启动后台接收线程。

        Args:
            port:         串口设备路径，如 /dev/ttyUSB0
            baudrate:     波特率，默认 115200
            srcSystem:    MAVLink 源系统 ID
            srcComponent: MAVLink 源组件 ID

        Raises:
            ConnectionError: 串口打开失败时抛出
        """
        try:
            self._serial = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            raise ConnectionError(
                f"无法打开串口 {port} (波特率 {baudrate}): {e}"
            )

        self._mav = MAVLink(
            self._serial, srcSystem=srcSystem, srcComponent=srcComponent
        )

        # 后台接收线程
        self._running = False
        self._recv_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._start_receive_thread()

    # ==================== 发送方法 ====================

    def send_chassis_velocity(self, vx: float, vy: float, vw: float) -> None:
        """发送底盘目标速度指令 (MSG ID 201)。

        Args:
            vx: X 轴线速度 (m/s)
            vy: Y 轴线速度 (m/s)
            vw: 旋转角速度 (rad/s)
        """
        with self._lock:
            self._mav.chassis_velocity_cmd_send(vx, vy, vw)

    def send_arm_action(self, action_id: int) -> None:
        """发送机械臂动作指令 (MSG ID 200)。

        Args:
            action_id: 动作 ID（V3）：
                ARM_ACTION_IDLE         (0) 停止/空闲
                ARM_DRAW_KFS_20cm       (1) 绘制 KFS 20cm
                ARM_DRAW_KFS_40cm       (2) 绘制 KFS 40cm
                ARM_DRAW_KFS_BELOW_20cm (3) 从下方绘制 KFS 20cm
        """
        with self._lock:
            self._mav.arm_control_send(action_id)

    def send_climbing_cmd(self, command_id: int) -> None:
        """发送攀爬机构指令 (MSG ID 202)。

        Args:
            command_id: 指令 ID →
                CLIMBING_CMD_INIT_POSE              (0) 初始 20cm 姿态
                CLIMBING_CMD_PREPARE_40CM           (1) 升至 40cm 准备
                CLIMBING_CMD_EXECUTE_UP_20CM        (2) 上爬 20cm
                CLIMBING_CMD_EXECUTE_UP_40CM        (3) 上爬 40cm
                CLIMBING_CMD_EXECUTE_DOWN_20CM      (4) 下降 20cm
                CLIMBING_CMD_WEAPON_HEAD_CLAMP_START(5) 武器头夹紧
                CLIMBING_CMD_WEAPON_ROD_DOCK_START  (6) 武器杆对接
        """
        with self._lock:
            self._mav.climbing_cmd_send(command_id)

    def send_clamping_cmd(self, command_id: int) -> None:
        """发送夹爪机构指令 (MSG ID 203)。

        Args:
            command_id: 指令 ID →
                CLAMPING_CMD_MOVE_TO_PARALLEL (0) 移至 90° 夹持角
                CLAMPING_CMD_GRAB             (1) 闭合夹爪
                CLAMPING_CMD_MOVE_TO_RESET    (2) 复位 0°
                CLAMPING_CMD_RELEASE          (3) 释放夹爪
                CLAMPING_CMD_MOVE_TO_DOCK     (4) 升至对接位
                CLAMPING_CMD_ADJUST           (5) 底盘调整
                CLAMPING_CMD_START            (6) 开始夹紧流程
        """
        with self._lock:
            self._mav.clamping_cmd_send(command_id)

    # ==================== 接收线程 ====================

    def _start_receive_thread(self) -> None:
        """启动后台接收线程。"""
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="R2-Recv", daemon=True
        )
        self._recv_thread.start()

    def _recv_loop(self) -> None:
        """后台持续读取串口、解析并打印收到的消息名称。"""
        while self._running:
            try:
                if self._serial.is_open and self._serial.in_waiting > 0:
                    raw = self._serial.read(self._serial.in_waiting)
                    msg = self._mav.parse_char(raw)
                    if msg is not None:
                        msg_type = msg.get_type()
                        print(f"[R2] 收到消息: {msg_type}")
                else:
                    time.sleep(0.01)
            except serial.SerialException as e:
                print(f"[R2] 串口异常: {e}")
                break
            except Exception as e:
                print(f"[R2] 接收错误: {e}")

    def stop_receiving(self) -> None:
        """停止后台接收线程。"""
        self._running = False
        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2.0)

    # ==================== 资源管理 ====================

    def close(self) -> None:
        """关闭串口连接并停止接收线程。"""
        self.stop_receiving()
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            print("[R2] 串口已关闭")

    @property
    def is_connected(self) -> bool:
        """是否已连接到串口。"""
        return self._serial is not None and self._serial.is_open

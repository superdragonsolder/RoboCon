"""
R2Communication - R2 机器人上下位机通信类
基于 R2_Protocol.py (MAVLink) 和 pyserial 实现串口通信。
支持串口断线自动重连（指数退避）。
"""

import logging
import threading
import time
import sys

# 确保错误信息不被静默丢弃
logging.basicConfig(
    level=logging.WARNING,
    format="[R2Comm] %(levelname)s: %(message)s",
    stream=sys.stderr,
)

import serial

from R2_Protocol import (
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

_logger = logging.getLogger(__name__)


class R2Communication:
    """R2 机器人上下位机通信管理类。

    封装了基于 MAVLink 协议的串口收发功能，支持：
    - 底盘速度控制
    - 机械臂动作控制
    - 攀爬机构控制
    - 夹爪机构控制
    - 后台多线程数据接收
    - 串口断线自动重连（指数退避，最大间隔 5s）
    """

    # 重连参数
    RECONNECT_BASE_DELAY = 0.1    # 初始重连间隔 (秒)
    RECONNECT_MAX_DELAY = 5.0     # 最大重连间隔 (秒)
    RECONNECT_BACKOFF = 2.0       # 退避因子

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200,
                 srcSystem: int = 1, srcComponent: int = 1):
        """初始化串口连接和 MAVLink 协议实例，并启动后台接收线程。

        Args:
            port: 串口设备路径，如 /dev/ttyUSB0
            baudrate: 波特率，默认 115200
            srcSystem: MAVLink 源系统 ID
            srcComponent: MAVLink 源组件 ID

        Raises:
            ConnectionError: 串口打开失败时抛出
        """
        # 保存参数用于重连
        self._port = port
        self._baudrate = baudrate
        self._srcSystem = srcSystem
        self._srcComponent = srcComponent

        # 状态标记
        self._last_send_time: float = 0.0
        self._reconnecting = False
        self._connection_lost = False

        # ---------- 打开串口 ----------
        self._serial: serial.Serial | None = None
        self._mav: MAVLink | None = None
        self._open_serial()

        # ---------- 后台接收线程 ----------
        self._running = False
        self._recv_thread: threading.Thread | None = None
        self._lock = threading.Lock()  # 保护发送操作的线程安全
        self._start_receive_thread()

    def _open_serial(self) -> None:
        """打开串口并初始化 MAVLink。失败时标记断线。"""
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=1)
            self._mav = MAVLink(
                self._serial,
                srcSystem=self._srcSystem,
                srcComponent=self._srcComponent,
            )
            self._connection_lost = False
            _logger.info(f"[R2] 串口 {self._port} 已打开")
        except (serial.SerialException, OSError) as e:
            self._serial = None
            self._mav = None
            self._connection_lost = True
            _logger.error(f"[R2] 串口打开失败 ({self._port}): {e}")
            raise ConnectionError(f"无法打开串口 {self._port}: {e}")

    def _reconnect_loop(self) -> None:
        """后台重连循环：指数退避重试直到连接恢复。"""
        if self._reconnecting:
            return
        self._reconnecting = True
        delay = self.RECONNECT_BASE_DELAY
        attempt = 0

        while self._running and self._connection_lost:
            attempt += 1
            _logger.warning(
                f"[R2] 串口重连尝试 #{attempt} (间隔 {delay:.1f}s)..."
            )
            # 先关闭旧句柄
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                self._mav = None

            # 尝试重连
            try:
                self._serial = serial.Serial(self._port, self._baudrate, timeout=1)
                self._mav = MAVLink(
                    self._serial,
                    srcSystem=self._srcSystem,
                    srcComponent=self._srcComponent,
                )
                self._connection_lost = False
                self._reconnecting = False
                _logger.info(f"[R2] ✅ 串口重连成功! (尝试 {attempt} 次)")
                return
            except (serial.SerialException, OSError) as e:
                _logger.error(f"[R2] 重连失败: {e}")

            # 指数退避等待
            time.sleep(min(delay, self.RECONNECT_MAX_DELAY))
            delay *= self.RECONNECT_BACKOFF

        self._reconnecting = False
        if self._connection_lost:
            _logger.error("[R2] 重连循环退出，串口仍未恢复")

    # ==================== 发送方法 ====================

    def send_chassis_velocity(self, vx: float, vy: float, vw: float) -> None:
        """发送底盘目标速度指令。

        Args:
            vx: X 轴线速度 (m/s)
            vy: Y 轴线速度 (m/s)
            vw: 旋转角速度 (rad/s)
        """
        self._last_send_time = time.monotonic()
        if self._connection_lost:
            _logger.debug("[R2] 串口断线，底盘速度指令丢弃")  # 降级为 DEBUG，不再刷屏
            return
        try:
            with self._lock:
                if self._mav is not None:
                    self._mav.chassis_velocity_cmd_send(vx, vy, vw)
        except (serial.SerialException, OSError) as e:
            _logger.error(f"[R2] 发送底盘速度失败: {e}")
            self._connection_lost = True
            if not self._reconnecting:
                threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def send_arm_action(self, action_id: int) -> None:
        """发送机械臂动作指令 (MSG ID 200)。

        Args:
            action_id: 动作 ID，使用 ARM_ACTION_CMD 枚举值（V3）：
                       ARM_ACTION_IDLE         (0) - 停止/空闲
                       ARM_DRAW_KFS_20cm       (1) - 绘制 KFS 20cm
                       ARM_DRAW_KFS_40cm       (2) - 绘制 KFS 40cm
                       ARM_DRAW_KFS_BELOW_20cm (3) - 从下方绘制 KFS 20cm
        """
        if self._connection_lost:
            _logger.warning("[R2] 串口断线，机械臂指令丢弃")
            return
        try:
            with self._lock:
                if self._mav is not None:
                    self._mav.arm_control_send(action_id)
        except (serial.SerialException, OSError) as e:
            _logger.error(f"[R2] 发送机械臂指令失败: {e}")
            self._connection_lost = True
            if not self._reconnecting:
                threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def send_climbing_cmd(self, command_id: int) -> None:
        """发送攀爬机构指令 (MSG ID 202)。

        Args:
            command_id: 指令 ID，使用 CLIMBING_CMD_TYPE 枚举值：
                        CLIMBING_CMD_INIT_POSE              (0) 初始 20cm 准备姿态
                        CLIMBING_CMD_PREPARE_40CM           (1) 升至 40cm 准备姿态
                        CLIMBING_CMD_EXECUTE_UP_20CM        (2) 执行 20cm 攀爬
                        CLIMBING_CMD_EXECUTE_UP_40CM        (3) 执行 40cm 攀爬
                        CLIMBING_CMD_EXECUTE_DOWN_20CM      (4) 执行 20cm 下降
                        CLIMBING_CMD_WEAPON_HEAD_CLAMP_START(5) 武器头夹紧
                        CLIMBING_CMD_WEAPON_ROD_DOCK_START  (6) 武器杆对接
        """
        if self._connection_lost:
            _logger.warning("[R2] 串口断线，攀爬指令丢弃")
            return
        try:
            with self._lock:
                if self._mav is not None:
                    self._mav.climbing_cmd_send(command_id)
        except (serial.SerialException, OSError) as e:
            _logger.error(f"[R2] 发送攀爬指令失败: {e}")
            self._connection_lost = True
            if not self._reconnecting:
                threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def send_clamping_cmd(self, command_id: int) -> None:
        """发送夹爪机构指令 (MSG ID 203)。

        Args:
            command_id: 指令 ID，使用 CLAMPING_CMD_TYPE 枚举值：
                        CLAMPING_CMD_MOVE_TO_PARALLEL (0) 移至夹持角度 (90°)
                        CLAMPING_CMD_GRAB             (1) 闭合夹爪
                        CLAMPING_CMD_MOVE_TO_RESET    (2) 复位至 0°
                        CLAMPING_CMD_RELEASE          (3) 释放夹爪
                        CLAMPING_CMD_MOVE_TO_DOCK     (4) 升至对接位
                        CLAMPING_CMD_ADJUST           (5) 底盘调整
                        CLAMPING_CMD_START            (6) 开始夹紧流程
        """
        if self._connection_lost:
            _logger.warning("[R2] 串口断线，夹爪指令丢弃")
            return
        try:
            with self._lock:
                if self._mav is not None:
                    self._mav.clamping_cmd_send(command_id)
        except (serial.SerialException, OSError) as e:
            _logger.error(f"[R2] 发送夹爪指令失败: {e}")
            self._connection_lost = True
            if not self._reconnecting:
                threading.Thread(target=self._reconnect_loop, daemon=True).start()

    # ==================== 接收线程 ====================

    def _start_receive_thread(self) -> None:
        """启动后台接收线程。"""
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, name="R2-Recv", daemon=True
        )
        self._recv_thread.start()

    def _recv_loop(self) -> None:
        """后台接收循环：持续读取串口数据，检测断线并触发重连。"""
        while self._running:
            try:
                if (self._serial is not None
                        and self._serial.is_open
                        and self._serial.in_waiting > 0):
                    raw = self._serial.read(self._serial.in_waiting)
                    if self._mav is not None:
                        msg = self._mav.parse_char(raw)
                        if msg is not None:
                            msg_type = msg.get_type()
                            _logger.debug(f"[R2] 收到消息: {msg_type}")
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as e:
                _logger.error(f"[R2] 串口读取异常: {e}")
                self._connection_lost = True
                if not self._reconnecting:
                    threading.Thread(
                        target=self._reconnect_loop, daemon=True
                    ).start()
                time.sleep(0.5)  # 避免重连期间高频重试
            except Exception as e:
                _logger.error(f"[R2] 接收错误: {e}")

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
            _logger.info("[R2] 串口已关闭")

    def __enter__(self) -> "R2Communication":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    @property
    def is_connected(self) -> bool:
        """是否已连接到串口。"""
        return (self._serial is not None
                and self._serial.is_open
                and not self._connection_lost)

    @property
    def last_send_time(self) -> float:
        """最近一次底盘速度指令的发送时间 (time.monotonic)。"""
        return self._last_send_time

    @property
    def is_connection_lost(self) -> bool:
        """串口是否处于断线状态。"""
        return self._connection_lost


# ==================== 使用示例 ====================
if __name__ == "__main__":
    # 方式一：手动管理生命周期
    r2 = R2Communication(port="/dev/ttyUSB0", baudrate=115200)
    try:
        # 发送底盘速度
        r2.send_chassis_velocity(vx=0.5, vy=0.0, vw=0.0)

        # 发送机械臂动作 (V3: 可选 20cm/40cm/下方)
        r2.send_arm_action(ARM_DRAW_KFS_20cm)

        # 发送攀爬指令
        r2.send_climbing_cmd(CLIMBING_CMD_PREPARE_40CM)

        # 发送夹爪指令
        r2.send_clamping_cmd(CLAMPING_CMD_GRAB)

        # 保持运行，让后台线程接收数据
        time.sleep(5)
    finally:
        r2.close()

    # 方式二：使用上下文管理器
    with R2Communication(port="/dev/ttyUSB0") as r2:
        r2.send_chassis_velocity(0.0, 0.0, 0.5)
        time.sleep(2)

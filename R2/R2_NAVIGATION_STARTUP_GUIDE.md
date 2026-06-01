# R2 机器人导航启动指南

> 最后更新: 2026-05-20 (V3 协议更新)

---
ssh ssr01@192.168.20.167
## 1. 系统概述

R2 机器人导航系统基于 ROS2 + MAVLink 串口协议，包含以下核心模块：

| 文件 | 功能 |
|------|------|
| `mission_sequence_pid.py` | PID 自主导航节点，执行预设任务序列 |
| `R2Communication.py` / `r2_communication.py` | MAVLink 串口通信封装 |
| `R2_Protocol.py` | MAVLink 协议实现（由 `R2.xml` 自动生成） |
| `keyboard_control.py` | 键盘手动遥控节点 |

**坐标系约定（右手系）：**

- X → 前 (forward)
- Y → 左 (left)
- Z → 上 (up)
- +Z 旋转 → 逆时针（左转）

**相机安装位姿（相对于 `base_link`）：**

| 方向 | 偏移量 | ROS 坐标 |
|------|--------|----------|
| 前 (X) | +32cm | x = 0.32 |
| 左 (Y) | +23cm | y = 0.23 |
| 高 (Z) | +83cm | z = 0.83 |

---
cd ~/connect/R2_ws/src/odin_ros_driver

# 用 set_param.sh 动态切换到 SLAM 模式（或直接改 YAML）
./set_param.sh custom_map_mode 1
cd ~/connect/R2_ws && source install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py

cd ~/connect/R2_ws/src/odin_ros_driver
./set_param.sh save_map 1

cd ~/connect/R2_ws/src/odin_ros_driver
./set_param.sh custom_map_mode 2

# 更新地图路径（如果文件名变了）
# 编辑 config/control_command.yaml 中的 relocalization_map_abs_path

# 前提：Odin 已启动（重定位模式下）
cd ~/connect/R2_ws && source install/setup.bash

# 默认摆动 30°
python3 ~/connect/relocalization_wiggle.py

## 2. 前置条件

### 2.1 硬件连接

- 串口线连接至上位机，默认设备路径 `/dev/ttyACM0`
- 确认串口权限：

```bash
ls -la /dev/ttyACM0
# 若无权限，将用户加入 dialout 组：
sudo usermod -aG dialout $USER
# 重新登录生效
```

### 2.2 软件依赖

```bash
# ROS2 (Humble 或更高)
# pyserial
pip3 install pyserial
```

### 2.3 编译工作空间

```bash
cd ~/connect/R2_ws
colcon build --packages-select r2_communication odin_ros_driver
source install/setup.bash
```

---

## 3. 启动方式

### 3.1 方式一：自动任务导航（PID 自主导航）

## 重定位
cd ~/connect/R2_ws
source install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py

## 检查map——odomTF发布 另开终端检查
ros2 run tf2_ros tf2_echo map odom

运行 `mission_sequence_pid.py`，机器人按预设任务序列自主移动。

```bash
# 1. Source ROS2 环境
cd ~/connect/R2_ws
source install/setup.bash

# 2. 启动导航节点
python3 ~/connect/mission_sequence_pid.py
```

**任务序列：**

| 步骤 | 动作 | 说明 |
|------|------|------|
| 0 | 攀爬初始化 | 发送 `CLIMBING_CMD=0` (INIT_POSE) |
| 1 | 前进 2m | PID 直线前进 |
| 2 | 停顿 5s | + 姿态吸附校准（90° 倍数） |
| 3 | 右转 90° | 原地旋转 |
| 4 | 前进 5m → 停 5s | ×2 次 |
| 5 | 前进 15m | 分 3 段 × 5m |
| 6 | 转向 180° → 返回起点 | 对齐初始位置和朝向 |

**运行时可通过 `/move_relative` 话题随时覆盖任务（见 3.3）。**

---

### 3.2 方式二：键盘手动遥控

```bash
cd ~/connect/R2_ws
source install/setup.bash
ros2 launch r2_communication keyboard_control.launch.py
```

**带自定义参数启动：**

```bash
cd ~/connect/R2_ws
source install/setup.bash
ros2 launch r2_communication keyboard_control.launch.py \
    port:=/dev/ttyUSB0 \
    linear_step:=0.2 \
    angular_step:=0.2
```

**按键映射：**

| 类别 | 按键 | 功能 |
|------|------|------|
| 底盘 | `W` / `S` | 前进 / 后退 |
| 底盘 | `A` / `D` | 左移 / 右移 |
| 底盘 | `Q` / `E` | 左转 / 右转 |
| 底盘 | `Space` | 急停（速度归零） |
| 机械臂 | `1` | 停止/空闲 |
| 机械臂 | `2` | 绘制 KFS |
| 攀爬 | `Z` | 初始 20cm 姿态 |
| 攀爬 | `X` | 升至 40cm 准备 |
| 攀爬 | `C` | 上爬 20cm |
| 攀爬 | `V` | 上爬 40cm |
| 攀爬 | `B` | 下降 20cm |
| 攀爬 | `N` | 武器头夹紧 |
| 攀爬 | `M` | 武器杆对接 |
| 夹爪 | `U` | 移至 90° 夹持角 |
| 夹爪 | `I` | 闭合夹爪 |
| 夹爪 | `O` | 复位 0° |
| 夹爪 | `P` | 释放夹爪 |
| 夹爪 | `J` | 升至对接位 |
| 夹爪 | `K` | 底盘调整 |
| 夹爪 | `L` | 开始夹紧流程 |
| 全局 | `H` | 显示帮助 |
| 全局 | `Esc` / `Ctrl+C` | 退出 |

---

### 3.3 方式三：通过 ROS2 话题发送相对位移指令

`mission_sequence_pid.py` 订阅 `/move_relative` 话题（类型 `geometry_msgs/msg/Point`），可在导航任务运行期间覆盖当前动作。

```bash
cd ~/connect/R2_ws
source install/setup.bash
python3 ~/connect/r2_behavior_tree.py

# ═══ 底盘移动 ═══
# W 前进
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.5, y: 0.0, z: 0.0}"
# S 后退
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: -0.5, y: 0.0, z: 0.0}"
# A 左移
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.5, z: 0.0}"
# D 右移
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: -0.5, z: 0.0}"
# Q 左转
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 1.5708}"
# E 右转
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -1.5708}"
# Space 急停
ros2 topic pub --once /r2/estop std_msgs/msg/Empty

# ═══ 机械臂 ═══
# 1 空闲
ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{data: 0}"
# 2 绘制KFS
ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{data: 1}"

# ═══ 攀爬 ═══
# Z 初始20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 0}"
# X 准备40cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 1}"
# C 上爬20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 2}"
# V 上爬40cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 3}"
# B 下降20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 4}"
# N 武器头夹紧
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 5}"
# M 武器杆对接
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 6}"

# ═══ 夹爪 ═══
# U 夹持角90°
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 0}"
# I 闭合
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 1}"
# O 复位0°
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 2}"
# P 释放
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 3}"
# J 升至对接位
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 4}"
# K 底盘调整
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 5}"
# L 开始夹紧流程
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 6}"

# ═══ 下台阶（相机自动检测） ═══
# 触发下台阶自动流程：停车 → 启动深度相机 → 0.05m/s慢行 → 检测到楼梯 → 攀爬下降20cm
ros2 topic pub --once /r2/stairs_down std_msgs/msg/Empty

# 下台阶流程详细说明见 3.4 节
> **字段含义**：`x` = dx (前向位移, m)，`y` = dy (左向位移, m)，`z` = dtheta (旋转增量, rad)




---

### 3.4 方式四：下台阶自动流程（深度相机检测）

通过 `/r2/stairs_down` 话题触发全自动下台阶流程，由行为树中的 `下台阶自动流程` 节点执行。

**触发命令：**

```bash
ros2 topic pub --once /r2/stairs_down std_msgs/msg/Empty
```

**自动流程（状态机）：**

| 状态 | 动作 | 超时/条件 |
|------|------|-----------|
| ① STOPPING | 发送零速，等待 0.5s 确保底盘静止 | 0.5s 后自动进入下一步 |
| ② CAMERA_INIT | 初始化 Berxel 100R 深度相机，启动深度流 | 失败则跳过检测直接下降 |
| ③ MOVING_SLOW | 以 **0.05 m/s** 慢速前进，每 tick 轮询相机检测台阶 | 最长 60s 超时保护 |
| ④ STAIRS_DETECTED | 停速 → 等待 0.3s → 发送 `CLIMBING_CMD_DOWN_20CM` | 检测到台阶高度突变时触发 |
| ⑤ DONE | 完成，行为树返回 SUCCESS，恢复正常导航 | — |

**楼梯检测逻辑：**

- 使用滑动窗口（默认 3 帧）确认距离突变量
- 当 ROI 中心区域距离突增 **20±2cm** 时判定为检测到台阶
- 检测到后立即制动并执行攀爬下降

**可调参数（`_shared` 默认值）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `stairs_down_step_height_cm` | 20.0 | 台阶高度 (cm) |
| `stairs_down_tolerance_cm` | 2.0 | 检测容差 (cm) |
| `stairs_down_confirm_frames` | 3 | 确认帧数 |
| `stairs_down_conversion_factor` | 7900.0 | 深度像素→距离转换因子 |
| `stairs_down_roi_size` | 10 | 测量区域 (px) |
| `stairs_down_slow_speed` | 0.05 | 慢行速度 (m/s) |
| `stairs_down_camera_timeout_ms` | 5 | 相机帧读取超时 (ms) |

**相机不可用时的降级策略：**

若 Berxel 相机未连接或驱动加载失败，`下台阶自动流程` 节点会自动跳过相机检测步骤，直接执行攀爬下降 20cm。

---

## 4. 关键 ROS2 参数参考

### 4.1 串口 & 坐标帧

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyACM0` | 串口设备路径 |
| `serial_baudrate` | `115200` | 波特率 |
| `global_frame` | `odom` | 全局坐标系 |
| `base_frame` | `odin1_base_link` | 机器人基座坐标系 |
| `odom_topic` | `/odin1/odometry_high` | 里程计话题 |
| `cmd_vel_topic` | `/cmd_vel` | 速度指令话题 |

### 4.2 PID 控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_dist` | 0.95 | 距离 P 增益 |
| `kp_heading` | 1.2 | 航向 P 增益 |
| `kp_lateral` | 0.8 | 横向 P 增益 |
| `kp_turn` | 1.4 | 旋转 P 增益 |
| `max_linear` | 0.32 m/s | 最大线速度 |
| `max_angular` | 0.55 rad/s | 最大角速度 |
| `pos_tolerance` | 0.10 m | 到位容差 |
| `turn_tolerance_rad` | 0.08 rad | 旋转到位容差 |

### 4.3 任务参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hold_sec` | 5.0 s | 停顿时间 |
| `segment_timeout_sec` | 120.0 s | 单段超时 |
| `align_snap_deg` | 90.0° | 姿态吸附角度 |
| `forward_straight_mode` | true | 直行模式 |
| `align_before_each_forward` | true | 每段前进前先对齐朝向 |

---

## 5. MAVLink 协议消息速查

协议定义文件：`R2.xml`，由 `R2_Protocol.py` 实现。

| 消息 ID | 名称 | 用途 |
|---------|------|------|
| 200 | `ARM_CONTROL` | 机械臂控制 |
| 201 | `CHASSIS_VELOCITY_CMD` | 底盘速度指令 (vx, vy, vw) |
| 202 | `CLIMBING_CMD` | 攀爬机构指令 |
| 203 | `CLAMPING_CMD` | 夹爪机构指令 |

**攀爬指令枚举 (`CLIMBING_CMD_TYPE`)：**

| 值 | 枚举名 | 说明 |
|----|--------|------|
| 0 | `CLIMBING_CMD_INIT_POSE` | 初始 20cm 准备姿态 |
| 1 | `CLIMBING_CMD_PREPARE_40CM` | 升至 40cm 准备 |
| 2 | `CLIMBING_CMD_EXECUTE_UP_20CM` | 上爬 20cm |
| 3 | `CLIMBING_CMD_EXECUTE_UP_40CM` | 上爬 40cm |
| 4 | `CLIMBING_CMD_EXECUTE_DOWN_20CM` | 下降 20cm |
| 5 | `CLIMBING_CMD_WEAPON_HEAD_CLAMP_START` | 武器头夹紧 |
| 6 | `CLIMBING_CMD_WEAPON_ROD_DOCK_START` | 武器杆对接 |

**夹爪指令枚举 (`CLAMPING_CMD_TYPE`)：**

| 值 | 枚举名 | 说明 |
|----|--------|------|
| 0 | `CLAMPING_CMD_MOVE_TO_PARALLEL` | 移至 90° 夹持角 |
| 1 | `CLAMPING_CMD_GRAB` | 闭合夹爪 |
| 2 | `CLAMPING_CMD_MOVE_TO_RESET` | 复位 0° |
| 3 | `CLAMPING_CMD_RELEASE` | 释放夹爪 |
| 4 | `CLAMPING_CMD_MOVE_TO_DOCK` | 升至对接位 |
| 5 | `CLAMPING_CMD_ADJUST` | 底盘调整 |
| 6 | `CLAMPING_CMD_START` | 开始夹紧流程 |

---

## 6. 常用调试命令

```bash
# 查看 TF 变换是否就绪
ros2 run tf2_ros tf2_echo odom odin1_base_link

# 查看里程计数据
ros2 topic echo /odin1/odometry_high

# 查看当前节点列表
ros2 node list

# 查看 mission_sequence_pid 的参数
ros2 param list /mission_sequence_pid

# 动态修改参数（示例：降低最大速度）
ros2 param set /mission_sequence_pid max_linear 0.2
```

---

## 7. 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 串口打开失败 | 设备路径错误或权限不足 | `ls /dev/ttyACM*` 确认设备；`sudo usermod -aG dialout $USER` |
| TF 未就绪 | odometry 节点未启动 | 检查 `odin_ros_driver` 是否运行 |
| 里程计超时 | 传感器数据中断 | 检查 Odometry 话题发布频率 |
| 机器人不移动 | 下位机未响应 | 检查串口连接和 MAVLink 心跳 |
| 任务超时 | 目标点无法到达 | 增大 `segment_timeout_sec` 或检查定位精度 |

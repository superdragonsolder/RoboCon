# R2 行为树系统 — 完整参考手册

> 文件: `r2_behavior_tree.py` | 最后更新: 2026-05-23

---

## 目录

1. [系统架构概览](#1-系统架构概览)
2. [启动流程](#2-启动流程)
3. [行为树结构](#3-行为树结构)
4. [节点详解](#4-节点详解)
   - [4.1 条件节点](#41-条件节点)
   - [4.2 动作节点](#42-动作节点)
   - [4.3 机构控制节点](#43-机构控制节点)
   - [4.4 下台阶自动流程](#44-下台阶自动流程)
5. [ROS2 话题控制](#5-ros2-话题控制)
6. [共享变量表](#6-共享变量表)
7. [比赛加固机制](#7-比赛加固机制)
8. [任务脚本使用](#8-任务脚本使用)

---

## 1. 系统架构概览

```
┌──────────────────────────────────────────────────────────────┐
│                      mission_script.py                       │
│               (任务脚本，通过 ROS2 Topic 发送指令)              │
└─────────────────────┬────────────────────────────────────────┘
                      │ ROS2 Topics (7个)
                      ▼
┌──────────────────────────────────────────────────────────────┐
│                  r2_behavior_tree.py                          │
│              (行为树宿主节点, 20Hz 主循环)                     │
│                                                              │
│  ┌── 串口: /dev/ttyACM0 @ 115200                             │
│  ├── 定位: /odin1/odometry (Odometry)                         │
│  └── 行为树: py_trees (Sequence / Selector)                   │
└─────────────────────┬────────────────────────────────────────┘
                      │ MAVLink 串口协议
                      ▼
┌──────────────────────────────────────────────────────────────┐
│                    底盘 MCU / 执行器                           │
│          (底盘电机、攀爬机构、机械臂、夹爪)                      │
└──────────────────────────────────────────────────────────────┘
```

### 核心文件

| 文件 | 功能 |
|:--|:--|
| `r2_behavior_tree.py` | **行为树主程序** — ROS2 节点, 20Hz tick, 所有节点定义 |
| `R2Communication.py` | 串口通信层, MAVLink 协议封装 |
| `R2_Protocol.py` | MAVLink 消息 ID 和命令常量 |
| `mission_script.py` | 比赛任务脚本模板, 通过 Topic 发送指令 |
| `howlong/berxel/stair_down_detector.py` | 下台阶检测器 (Berxel 深度相机 ROI 比例检测) |
| `howlong/berxel/src/camera_manager.py` | Berxel Hawk 相机管理 |
| `howlong/berxel/src/distance_measurer.py` | 深度距离测量工具 |

---

## 2. 启动流程

### 2.1 完整启动步骤

```bash
# 第 1 步: Source ROS2 环境
source ~/connect/R2_ws/install/setup.bash

# 第 2 步: 启动行为树（必须最先启动）
python3 ~/connect/r2_behavior_tree.py

# 第 3 步: 运行任务脚本（在另一个终端）
python3 ~/connect/mission_script.py
```

### 2.2 行为树启动时序

```
R2BehaviorTreeHost.__init__()
├── 1. 声明 ROS2 参数 (serial_port, rate_hz, PID 参数等)
├── 2. 禁用 Python 自动 GC，切换手动定时触发
├── 3. 打开串口 → R2Communication(port, baudrate)
├── 4. 创建 TF2 监听器
├── 5. 订阅里程计 → /odin1/odometry
├── 6. 订阅 7 个控制话题:
│   ├── /move_relative      → 底盘位移指令
│   ├── /r2/arm_cmd         → 机械臂指令
│   ├── /r2/climbing_cmd    → 攀爬指令
│   ├── /r2/clamping_cmd    → 夹爪指令
│   ├── /r2/estop           → 急停指令
│   ├── /r2/stairs_down     → 下台阶触发
│   └── /r2/snap_wall       → 吸附墙
├── 7. 初始化共享变量 _shared (PID 参数来自 ROS2 params)
├── 8. 构建行为树 → _build_tree()
├── 9. 打印树结构 (Unicode)
├── 10. 创建定时器:
│   ├── 20Hz 主循环 (_tick_tree)
│   ├── 2s 状态打印
│   ├── 10s 手动 GC
│   └── 1s 串口状态检查
└── 11. 进入 ROS2 spin 循环
```

### 2.3 关闭流程

```
Ctrl+C → destroy_node()
├── 1. 发送底盘零速
├── 2. 关闭串口
└── 3. 恢复 Python 自动 GC
```

### 2.4 ROS2 参数表（可通过命令行覆盖）

| 参数 | 默认值 | 说明 |
|:--|:--|:--|
| `serial_port` | `/dev/ttyACM0` | 串口设备路径 |
| `serial_baudrate` | `115200` | 串口波特率 |
| `rate_hz` | `20.0` | 行为树 tick 频率 |
| `odom_topic` | `/odin1/odometry` | 里程计话题 |
| `pos_tolerance` | `0.10` | 到达判定阈值 (m) |
| `kp_dist` | `0.95` | 距离 P 增益 |
| `kp_heading` | `1.2` | 航向 P 增益 |
| `max_linear` | `0.32` | 最大线速度 (m/s) |
| `max_angular` | `0.55` | 最大角速度 (rad/s) |
| `slowdown_distance` | `0.20` | 减速距离 (m) |
| `wall_yaw_offset` | `0.0` | 墙方向偏移 (rad) |
| `watchdog_timeout_sec` | `0.5` | 看门狗超时 (s) |
| `gc_interval_sec` | `10.0` | 手动 GC 间隔 (s) |

覆盖示例:
```bash
python3 r2_behavior_tree.py --ros-args -p serial_port:=/dev/ttyUSB0 -p max_linear:=0.25
```

---

## 3. 行为树结构

```
R2导航根序列 (Sequence, memory=False)           ← 每 tick 从零评估
├── 更新当前位姿 (动作)                          ← 始终 SUCCESS
├── 定位保护 (Selector, memory=False)            ← 无记忆，每 tick 重评估
│   ├── 检查雷达重定位 (条件)                     ← SUCCESS → 通过
│   └── 底盘紧急制动 (动作)                       ← FAILURE → 制动并中断链条
├── 看门狗检查 (条件)                             ← 串口断线时 FAILURE
└── 任务分流 (Selector, memory=False)            ← 下台阶优先
    ├── 下台阶任务 (Sequence, memory=True)       ← 有记忆，完成后回导航
    │   ├── 检查下台阶触发 (条件)                 ← /r2/stairs_down
    │   └── 下台阶自动流程 (动作)                 ← 6 状态状态机
    └── 导航执行流 (Sequence, memory=False)      ← 默认导航
        ├── 设置相对目标 (动作)                   ← 计算全局目标
        ├── 计算PID速度 (动作)                    ← RUNNING→SUCCESS
        └── 是否到达目标 (条件)                    ← 到达校验
```

### 组合器语义

| 组合器 | 记忆 | 行为 |
|:--|:--|:--|
| **Sequence** | `False` | 子节点从左到右顺序执行。任一 `FAILURE` 立即终止，返回 `FAILURE` |
| **Sequence** | `True` | 同上，但 `SUCCESS` 的子节点下一 tick 跳过（从上次中断处继续） |
| **Selector** | `False` | 每 tick 从第一个子节点重新评估。任一 `SUCCESS` 立即返回 |
| **Selector** | `True` | 记住上次成功的子节点，后续 tick 跳过前面的子节点 |

### 关键设计决策

1. **定位保护 Selector 无记忆** → 定位恢复后能自动退出紧急制动
2. **任务分流 Selector 无记忆** → 下台阶任务优先于导航
3. **下台阶 Sequence 有记忆** → 状态机跨 tick 保持
4. **根 Sequence 无记忆** → 每 tick 完整评估，确保安全链始终执行

---

## 4. 节点详解

### 4.1 条件节点

#### 检查雷达重定位 `检查雷达重定位()`

| 属性 | 值 |
|:--|:--|
| 类型 | Condition |
| 返回 | `SUCCESS` — 定位新鲜 / `FAILURE` — 超时 |

**逻辑**: 检查 `odom_last_arrival` 距当前时间是否超过 `odom_timeout_sec`（默认 0.5s）

#### 是否到达目标 `是否到达目标()`

| 属性 | 值 |
|:--|:--|
| 类型 | Condition |
| 返回 | `SUCCESS` — `dist ≤ pos_tolerance` / `FAILURE` — 未到达 |

**逻辑**: 计算 `current_pose` 与 `global_target` 的欧氏距离

#### 看门狗检查 `看门狗检查()`

| 属性 | 值 |
|:--|:--|
| 类型 | Condition |
| 返回 | `SUCCESS` — 正常 / `FAILURE` — 超时未发送指令 |

**逻辑**: 若 `R2Communication.last_send_time` 距当前时间 > `watchdog_timeout_sec`（0.5s），触发保护。此节点位于根序列中，`FAILURE` 会中断整个行为树链条，迫使底盘停止。

---

### 4.2 动作节点

#### 更新当前位姿 `更新当前位姿()`

| 属性 | 值 |
|:--|:--|
| 类型 | Action |
| 返回 | 始终 `SUCCESS` |

> 位姿数据由 `_on_odom()` 回调实时写入 `_shared`，此节点仅作标记位。

#### 底盘紧急制动 `底盘紧急制动()`

| 属性 | 值 |
|:--|:--|
| 类型 | Action |
| 返回 | `FAILURE`（向父节点传播异常） |

**行为**: 发送 `Vx=0, Vy=0, Vω=0`。返回 `FAILURE` 以确保上层 Sequence 感知到定位丢失并终止后续导航。

#### 设置相对目标 `设置相对目标()`

| 属性 | 值 |
|:--|:--|
| 类型 | Action |
| 返回 | 始终 `SUCCESS` |

**行为**:
- 读取 `relative_command` (dx, dy, dtheta)
- 无指令 (`dx=dy=dtheta≈0`): 直接放行
- 纯旋转 (`dx=dy=0, dtheta≠0`): **吸附到最近 90° 墙方向**（含 `wall_yaw_offset` 偏移），写入 `target_yaw`
- 位移指令: 计算全局目标坐标，同时**解锁攀爬锁** (`climbing_active=False`)

#### 计算PID速度 `计算PID速度()`

| 属性 | 值 |
|:--|:--|
| 类型 | Action |
| 返回 | `RUNNING` → `SUCCESS` |

**行为**:
1. **攀爬锁检查**: 若 `climbing_active=True`，直写串口 `V=0`，返回 `SUCCESS`（不阻塞树但锁定底盘）
2. **纯旋转追踪**: 若 `target_yaw ≠ None`，仅旋转直到 `|err| < 0.06 rad`（≈3.4°）
3. **位置 PID 追踪**: 距离误差 → 线速度, 航向误差 → 角速度

**PID 公式**:
- $v = \min(v_{max},\ k_p^{dist} \cdot dist)$，(距离 < `slowdown_distance` 时线性减速)
- 航向偏差 > 0.8 rad 时 $v \times 0.2$
- $\omega = \text{clamp}(k_p^{heading} \cdot heading\_err,\ -\omega_{max},\ \omega_{max})$

#### 便捷移动节点

| 节点 | 参数 | 等价调用 |
|:--|:--|:--|
| `前进(d)` | distance (m) | `_相对移动(dx=d)` |
| `后退(d)` | distance (m) | `_相对移动(dx=-d)` |
| `左移(d)` | distance (m) | `_相对移动(dy=d)` |
| `右移(d)` | distance (m) | `_相对移动(dy=-d)` |
| `左转90度()` | — | `_相对移动(dtheta=π/2)` |
| `右转90度()` | — | `_相对移动(dtheta=-π/2)` |
| `急停()` | — | `send_chassis_velocity(0,0,0)` |

> 所有 `_相对移动` 子类在 `initialise()` 时锁定当前位姿计算全局目标，`update()` 中执行 PID 追踪 → `terminate()` 发送零速。

---

### 4.3 机构控制节点

#### 机械臂

| 节点 | MAVLink 命令 | 说明 |
|:--|:--|:--|
| `机械臂空闲()` | `ARM_ACTION_IDLE` | 机械臂归位 |
| `机械臂绘制KFS()` | `ARM_DRAW_KFS_20cm` | 绘制 KFS 20cm |

#### 攀爬机构

| 节点 | MAVLink 命令 | 说明 |
|:--|:--|:--|
| `攀爬初始姿态()` | `CLIMBING_CMD_INIT_POSE` | 初始 20cm |
| `攀爬准备40cm()` | `CLIMBING_CMD_PREPARE_40CM` | 升至 40cm |
| `攀爬上爬20cm()` | `CLIMBING_CMD_EXECUTE_UP_20CM` | 上爬 20cm |
| `攀爬上爬40cm()` | `CLIMBING_CMD_EXECUTE_UP_40CM` | 上爬 40cm |
| `攀爬下降20cm()` | `CLIMBING_CMD_EXECUTE_DOWN_20CM` | 下降 20cm |

> **攀爬时底盘锁定机制**（上坡和下坡统一）：
> 1. 清空 `relative_command`, `global_target`, `target_yaw`
> 2. 设置 `climbing_active = True`（PID 强制零速）
> 3. 连续 3 次 `chassis_velocity_cmd_send(0,0,0)` 暴力刹车
> 4. **直写串口** `climbing_cmd_send()`（绕过 `R2Communication` 连接检查）
> 5. 下一个位移指令自动解锁 `climbing_active = False`

#### 夹爪机构

| 节点 | MAVLink 命令 | 说明 |
|:--|:--|:--|
| `夹爪夹持角()` | `CLAMPING_CMD_MOVE_TO_PARALLEL` | 移至 90° |
| `夹爪闭合()` | `CLAMPING_CMD_GRAB` | 闭合 |
| `夹爪复位()` | `CLAMPING_CMD_MOVE_TO_RESET` | 复位 0° |
| `夹爪释放()` | `CLAMPING_CMD_RELEASE` | 释放 |

---

### 4.4 下台阶自动流程

> **详细设计**：使用 `StairDownDetector`（Berxel 深度相机 + ROI 比例检测）

#### 触发链

```
/r2/stairs_down (Empty) → _on_stairs_down()
  → stairs_down_triggered = True
  → 清空移动目标
  → 行为树 "任务分流" Selector 优先进入下台阶任务
```

#### 6 状态状态机

```
STOPPING ──0.5s──▶ CAMERA_INIT ──成功──▶ MOVING_SLOW ──检测到──▶ STAIRS_DETECTED ──0.5s──▶ DONE
    │                   │                    │                      │
    │                   └──失败──▶ STAIRS_DETECTED                  │
    │                                           │                  │
    └──相机不可用────────────────────────────────┘                  │
    ┌──────────────────────────────────────────────────────────────┘
    ▼
  SUCCESS / FAILURE (terminate: 停底盘 + 释放相机)
```

| 状态 | 名称 | 行为 |
|:--|:--|:--|
| 0 | `STOPPING` | 零速 0.5s 确保静止。相机可用 → CAMERA_INIT，否则 → STAIRS_DETECTED |
| 1 | `CAMERA_INIT` | 创建 `StairDownDetector` + 开启深度流。失败 → STAIRS_DETECTED |
| 2 | `MOVING_SLOW` | **0.05 m/s 慢速前进**，每帧调用 `StairDownDetector.check_stair_down()`。**60s 超时**强制触发 |
| 3 | `STAIRS_DETECTED` | **同上坡逻辑**：`climbing_active=True` + 暴力刹车 3 次 + 直写串口 `CLIMBING_CMD_EXECUTE_DOWN_20CM` |
| 4 | `DONE` | 清零 `stairs_down_triggered`，返回 `SUCCESS` |
| 5 | `ERROR` | 返回 `FAILURE` |

#### StairDownDetector 检测参数

| 参数 | 默认值 | 说明 |
|:--|:--|:--|
| ROI 尺寸 | 300×20 px | 图像中心横条 |
| 目标距离 | 0.57 m (57 cm) | 地面到相机的期望距离 |
| 容差 | ±0.03 m (3 cm) | 有效距离范围 |
| 阈值比例 | 95% | ROI 内有效像素占比超过此值 → 检测到楼梯 |
| 确认帧数 | 5 帧 | 连续满足阈值才触发 |

---

## 5. ROS2 话题控制

### 话题一览

| 话题 | 消息类型 | 方向 | 功能 |
|:--|:--|:--|:--|
| `/move_relative` | `geometry_msgs/Point` | 输入 | 底盘相对位移 |
| `/r2/arm_cmd` | `std_msgs/Int32` | 输入 | 机械臂动作 |
| `/r2/climbing_cmd` | `std_msgs/Int32` | 输入 | 攀爬机构 |
| `/r2/clamping_cmd` | `std_msgs/Int32` | 输入 | 夹爪机构 |
| `/r2/estop` | `std_msgs/Empty` | 输入 | 紧急制动 |
| `/r2/stairs_down` | `std_msgs/Empty` | 输入 | 触发下台阶流程 |
| `/r2/snap_wall` | `std_msgs/Empty` | 输入 | 吸附最近墙方向 |
| `/odin1/odometry` | `nav_msgs/Odometry` | 输入 | 里程计定位 |

### 底盘移动 — `/move_relative`

`geometry_msgs/Point`: `x`=dx(m), `y`=dy(m), `z`=dθ(rad)

```bash
# 前进 1m
ros2 topic pub --once /move_relative geometry_msgs/Point "{x: 1.0, y: 0.0, z: 0.0}"

# 左移 0.3m
ros2 topic pub --once /move_relative geometry_msgs/Point "{x: 0.0, y: 0.3, z: 0.0}"

# 右移 0.3m
ros2 topic pub --once /move_relative geometry_msgs/Point "{x: 0.0, y: -0.3, z: 0.0}"

# 左转 90° (π/2)
ros2 topic pub --once /move_relative geometry_msgs/Point "{x: 0.0, y: 0.0, z: 1.5708}"

# 右转 90° (-π/2)
ros2 topic pub --once /move_relative geometry_msgs/Point "{x: 0.0, y: 0.0, z: -1.5708}"

# 组合: 前进 1m + 左转 45°
ros2 topic pub --once /move_relative geometry_msgs/Point "{x: 1.0, y: 0.0, z: 0.7854}"
```

### 机械臂 — `/r2/arm_cmd`

`std_msgs/Int32`: `data` = 0~1

```bash
ros2 topic pub --once /r2/arm_cmd std_msgs/Int32 "{data: 0}"   # 空闲
ros2 topic pub --once /r2/arm_cmd std_msgs/Int32 "{data: 1}"   # 绘制 KFS
```

### 攀爬机构 — `/r2/climbing_cmd`

`std_msgs/Int32`: `data` = 0~4

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/Int32 "{data: 0}"  # 初始 20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/Int32 "{data: 1}"  # 准备 40cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/Int32 "{data: 2}"  # 上爬 20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/Int32 "{data: 3}"  # 上爬 40cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/Int32 "{data: 4}"  # 下降 20cm
```

### 夹爪 — `/r2/clamping_cmd`

`std_msgs/Int32`: `data` = 0~3

```bash
ros2 topic pub --once /r2/clamping_cmd std_msgs/Int32 "{data: 0}"  # 夹持角 90°
ros2 topic pub --once /r2/clamping_cmd std_msgs/Int32 "{data: 1}"  # 闭合
ros2 topic pub --once /r2/clamping_cmd std_msgs/Int32 "{data: 2}"  # 复位 0°
ros2 topic pub --once /r2/clamping_cmd std_msgs/Int32 "{data: 3}"  # 释放
```

### 急停 / 下台阶 / 吸附墙

```bash
# 急停
ros2 topic pub --once /r2/estop std_msgs/msg/Empty

# 下台阶自动流程
ros2 topic pub --once /r2/stairs_down std_msgs/msg/Empty

# 吸附最近墙方向
ros2 topic pub --once /r2/snap_wall std_msgs/msg/Empty
```

---

## 6. 共享变量表

模块级 `_shared` 字典是行为树所有节点的公共黑板。

### 位姿与目标

| 键 | 类型 | 默认值 | 读写者 |
|:--|:--|:--|:--|
| `current_pose` | `(x, y, yaw)` | `(0,0,0)` | `_on_odom` 写入, 所有节点读取 |
| `global_target` | `(tx, ty)` | `(0,0)` | `设置相对目标` / 回调写入, `计算PID速度` 读取 |
| `target_yaw` | `float\|None` | `None` | `设置相对目标` / 吸附墙写入, `计算PID速度` 读取 |
| `relative_command` | `(dx,dy,dθ)` | `(0,0,0)` | `/move_relative` 回调写入, `设置相对目标` 消费 |

### PID 参数

| 键 | 默认值 | 说明 |
|:--|:--|:--|
| `pos_tolerance` | `0.10` | 到达判定距离 (m) |
| `kp_dist` | `0.95` | 距离 P 增益 |
| `kp_heading` | `1.2` | 航向 P 增益 |
| `max_linear` | `0.32` | 最大线速度 (m/s) |
| `max_angular` | `0.55` | 最大角速度 (rad/s) |
| `slowdown_distance` | `0.20` | 减速开始距离 (m) |

### 安全

| 键 | 默认值 | 说明 |
|:--|:--|:--|
| `odom_last_arrival` | `None` | 最近里程计到达时间 |
| `odom_timeout_sec` | `0.5` | 定位超时阈值 |
| `watchdog_timeout_sec` | `0.5` | 看门狗超时阈值 |
| `serial_connected` | `True` | 串口连接状态 |
| `climbing_active` | `False` | **攀爬锁** — True 时 PID 强制零速 |
| `wall_yaw_offset` | `0.0` | 墙方向偏移 (rad) |

### 下台阶检测

| 键 | 默认值 | 说明 |
|:--|:--|:--|
| `stairs_down_triggered` | `False` | 下台阶触发标记 |
| `stairs_down_slow_speed` | `0.05` | 慢行速度 (m/s) |
| `stairs_down_camera_timeout_ms` | `5` | 相机帧超时 (ms) |

> 注: `stairs_down_step_height_cm`, `stairs_down_tolerance_cm`, `stairs_down_confirm_frames`, `stairs_down_conversion_factor`, `stairs_down_roi_size` 保留但不再使用（检测参数已内置于 `StairDownDetector`）。

---

## 7. 比赛加固机制

### 7.1 看门狗 (Watchdog)

- **节点**: `看门狗检查()`
- **位置**: 根序列第 3 个节点
- **逻辑**: 若 `R2Communication.last_send_time` 距现在 > 0.5s → `FAILURE` → 根序列中断
- **效果**: 串口断线或树卡死时自动停止底盘

### 7.2 定位保护

- **节点**: `定位保护` Selector (无记忆)
- **逻辑**: 每 tick 检查 `odom_last_arrival`。定位丢失 > 0.5s → `底盘紧急制动`
- **恢复**: 无记忆设计 → 定位恢复后下一 tick 自动通过

### 7.3 手动 GC

- Python 自动 GC 已禁用
- 每 10s 手动触发 `gc.collect()`
- 避免 GC 暂停导致行为树 tick 抖动

### 7.4 异常保护

```python
# _tick_tree 中的异常捕获
try:
    self.tree.tick()
except Exception:
    self._r2.send_chassis_velocity(0.0, 0.0, 0.0)  # 兜底制动
```

---

## 8. 任务脚本使用

### 8.1 动作函数速查

| 函数 | 说明 |
|:--|:--|
| `急停()` | 立即制动 |
| `前进(d)` | 前进 d 米 (负数=后退) |
| `左移(d)` | 左移 d 米 (负数=右移) |
| `右移(d)` | 右移 d 米 |
| `旋转(deg)` | 旋转 deg° (正=左转, 负=右转) |
| `组合移动(前向,左移,旋转)` | 同时前进+平移+旋转 |
| `机械臂(n)` | 0=空闲, 1=绘制KFS |
| `攀爬(n)` | 0=初始, 1=准备40, 2=上20, 3=上40, 4=下20 |
| `夹爪(n)` | 0=夹持角, 1=闭合, 2=复位, 3=释放 |
| `下台阶()` | 全自动相机检测下台阶 |
| `吸附墙()` | 旋转到最近墙方向 |

### 8.2 等待函数

| 函数 | 说明 |
|:--|:--|
| `等待秒(s)` | 简单等待 |
| `等待前进完成(d, 额外秒)` | 估算移动时间 + 缓冲 |
| `等待旋转完成(a, 额外秒)` | 估算旋转时间 + 缓冲 |
| `等待下台阶完成(最大秒)` | 最长 65s (内部 60s 超时) |
| `等待吸附完成()` | 固定 5s |

### 8.3 计时参考

| 动作 | 估算时间 (含余量) |
|:--|:--|
| 1m 前进 | ~5s |
| 2m 前进 | ~9s |
| 5m 前进 | ~20s |
| 45° 旋转 | ~4s |
| 90° 旋转 | ~6s |
| 180° 旋转 | ~11s |
| 上/下坡 | ~10s |
| 下台阶自动流程 | ~65s (最长) |

### 8.4 示例任务序列

```python
def 执行任务序列():
    急停()
    等待秒(0.5)
    攀爬(0)        # 初始姿态
    等待秒(2.0)
    吸附墙()
    等待吸附完成()

    前进(1.1)
    等待前进完成(1.1)

    旋转(-90)      # 右转 90°
    等待秒(6.0)

    前进(0.3)
    等待前进完成(0.2)

    攀爬(2)        # 上爬 20cm
    等待秒(10.0)

    前进(0.52)
    等待前进完成(0.52)

    攀爬(2)        # 上爬 20cm
    等待秒(10.0)

    前进(0.50)
    等待前进完成(0.50)

    下台阶()        # 自动检测下台阶
    等待下台阶完成()

    急停()
    print("✅ 任务完成")
```

# R2 行为树 ROS2 Topic 控制参考

> 文件: `r2_behavior_tree.py` | 最后更新: 2026-05-15

---

## 1. 概览

行为树节点可通过以下 5 个 ROS2 话题直接控制：

| 话题 | 消息类型 | 功能 |
|:--|:--|:--|
| `/move_relative` | `geometry_msgs/msg/Point` | 底盘相对位移 |
| `/r2/arm_cmd` | `std_msgs/msg/Int32` | 机械臂动作 |
| `/r2/climbing_cmd` | `std_msgs/msg/Int32` | 攀爬机构 |
| `/r2/clamping_cmd` | `std_msgs/msg/Int32` | 夹爪机构 |
| `/r2/estop` | `std_msgs/msg/Empty` | 紧急制动 |
| `/r2/stairs_down` | `std_msgs/msg/Empty` | 触发下台阶自动流程 |

> **前提**: 行为树节点 (`python3 ~/connect/r2_behavior_tree.py`) 必须先启动。

---

## 2. 底盘移动 — `/move_relative`

**消息**: `geometry_msgs/msg/Point`
**字段**: `x`=dx(前向位移,m), `y`=dy(左向位移,m), `z`=dtheta(旋转增量,rad)

### 2.1 前进

```bash
# 前进 1 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"

# 前进 0.5 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.5, y: 0.0, z: 0.0}"

# 前进 2 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

对应节点: `前进(distance)`

### 2.2 后退

```bash
# 后退 0.5 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: -0.5, y: 0.0, z: 0.0}"

# 后退 1 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: -1.0, y: 0.0, z: 0.0}"
```

对应节点: `后退(distance)`

### 2.3 左移

```bash
# 左移 0.3 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.3, z: 0.0}"

# 左移 0.5 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.5, z: 0.0}"
```

对应节点: `左移(distance)`

### 2.4 右移

```bash
# 右移 0.3 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: -0.3, z: 0.0}"

# 右移 0.5 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: -0.5, z: 0.0}"
```

对应节点: `右移(distance)`

### 2.5 左转

```bash
# 左转 90° (π/2 ≈ 1.5708 rad)
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 1.5708}"

# 左转 45° (π/4 ≈ 0.7854 rad)
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 0.7854}"

# 左转 180° (π ≈ 3.1416 rad)
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 3.1416}"
```

对应节点: `左转90度()`

### 2.6 右转

```bash
# 右转 90° (-π/2 ≈ -1.5708 rad)
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -1.5708}"

# 右转 45° (-π/4 ≈ -0.7854 rad)
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -0.7854}"

# 右转 180° (-π ≈ -3.1416 rad)
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -3.1416}"
```

对应节点: `右转90度()`

### 2.7 组合移动

```bash
# 前进 1 米 + 同时左转 90°
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 1.5708}"

# 前进 2 米 + 右移 0.5 米
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 2.0, y: -0.5, z: 0.0}"
```

---

## 3. 机械臂 — `/r2/arm_cmd`

**消息**: `std_msgs/msg/Int32`
**data 值**: `0`=空闲, `1`=绘制KFS

### 3.1 机械臂空闲

```bash
ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{data: 0}"
```

对应节点: `机械臂空闲()`
MAVLink: `ARM_CONTROL` (msg ID 200), `ARM_ACTION_IDLE`

### 3.2 机械臂绘制 KFS

```bash
ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{data: 1}"
```

对应节点: `机械臂绘制KFS()`
MAVLink: `ARM_CONTROL` (msg ID 200), `ARM_DRAW_KFS`

---

## 4. 攀爬机构 — `/r2/climbing_cmd`

**消息**: `std_msgs/msg/Int32`
**data 值**: `0`-`4`

### 4.1 攀爬初始姿态 (20cm)

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 0}"
```

对应节点: `攀爬初始姿态()`
MAVLink: `CLIMBING_CMD` (msg ID 202), `CLIMBING_CMD_INIT_POSE`

### 4.2 攀爬准备 40cm

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 1}"
```

对应节点: `攀爬准备40cm()`
MAVLink: `CLIMBING_CMD_PREPARE_40CM`

### 4.3 攀爬上爬 20cm

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 2}"
```

对应节点: `攀爬上爬20cm()`
MAVLink: `CLIMBING_CMD_EXECUTE_UP_20CM`

### 4.4 攀爬上爬 40cm

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 3}"
```

对应节点: `攀爬上爬40cm()`
MAVLink: `CLIMBING_CMD_EXECUTE_UP_40CM`

### 4.5 攀爬下降 20cm

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 4}"
```

对应节点: `攀爬下降20cm()`
MAVLink: `CLIMBING_CMD_EXECUTE_DOWN_20CM`

---

## 5. 夹爪机构 — `/r2/clamping_cmd`

**消息**: `std_msgs/msg/Int32`
**data 值**: `0`-`3`

### 5.1 夹爪移至夹持角 (90°)

```bash
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 0}"
```

对应节点: `夹爪夹持角()`
MAVLink: `CLAMPING_CMD` (msg ID 203), `CLAMPING_CMD_MOVE_TO_PARALLEL`

### 5.2 夹爪闭合

```bash
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 1}"
```

对应节点: `夹爪闭合()`
MAVLink: `CLAMPING_CMD_GRAB`

### 5.3 夹爪复位 (0°)

```bash
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 2}"
```

对应节点: `夹爪复位()`
MAVLink: `CLAMPING_CMD_MOVE_TO_RESET`

### 5.4 夹爪释放

```bash
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 3}"
```

对应节点: `夹爪释放()`
MAVLink: `CLAMPING_CMD_RELEASE`

---

## 6. 紧急制动 — `/r2/estop`

**消息**: `std_msgs/msg/Empty`（无数据字段）

```bash
ros2 topic pub --once /r2/estop std_msgs/msg/Empty
```

对应节点: `急停()` / `底盘紧急制动()`
效果: 立即发送 Vx=0, Vy=0, Vω=0，同时清空移动目标防止恢复后继续移动。

---

## 7. 一键速查

### 移动

```bash
# 前进 1m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"
# 后退 0.5m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: -0.5, y: 0.0, z: 0.0}"
# 左移 0.3m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.3, z: 0.0}"
# 右移 0.3m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: -0.3, z: 0.0}"
# 左转 90°
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 1.5708}"
# 右转 90°
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -1.5708}"
```

### 机械臂

```bash
ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{data: 0}"   # 空闲
ros2 topic pub --once /r2/arm_cmd std_msgs/msg/Int32 "{data: 1}"   # 绘制KFS
```

### 攀爬

```bash
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 0}"  # 初始20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 1}"  # 准备40cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 2}"  # 上爬20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 3}"  # 上爬40cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 4}"  # 下降20cm
```

### 夹爪

```bash
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 0}"  # 夹持角90°
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 1}"  # 闭合
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 2}"  # 复位0°
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 3}"  # 释放
```

### 急停

```bash
ros2 topic pub --once /r2/estop std_msgs/msg/Empty
```

### 下台阶（相机检测自动流程）

```bash
# 触发下台阶自动流程：
# 停车 → 启动深度相机 → 0.05m/s慢行 → 检测到楼梯 → 攀爬下降20cm
ros2 topic pub --once /r2/stairs_down std_msgs/msg/Empty
```

---

## 8. 角度换算表

| 角度 | 弧度值 |
|:--|:--|
| 左转 15° | `0.2618` |
| 左转 30° | `0.5236` |
| 左转 45° | `0.7854` |
| 左转 60° | `1.0472` |
| **左转 90°** | **`1.5708`** |
| 左转 180° | `3.1416` |
| 右转 15° | `-0.2618` |
| 右转 30° | `-0.5236` |
| 右转 45° | `-0.7854` |
| 右转 60° | `-1.0472` |
| **右转 90°** | **`-1.5708`** |
| 右转 180° | `-3.1416` |

---

## 9. 完整示例：自动夹取任务

```bash
# 1) 前进 2m 靠近目标
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"

# 2) 夹爪移至夹持角
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 0}"

# 3) 微调前进 0.3m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.3, y: 0.0, z: 0.0}"

# 4) 闭合夹爪
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 1}"

# 5) 后退 1m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: -1.0, y: 0.0, z: 0.0}"

# 6) 上爬 20cm
ros2 topic pub --once /r2/climbing_cmd std_msgs/msg/Int32 "{data: 2}"

# 7) 右转 90°
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -1.5708}"

# 8) 前进 1m
ros2 topic pub --once /move_relative geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"

# 9) 释放夹爪
ros2 topic pub --once /r2/clamping_cmd std_msgs/msg/Int32 "{data: 3}"

# 10) 急停
ros2 topic pub --once /r2/estop std_msgs/msg/Empty
```

---

## 10. 比赛任务脚本模板

> 📄 文件: `~/connect/mission_script.py`

如果比赛需要执行多步骤任务序列，用话题手动逐条发送容易出错。推荐使用 `mission_script.py` 脚本模板：

```bash
# 1. 启动行为树
python3 ~/connect/r2_behavior_tree.py

# 2. 另开终端，运行任务脚本
python3 ~/connect/mission_script.py
```

脚本内提供了封装好的动作函数（`前进()`、`旋转()`、`攀爬()`、`夹爪()`、`下台阶()` 等），只需在 `执行任务序列()` 函数中按顺序编写步骤即可。

详细修改指南见 `mission_script.py` 顶部注释。

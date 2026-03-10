# R2 ROS2 启动文件说明

## 📋 可用的启动文件

### 1. `r2_bringup.launch.py` - 通用启动（推荐）

**用途**: 默认启动文件，支持参数自定义

**启动命令**:
```bash
# 默认：冷启动 + 模拟硬件
ros2 launch r2_decision_ros2 r2_bringup.launch.py

# 自定义启动模式
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode:=cold_start
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode:=return_to_mc
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode:=return_to_mf

# 使用真实硬件
ros2 launch r2_decision_ros2 r2_bringup.launch.py use_sim_hardware:=false

# 组合参数
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode:=return_to_mc use_sim_hardware:=false
```

**参数**:
- `launch_mode`: 启动模式 (cold_start | return_to_mc | return_to_mf)
- `use_sim_hardware`: 硬件模式 (true=模拟 | false=真实)

---

### 2. `r2_cold_start.launch.py` - 冷启动模式

**用途**: 比赛开始，正常完整流程

**启动命令**:
```bash
ros2 launch r2_decision_ros2 r2_cold_start.launch.py
```

**流程**: 启动 → 武馆 → 梅林 → 对抗区

**使用场景**:
- 比赛开始
- 完整流程测试
- 常规任务执行

---

### 3. `r2_return_mc.launch.py` - 返回武馆模式

**用途**: 武器组装失败，返回武馆重新拼接

**启动命令**:
```bash
ros2 launch r2_decision_ros2 r2_return_mc.launch.py
```

**流程**: 启动 → 武馆（重拼） → 梅林 → 对抗区

**使用场景**:
- 武器头组装失败
- 二维码识别错误
- 需要重新获取武器头

---

### 4. `r2_return_mf.launch.py` - 返回梅林模式

**用途**: KFS收集不足，返回梅林区域补充

**启动命令**:
```bash
ros2 launch r2_decision_ros2 r2_return_mf.launch.py
```

**流程**: 启动 → 梅林（补拿） → 对抗区

**使用场景**:
- 梅林阶段KFS收集失败
- 需要补充更多KFS
- 策略调整需要更多方块

---

### 5. `r2_real_hardware.launch.py` - 真实硬件模式

**用途**: 实际比赛，连接真实硬件设备

**启动命令**:
```bash
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py

# 可选：指定启动模式
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py launch_mode:=cold_start
```

**特点**:
- 强制使用真实硬件（`use_sim_hardware=false`）
- 需要硬件设备已连接
- 适用于实际比赛环境

**使用场景**:
- 实际比赛
- 真实硬件调试
- 现场测试

---

### 6. `r2_test_path_planning.launch.py` - 路径规划测试

**用途**: 仅测试梅林路径规划算法

**启动命令**:
```bash
ros2 launch r2_decision_ros2 r2_test_path_planning.launch.py
```

**特点**:
- 仅启动任务状态机节点
- 直接跳到梅林阶段
- 快速测试路径规划

**使用场景**:
- 调试BFS算法
- 验证路径规划
- 梅林阶段单独测试

---

## 🎯 启动模式对照表

| 启动文件 | launch_mode | use_sim_hardware | 节点数 | 用途 |
|---------|-------------|------------------|-------|------|
| r2_bringup.launch.py | 参数可选 | 参数可选 | 3 | 通用启动 |
| r2_cold_start.launch.py | cold_start | true | 3 | 冷启动 |
| r2_return_mc.launch.py | return_to_mc | true | 3 | 返回武馆 |
| r2_return_mf.launch.py | return_to_mf | true | 3 | 返回梅林 |
| r2_real_hardware.launch.py | 参数可选 | false | 3 | 真实硬件 |
| r2_test_path_planning.launch.py | test | true | 1 | 路径测试 |

---

## 📊 启动流程对比

### 冷启动 (cold_start)
```
启动 → 传感器检查 → 系统校准
  ↓
武馆 → 旋转搜索 → 移动 → 抓取 → 组装
  ↓
梅林 → 路径规划 → 导航 → 收集KFS
  ↓
对抗区 → 进场 → 放置策略
  ↓
结束
```

### 返回武馆 (return_to_mc)
```
启动 → 快速检查
  ↓
武馆 → 重新抓取 → 重新组装
  ↓
梅林 → 路径规划 → 收集KFS
  ↓
对抗区 → 放置策略
  ↓
结束
```

### 返回梅林 (return_to_mf)
```
启动 → 快速检查
  ↓
梅林 → 路径规划 → 补充KFS
  ↓
对抗区 → 放置策略
  ↓
结束
```

---

## 🔧 常用启动命令组合

### 场景1: 比赛开始（默认）
```bash
ros2 launch r2_decision_ros2 r2_cold_start.launch.py
```

### 场景2: 比赛开始（真实硬件）
```bash
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py
```

### 场景3: 武器组装失败，重试
```bash
ros2 launch r2_decision_ros2 r2_return_mc.launch.py
```

### 场景4: KFS不足，返回梅林
```bash
ros2 launch r2_decision_ros2 r2_return_mf.launch.py
```

### 场景5: 调试路径规划算法
```bash
ros2 launch r2_decision_ros2 r2_test_path_planning.launch.py
```

### 场景6: 自定义参数启动
```bash
ros2 launch r2_decision_ros2 r2_bringup.launch.py \
    launch_mode:=return_to_mc \
    use_sim_hardware:=false
```

---

## 🛠️ 编译和安装

启动前需要先编译：

```bash
cd /home/yf/ros2_ws

# 编译ROS2包
colcon build --packages-select r2_decision_ros2

# 加载环境
source install/setup.bash
```

---

## 📝 查看可用参数

```bash
# 查看启动文件支持的参数
ros2 launch r2_decision_ros2 r2_bringup.launch.py --show-args
```

输出示例：
```
Arguments (pass arguments as '<name>:=<value>'):

    'launch_mode':
        Launch mode: cold_start, return_to_mc, return_to_mf
        (default: 'cold_start')

    'use_sim_hardware':
        Use simulated hardware (true) or real hardware (false)
        (default: 'true')
```

---

## 🐛 故障排查

### 问题1: 找不到启动文件

```bash
# 检查包是否安装
ros2 pkg list | grep r2_decision

# 重新编译和加载
colcon build --packages-select r2_decision_ros2
source install/setup.bash
```

### 问题2: 参数不生效

```bash
# 使用 := 而不是 =
# ✓ 正确
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode:=cold_start

# ✗ 错误
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode=cold_start
```

### 问题3: 节点无法启动

```bash
# 检查节点是否已编译
ls install/r2_decision_ros2/lib/r2_decision_ros2/

# 应该看到：
# - launch_manager_node
# - safety_guard_node
# - mission_state_node
```

---

## 📚 相关文档

- `STARTUP_GUIDE.md` - 完整启动指南
- `README.md` - 项目总览
- `docs/path_planner_integration.md` - 路径规划文档

---

## 🎯 快速参考卡片

```bash
# === 最常用的3个命令 ===
ros2 launch r2_decision_ros2 r2_cold_start.launch.py      # 冷启动
ros2 launch r2_decision_ros2 r2_return_mc.launch.py       # 返回武馆
ros2 launch r2_decision_ros2 r2_return_mf.launch.py       # 返回梅林

# === 真实硬件 ===
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py   # 真实硬件模式

# === 测试调试 ===
ros2 launch r2_decision_ros2 r2_test_path_planning.launch.py  # 路径规划测试
```

**选择启动文件而非修改代码，让系统更灵活！** 🚀

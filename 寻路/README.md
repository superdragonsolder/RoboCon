# R2 决策树骨架（ROBOCON 武林探秘）

该仓库提供一个**可运行的 C++ 骨架**，用于表达 R2 三阶段流程：

1. `launch` 开机自检与分流
2. `MC` 武馆取端头并配合组装
3. `MF` 梅林寻路与夹取 KFS
4. `CF` 对抗区放置 KFS

实现重点：
- 用 `flag` 总线做节点间通信
- 用状态机管理流程、重试与异常分支
- 把规则约束做成守卫函数，避免明显犯规
- **硬件抽象层（HAL）**：灵活集成多种传感器和执行器

## 快速运行（C++ 单机演示）

```bash
cd /home/yf/ros2_ws
cmake -S . -B build
cmake --build build -j
./build/cpp/r2_decision_demo
```

## 硬件集成示例

```bash
./build/cpp/r2_hardware_example
```

输出显示模拟硬件设备的数据流和控制过程。

## ROS2 多节点版本

ROS2 包：`ros2/r2_decision_ros2`，包含三个节点：
- `launch_manager_node`：发布启动分流 flag
- `safety_guard_node`：发布规则守卫 flag
- `mission_state_node`：驱动状态机并发布中文阶段/日志

### 话题约定
- 输入：`/r2/flags/in`（`std_msgs/String`，格式 `key=value`）
- 输出：`/r2/phase`、`/r2/log`（均为中文文本）

### ROS2 构建运行

```bash
cd /home/yf/ros2_ws/ros2
colcon build --packages-select r2_decision_ros2
source install/setup.bash
ros2 launch r2_decision_ros2 r2_bringup.launch.py
```

## 目录结构

- `cpp/include/r2_decision_cpp/`：C++ 头文件（flag_bus、models、state_machine、strategy、**hardware_interface**、**hardware_manager**）
- `cpp/src/`：C++ 实现与单机演示入口
- `ros2/r2_decision_ros2/`：ROS2 包（节点、launch、配置）
- `docs/`：设计说明与改进建议
  - `r2_design_notes.md` - 决策树设计笔记
  - `hardware_integration_guide.md` - 硬件集成教程
  - `hardware_architecture.md` - 硬件架构详解
  - `detailed_hardware_process_flow.md` - **完整流程详解（含每步硬件操作）**

## 硬件集成

### 支持的硬件

- **传感器**：激光测距仪、二维码扫描器、单目相机、双目相机、雷达
- **执行器**：舵轮底盘、机械臂

### 快速集成

1. 继承 `LaserSensorInterface`、`CameraInterface` 等基类
2. 实现驱动逻辑
3. 在 `mission_state_node` 中通过 `HardwareManager` 注册
4. 在决策循环中调用 `update()` 和 `get_*()`

详见 [`docs/hardware_integration_guide.md`](docs/hardware_integration_guide.md) 和 [`docs/hardware_architecture.md`](docs/hardware_architecture.md)

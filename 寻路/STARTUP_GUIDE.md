# R2 决策系统启动流程指南

## 🚀 快速启动（3步）

```bash
# 1. 进入工作目录
cd /home/yf/ros2_ws

# 2. 编译项目
cmake --build build -j4

# 3. 运行演示程序
./build/cpp/r2_decision_demo
```

---

## 📋 详细启动流程

### 方案一：独立C++程序（推荐用于测试）

#### 1.1 编译项目

```bash
cd /home/yf/ros2_ws

# 首次编译或修改CMakeLists.txt后需要重新配置
cmake -S . -B build

# 编译所有目标
cmake --build build -j4
```

**编译输出：**
```
[ 45%] Built target r2_decision_core          # 核心库
[ 72%] Built target r2_hardware_example       # 硬件演示
[ 90%] Built target r2_decision_demo          # 完整决策系统
[100%] Built target r2_path_planner_demo      # 路径规划演示
```

#### 1.2 运行完整决策系统

```bash
./build/cpp/r2_decision_demo
```

**程序流程：**
```
启动 → 硬件初始化 → 武馆 → 梅林（路径规划） → 对抗区 → 结束
```

**预期输出：**
- 总日志行数：120+
- 梅林阶段收集：4个KFS
- 对抗区阶段：放置策略执行
- 最终状态：成功结束

#### 1.3 运行路径规划演示

```bash
./build/cpp/r2_path_planner_demo
```

**功能：**
- 仅演示梅林路径规划算法
- 随机生成KFS分布
- BFS搜索最优路径
- 可视化路径追踪

#### 1.4 运行硬件演示

```bash
./build/cpp/r2_hardware_example
```

**功能：**
- 演示所有硬件接口
- 传感器数据读取
- 执行器控制测试
- 不涉及决策逻辑

---

### 方案二：ROS2多节点系统（用于实际比赛）

#### 2.1 编译ROS2包

```bash
cd /home/yf/ros2_ws

# 使用colcon编译ROS2包
colcon build --packages-select r2_decision_ros2

# 或者编译所有包
colcon build
```

#### 2.2 加载环境变量

```bash
# 加载ROS2 Humble环境
source /opt/ros/humble/setup.bash

# 加载本地工作空间
source /home/yf/ros2_ws/install/setup.bash
```

#### 2.3 启动系统（通过不同launch文件）

**💡 推荐方式：根据场景选择对应的launch文件，无需修改代码！**

##### 场景1: 比赛开始 - 冷启动模式
```bash
ros2 launch r2_decision_ros2 r2_cold_start.launch.py
```
**流程**: 启动 → 武馆 → 梅林 → 对抗区

##### 场景2: 武器组装失败 - 返回武馆模式
```bash
ros2 launch r2_decision_ros2 r2_return_mc.launch.py
```
**流程**: 启动 → 武馆(重拼) → 梅林 → 对抗区

##### 场景3: KFS不足 - 返回梅林模式
```bash
ros2 launch r2_decision_ros2 r2_return_mf.launch.py
```
**流程**: 启动 → 梅林(补拿) → 对抗区

##### 场景4: 真实硬件模式
```bash
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py
```
**特点**: 强制使用真实硬件设备

##### 场景5: 路径规划测试
```bash
ros2 launch r2_decision_ros2 r2_test_path_planning.launch.py
```
**特点**: 仅测试梅林路径规划算法

##### 场景6: 通用启动（自定义参数）
```bash
# 默认冷启动 + 模拟硬件
ros2 launch r2_decision_ros2 r2_bringup.launch.py

# 自定义启动模式
ros2 launch r2_decision_ros2 r2_bringup.launch.py launch_mode:=return_to_mc

# 使用真实硬件
ros2 launch r2_decision_ros2 r2_bringup.launch.py use_sim_hardware:=false

# 组合参数
ros2 launch r2_decision_ros2 r2_bringup.launch.py \
    launch_mode:=return_to_mc \
    use_sim_hardware:=false
```

**可用启动文件**:
| 文件 | 用途 | launch_mode |
|-----|------|-------------|
| `r2_cold_start.launch.py` | 冷启动 | cold_start |
| `r2_return_mc.launch.py` | 返回武馆 | return_to_mc |
| `r2_return_mf.launch.py` | 返回梅林 | return_to_mf |
| `r2_real_hardware.launch.py` | 真实硬件 | 可选 |
| `r2_test_path_planning.launch.py` | 路径测试 | test |
| `r2_bringup.launch.py` | 通用启动 | 参数可选 |

**详细说明**: 查看 `ros2/r2_decision_ros2/LAUNCH_FILES.md`

**启动内容（所有launch文件）**:
- `launch_manager_node` - 启动管理和模式选择
- `safety_guard_node` - 安全监控
- `mission_state_node` - 任务状态机

**节点通信**:
```
/r2/flags/in    ← 输入标志
/r2/flags/out   → 输出标志
/r2/phase       → 阶段状态
/r2/log         → 日志输出
```

#### 2.4 监控系统状态

#### 2.4 监控系统状态

**查看所有节点：**
```bash
ros2 node list
```

**查看话题列表：**
```bash
ros2 topic list
```

**监听日志输出：**
```bash
ros2 topic echo /r2/log
```

**监听阶段状态：**
```bash
ros2 topic echo /r2/phase
```

---

## 🔧 启动前配置

### ⚠️ 重要：无需修改代码

**新方式**: 通过选择不同的launch文件来控制启动模式和硬件配置。

### 配置方式对照

| 需求 | 旧方式（修改代码） | 新方式（选择launch文件） |
|-----|------------------|----------------------|
| 冷启动模式 | 修改main.cpp | `r2_cold_start.launch.py` |
| 返回武馆 | 修改main.cpp | `r2_return_mc.launch.py` |
| 返回梅林 | 修改main.cpp | `r2_return_mf.launch.py` |
| 真实硬件 | 修改main.cpp | `r2_real_hardware.launch.py` |
| 自定义参数 | 修改main.cpp | `r2_bringup.launch.py` + 参数 |

### 旧配置1：硬件设备连接（仅真实硬件需要）

### 旧配置1：硬件设备连接（仅真实硬件需要）

**⚠️ 仅在使用真实硬件时需要修改（不推荐修改代码）**

真实硬件模式推荐使用：
```bash
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py
```

如果需要自定义硬件驱动，编辑 `cpp/src/main.cpp`：

```cpp
// 替换模拟硬件为真实驱动
// hw_manager->register_hardware("laser", std::make_shared<MockLaserSensor>());
hw_manager->register_hardware("laser", std::make_shared<RealLaserSensor>("/dev/ttyUSB0"));

// hw_manager->register_hardware("chassis", std::make_shared<MockChassis>());
hw_manager->register_hardware("chassis", std::make_shared<RealChassis>("/dev/can0"));
```

### 旧配置2：启动模式选择（已废弃）

**❌ 不再需要修改代码！**

旧方式（修改 `main.cpp`）：
```cpp
// 不推荐
bus.set("launch_mode", "cold_start");
```

**✅ 新方式（选择launch文件）：**
```bash
# 直接启动对应模式
ros2 launch r2_decision_ros2 r2_cold_start.launch.py
ros2 launch r2_decision_ros2 r2_return_mc.launch.py
ros2 launch r2_decision_ros2 r2_return_mf.launch.py
```

### 旧配置3：定时参数调整（保留）

编辑 `cpp/src/main.cpp` 中的 `TimingPolicy`：

```cpp
r2::TimingPolicy timing;
timing.wait_before_grab_ms = 200;      // 抓取前等待
timing.grab_timeout_ms = 500;          // 抓取超时
timing.assemble_timeout_ms = 500;      // 组装超时
timing.mc_move_timeout_ms = 3000;      // 武馆移动超时
timing.mf_move_timeout_ms = 4000;      // 梅林移动超时
timing.cf_move_timeout_ms = 3000;      // 对抗区移动超时
```

---

## 🎯 启动流程详解

### 程序启动顺序

```
1. 主程序初始化
   ├─ 创建 FlagBus (标志总线)
   ├─ 创建 HardwareManager (硬件管理器)
   └─ 注册所有硬件设备

2. 硬件系统初始化
   ├─ laser.initialize()           # 激光传感器
   ├─ qr_scanner.initialize()      # 二维码扫描
   ├─ mono_camera.initialize()     # 单目相机
   ├─ stereo_camera.initialize()   # 双目相机
   ├─ lidar.initialize()           # 激光雷达
   ├─ chassis.initialize()         # 底盘
   └─ arm.initialize()             # 机械臂

3. 创建状态机
   └─ R2DecisionStateMachine (传入bus + hw_manager + timing)

4. 设置初始标志
   ├─ match_started = false
   ├─ r1_left_mc = false
   └─ launch_mode = cold_start

5. 进入主循环
   while (比赛进行中) {
       ├─ 执行 state_machine.tick(ctx)
       ├─ 输出实时日志
       └─ 检查阶段是否为"结束"
   }

6. 清理资源
   ├─ 输出最终统计
   └─ hw_manager.shutdown_all()
```

### 阶段执行流程

#### 【启动阶段】
```
1. 检查所有传感器状态
   ├─ 激光传感器 ready check
   ├─ 单目相机 ready check
   └─ 输出传感器状态

2. 执行系统校准
   ├─ 底盘归零
   ├─ 机械臂归零
   └─ 传感器标定

3. 选择启动模式
   └─ 根据 launch_mode 标志决定下一阶段
```

#### 【武馆阶段】（4步）
```
步骤1: 旋转搜索武器头
   └─ 底盘旋转 + 激光扫描

步骤2: 前进到武器架
   └─ 底盘前进

步骤3: 抓取武器头
   ├─ 机械臂打开夹爪
   ├─ 移动到抓取位置
   └─ 闭合夹爪

步骤4: 二维码识别并组装
   ├─ 扫描二维码
   └─ 执行组装动作
```

#### 【梅林阶段】（3步）- **集成路径规划**
```
步骤1: 初始化网格并规划路径
   ├─ 创建 MeiHuaGrid (4x3网格)
   ├─ 随机生成 KFS 分布 (3个R1 + 4个R2 + 1个假方块)
   ├─ 创建 PathPlanner
   ├─ 执行 BFS 搜索最优路径
   └─ 保存路径到 ctx.meilin_path

步骤2: 导航到梅林入口
   └─ 底盘移动到 (1,2) 起点

步骤3: 执行规划路径
   for 每一步 in 路径:
       ├─ 底盘按 W/A/S/D 指令移动
       ├─ 检测是否相邻 R2
       └─ 如果相邻 → 机械臂抓取 KFS
```

#### 【对抗区阶段】（2步）
```
步骤1: 进入对抗区
   └─ 底盘移动

步骤2: 放置 KFS 策略
   ├─ 识别当前局面
   ├─ 选择阻断或赢线策略
   └─ 机械臂放置方块
```

#### 【重试阶段】
```
如果前面阶段失败 → 重试
├─ retry_count++
├─ 检查重试次数 (最多3次)
└─ 返回上一阶段或进入结束
```

#### 【结束阶段】
```
1. 输出最终统计
   ├─ 最终阶段
   ├─ 携带 KFS 数量
   ├─ 重试次数
   └─ 总日志行数

2. 关闭硬件系统
3. 退出程序
```

---

## 📊 启动检查清单

### 编译前检查

- [ ] 已安装 CMake 3.16+
- [ ] 已安装 C++17 编译器 (g++/clang++)
- [ ] 已安装 ROS2 Humble（如果使用ROS2模式）
- [ ] 所有源文件无语法错误

### 硬件连接检查（真实硬件模式）

- [ ] 激光传感器连接 (检查串口设备)
- [ ] 二维码扫描仪连接
- [ ] 相机连接 (单目/双目)
- [ ] 激光雷达连接
- [ ] 底盘CAN总线连接
- [ ] 机械臂连接

### 运行时检查

- [ ] 程序正常启动
- [ ] 硬件初始化成功 (查看 "✓ xxx initialized" 日志)
- [ ] 标志总线工作正常
- [ ] 各阶段按顺序执行
- [ ] 日志输出正常

---

## 🐛 常见启动问题

### 问题1：编译错误

**症状：** `cmake --build build` 失败

**解决：**
```bash
# 清理构建目录
rm -rf build

# 重新配置
cmake -S . -B build

# 重新编译
cmake --build build -j4
```

### 问题2：找不到可执行文件

**症状：** `./build/cpp/r2_decision_demo: No such file`

**解决：**
```bash
# 检查编译是否成功
ls -l build/cpp/

# 如果没有，重新编译
cmake --build build -j4 --verbose
```

### 问题3：硬件初始化失败

**症状：** 日志显示 "✗ xxx initialization failed"

**解决：**
- 检查硬件连接
- 检查设备权限 (`sudo chmod 666 /dev/ttyUSB0`)
- 使用模拟硬件测试 (`MockXXX`)

### 问题4：ROS2节点无法启动

**症状：** `ros2 run` 找不到包

**解决：**
```bash
# 重新编译ROS2包
colcon build --packages-select r2_decision_ros2

# 重新加载环境
source install/setup.bash

# 检查包是否存在
ros2 pkg list | grep r2_decision
```

### 问题5：路径规划失败

**症状：** 梅林阶段输出 "✗ 规划失败"

**原因：**
- 网格中所有格子都是障碍物
- 起点被假方块堵住
- 出口都被堵住

**解决：**
- 重新运行（随机生成会改变）
- 检查网格初始状态日志
- 调试 `path_planner.cpp`

---

## 📝 启动日志示例

### 完整启动输出

```
========================================
  R2 决策系统演示程序
========================================

→ 初始化硬件管理器...
  注册硬件: laser
  注册硬件: qr_scanner
  注册硬件: mono_camera
  注册硬件: stereo_camera
  注册硬件: lidar
  注册硬件: chassis
  注册硬件: arm

→ 初始化所有硬件设备...
  ✓ laser initialized
  ✓ qr_scanner initialized
  ✓ mono_camera initialized
  ✓ stereo_camera initialized
  ✓ lidar initialized
  ✓ chassis initialized
  ✓ arm initialized

→ 创建决策状态机...
→ 设置初始标志...
→ 开始执行决策流程...

========== 实时日志输出 ==========

[启动] ========== 启动阶段开始 ==========
[启动] 目标: 系统初始化和启动模式选择
[启动] → 第1步: 传感器检查
...
[梅林] → 第1步: 初始化梅林网格并规划路径
[梅林]   → 创建梅花桩网格模型
[梅林]     生成随机KFS分布
[梅林]     梅花桩初始状态:
[梅林]       行4: 空 R1 R2 
[梅林]       行3: 空 R2 R1 
[梅林]       行2: 空 X  R1 
[梅林]       行1: R2 R2 空 
[梅林]   → 开始BFS搜索最优路径...
[梅林]     ✓ 找到最优路径，步数: 4
...
[对抗区] 对抗区阶段完成

========== 比赛流程结束 ==========

【最终统计】
- 最终阶段: 结束
- 当前携带R2 KFS数量: 4
- 重试次数: 0
- 总日志行数: 120

✓ 硬件系统已关闭
```

---

## 🎯 快速命令参考

```bash
# === 编译相关 ===
cmake -S . -B build                    # 配置项目
cmake --build build -j4                # 并行编译
cmake --build build --target clean     # 清理编译

# === 运行相关 ===
./build/cpp/r2_decision_demo           # 完整系统
./build/cpp/r2_path_planner_demo       # 路径规划
./build/cpp/r2_hardware_example        # 硬件测试

# === ROS2相关 ===
colcon build                           # 编译所有ROS2包
source install/setup.bash              # 加载环境
ros2 launch r2_decision_ros2 r2_bringup.launch.py  # 启动系统
ros2 topic echo /r2/log                # 查看日志
ros2 node list                         # 查看节点

# === 调试相关 ===
gdb ./build/cpp/r2_decision_demo       # GDB调试
valgrind ./build/cpp/r2_decision_demo  # 内存检查
```

---

## 📚 相关文档

- `README.md` - 项目总览
- `docs/path_planner_integration.md` - 路径规划集成文档
- `docs/detailed_hardware_process_flow.md` - 硬件流程详解
- `PATH_PLANNER_SUMMARY.md` - 路径规划总结
- `UPGRADE_SUMMARY.md` - 硬件集成升级总结

---

**祝您启动顺利！如有问题请查阅文档或联系开发者。** 🚀

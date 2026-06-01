# R2 机器人导航系统 — 移植部署指南

> 解压后请先阅读本文，按步骤完成环境配置。

---

## 1. 目录结构

```
connect/
├── R2_ws/                          # ROS2 工作空间（需重新编译）
│   └── src/
│       ├── odin_ros_driver/        # Odin 传感器 ROS2 驱动（C++）
│       └── r2_communication/       # R2 MAVLink 串口通信包（Python）
├── howlong/                        # 深度相机楼梯检测模块
│   ├── BerxelSdkDriver/            # Berxel 100R 相机 SDK（含 .so 原生库）
│   └── src/                        # 相机管理、距离检测、可视化
├── R2.xml                          # MAVLink 协议定义文件
├── R2_Protocol.py                  # 由 R2.xml 自动生成的协议实现
├── R2Communication.py              # MAVLink 串口通信封装
├── r2_behavior_tree.py             # 行为树导航主节点
├── r2_behavior_tree_groot.xml      # Groot 行为树可视化文件
├── mission_sequence_pid.py         # PID 自主任务导航
├── mission_script.py               # 任务脚本
├── relocalization_wiggle.py        # 重定位摆动脚本
├── generate_groot_xml.py           # Groot XML 生成工具
├── image/
│   └── cam_in_ex.txt               # 相机外参标定文件
├── R2_NAVIGATION_STARTUP_GUIDE.md  # 启动指南（必读）
├── R2_ROS2_TOPIC_REFERENCE.md      # ROS2 话题参考
└── R2_BEHAVIOR_TREE_NODES.md       # 行为树节点说明
```

---

## 2. 前置条件

### 2.1 硬件

| 项目 | 要求 |
|------|------|
| 上位机 | x86_64 Linux（Ubuntu 22.04 推荐） |
| 串口 | USB 转串口连接 R2 下位机 |
| 深度相机 | Berxel 100R（可选，下台阶功能需要） |

### 2.2 软件

```bash
# ROS2 Humble（必须）
# 安装方法: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# ROS2 系统依赖
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-pcl-conversions \
    ros-humble-message-filters \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs

# Python 依赖
pip3 install pyserial numpy opencv-python py_trees

# colcon 构建工具
sudo apt install python3-colcon-common-extensions
```

### 2.3 串口权限

```bash
sudo usermod -aG dialout $USER
# 重新登录或重启生效
```

---

## 3. 部署步骤

### 3.1 解压

```bash
# 解压到任意目录（推荐 ~/connect）
tar xzf r2_project.tar.gz -C ~/connect
```

### 3.2 编译 ROS2 工作空间

```bash
cd ~/connect/R2_ws
colcon build --packages-select r2_communication odin_ros_driver
source install/setup.bash
```

> 编译成功后，可将 `source ~/connect/R2_ws/install/setup.bash` 加入 `~/.bashrc`。

### 3.3 确认串口设备

```bash
ls -la /dev/ttyACM* /dev/ttyUSB*
```

记下实际设备名（如 `/dev/ttyUSB0`），启动时通过 `port:=` 参数指定。

### 3.4 验证启动

```bash
# 终端1：启动 Odin 传感器驱动
cd ~/connect/R2_ws && source install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py

# 终端2：键盘遥控测试（注意替换串口设备名）
cd ~/connect/R2_ws && source install/setup.bash
ros2 launch r2_communication keyboard_control.launch.py port:=/dev/ttyUSB0
```

---

## 4. 关键注意事项

### 4.1 串口设备名

不同机器的串口设备名可能不同（`/dev/ttyACM0`、`/dev/ttyUSB0` 等），所有启动命令都支持 `port:=` 参数覆盖。

### 4.2 Berxel 相机原生库

`howlong/BerxelSdkDriver/libs/linux/` 下的 `.so` 文件是为 **x86_64 Linux** 编译的。如果目标机器架构不同（如 ARM/Jetson），需要联系 Berxel 获取对应架构的 SDK。

### 4.3 相机标定文件

`image/cam_in_ex.txt` 包含相机外参。如果更换了相机或安装位置，需要重新标定。

### 4.4 地图文件

如果之前保存了 SLAM 地图，确保 `R2_ws/src/odin_ros_driver/map/` 目录中的地图文件已包含。

### 4.5 已修复的硬编码路径

`behavior_tree.launch.py` 中的脚本路径已改为相对路径，不再依赖 `~/connect`。只要保持压缩包内的目录结构不变，放在任意路径下都能正常运行。

---

## 5. 快速命令速查

```bash
# 编译
cd ~/connect/R2_ws && colcon build --packages-select r2_communication odin_ros_driver

# 键盘遥控
ros2 launch r2_communication keyboard_control.launch.py port:=/dev/ttyUSB0

# PID 自主导航
python3 ~/connect/mission_sequence_pid.py

# 行为树导航
ros2 launch r2_communication behavior_tree.launch.py port:=/dev/ttyUSB0

# 重定位摆动
python3 ~/connect/relocalization_wiggle.py --port /dev/ttyUSB0

# 下台阶检测
ros2 topic pub --once /r2/stairs_down std_msgs/msg/Empty
```

---

## 6. 故障排查

| 问题 | 解决方法 |
|------|----------|
| 串口打不开 | `ls /dev/tty*` 确认设备名；检查 `dialout` 组权限 |
| `colcon build` 失败 | 确认 ROS2 环境已 source（`source /opt/ros/humble/setup.bash`） |
| `import BerxelSdkDriver` 失败 | 确认 `howlong/` 在 `PYTHONPATH` 中，或从 `howlong/` 目录运行脚本 |
| TF 变换不可用 | 确认 `odin_ros_driver` 已启动，检查 `ros2 run tf2_ros tf2_echo odom odin1_base_link` |
| 机器人不移动 | 检查串口连接和 MAVLink 心跳，尝试重新插拔串口线 |

---

详细启动说明请阅读 `R2_NAVIGATION_STARTUP_GUIDE.md`。

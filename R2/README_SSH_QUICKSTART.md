# R2 远程 SSH 快速启动与调试指南

> 适用：Jetson Nano / RDK X5，Username: `ssr01` / `sunrise`

---

## 1. 连接

```bash
# Jetson Nano
ssh ssr01@192.168.149.167

# RDK X5
ssh sunrise@192.168.41.239
```

> IP 变动时，用 HDMI+键盘 登录后 `ip addr show | grep "inet " | grep -v 127` 查看

---

## 2. 环境加载（每次 SSH 后必做）

```bash
source /opt/ros/humble/setup.bash
source ~/connect/R2_ws/install/setup.bash
```

> 设别名方便使用：
> ```bash
> echo 'alias r2env="source /opt/ros/humble/setup.bash && source ~/connect/R2_ws/install/setup.bash"' >> ~/.bashrc
> ```
> 以后只需 `r2env` 即可加载环境。

---

## 3. 启动方式

### 3.1 前提：硬件检查

```bash
# 确认串口设备
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# 若无权限
sudo usermod -aG dialout $USER
# 然后 exit 重新登录
```

### 3.2 奥丁雷达（Odin）

```bash
source /opt/ros/humble/setup.bash
source ~/connect/R2_ws/install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

> ⚠️ rviz2 在 SSH 下必定崩溃（无显示器），**忽略即可**，不影响数据。

### 3.3 键盘遥控

```bash
ros2 launch r2_communication keyboard_control.launch.py
# 自定义串口和步长
ros2 launch r2_communication keyboard_control.launch.py port:=/dev/ttyUSB0 linear_step:=0.3
```

### 3.4 行为树导航

```bash
python3 ~/connect/r2_behavior_tree.py
```

### 3.5 PID 自主任务

```bash
python3 ~/connect/mission_sequence_pid.py
```

### 3.6 重定位摆动

```bash
# 前提：Odin 已启动（重定位模式下）
python3 ~/connect/relocalization_wiggle.py
```

---

## 4. 同时启动多个节点

### 方案 A：tmux（推荐）

```bash
# 安装一次
sudo apt install -y tmux

# 创建会话
tmux new -s r2

# Odin 雷达
source /opt/ros/humble/setup.bash
source ~/connect/R2_ws/install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py

# Ctrl+B, C → 新窗口 → 键盘控制
source /opt/ros/humble/setup.bash
source ~/connect/R2_ws/install/setup.bash
ros2 launch r2_communication keyboard_control.launch.py

# Ctrl+B, D → 断开会话（后台继续运行）
# tmux attach -t r2 → 重新进入
```

### 方案 B：多个 SSH 终端

开两个终端窗口，分别 SSH 进去启动不同节点。

---

## 5. 常用调试命令

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看 TF 变换
ros2 run tf2_ros tf2_echo odom odin1_base_link

# 查看里程计
ros2 topic echo /odin1/odometry_high

# 查看话题发布频率
ros2 topic hz /odin1/odometry_high

# 发送底盘急停
ros2 topic pub --once /r2/estop std_msgs/msg/Empty

# 动态调参
ros2 param set /mission_sequence_pid max_linear 0.2

# 查看话题详细信息
ros2 topic info /odin1/odometry_high

# 录制 bag（调试用）
ros2 bag record -a -o ~/debug_bag
```

---

## 6. 编译相关

```bash
cd ~/connect/R2_ws
source /opt/ros/humble/setup.bash

# 全量编译（单线程，Jetson Nano 防 OOM）
colcon build --executor sequential --parallel-workers 1

# 只编译某个包
colcon build --packages-select r2_communication
colcon build --packages-select odin_ros_driver --executor sequential --parallel-workers 1

# 清理后重编
rm -rf build install log
colcon build --packages-select r2_communication odin_ros_driver --executor sequential --parallel-workers 1
```

---

## 7. 故障排查

| 现象 | 可能原因 | 命令 |
|------|----------|------|
| 串口打不开 | 设备路径错误/权限 | `ls /dev/ttyACM*` → `sudo usermod -aG dialout $USER` |
| 机器人不动 | 下位机无响应 | 检查串口连接，查看 `keyboard_control` 输出 |
| rviz2 报错 | SSH 无显示器 | **忽略**，不影响功能 |
| 编译 OOM 被杀 | Nano 内存不足 | 加 swap：`sudo fallocate -l 1G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile` |
| cloud_reprojection 退出 | 正常（sendreprojection=0） | 无需处理 |

---

## 8. 系统维护

```bash
# 查看内存
free -h

# 查看 CPU 温度
cat /sys/class/thermal/thermal_zone*/temp

# 查看磁盘
df -h

# 查看 swap
swapon --show

# 永久保留 swap（防止重启丢失）
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## 9. 快捷命令速查

| 操作 | 命令 |
|------|------|
| 连 Nano | `ssh ssr01@192.168.149.167` |
| 连 RDK X5 | `ssh sunrise@192.168.41.239` |
| 加载环境 | `source /opt/ros/humble/setup.bash && source ~/connect/R2_ws/install/setup.bash` |
| 启动雷达 | `ros2 launch odin_ros_driver odin1_ros2.launch.py` |
| 键盘遥控 | `ros2 launch r2_communication keyboard_control.launch.py` |
| 急停 | `ros2 topic pub --once /r2/estop std_msgs/msg/Empty` |
| 查节点 | `ros2 node list` |
| 编译 | `cd ~/connect/R2_ws && colcon build --executor sequential --parallel-workers 1` |

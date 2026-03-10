# R2 决策系统快速启动卡片

## 🚀 三步启动

```bash
cd /home/yf/ros2_ws                              # 1. 进入目录
colcon build --packages-select r2_decision_ros2 # 2. 编译ROS2包
source install/setup.bash                        # 3. 加载环境
```

---

## 🎯 启动命令速查

### C++ 单机模式（测试推荐）

```bash
./build/cpp/r2_decision_demo           # 完整系统
./build/cpp/r2_path_planner_demo       # 路径规划测试
./build/cpp/r2_hardware_example        # 硬件接口测试
```

### ROS2 多节点模式（比赛推荐）

```bash
# === 最常用 ===
ros2 launch r2_decision_ros2 r2_cold_start.launch.py    # ⭐ 冷启动
ros2 launch r2_decision_ros2 r2_return_mc.launch.py     # 返回武馆
ros2 launch r2_decision_ros2 r2_return_mf.launch.py     # 返回梅林

# === 真实硬件 ===
ros2 launch r2_decision_ros2 r2_real_hardware.launch.py # 真实设备

# === 测试调试 ===
ros2 launch r2_decision_ros2 r2_test_path_planning.launch.py  # 路径测试

# === 自定义参数 ===
ros2 launch r2_decision_ros2 r2_bringup.launch.py \
    launch_mode:=cold_start \
    use_sim_hardware:=false
```

---

## 📊 启动模式选择表

| 场景 | Launch文件 | 说明 |
|-----|-----------|------|
| 比赛开始 | `r2_cold_start.launch.py` | 完整流程 |
| 武器失败 | `r2_return_mc.launch.py` | 重拼武器 |
| KFS不足 | `r2_return_mf.launch.py` | 补充KFS |
| 真实比赛 | `r2_real_hardware.launch.py` | 真实硬件 |
| 调试算法 | `r2_test_path_planning.launch.py` | 路径测试 |

---

## 🔍 监控命令

```bash
ros2 node list                    # 查看节点
ros2 topic list                   # 查看话题
ros2 topic echo /r2/log           # 查看日志
ros2 topic echo /r2/phase         # 查看阶段
```

---

## 💡 参数说明

| 参数 | 值 | 说明 |
|-----|---|------|
| `launch_mode` | `cold_start` | 冷启动 |
|  | `return_to_mc` | 返回武馆 |
|  | `return_to_mf` | 返回梅林 |
| `use_sim_hardware` | `true` | 模拟硬件 |
|  | `false` | 真实硬件 |

---

## 📚 完整文档

- `STARTUP_GUIDE.md` - 完整启动指南
- `ros2/r2_decision_ros2/LAUNCH_FILES.md` - Launch文件详解
- `PATH_PLANNER_SUMMARY.md` - 路径规划总结

---

**无需修改代码，选择对应的launch文件即可！** 🎉

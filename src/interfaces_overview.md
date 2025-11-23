# 移动机器人（含机械臂）通信接口总览

下面用一张 Mermaid 图展示典型模块与主要 topic/service/action 的数据流关系（示例命名以 `/robot_alpha/...` 为前缀）。

```mermaid
graph LR
  subgraph Perception
    P(Lidar / Camera / IMU)
  end
  Localization[Localization\n(odom + tf)]
  Planner[Planner\n(global & local)]
  Controller[Base Controller\n(cmd_vel)]
  Base[Base Driver]
  Arm[Manipulator\n(trajectory/action)]
  Gripper[Gripper]
  Task[Task Manager]
  Diagnostics[Diagnostics / Heartbeat]
  Safety[Safety / E-Stop]

  P -->|/robot_alpha/sensor/lidar_front\n(sensor_msgs/LaserScan 或 sensor_msgs/Range)| Localization
  P -->|/robot_alpha/sensor/camera_rgb\n(sensor_msgs/Image)| Planner
  Localization -->|/robot_alpha/odom\n(nav_msgs/Odometry) + tf| Planner
  Planner -->|/robot_alpha/plan/global_path\n(nav_msgs/Path, QoS: reliable+transient_local)| Controller
  Planner -->|/robot_alpha/plan/local_velocity\n(geometry_msgs/Twist)| Controller
  Controller -->|/robot_alpha/cmd_vel\n(geometry_msgs/Twist)| Base
  Controller -->|/robot_alpha/arm/command\n(trajectory_msgs/JointTrajectory)| Arm
  Arm -->|/robot_alpha/arm/follow_joint_trajectory (action)| Task
  Task -->|/robot_alpha/task/execute (action)| Planner
  Gripper -->|/robot_alpha/gripper/state\n(sensor_msgs/JointState 或 std_msgs/Bool)| Task

  Diagnostics -->|/robot_alpha/diagnostics\n(diagnostic_msgs/DiagnosticArray)| All[All Nodes]
  Safety -->|/robot_alpha/emergency/stop\n(std_srvs/SetBool)| Controller
  Safety -->|/robot_alpha/emergency/stop| Arm

  classDef module fill:#f9f,stroke:#333,stroke-width:1px;
  class Perception,Localization,Planner,Controller,Base,Arm,Gripper,Task,Diagnostics,Safety module;
```

**图例与说明**
- **Topic 示例**: `/robot_alpha/sensor/lidar_front`、`/robot_alpha/cmd_vel`、`/robot_alpha/plan/global_path` 等。
- **Action 示例**: `/robot_alpha/arm/follow_joint_trajectory`（机械臂轨迹跟随）、`/robot_alpha/task/execute`（高层任务）。
- **QoS 建议**:
  - 传感器（IMU）: `reliable`, depth=50
  - 高带宽视觉/点云: `best_effort` 或根据网络选择
  - 控制/动作/诊断: `reliable`, depth=10; 全局路径: `transient_local`（latched-like）
- **TF**: 必须包含 `map`, `odom`, `base_link`, `arm_base_link`, `ee_link`。

**下一步建议**
- 如果需要我可以基于你提供的模块清单：
  - 生成详细通信矩阵（publisher/subscriber/service/action）
  - 自动产出对应的 `.msg/.srv/.action` 草案
  - 将 Mermaid 渲染为 PNG/SVG（若需要用于文档或报告）

---
文件路径：`/home/yf/ros2_ws/src/interfaces_overview.md`

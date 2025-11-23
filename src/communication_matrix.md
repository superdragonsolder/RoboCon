# 通信矩阵 — 移动机器人（含机械臂）

说明：本表按模块列出主要的 publishers/subscribers、services、actions、消息类型、建议 QoS 与频率范围。所有 topic/service/action 前缀示例统一使用 `/robot_alpha/...`。

---

## 1. 底盘（Chassis）
| 方向 | 名称 | Topic/Service/Action | 消息类型 | QoS 建议 | 频率/说明 |
|---:|---|---|---|---|---|
| 发布 | 里程计 | `/robot_alpha/base/odom` | `nav_msgs/Odometry` | reliable, depth=50 | 50-100 Hz（里程计） |
| 发布 | 位姿（可选） | `/robot_alpha/base/pose` | `geometry_msgs/PoseStamped` | reliable, transient_local opt. | 10-50 Hz |
| 发布 | 舵轮状态 | `/robot_alpha/base/wheel_state` | `sensor_msgs/JointState` | reliable | 20-100 Hz |
| 发布 | 短距激光（前/右/后/左） | `/robot_alpha/sensor/range_front` 等 | `sensor_msgs/Range` | reliable, depth=10 | 10-50 Hz |
| 订阅 | 速度命令 | `/robot_alpha/cmd_vel` | `geometry_msgs/Twist` | reliable | up to 50-200 Hz |
| 订阅 | 紧急停止 | `/robot_alpha/safety/emergency_stop` | `std_msgs/Bool` | reliable, low-latency | 事件驱动 |
| 服务 | 设置模式 | `/robot_alpha/base/set_mode` | `SetMode.srv` (自定义) | reliable | 同步调用 |
| 建议 | 台阶动作（可 action） | `/robot_alpha/base/step_climb` (建议为 action) | `StepClimb.action` | reliable | 支持 cancel & feedback |

---

## 2. 底盘四个方向短距激光（Short-range Range Sensors）
| 方向 | 名称 | Topic | 消息类型 | QoS | 说明 |
|---:|---|---|---|---|---|
| 发布 | 前短距 | `/robot_alpha/sensor/range_front` | `sensor_msgs/Range` | reliable, depth=10 | 精确近场对齐/上坡判定 |
| 发布 | 右短距 | `/robot_alpha/sensor/range_right` | `sensor_msgs/Range` | reliable, depth=10 | |
| 发布 | 后短距 | `/robot_alpha/sensor/range_back` | `sensor_msgs/Range` | reliable, depth=10 | |
| 发布 | 左短距 | `/robot_alpha/sensor/range_left` | `sensor_msgs/Range` | reliable, depth=10 | |
| 发布 | 聚合（可选） | `/robot_alpha/sensor/range_array` | `RangeArray.msg` (自定义) | reliable | 同步多个传感器读数 |

---

## 3. 单目相机（QR 扫描）
| 方向 | 名称 | Topic/Service | 消息类型 | QoS | 频率/说明 |
|---:|---|---|---|---|---|
| 发布 | 原始图像 | `/robot_alpha/camera/qr/image_raw` | `sensor_msgs/Image` | best_effort / reliable | 5-15 Hz |
| 发布 | 相机信息 | `/robot_alpha/camera/qr/camera_info` | `sensor_msgs/CameraInfo` | reliable | 静态或更新时发布 |
| 发布 | QR 检测 | `/robot_alpha/camera/qr/detections` | `QrDetection.msg` (自定义或 `vision_msgs/Detection2D`) | reliable | 事件驱动或 1-10 Hz |
| 服务 | 单次扫描 | `/robot_alpha/camera/qr/scan_once` | `QrScan.srv` (自定义) | reliable | 阻塞或超时返回 |

---

## 4. 双目相机（方块识别）
| 方向 | 名称 | Topic/Service | 消息类型 | QoS | 频率/说明 |
|---:|---|---|---|---|---|
| 发布 | 左图像 | `/robot_alpha/camera/stereo/left/image_raw` | `sensor_msgs/Image` | best_effort / reliable | 5-15 Hz |
| 发布 | 右图像 | `/robot_alpha/camera/stereo/right/image_raw` | `sensor_msgs/Image` | best_effort / reliable | 5-15 Hz |
| 发布 | 识别结果 | `/robot_alpha/perception/blocks` | `BlockList.msg` (自定义) 或 `vision_msgs/Detection3D` | reliable / best_effort | 2-10 Hz |
| 服务 | 查询方块 | `/robot_alpha/perception/block_query` | `GetBlock.srv` (自定义) | reliable | 同步查询 |
| 发布 | 点云（选配） | `/robot_alpha/camera/stereo/point_cloud` | `sensor_msgs/PointCloud2` | best_effort | 1-5 Hz |

---

## 5. 机械臂（Manipulator）
| 方向 | 名称 | Topic/Service/Action | 消息类型 | QoS | 频率/说明 |
|---:|---|---|---|---|---|
| 发布 | 关节状态 | `/robot_alpha/arm/joint_states` | `sensor_msgs/JointState` | reliable | 50-200 Hz |
| Action | 轨迹跟随 | `/robot_alpha/arm/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` (action) | reliable | 支持 feedback/cancel |
| Action | 抓取/放置 | `/robot_alpha/arm/pick_place` | `PickPlace.action` (自定义) | reliable | 高层抓取任务 |
| 服务 | 夹爪命令 | `/robot_alpha/arm/gripper_cmd` | `GripperCommand.srv` 或 action | reliable | 同步或异步控制夹爪 |
| 服务 | 校准 | `/robot_alpha/arm/calibrate` | `std_srvs/Empty` | reliable | 校准/归零 |

---

## 6. 大范围激光雷达（LIDAR，用于定位/SLAM）
| 方向 | 名称 | Topic/Service | 消息类型 | QoS | 频率/说明 |
|---:|---|---|---|---|---|
| 发布 | 激光扫描 | `/robot_alpha/lidar/scan` | `sensor_msgs/LaserScan` 或 `sensor_msgs/PointCloud2` | best_effort (高带宽) / reliable | 5-20 Hz |
| 发布 | 定位结果 | `/robot_alpha/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` / `nav_msgs/Odometry` | reliable | 5-20 Hz |
| 服务 | 定位重置 | `/robot_alpha/localization/reset` | `std_srvs/Empty` | reliable | 同步调用 |
| 发布 | 地图 | `/robot_alpha/mapping/map` | `nav_msgs/OccupancyGrid` | reliable + transient_local | 发布时（地图更新） |

---

## 7. 短距激光坐标（精细定位）
| 方向 | 名称 | Topic | 消息类型 | QoS | 说明 |
|---:|---|---|---|---|---|
| 发布 | 短距坐标（近场精确） | `/robot_alpha/sensor/precise_pose` | `geometry_msgs/PoseStamped` (可选) | reliable | 由短距传感器与融合模块发布，用于抓取对齐 |
| 发布 | 短距数组 | `/robot_alpha/sensor/range_array` | `RangeArray.msg` (自定义) | reliable | 聚合四路短距数据 |

---

## 8. 高层任务/调度（Task Manager）
| 方向 | 名称 | Topic/Service/Action | 消息类型 | QoS | 说明 |
|---:|---|---|---|---|---|
| Action | 执行任务 | `/robot_alpha/task/execute` | `Task.action` (自定义) | reliable | 复合任务（导航+抓取等） |
| 发布 | 任务状态 | `/robot_alpha/task/status` | `TaskStatus.msg` (自定义) | reliable | 事件或周期性发布 |
| 服务 | 取消所有任务 | `/robot_alpha/task/cancel_all` | `std_srvs/Empty` | reliable | 运维/控制优先级 |

---

## 9. 诊断 / 心跳 / 安全
| 方向 | 名称 | Topic/Service | 消息类型 | QoS | 频率/说明 |
|---:|---|---|---|---|---|
| 发布 | 心跳 | `/robot_alpha/health/heartbeat` | `Heartbeat.msg` (自定义) | reliable | 1 Hz |
| 发布 | 诊断 | `/robot_alpha/diagnostics` | `diagnostic_msgs/DiagnosticArray` | reliable | 1 Hz 或状态变化时 |
| 发布/订阅 | 紧急停止 | `/robot_alpha/safety/emergency_stop` | `std_msgs/Bool` 或 `std_srvs/SetBool` | reliable, low-latency | 立即停止控制回路 |
| 服务 | 清除 estop | `/robot_alpha/safety/clear_estop` | `std_srvs/SetBool` (需权限) | reliable | 管理员解锁 |

---

## 10. TF / 坐标框架（约定）
| Frame | 说明 |
|---|---|
| `map` | 全局地图坐标系 |
| `odom` | 里程计坐标系 |
| `base_link` | 车体中心 |
| `lidar_link` / `laser_link` | 大范围激光雷达 |
| `range_front_link` / `range_right_link` / `range_back_link` / `range_left_link` | 四路短距传感器各自 frame |
| `camera_qr_link` / `camera_stereo_left_link` / `camera_stereo_right_link` | 相机 frames |
| `arm_base_link` / `ee_link` | 机械臂基座与末端 |

发布频率建议：`odom->base_link` 50-100Hz，`map->odom` 1-10Hz；传感器到 `base_link` 的静态 TF 使用 `static_transform_publisher`。

---

## 附：自定义消息 / 服务 / 动作 简要模板
- `PickPlace.action`:
  - Goal: `geometry_msgs/PoseStamped target_pose`, `string object_id`, `float32 timeout_seconds`
  - Feedback: `float32 progress`
  - Result: `bool success`, `string info`

- `RangeArray.msg`:
  - `builtin_interfaces/Time stamp`
  - `float32[] ranges` # 建议长度 4（front,right,back,left）
  - `string[] labels`

- `Heartbeat.msg`:
  - `builtin_interfaces/Time stamp`
  - `string node_name`
  - `uint8 status` # 0=OK,1=WARN,2=ERROR
  - `string message`

---

文件路径：`/home/yf/ros2_ws/src/communication_matrix.md`

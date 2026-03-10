# R2 决策系统硬件集成方案

## 核心架构

### 三层设计

```
┌─────────────────────────────────────────────────────┐
│          应用层：决策逻辑 (mission_state_node)      │
│  - 状态机驱动 (StateMachine::tick)                 │
│  - 传感器数据融合                                  │
│  - 控制命令下发                                    │
└──────────────┬──────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────┐
│      硬件抽象层：HardwareManager                     │
│  - 设备注册与生命周期管理                          │
│  - 统一的接口规范（Sensor/Actuator Interface）     │
│  - 故障恢复与健康检测                              │
└──────────────┬──────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────┐
│        硬件驱动层：具体实现                         │
│  ├─ 传感器驱动（Laser/Camera/QR/Lidar）          │
│  ├─ 执行器驱动（Chassis/Arm）                     │
│  └─ 模拟实现（Mock硬件，用于开发）                │
└─────────────────────────────────────────────────────┘
```

---

## 文件组织

```
cpp/
├── include/r2_decision_cpp/
│   ├── hardware_interface.hpp      ← 硬件接口定义
│   │   ├── SensorInterface
│   │   ├── ActuatorInterface
│   │   ├── LaserSensorInterface
│   │   ├── QRScannerInterface
│   │   ├── MonoCameraInterface
│   │   ├── StereoCameraInterface
│   │   ├── LidarInterface
│   │   ├── ChassisInterface
│   │   └── ArmInterface
│   │
│   ├── hardware_manager.hpp        ← 设备管理器
│   │   └── HardwareManager
│   │
│   └── mock_hardware.hpp           ← 模拟实现（开发用）
│       ├── MockLaserSensor
│       ├── MockQRScanner
│       ├── MockMonoCamera
│       ├── MockStereoCamera
│       ├── MockLidar
│       ├── MockChassis
│       └── MockArm
│
└── src/
    ├── hardware_manager.cpp        ← 管理器实现
    └── hardware_example.cpp        ← 使用示例
```

---

## 快速集成步骤

### 第1步：定义你的硬件驱动

```cpp
// file: cpp/src/hardware/my_laser_sensor.cpp
#include "r2_decision_cpp/hardware_interface.hpp"
#include <serial/serial.h>

class MyLaserSensor : public r2::LaserSensorInterface {
private:
    serial::Serial port_;
    r2::LaserData data_;

public:
    bool connect() override {
        port_.open();
        return port_.isOpen();
    }

    void update() override {
        // 从硬件读取数据
        uint8_t buffer[16];
        port_.read(buffer, 16);
        // 解析数据到 data_
    }

    r2::LaserData get_data() const override { return data_; }
    // ... 其他接口实现
};
```

### 第2步：在节点中注册并使用

```cpp
// in mission_state_node.cpp
class MissionStateNode : public rclcpp::Node {
private:
    r2::HardwareManager hw_manager_;
    std::shared_ptr<r2::LaserSensorInterface> laser_;

    void init_hardware() {
        // 注册真实硬件或模拟硬件
        bool use_sim = this->declare_parameter("use_simulation", false).as_bool();

        if (use_sim) {
            auto sensor = std::make_shared<r2::mock::MockLaserSensor>();
            hw_manager_.register_laser("laser", sensor);
        } else {
            auto sensor = std::make_shared<MyLaserSensor>("/dev/ttyUSB0", 115200);
            hw_manager_.register_laser("laser", sensor);
        }

        laser_ = hw_manager_.get_laser("laser");
    }

    void on_tick() {
        // 更新传感器
        laser_->update();
        auto data = laser_->get_data();

        // 基于传感器数据做决策
        if (data.front_distance < 0.3f) {
            // 碰撞风险
            bus_.publish("collision_alert", true);
        }
    }
};
```

### 第3步：编译并测试

```bash
cd /home/yf/ros2_ws
cmake -S . -B build && cmake --build build -j
./build/cpp/r2_hardware_example  # 测试模拟硬件
ros2 launch r2_decision_ros2 r2_bringup.launch.py  # 测试真实硬件
```

---

## 硬件接口说明

### 传感器接口 (SensorInterface)

```cpp
class SensorInterface {
public:
    virtual void update() = 0;           // 读取数据（非阻塞）
    virtual bool is_ready() const = 0;   // 健康检测
    virtual bool connect() = 0;          // 连接硬件
    virtual void disconnect() = 0;       // 断开连接
};
```

**关键要点**：
- `update()` 必须**非阻塞**（使用线程或异步I/O）
- 数据通过 `get_*()` 方法返回，不应在 `update()` 中阻塞
- `is_ready()` 用于故障检测

### 执行器接口 (ActuatorInterface)

```cpp
class ActuatorInterface {
public:
    virtual bool execute_command() = 0;  // 执行控制命令
    virtual void stop() = 0;             // 紧急停止
    virtual bool is_ready() const = 0;   // 状态检测
    virtual bool connect() = 0;          // 连接硬件
    virtual void disconnect() = 0;       // 断开连接
};
```

**关键要点**：
- 命令应**队列化**，避免实时任务中的阻塞
- `stop()` 需要**可靠**且**快速**响应
- 执行状态通过 `get_*_feedback()` 返回

---

## 常见硬件集成模式

### 模式1：串口设备（激光、QR码扫描器）

```cpp
class SerialSensorBase : public SensorInterface {
protected:
    serial::Serial port_;

public:
    bool connect() override {
        return port_.open() && port_.isOpen();
    }

    void update() override {
        // 非阻塞读取
        if (port_.available() > 0) {
            parse_data();
        }
    }
};
```

### 模式2：ROS2 话题订阅（相机、雷达）

```cpp
class ROS2SensorAdapter : public SensorInterface {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

public:
    ROS2SensorAdapter(rclcpp::Node* node) {
        sub_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            std::bind(&ROS2SensorAdapter::on_image, this, std::placeholders::_1));
    }

    void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        // 异步处理
        process_image(msg);
    }
};
```

### 模式3：MoveIt 控制（机械臂）

```cpp
class MoveItArmAdapter : public ArmInterface {
private:
    moveit::planning_interface::MoveGroupInterface move_group_;

public:
    void move_to_pose(float x, float y, float z, 
                      float rx, float ry, float rz) override {
        geometry_msgs::msg::Pose target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        // ... 设置旋转 ...

        move_group_.setPoseTarget(target);
        move_group_.plan(plan);  // 规划
        move_group_.execute(plan);  // 执行
    }
};
```

---

## 模拟 vs 真实硬件切换

### 运行时切换

在 `config/params.yaml` 中：

```yaml
mission_state_node:
  ros__parameters:
    use_simulation: false
    hardware:
      laser_enabled: true
      camera_enabled: true
      arm_enabled: true
```

在节点中：

```cpp
bool use_sim = this->declare_parameter("hardware.laser_enabled", false).as_bool();

if (use_sim) {
    auto laser = std::make_shared<r2::mock::MockLaserSensor>();
} else {
    auto laser = std::make_shared<MyLaserSensor>(...);
}
```

### 混合模式（推荐用于集成测试）

```cpp
// 真实激光 + 模拟机械臂
hw_manager_.register_laser("laser", std::make_shared<MyLaserSensor>());
hw_manager_.register_arm("arm", std::make_shared<r2::mock::MockArm>());
```

---

## 常见问题

### Q1：如何处理传感器延迟？

```cpp
// 使用时间戳和缓冲
struct TimestampedData {
    r2::LaserData data;
    std::chrono::system_clock::time_point timestamp;
};

class BufferedLaser : public LaserSensorInterface {
private:
    std::deque<TimestampedData> buffer_;

    void on_tick() {
        // 清理过期数据（>200ms）
        auto now = std::chrono::system_clock::now();
        while (!buffer_.empty() && 
               std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - buffer_.front().timestamp).count() > 200) {
            buffer_.pop_front();
        }
    }
};
```

### Q2：如何处理硬件故障恢复？

```cpp
class ResilientSensor : public SensorInterface {
private:
    int failure_count_ = 0;
    static constexpr int MAX_FAILURES = 3;

    bool reconnect() {
        if (failure_count_ >= MAX_FAILURES) {
            return false;  // 放弃重连
        }
        failure_count_++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100 * failure_count_));
        return connect();
    }
};
```

### Q3：如何同时控制多个相同类型的设备？

```cpp
// 使用不同的名称
hw_manager_.register_laser("laser_front", front_laser);
hw_manager_.register_laser("laser_rear", rear_laser);

auto front = hw_manager_.get_laser("laser_front");
auto rear = hw_manager_.get_laser("laser_rear");
```

---

## 性能注意事项

| 优先级 | 考虑事项 |
|------|--------|
| 🔴 高 | `update()` 必须非阻塞，完成时间 <50ms |
| 🔴 高 | `get_*()` 方法应返回缓存数据，不进行计算 |
| 🟡 中 | 使用线程池处理密集计算（图像处理、点云处理） |
| 🟡 中 | 对传感器数据做时间戳检查，发现异常时告警 |
| 🟢 低 | 定期检查设备连接状态，日志记录 |

---

## 下一步建议

1. **实现你的硬件驱动**：根据实际设备选择合适的通信方式
2. **集成测试**：先用模拟 + 单个真实设备混合测试
3. **性能优化**：使用 ROS2 话题而非串口，利用硬件时间戳
4. **故障处理**：为每个设备添加超时检测和错误恢复
5. **监控面板**：使用 `rqt` 或 `foxglove` 可视化所有传感器实时数据


# 硬件集成指南

## 架构概览

R2 决策系统提供了一个**硬件抽象层（HAL）**，支持灵活、模块化的多设备集成。所有硬件设备分为两类：

```
┌──────────────────────────────┐
│   HardwareManager（管理器）   │ ← 统一管理所有设备
└──────────────────────────────┘
         ↙                    ↘
    传感器（Sensors）      执行器（Actuators）
    ├─ 激光测距仪          ├─ 底盘（舵轮）
    ├─ 二维码扫描器        └─ 机械臂
    ├─ 单目相机
    ├─ 双目相机
    └─ 雷达
```

---

## 1. 定义硬件实现

每种硬件需要继承对应的接口，实现具体的驱动逻辑。

### 示例：激光测距仪实现

假设您使用的是 **RPlidar A1** 或类似设备：

```cpp
#include "r2_decision_cpp/hardware_interface.hpp"
#include <serial/serial.h>  // 串口库

class LaserSensorImpl : public r2::LaserSensorInterface {
private:
    serial::Serial port_;
    r2::LaserData data_;
    std::string device_name_;

public:
    LaserSensorImpl(const std::string& port, uint32_t baudrate, 
                   const std::string& name = "laser")
        : device_name_(name), port_(port, baudrate) {}

    bool connect() override {
        if (!port_.isOpen()) {
            port_.open();
            return port_.isOpen();
        }
        return true;
    }

    void disconnect() override {
        if (port_.isOpen()) {
            port_.close();
        }
    }

    void update() override {
        // 从串口读取四个激光数据
        uint8_t buffer[20];
        if (port_.available() >= 20) {
            port_.read(buffer, 20);
            // 解析协议：buffer[0-3] = front/rear/left/right
            data_.front_distance = *((float*)&buffer[0]);
            data_.rear_distance = *((float*)&buffer[4]);
            data_.left_distance = *((float*)&buffer[8]);
            data_.right_distance = *((float*)&buffer[12]);
        }
    }

    bool is_ready() const override {
        return port_.isOpen() && data_.front_distance > 0;
    }

    std::string get_name() const override {
        return device_name_;
    }

    r2::LaserData get_data() const override {
        return data_;
    }
};
```

### 示例：双目相机实现

```cpp
#include "r2_decision_cpp/hardware_interface.hpp"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>  // Intel RealSense SDK

class StereoCameraImpl : public r2::StereoCameraInterface {
private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    cv::CascadeClassifier cascade_;
    std::string device_name_;

public:
    StereoCameraImpl(const std::string& name = "stereo_camera")
        : device_name_(name) {
        // 加载分类器（用于检测方块等）
        cascade_.load("/path/to/cascade.xml");
    }

    bool connect() override {
        try {
            cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
            cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            pipe_.start(cfg_);
            return true;
        } catch (...) {
            return false;
        }
    }

    void disconnect() override {
        pipe_.stop();
    }

    void update() override {
        // 定期获取帧，但不在这里处理（非阻塞）
    }

    bool is_ready() const override {
        return true;  // RealSense 总是就绪
    }

    std::string get_name() const override {
        return device_name_;
    }

    r2::StereoVisionResult detect_and_depth(const std::string& class_name) override {
        r2::StereoVisionResult result;

        try {
            rs2::frameset frames = pipe_.wait_for_frames();
            rs2::depth_frame depth = frames.get_depth_frame();
            rs2::video_frame color = frames.get_color_frame();

            int width = color.get_width();
            int height = color.get_height();
            cv::Mat color_mat(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

            // 使用级联分类器检测
            std::vector<cv::Rect> detections;
            cascade_.detectMultiScale(color_mat, detections, 1.1, 4);

            if (!detections.empty()) {
                cv::Rect detection = detections[0];
                int center_x = detection.x + detection.width / 2;
                int center_y = detection.y + detection.height / 2;

                float dist = depth.get_distance(center_x, center_y);
                auto point = depth.get_3d_coordinates(center_x, center_y, dist);

                result.detected = true;
                result.x_3d = point[0];
                result.y_3d = point[1];
                result.z_3d = point[2];
                result.depth = dist;
                result.object_class = class_name;
                result.confidence = 0.85f;
            }
        } catch (...) {
            result.detected = false;
        }

        return result;
    }
};
```

### 示例：底盘控制器实现

```cpp
#include "r2_decision_cpp/hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ChassisImpl : public r2::ChassisInterface {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Node* node_;
    r2::ChassisState state_;
    std::string device_name_;

public:
    ChassisImpl(rclcpp::Node* node, const std::string& name = "chassis")
        : node_(node), device_name_(name) {
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
    }

    bool connect() override {
        return cmd_pub_ != nullptr;
    }

    void disconnect() override {}

    void set_velocity(float vx, float vy, float wz) override {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = wz;
        cmd_pub_->publish(msg);
    }

    void set_position(float x, float y, float theta) override {
        // 通常使用导航堆栈完成
        // 这里仅更新状态，实际导航由上层处理
        state_.pos_x = x;
        state_.pos_y = y;
        state_.pos_theta = theta;
    }

    r2::ChassisState get_state() const override {
        return state_;
    }

    bool execute_command() override { return true; }
    void stop() override { set_velocity(0, 0, 0); }
    bool is_ready() const override { return cmd_pub_ != nullptr; }
    std::string get_name() const override { return device_name_; }
};
```

---

## 2. 在 ROS2 节点中集成硬件

修改 `mission_state_node.cpp`，引入硬件管理器：

```cpp
#include "r2_decision_cpp/hardware_manager.hpp"

class MissionStateNode : public rclcpp::Node {
private:
    r2::HardwareManager hw_manager_;
    
    // 传感器指针（缓存，便于快速访问）
    std::shared_ptr<r2::LaserSensorInterface> laser_;
    std::shared_ptr<r2::MonoCameraInterface> mono_camera_;
    std::shared_ptr<r2::StereoCameraInterface> stereo_camera_;
    std::shared_ptr<r2::QRScannerInterface> qr_scanner_;
    std::shared_ptr<r2::ChassisInterface> chassis_;
    std::shared_ptr<r2::ArmInterface> arm_;

public:
    MissionStateNode() : Node("mission_state_node"), sm_(bus_) {
        // ... 其他初始化 ...

        // 注册所有硬件设备
        if (!init_hardware()) {
            RCLCPP_ERROR(this->get_logger(), "硬件初始化失败");
            rclcpp::shutdown();
            return;
        }
    }

private:
    bool init_hardware() {
        try {
            // 注册激光传感器
            auto laser_impl = std::make_shared<LaserSensorImpl>(
                "/dev/ttyUSB0", 115200, "laser_front");
            hw_manager_.register_laser("laser_front", laser_impl);
            laser_ = hw_manager_.get_laser("laser_front");

            // 注册双目相机
            auto stereo_impl = std::make_shared<StereoCameraImpl>("realsense");
            hw_manager_.register_stereo_camera("realsense", stereo_impl);
            stereo_camera_ = hw_manager_.get_stereo_camera("realsense");

            // 注册单目相机（二维码识别）
            auto mono_impl = std::make_shared<MonoCameraImpl>("qr_camera");
            hw_manager_.register_mono_camera("qr_camera", mono_impl);
            mono_camera_ = hw_manager_.get_mono_camera("qr_camera");

            // 注册二维码扫描器
            auto qr_impl = std::make_shared<QRScannerImpl>("/dev/ttyUSB1");
            hw_manager_.register_qr_scanner("qr_scanner", qr_impl);
            qr_scanner_ = hw_manager_.get_qr_scanner("qr_scanner");

            // 注册底盘
            auto chassis_impl = std::make_shared<ChassisImpl>(this, "chassis");
            hw_manager_.register_chassis("chassis", chassis_impl);
            chassis_ = hw_manager_.get_chassis("chassis");

            // 注册机械臂（模拟或真实MoveIt接口）
            auto arm_impl = std::make_shared<ArmImpl>(this, "manipulator");
            hw_manager_.register_arm("manipulator", arm_impl);
            arm_ = hw_manager_.get_arm("manipulator");

            // 初始化所有设备
            if (!hw_manager_.initialize_all()) {
                RCLCPP_WARN(this->get_logger(), "某些设备初始化不完全");
            }

            RCLCPP_INFO(this->get_logger(), "硬件初始化成功");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "硬件初始化异常: %s", e.what());
            return false;
        }
    }

    void on_tick() {
        // 更新所有传感器
        if (laser_) laser_->update();
        if (stereo_camera_) stereo_camera_->update();
        if (mono_camera_) mono_camera_->update();
        if (qr_scanner_) qr_scanner_->update();

        // 读取传感器数据并更新决策状态
        if (laser_) {
            auto laser_data = laser_->get_data();
            // 如果碰撞，发布警告
            if (laser_data.front_distance < 0.3f) {
                bus_.publish("collision_detected", true);
            }
        }

        // 执行状态机
        const auto prev_logs = ctx_.logs.size();
        const auto p = sm_.tick(ctx_);

        // ... 发布结果 ...

        // 当进入对抗区时，控制机械臂放置方块
        if (p == r2::Phase::对抗区 && arm_) {
            if (!ctx_.holding_r2_kfs) return;
            
            // 移动到放置位置
            arm_->move_to_pose(0.5f, 0.0f, 0.1f, 0, 0, 0);
            arm_->wait_for_completion(5.0f);
            
            // 打开夹爪释放
            arm_->gripper_open();
            RCLCPP_INFO(this->get_logger(), "[对抗区] 已释放KFS");
        }
    }
};
```

---

## 3. 模拟模式（开发测试）

当硬件不可用时，可以使用**模拟实现**进行快速迭代开发：

```cpp
// 模拟激光传感器
class MockLaserSensor : public r2::LaserSensorInterface {
private:
    r2::LaserData data_;
    int counter_ = 0;

public:
    bool connect() override { return true; }
    void disconnect() override {}

    void update() override {
        // 周期性模拟数据变化
        data_.front_distance = 1.0f + 0.1f * std::sin(counter_ * 0.1f);
        data_.left_distance = 0.8f + 0.05f * std::sin(counter_ * 0.15f);
        counter_++;
    }

    bool is_ready() const override { return true; }
    std::string get_name() const override { return "mock_laser"; }
    r2::LaserData get_data() const override { return data_; }
};
```

使用模拟模式：

```cpp
// 在开发环境中
bool use_simulation = this->declare_parameter("use_simulation", false).as_bool();

if (use_simulation) {
    auto mock_laser = std::make_shared<MockLaserSensor>();
    hw_manager_.register_laser("laser_front", mock_laser);
    // ... 其他模拟设备 ...
} else {
    // ... 真实硬件 ...
}
```

---

## 4. 使用建议

| 场景 | 推荐做法 |
|------|--------|
| 早期开发 | 使用模拟设备，快速迭代决策逻辑 |
| 集成测试 | 混合真实+模拟（如真实激光+模拟相机） |
| 现场调试 | 所有设备真实，添加详细日志 |
| 故障排查 | 逐个禁用设备，隔离问题 |

---

## 5. 文件清单

新增文件：
- `cpp/include/r2_decision_cpp/hardware_interface.hpp` - 硬件接口定义
- `cpp/include/r2_decision_cpp/hardware_manager.hpp` - 硬件管理器头文件
- `cpp/src/hardware_manager.cpp` - 硬件管理器实现

需要实现的文件（由用户提供）：
- `cpp/src/hardware/laser_sensor_impl.cpp` - 激光传感器驱动
- `cpp/src/hardware/stereo_camera_impl.cpp` - 双目相机驱动
- `cpp/src/hardware/mono_camera_impl.cpp` - 单目相机驱动
- `cpp/src/hardware/qr_scanner_impl.cpp` - 二维码扫描器驱动
- `cpp/src/hardware/chassis_impl.cpp` - 底盘控制驱动
- `cpp/src/hardware/arm_impl.cpp` - 机械臂控制驱动


#pragma once

#include "r2_decision_cpp/hardware_interface.hpp"
#include <cmath>

namespace r2 {
namespace mock {

// ============================================================================
// 模拟传感器实现
// ============================================================================

/// 模拟激光测距仪
class MockLaserSensor : public LaserSensorInterface {
private:
    LaserData data_;
    int tick_count_ = 0;
    bool ready_ = false;

public:
    bool connect() override {
        ready_ = true;
        return true;
    }

    void disconnect() override { ready_ = false; }

    void update() override {
        // 周期性模拟传感器数据变化
        float t = tick_count_ * 0.1f;
        data_.front_distance = 1.0f + 0.2f * std::sin(t);
        data_.rear_distance = 0.9f + 0.15f * std::sin(t + 1.57f);
        data_.left_distance = 0.8f + 0.1f * std::sin(t + 3.14f);
        data_.right_distance = 0.85f + 0.15f * std::sin(t + 4.71f);
        tick_count_++;
    }

    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_laser"; }
    LaserData get_data() const override { return data_; }
};

/// 模拟二维码扫描器
class MockQRScanner : public QRScannerInterface {
private:
    bool ready_ = false;
    int scan_counter_ = 0;

public:
    bool connect() override {
        ready_ = true;
        return true;
    }

    void disconnect() override { ready_ = false; }
    void update() override {}
    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_qr_scanner"; }

    QRScanResult scan() override {
        QRScanResult result;
        if (scan_counter_++ % 3 == 0) {
            result.detected = true;
            result.qr_code = "R2_001_MC_01";
            result.confidence = 0.95f;
        } else {
            result.detected = false;
        }
        return result;
    }
};

/// 模拟单目相机
class MockMonoCamera : public MonoCameraInterface {
private:
    bool ready_ = false;
    int detect_counter_ = 0;

public:
    bool connect() override {
        ready_ = true;
        return true;
    }

    void disconnect() override { ready_ = false; }
    void update() override {}
    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_mono_camera"; }

    VisionResult detect_object(const std::string& class_name) override {
        VisionResult result;
        if (detect_counter_++ % 2 == 0) {
            result.detected = true;
            result.target_x = 320.0f + 50.0f * std::sin(detect_counter_ * 0.1f);
            result.target_y = 240.0f + 30.0f * std::cos(detect_counter_ * 0.15f);
            result.confidence = 0.88f;
            result.object_class = class_name;
        }
        return result;
    }
};

/// 模拟双目相机
class MockStereoCamera : public StereoCameraInterface {
private:
    bool ready_ = false;
    int detect_counter_ = 0;

public:
    bool connect() override {
        ready_ = true;
        return true;
    }

    void disconnect() override { ready_ = false; }
    void update() override {}
    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_stereo_camera"; }

    StereoVisionResult detect_and_depth(const std::string& class_name) override {
        StereoVisionResult result;
        if (detect_counter_++ % 3 == 0) {
            result.detected = true;
            result.depth = 0.5f + 0.1f * std::sin(detect_counter_ * 0.1f);
            result.x_3d = 0.3f + 0.05f * std::cos(detect_counter_ * 0.1f);
            result.y_3d = 0.0f + 0.02f * std::sin(detect_counter_ * 0.2f);
            result.z_3d = result.depth;
            result.object_class = class_name;
            result.confidence = 0.92f;
        }
        return result;
    }
};

/// 模拟雷达
class MockLidar : public LidarInterface {
private:
    bool ready_ = false;
    int tick_count_ = 0;

public:
    bool connect() override {
        ready_ = true;
        return true;
    }

    void disconnect() override { ready_ = false; }

    void update() override {}
    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_lidar"; }

    LidarData get_point_cloud() override {
        LidarData data;
        data.angle_resolution = 1.0f;
        data.num_samples = 360;
        data.distances.resize(360);

        // 生成圆形障碍物模拟
        for (int i = 0; i < 360; ++i) {
            float angle = i * 3.14159f / 180.0f;
            float base_dist = 2.0f;
            float obstacle_dist = 1.0f + 0.3f * std::sin(tick_count_ * 0.05f);

            // 检测范围内的障碍物
            if (i > 90 && i < 180) {
                data.distances[i] = obstacle_dist;
            } else {
                data.distances[i] = base_dist;
            }
        }
        tick_count_++;
        return data;
    }
};

// ============================================================================
// 模拟执行器实现
// ============================================================================

/// 模拟底盘控制
class MockChassis : public ChassisInterface {
private:
    ChassisState state_;
    bool ready_ = false;
    float target_vx_ = 0, target_vy_ = 0, target_wz_ = 0;

public:
    bool connect() override {
        ready_ = true;
        return true;
    }

    void disconnect() override { ready_ = false; }

    void set_velocity(float vx, float vy, float wz) override {
        target_vx_ = vx;
        target_vy_ = vy;
        target_wz_ = wz;
        
        // 模拟底盘加速
        state_.linear_x += (target_vx_ - state_.linear_x) * 0.1f;
        state_.linear_y += (target_vy_ - state_.linear_y) * 0.1f;
        state_.angular_z += (target_wz_ - state_.angular_z) * 0.1f;
    }

    void set_position(float x, float y, float theta) override {
        state_.pos_x = x;
        state_.pos_y = y;
        state_.pos_theta = theta;
    }

    ChassisState get_state() const override { return state_; }
    bool execute_command() override { return true; }
    void stop() override { set_velocity(0, 0, 0); }
    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_chassis"; }
};

/// 模拟机械臂
class MockArm : public ArmInterface {
private:
    ArmFeedback feedback_;
    bool ready_ = false;
    bool executing_ = false;

public:
    bool connect() override {
        ready_ = true;
        feedback_.gripper_open = true;
        return true;
    }

    void disconnect() override { ready_ = false; }

    void move_to_pose(float x, float y, float z, float rx, float ry,
                      float rz) override {
        executing_ = true;
        feedback_.is_moving = true;
    }

    void move_to_joint_angles(const std::vector<float>& angles) override {
        if (angles.size() >= 3) {
            feedback_.joint_1_angle = angles[0];
            feedback_.joint_2_angle = angles[1];
            feedback_.joint_3_angle = angles[2];
        }
        executing_ = true;
        feedback_.is_moving = true;
    }

    void gripper_open() override { feedback_.gripper_open = true; }
    void gripper_close() override { feedback_.gripper_open = false; }

    ArmFeedback get_feedback() const override { return feedback_; }

    void wait_for_completion(float timeout_sec) override {
        // 模拟完成时间
        feedback_.is_moving = false;
    }

    bool execute_command() override { return true; }
    void stop() override { executing_ = false; }
    bool is_ready() const override { return ready_; }
    std::string get_name() const override { return "mock_arm"; }
};

} // namespace mock
} // namespace r2

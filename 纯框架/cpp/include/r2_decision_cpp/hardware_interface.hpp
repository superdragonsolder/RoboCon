#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace r2 {

// ============================================================================
// 传感器数据结构
// ============================================================================

/// 激光测距仪数据（前后左右四个）
struct LaserData {
    float front_distance = 0.0f;   // 前方距离（米）
    float rear_distance = 0.0f;    // 后方距离（米）
    float left_distance = 0.0f;    // 左方距离（米）
    float right_distance = 0.0f;   // 右方距离（米）
};

/// 二维码扫描结果
struct QRScanResult {
    bool detected = false;
    std::string qr_code;           // 识别的二维码内容
    float confidence = 0.0f;       // 识别置信度 [0, 1]
};

/// 单目相机目标检测结果
struct VisionResult {
    bool detected = false;
    float target_x = 0.0f;         // 目标像素X坐标
    float target_y = 0.0f;         // 目标像素Y坐标
    float confidence = 0.0f;       // 置信度 [0, 1]
    std::string object_class;      // 识别的对象类别（如"fake_kfs", "real_kfs", "block"）
};

/// 双目相机深度与识别结果
struct StereoVisionResult {
    bool detected = false;
    float depth = 0.0f;            // 深度（米）
    float x_3d = 0.0f;             // 3D坐标X（米）
    float y_3d = 0.0f;             // 3D坐标Y（米）
    float z_3d = 0.0f;             // 3D坐标Z（米）
    std::string object_class;      // 对象类别（"block", "kfs_holder"等）
    float confidence = 0.0f;
};

/// 机械臂状态反馈
struct ArmFeedback {
    bool is_moving = false;
    bool gripper_open = true;
    float joint_1_angle = 0.0f;    // 关节1角度（度）
    float joint_2_angle = 0.0f;
    float joint_3_angle = 0.0f;
    bool force_sensor_triggered = false;  // 碰撞检测
    float gripper_force = 0.0f;    // 夹持力（牛）
};

/// 雷达点云简化信息
struct LidarData {
    std::vector<float> distances;  // 各方向距离（极坐标半径）
    float angle_resolution = 1.0f; // 角度分辨率（度）
    int num_samples = 360;         // 采样点数
};

/// 底盘运动状态
struct ChassisState {
    float linear_x = 0.0f;         // X方向速度（m/s）
    float linear_y = 0.0f;         // Y方向速度（m/s）
    float angular_z = 0.0f;        // Z轴旋转速度（rad/s）
    float pos_x = 0.0f;            // 位置X（米）
    float pos_y = 0.0f;            // 位置Y（米）
    float pos_theta = 0.0f;        // 方向角（弧度）
};

// ============================================================================
// 硬件接口基类
// ============================================================================

/// 传感器基类
class SensorInterface {
public:
    virtual ~SensorInterface() = default;
    
    /// 读取传感器数据（应为非阻塞）
    virtual void update() = 0;
    
    /// 检查传感器是否就绪
    virtual bool is_ready() const = 0;
    
    /// 获取传感器名称
    virtual std::string get_name() const = 0;
    
    /// 连接硬件设备
    virtual bool connect() = 0;
    
    /// 断开连接
    virtual void disconnect() = 0;
};

/// 执行器基类（电机、舵轮、机械臂等）
class ActuatorInterface {
public:
    virtual ~ActuatorInterface() = default;
    
    /// 发送控制命令
    virtual bool execute_command() = 0;
    
    /// 停止执行
    virtual void stop() = 0;
    
    /// 检查执行器是否就绪
    virtual bool is_ready() const = 0;
    
    /// 获取执行器名称
    virtual std::string get_name() const = 0;
    
    /// 连接硬件设备
    virtual bool connect() = 0;
    
    /// 断开连接
    virtual void disconnect() = 0;
};

// ============================================================================
// 具体传感器接口
// ============================================================================

/// 激光测距仪传感器
class LaserSensorInterface : public SensorInterface {
public:
    virtual LaserData get_data() const = 0;
};

/// 二维码扫描器
class QRScannerInterface : public SensorInterface {
public:
    virtual QRScanResult scan() = 0;
};

/// 单目相机
class MonoCameraInterface : public SensorInterface {
public:
    virtual VisionResult detect_object(const std::string& class_name) = 0;
};

/// 双目相机
class StereoCameraInterface : public SensorInterface {
public:
    virtual StereoVisionResult detect_and_depth(const std::string& class_name) = 0;
};

/// 雷达传感器
class LidarInterface : public SensorInterface {
public:
    virtual LidarData get_point_cloud() = 0;
};

// ============================================================================
// 具体执行器接口
// ============================================================================

/// 底盘驱动执行器（舵轮）
class ChassisInterface : public ActuatorInterface {
public:
    virtual void set_velocity(float vx, float vy, float wz) = 0;
    virtual void set_position(float x, float y, float theta) = 0;
    virtual ChassisState get_state() const = 0;
};

/// 机械臂控制执行器
class ArmInterface : public ActuatorInterface {
public:
    virtual void move_to_pose(float x, float y, float z, float rx, float ry, float rz) = 0;
    virtual void move_to_joint_angles(const std::vector<float>& angles) = 0;
    virtual void gripper_open() = 0;
    virtual void gripper_close() = 0;
    virtual ArmFeedback get_feedback() const = 0;
    virtual void wait_for_completion(float timeout_sec) = 0;
};

} // namespace r2

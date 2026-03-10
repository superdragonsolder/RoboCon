#pragma once

#include "r2_decision_cpp/hardware_interface.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace r2 {

/// 硬件设备管理器
class HardwareManager {
public:
    HardwareManager() = default;
    ~HardwareManager();

    // ========================================================================
    // 传感器管理
    // ========================================================================

    /// 注册激光传感器
    bool register_laser(const std::string& name,
                        std::shared_ptr<LaserSensorInterface> sensor);

    /// 注册二维码扫描器
    bool register_qr_scanner(const std::string& name,
                             std::shared_ptr<QRScannerInterface> scanner);

    /// 注册单目相机
    bool register_mono_camera(const std::string& name,
                              std::shared_ptr<MonoCameraInterface> camera);

    /// 注册双目相机
    bool register_stereo_camera(const std::string& name,
                                std::shared_ptr<StereoCameraInterface> camera);

    /// 注册雷达
    bool register_lidar(const std::string& name,
                        std::shared_ptr<LidarInterface> lidar);

    // ========================================================================
    // 执行器管理
    // ========================================================================

    /// 注册底盘控制器
    bool register_chassis(const std::string& name,
                          std::shared_ptr<ChassisInterface> chassis);

    /// 注册机械臂控制器
    bool register_arm(const std::string& name,
                      std::shared_ptr<ArmInterface> arm);

    // ========================================================================
    // 设备查询与访问
    // ========================================================================

    /// 获取指定的激光传感器
    std::shared_ptr<LaserSensorInterface> get_laser(const std::string& name) const;

    /// 获取指定的二维码扫描器
    std::shared_ptr<QRScannerInterface> get_qr_scanner(const std::string& name) const;

    /// 获取指定的单目相机
    std::shared_ptr<MonoCameraInterface> get_mono_camera(const std::string& name) const;

    /// 获取指定的双目相机
    std::shared_ptr<StereoCameraInterface> get_stereo_camera(const std::string& name) const;

    /// 获取指定的雷达
    std::shared_ptr<LidarInterface> get_lidar(const std::string& name) const;

    /// 获取指定的底盘控制器
    std::shared_ptr<ChassisInterface> get_chassis(const std::string& name) const;

    /// 获取指定的机械臂控制器
    std::shared_ptr<ArmInterface> get_arm(const std::string& name) const;

    // ========================================================================
    // 初始化与清理
    // ========================================================================

    /// 初始化所有已注册的设备
    bool initialize_all();

    /// 检查所有设备是否就绪
    bool check_all_ready() const;

    /// 关闭所有设备
    void shutdown_all();

    /// 获取所有传感器列表
    std::vector<std::string> get_all_sensor_names() const;

    /// 获取所有执行器列表
    std::vector<std::string> get_all_actuator_names() const;

private:
    // 传感器存储
    std::map<std::string, std::shared_ptr<LaserSensorInterface>> lasers_;
    std::map<std::string, std::shared_ptr<QRScannerInterface>> qr_scanners_;
    std::map<std::string, std::shared_ptr<MonoCameraInterface>> mono_cameras_;
    std::map<std::string, std::shared_ptr<StereoCameraInterface>> stereo_cameras_;
    std::map<std::string, std::shared_ptr<LidarInterface>> lidars_;

    // 执行器存储
    std::map<std::string, std::shared_ptr<ChassisInterface>> chassis_controllers_;
    std::map<std::string, std::shared_ptr<ArmInterface>> arm_controllers_;
};

} // namespace r2

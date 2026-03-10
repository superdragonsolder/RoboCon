#include "r2_decision_cpp/hardware_manager.hpp"

namespace r2 {

HardwareManager::~HardwareManager() {
    shutdown_all();
}

// 激光传感器注册
bool HardwareManager::register_laser(const std::string& name,
                                      std::shared_ptr<LaserSensorInterface> sensor) {
    if (!sensor) return false;
    lasers_[name] = sensor;
    return sensor->connect();
}

// 二维码扫描器注册
bool HardwareManager::register_qr_scanner(const std::string& name,
                                          std::shared_ptr<QRScannerInterface> scanner) {
    if (!scanner) return false;
    qr_scanners_[name] = scanner;
    return scanner->connect();
}

// 单目相机注册
bool HardwareManager::register_mono_camera(const std::string& name,
                                           std::shared_ptr<MonoCameraInterface> camera) {
    if (!camera) return false;
    mono_cameras_[name] = camera;
    return camera->connect();
}

// 双目相机注册
bool HardwareManager::register_stereo_camera(const std::string& name,
                                             std::shared_ptr<StereoCameraInterface> camera) {
    if (!camera) return false;
    stereo_cameras_[name] = camera;
    return camera->connect();
}

// 雷达注册
bool HardwareManager::register_lidar(const std::string& name,
                                     std::shared_ptr<LidarInterface> lidar) {
    if (!lidar) return false;
    lidars_[name] = lidar;
    return lidar->connect();
}

// 底盘注册
bool HardwareManager::register_chassis(const std::string& name,
                                       std::shared_ptr<ChassisInterface> chassis) {
    if (!chassis) return false;
    chassis_controllers_[name] = chassis;
    return chassis->connect();
}

// 机械臂注册
bool HardwareManager::register_arm(const std::string& name,
                                   std::shared_ptr<ArmInterface> arm) {
    if (!arm) return false;
    arm_controllers_[name] = arm;
    return arm->connect();
}

// 获取激光传感器
std::shared_ptr<LaserSensorInterface> HardwareManager::get_laser(
    const std::string& name) const {
    auto it = lasers_.find(name);
    return it != lasers_.end() ? it->second : nullptr;
}

// 获取二维码扫描器
std::shared_ptr<QRScannerInterface> HardwareManager::get_qr_scanner(
    const std::string& name) const {
    auto it = qr_scanners_.find(name);
    return it != qr_scanners_.end() ? it->second : nullptr;
}

// 获取单目相机
std::shared_ptr<MonoCameraInterface> HardwareManager::get_mono_camera(
    const std::string& name) const {
    auto it = mono_cameras_.find(name);
    return it != mono_cameras_.end() ? it->second : nullptr;
}

// 获取双目相机
std::shared_ptr<StereoCameraInterface> HardwareManager::get_stereo_camera(
    const std::string& name) const {
    auto it = stereo_cameras_.find(name);
    return it != stereo_cameras_.end() ? it->second : nullptr;
}

// 获取雷达
std::shared_ptr<LidarInterface> HardwareManager::get_lidar(
    const std::string& name) const {
    auto it = lidars_.find(name);
    return it != lidars_.end() ? it->second : nullptr;
}

// 获取底盘控制器
std::shared_ptr<ChassisInterface> HardwareManager::get_chassis(
    const std::string& name) const {
    auto it = chassis_controllers_.find(name);
    return it != chassis_controllers_.end() ? it->second : nullptr;
}

// 获取机械臂控制器
std::shared_ptr<ArmInterface> HardwareManager::get_arm(
    const std::string& name) const {
    auto it = arm_controllers_.find(name);
    return it != arm_controllers_.end() ? it->second : nullptr;
}

// 初始化所有设备
bool HardwareManager::initialize_all() {
    bool success = true;

    for (auto& [name, sensor] : lasers_) {
        sensor->update();
        if (!sensor->is_ready()) success = false;
    }
    for (auto& [name, sensor] : qr_scanners_) {
        sensor->update();
        if (!sensor->is_ready()) success = false;
    }
    for (auto& [name, sensor] : mono_cameras_) {
        sensor->update();
        if (!sensor->is_ready()) success = false;
    }
    for (auto& [name, sensor] : stereo_cameras_) {
        sensor->update();
        if (!sensor->is_ready()) success = false;
    }
    for (auto& [name, sensor] : lidars_) {
        sensor->update();
        if (!sensor->is_ready()) success = false;
    }

    for (auto& [name, actuator] : chassis_controllers_) {
        if (!actuator->is_ready()) success = false;
    }
    for (auto& [name, actuator] : arm_controllers_) {
        if (!actuator->is_ready()) success = false;
    }

    return success;
}

// 检查所有设备就绪状态
bool HardwareManager::check_all_ready() const {
    for (const auto& [name, sensor] : lasers_) {
        if (!sensor->is_ready()) return false;
    }
    for (const auto& [name, sensor] : qr_scanners_) {
        if (!sensor->is_ready()) return false;
    }
    for (const auto& [name, sensor] : mono_cameras_) {
        if (!sensor->is_ready()) return false;
    }
    for (const auto& [name, sensor] : stereo_cameras_) {
        if (!sensor->is_ready()) return false;
    }
    for (const auto& [name, sensor] : lidars_) {
        if (!sensor->is_ready()) return false;
    }
    for (const auto& [name, actuator] : chassis_controllers_) {
        if (!actuator->is_ready()) return false;
    }
    for (const auto& [name, actuator] : arm_controllers_) {
        if (!actuator->is_ready()) return false;
    }
    return true;
}

// 关闭所有设备
void HardwareManager::shutdown_all() {
    for (auto& [name, sensor] : lasers_) {
        sensor->disconnect();
    }
    for (auto& [name, sensor] : qr_scanners_) {
        sensor->disconnect();
    }
    for (auto& [name, sensor] : mono_cameras_) {
        sensor->disconnect();
    }
    for (auto& [name, sensor] : stereo_cameras_) {
        sensor->disconnect();
    }
    for (auto& [name, sensor] : lidars_) {
        sensor->disconnect();
    }
    for (auto& [name, actuator] : chassis_controllers_) {
        actuator->stop();
        actuator->disconnect();
    }
    for (auto& [name, actuator] : arm_controllers_) {
        actuator->stop();
        actuator->disconnect();
    }
}

// 获取所有传感器名称
std::vector<std::string> HardwareManager::get_all_sensor_names() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : lasers_) names.push_back(name);
    for (const auto& [name, _] : qr_scanners_) names.push_back(name);
    for (const auto& [name, _] : mono_cameras_) names.push_back(name);
    for (const auto& [name, _] : stereo_cameras_) names.push_back(name);
    for (const auto& [name, _] : lidars_) names.push_back(name);
    return names;
}

// 获取所有执行器名称
std::vector<std::string> HardwareManager::get_all_actuator_names() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : chassis_controllers_) names.push_back(name);
    for (const auto& [name, _] : arm_controllers_) names.push_back(name);
    return names;
}

} // namespace r2

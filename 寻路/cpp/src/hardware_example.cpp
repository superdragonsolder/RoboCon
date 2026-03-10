#include <iostream>
#include "r2_decision_cpp/hardware_manager.hpp"
#include "r2_decision_cpp/mock_hardware.hpp"

int main() {
    std::cout << "=== R2 硬件集成示例 ===" << std::endl;
    std::cout << std::endl;

    // 创建硬件管理器
    r2::HardwareManager hw_manager;

    // ========================================================================
    // 注册模拟硬件设备（开发测试环境）
    // ========================================================================

    std::cout << "[初始化] 注册硬件设备..." << std::endl;

    // 注册传感器
    auto laser = std::make_shared<r2::mock::MockLaserSensor>();
    hw_manager.register_laser("laser_front", laser);
    std::cout << "  ✓ 激光测距仪已注册" << std::endl;

    auto qr_scanner = std::make_shared<r2::mock::MockQRScanner>();
    hw_manager.register_qr_scanner("qr_scanner", qr_scanner);
    std::cout << "  ✓ 二维码扫描器已注册" << std::endl;

    auto mono_camera = std::make_shared<r2::mock::MockMonoCamera>();
    hw_manager.register_mono_camera("mono_camera", mono_camera);
    std::cout << "  ✓ 单目相机已注册" << std::endl;

    auto stereo_camera = std::make_shared<r2::mock::MockStereoCamera>();
    hw_manager.register_stereo_camera("stereo_camera", stereo_camera);
    std::cout << "  ✓ 双目相机已注册" << std::endl;

    auto lidar = std::make_shared<r2::mock::MockLidar>();
    hw_manager.register_lidar("lidar", lidar);
    std::cout << "  ✓ 雷达已注册" << std::endl;

    // 注册执行器
    auto chassis = std::make_shared<r2::mock::MockChassis>();
    hw_manager.register_chassis("chassis", chassis);
    std::cout << "  ✓ 底盘控制器已注册" << std::endl;

    auto arm = std::make_shared<r2::mock::MockArm>();
    hw_manager.register_arm("arm", arm);
    std::cout << "  ✓ 机械臂已注册" << std::endl;

    std::cout << std::endl;

    // ========================================================================
    // 初始化所有设备
    // ========================================================================

    std::cout << "[初始化] 连接设备..." << std::endl;
    if (!hw_manager.initialize_all()) {
        std::cerr << "错误：某些设备初始化失败" << std::endl;
        return 1;
    }
    std::cout << "  ✓ 所有设备就绪" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 列出已注册的设备
    // ========================================================================

    std::cout << "[设备清单]" << std::endl;
    std::cout << "  传感器:" << std::endl;
    for (const auto& name : hw_manager.get_all_sensor_names()) {
        std::cout << "    - " << name << std::endl;
    }
    std::cout << "  执行器:" << std::endl;
    for (const auto& name : hw_manager.get_all_actuator_names()) {
        std::cout << "    - " << name << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // 模拟周期性读取传感器数据
    // ========================================================================

    std::cout << "[运行] 周期性采样传感器数据..." << std::endl;
    std::cout << std::endl;

    for (int cycle = 0; cycle < 5; ++cycle) {
        std::cout << "--- 周期 " << (cycle + 1) << " ---" << std::endl;

        // 更新所有传感器
        auto laser_ptr = hw_manager.get_laser("laser_front");
        if (laser_ptr) {
            laser_ptr->update();
            auto laser_data = laser_ptr->get_data();
            std::cout << "  [激光] 前=" << laser_data.front_distance << "m"
                      << " 后=" << laser_data.rear_distance << "m"
                      << " 左=" << laser_data.left_distance << "m"
                      << " 右=" << laser_data.right_distance << "m" << std::endl;
        }

        // 二维码扫描
        auto qr_ptr = hw_manager.get_qr_scanner("qr_scanner");
        if (qr_ptr) {
            qr_ptr->update();
            auto result = qr_ptr->scan();
            if (result.detected) {
                std::cout << "  [二维码] 检测到: " << result.qr_code
                          << " (置信度: " << result.confidence << ")" << std::endl;
            } else {
                std::cout << "  [二维码] 未检测到" << std::endl;
            }
        }

        // 单目相机检测
        auto mono_ptr = hw_manager.get_mono_camera("mono_camera");
        if (mono_ptr) {
            mono_ptr->update();
            auto result = mono_ptr->detect_object("block");
            if (result.detected) {
                std::cout << "  [单目] 检测到方块 @ (" << result.target_x << ", "
                          << result.target_y << ")" << std::endl;
            }
        }

        // 双目相机检测
        auto stereo_ptr = hw_manager.get_stereo_camera("stereo_camera");
        if (stereo_ptr) {
            stereo_ptr->update();
            auto result = stereo_ptr->detect_and_depth("block");
            if (result.detected) {
                std::cout << "  [双目] 检测到对象 @ 深度=" << result.depth << "m"
                          << " 3D坐标=(" << result.x_3d << ", " << result.y_3d
                          << ", " << result.z_3d << ")" << std::endl;
            }
        }

        // 雷达
        auto lidar_ptr = hw_manager.get_lidar("lidar");
        if (lidar_ptr) {
            lidar_ptr->update();
            auto data = lidar_ptr->get_point_cloud();
            std::cout << "  [雷达] 扫描360度，采样数=" << data.num_samples << std::endl;
        }

        // 控制执行器
        auto chassis_ptr = hw_manager.get_chassis("chassis");
        if (chassis_ptr) {
            // 发送速度命令
            chassis_ptr->set_velocity(0.5f, 0.0f, 0.1f);
            auto state = chassis_ptr->get_state();
            std::cout << "  [底盘] 速度=(" << state.linear_x << ", "
                      << state.linear_y << ", " << state.angular_z << ")" << std::endl;
        }

        // 机械臂
        auto arm_ptr = hw_manager.get_arm("arm");
        if (arm_ptr && cycle == 2) {
            std::cout << "  [机械臂] 执行动作..." << std::endl;
            arm_ptr->move_to_pose(0.3f, 0.0f, 0.2f, 0, 0, 0);
            arm_ptr->gripper_close();
            auto feedback = arm_ptr->get_feedback();
            std::cout << "    - 夹爪状态: " << (feedback.gripper_open ? "打开" : "闭合") << std::endl;
        }

        std::cout << std::endl;
    }

    // ========================================================================
    // 关闭所有设备
    // ========================================================================

    std::cout << "[清理] 关闭所有设备..." << std::endl;
    hw_manager.shutdown_all();
    std::cout << "  ✓ 所有设备已关闭" << std::endl;

    return 0;
}

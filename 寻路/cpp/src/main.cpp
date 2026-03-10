#include <iostream>
#include <memory>

#include "r2_decision_cpp/flag_bus.hpp"
#include "r2_decision_cpp/hardware_manager.hpp"
#include "r2_decision_cpp/mock_hardware.hpp"
#include "r2_decision_cpp/models.hpp"
#include "r2_decision_cpp/state_machine.hpp"

int main() {
    using namespace r2;

    std::cout << "=== R2 决策树完整流程演示（含硬件集成） ===" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 初始化硬件管理器
    // ========================================================================
    auto hw_manager = std::make_shared<HardwareManager>();

    // 注册模拟硬件设备
    hw_manager->register_laser("laser_front", std::make_shared<mock::MockLaserSensor>());
    hw_manager->register_qr_scanner("qr_scanner", std::make_shared<mock::MockQRScanner>());
    hw_manager->register_mono_camera("mono_camera", std::make_shared<mock::MockMonoCamera>());
    hw_manager->register_stereo_camera("stereo_camera", std::make_shared<mock::MockStereoCamera>());
    hw_manager->register_lidar("lidar", std::make_shared<mock::MockLidar>());
    hw_manager->register_chassis("chassis", std::make_shared<mock::MockChassis>());
    hw_manager->register_arm("arm", std::make_shared<mock::MockArm>());

    // 初始化所有硬件
    if (!hw_manager->initialize_all()) {
        std::cerr << "硬件初始化失败" << std::endl;
        return 1;
    }

    std::cout << "✓ 硬件系统已初始化" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 初始化决策系统
    // ========================================================================

    FlagBus bus;
    RobotContext ctx;

    // 配置模拟传感器
    bus.publish("simulate_grab_ok", true);
    bus.publish("simulate_qr_ok", true);
    bus.publish("simulate_assemble_ok", true);

    // 创建状态机，传入硬件管理器
    R2DecisionStateMachine sm(bus, hw_manager);

    std::cout << "✓ 决策系统已初始化" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 执行完整的比赛流程
    // ========================================================================

    std::cout << "========== 开始比赛流程 ==========" << std::endl;
    std::cout << std::endl;

    for (int tick = 0; tick < 50; ++tick) {
        Phase p = sm.tick(ctx);

        // 关键状态点：武馆阶段后标记R1已离开
        if (p == Phase::武馆 && tick > 5) {
            ctx.r1_left_mc = true;
        }

        // 输出每个周期产生的日志
        if (!ctx.logs.empty()) {
            // 只在这个周期新增的日志
            static size_t last_log_count = 0;
            for (size_t i = last_log_count; i < ctx.logs.size(); ++i) {
                std::cout << ctx.logs[i] << std::endl;
            }
            last_log_count = ctx.logs.size();
        }

        // 到达结束阶段时停止
        if (p == Phase::结束) {
            break;
        }
    }

    std::cout << std::endl;
    std::cout << "========== 比赛流程结束 ==========" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 输出最终统计
    // ========================================================================

    std::cout << "【最终统计】" << std::endl;
    std::cout << "- 最终阶段: " << to_chinese(sm.phase()) << std::endl;
    std::cout << "- 当前携带R2 KFS数量: " << ctx.holding_r2_kfs << std::endl;
    std::cout << "- 重试次数: " << ctx.retry_count << std::endl;
    std::cout << "- 总日志行数: " << ctx.logs.size() << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 关闭硬件
    // ========================================================================

    hw_manager->shutdown_all();
    std::cout << "✓ 硬件系统已关闭" << std::endl;

    return 0;
}

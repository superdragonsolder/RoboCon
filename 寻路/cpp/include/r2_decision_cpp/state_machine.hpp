#pragma once

#include "r2_decision_cpp/flag_bus.hpp"
#include "r2_decision_cpp/hardware_manager.hpp"
#include "r2_decision_cpp/models.hpp"
#include "r2_decision_cpp/path_planner.hpp"
#include <memory>

namespace r2 {

struct TimingPolicy {
    int wait_before_grab_ms = 200;
    int grab_timeout_ms = 500;
    int assemble_timeout_ms = 500;
    int wait_after_assemble_ms = 200;
    int mc_move_timeout_ms = 3000;
    int mf_move_timeout_ms = 4000;
    int cf_move_timeout_ms = 3000;
};

class LaunchRouter {
public:
    LaunchMode choose_mode(const RobotContext& ctx) const;
};

class R2DecisionStateMachine {
public:
    explicit R2DecisionStateMachine(FlagBus& bus, 
                                    std::shared_ptr<HardwareManager> hw_manager = nullptr,
                                    TimingPolicy timing = {});

    Phase tick(RobotContext& ctx);
    Phase phase() const { return phase_; }

private:
    // 启动阶段
    void run_launch(RobotContext& ctx);
    void check_startup_sensors(RobotContext& ctx);
    void perform_launch_calibration(RobotContext& ctx);

    // 武馆阶段
    void run_mc(RobotContext& ctx);
    void rotate_to_find_weapon_head(RobotContext& ctx);
    void move_to_weapon_rack(RobotContext& ctx);
    bool grab_weapon_head_with_timeout(RobotContext& ctx) const;
    bool assemble_with_qr(RobotContext& ctx) const;

    // 梅林阶段
    void run_mf(RobotContext& ctx);
    void plan_meilin_path(RobotContext& ctx);  // 新增：路径规划
    void execute_meilin_path(RobotContext& ctx);  // 新增：执行路径
    void navigate_to_kfs_zone(RobotContext& ctx);
    void scan_and_approach_kfs(RobotContext& ctx);
    int collect_kfs_in_mf(RobotContext& ctx) const;

    // 对抗区阶段
    void run_cf(RobotContext& ctx);
    void enter_conflict_zone(RobotContext& ctx);
    void place_kfs_strategy(RobotContext& ctx);

    // 重试阶段
    void run_retry(RobotContext& ctx);

    // 保护函数
    bool violates_mf_guard() const;
    void check_collision_safety(RobotContext& ctx);
    void log_sensor_status(RobotContext& ctx);

    FlagBus& bus_;
    std::shared_ptr<HardwareManager> hw_manager_;
    TimingPolicy timing_;
    LaunchRouter launch_router_;
    Phase phase_ = Phase::启动;
    int sub_step_ = 0;  // 追踪当前阶段的子步骤
    
    // 梅林路径规划器（延迟初始化）
    std::unique_ptr<r2_decision::MeiHuaGrid> meihua_grid_;
    std::unique_ptr<r2_decision::PathPlanner> path_planner_;
};

const char* to_chinese(LaunchMode mode);
const char* to_chinese(Phase phase);

}  // namespace r2

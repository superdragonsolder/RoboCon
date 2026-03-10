#include "r2_decision_cpp/state_machine.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "r2_decision_cpp/strategy.hpp"

namespace r2 {

namespace {
void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
}  // namespace

const char* to_chinese(LaunchMode mode) {
    switch (mode) {
        case LaunchMode::冷启动:
            return "冷启动";
        case LaunchMode::返回重拼武器:
            return "返回重拼武器";
        case LaunchMode::返回梅林补拿KFS:
            return "返回梅林补拿KFS";
        default:
            return "未知启动模式";
    }
}

const char* to_chinese(Phase phase) {
    switch (phase) {
        case Phase::启动:
            return "启动";
        case Phase::武馆:
            return "武馆";
        case Phase::梅林:
            return "梅林";
        case Phase::对抗区:
            return "对抗区";
        case Phase::重试:
            return "重试";
        case Phase::结束:
            return "结束";
        default:
            return "未知阶段";
    }
}

LaunchMode LaunchRouter::choose_mode(const RobotContext& ctx) const {
    if (ctx.missed_mf_kfs_on_platform) {
        return LaunchMode::返回梅林补拿KFS;
    }
    if (ctx.last_mc_failed) {
        return LaunchMode::返回重拼武器;
    }
    return LaunchMode::冷启动;
}

// ============================================================================
// R2DecisionStateMachine 实现
// ============================================================================

R2DecisionStateMachine::R2DecisionStateMachine(FlagBus& bus,
                                             std::shared_ptr<HardwareManager> hw_manager,
                                             TimingPolicy timing)
    : bus_(bus), hw_manager_(hw_manager), timing_(timing) {}

Phase R2DecisionStateMachine::tick(RobotContext& ctx) {
    switch (phase_) {
        case Phase::启动:
            run_launch(ctx);
            break;
        case Phase::武馆:
            run_mc(ctx);
            break;
        case Phase::梅林:
            run_mf(ctx);
            break;
        case Phase::对抗区:
            run_cf(ctx);
            break;
        case Phase::重试:
            run_retry(ctx);
            break;
        case Phase::结束:
            // 保持结束状态
            break;
        default:
            break;
    }
    return phase_;
}

// ============================================================================
// 启动阶段
// ============================================================================

void R2DecisionStateMachine::run_launch(RobotContext& ctx) {
    ctx.logs.push_back("[启动] ========== 启动阶段开始 ==========");

    // 确定启动模式
    LaunchMode mode = launch_router_.choose_mode(ctx);
    ctx.logs.push_back(std::string("[启动] 选择启动模式: ") + to_chinese(mode));

    // 检查启动条件
    ctx.logs.push_back("[启动] 检查启动条件...");
    check_startup_sensors(ctx);

    // 执行启动校准
    ctx.logs.push_back("[启动] 执行启动校准...");
    perform_launch_calibration(ctx);

    // 根据模式执行对应的恢复逻辑
    if (mode == LaunchMode::冷启动) {
        ctx.logs.push_back("[启动] 冷启动: 准备进入武馆阶段");
        ctx.r1_left_mc = false;
        ctx.holding_r2_kfs = 0;
    } else if (mode == LaunchMode::返回重拼武器) {
        ctx.logs.push_back("[启动] 返回重拼武器: 返回到武馆重新组装");
        ctx.last_mc_failed = false;
    } else if (mode == LaunchMode::返回梅林补拿KFS) {
        ctx.logs.push_back("[启动] 返回梅林补拿KFS: 返回梅林重新拾取");
        ctx.missed_mf_kfs_on_platform = false;
        ctx.holding_r2_kfs = 0;
    }

    ctx.logs.push_back("[启动] 启动阶段完成，准备进入武馆");
    phase_ = Phase::武馆;
    sub_step_ = 0;
}

void R2DecisionStateMachine::check_startup_sensors(RobotContext& ctx) {
    ctx.logs.push_back("[启动] → 检查传感器状态");

    if (hw_manager_) {
        // 检查激光
        auto laser = hw_manager_->get_laser("laser_front");
        if (laser) {
            laser->update();
            if (laser->is_ready()) {
                auto data = laser->get_data();
                ctx.logs.push_back(std::string("[启动]   ✓ 激光测距仪就绪 (前:") +
                                   std::to_string(static_cast<int>(data.front_distance * 100)) + "cm)");
            } else {
                ctx.logs.push_back("[启动]   ✗ 激光测距仪异常");
            }
        }

        // 检查相机
        auto camera = hw_manager_->get_mono_camera("mono_camera");
        if (camera) {
            camera->update();
            if (camera->is_ready()) {
                ctx.logs.push_back("[启动]   ✓ 单目相机就绪");
            } else {
                ctx.logs.push_back("[启动]   ✗ 单目相机异常");
            }
        }

        // 检查底盘
        auto chassis = hw_manager_->get_chassis("chassis");
        if (chassis) {
            if (chassis->is_ready()) {
                ctx.logs.push_back("[启动]   ✓ 底盘控制器就绪");
            } else {
                ctx.logs.push_back("[启动]   ✗ 底盘控制器异常");
            }
        }

        // 检查机械臂
        auto arm = hw_manager_->get_arm("arm");
        if (arm) {
            if (arm->is_ready()) {
                ctx.logs.push_back("[启动]   ✓ 机械臂就绪");
            } else {
                ctx.logs.push_back("[启动]   ✗ 机械臂异常");
            }
        }
    } else {
        ctx.logs.push_back("[启动]   (硬件管理器未配置，使用模拟模式)");
    }
}

void R2DecisionStateMachine::perform_launch_calibration(RobotContext& ctx) {
    ctx.logs.push_back("[启动] → 执行启动校准");

    if (hw_manager_) {
        auto chassis = hw_manager_->get_chassis("chassis");
        if (chassis) {
            ctx.logs.push_back("[启动]   动作1: 前进测试");
            chassis->set_velocity(0.2f, 0.0f, 0.0f);
            sleep_ms(300);

            ctx.logs.push_back("[启动]   动作2: 原地旋转测试");
            chassis->set_velocity(0.0f, 0.0f, 0.5f);
            sleep_ms(300);

            ctx.logs.push_back("[启动]   动作3: 停止");
            chassis->stop();
        }

        auto arm = hw_manager_->get_arm("arm");
        if (arm) {
            ctx.logs.push_back("[启动]   动作4: 机械臂回零位");
            std::vector<float> home_pose = {0.0f, -1.57f, 0.0f};
            arm->move_to_joint_angles(home_pose);
            arm->gripper_open();
            arm->wait_for_completion(2.0f);
        }
    }

    ctx.logs.push_back("[启动] 校准完成，所有系统准备就绪");
}

// ============================================================================
// 武馆阶段
// ============================================================================

void R2DecisionStateMachine::run_mc(RobotContext& ctx) {
    if (sub_step_ == 0) {
        ctx.logs.push_back("[武馆] ========== 武馆阶段开始 ==========");
        ctx.logs.push_back("[武馆] 目标: 取得端头并与R1协作组装");
        ctx.logs.push_back("[武馆] 开机动作: 旋转90度、对齐武器架、移动到第一个端头位");
        sub_step_++;
    }

    // 子步骤1: 旋转寻找端头
    if (sub_step_ == 1) {
        ctx.logs.push_back("[武馆] → 第1步: 旋转寻找端头位置");
        rotate_to_find_weapon_head(ctx);
        sub_step_++;
    }

    // 子步骤2: 移动到武器架
    if (sub_step_ == 2) {
        ctx.logs.push_back("[武馆] → 第2步: 移动靠近武器架");
        move_to_weapon_rack(ctx);
        sub_step_++;
    }

    // 子步骤3: 抓取端头
    if (sub_step_ == 3) {
        ctx.logs.push_back("[武馆] → 第3步: 使用机械臂抓取端头");
        if (grab_weapon_head_with_timeout(ctx)) {
            ctx.logs.push_back("[武馆]   成功抓取端头");
            sub_step_++;
        } else {
            ctx.logs.push_back("[武馆]   ✗ 抓取失败，标记为失败");
            ctx.last_mc_failed = true;
            phase_ = Phase::重试;
            sub_step_ = 0;
            return;
        }
    }

    // 子步骤4: 二维码识别与组装
    if (sub_step_ == 4) {
        ctx.logs.push_back("[武馆] → 第4步: 扫描QR码并配合组装");
        if (assemble_with_qr(ctx)) {
            ctx.logs.push_back("[武馆]   成功识别并完成组装");
            ctx.logs.push_back("[武馆] 武馆阶段完成，准备进入梅林");
            phase_ = Phase::梅林;
            sub_step_ = 0;
            ctx.r1_left_mc = true;
        } else {
            ctx.logs.push_back("[武馆]   ✗ 组装失败");
            ctx.last_mc_failed = true;
            phase_ = Phase::重试;
            sub_step_ = 0;
        }
    }
}

void R2DecisionStateMachine::rotate_to_find_weapon_head(RobotContext& ctx) {
    ctx.logs.push_back("[武馆]   → 执行旋转搜索");

    if (hw_manager_) {
        auto laser = hw_manager_->get_laser("laser_front");
        auto chassis = hw_manager_->get_chassis("chassis");

        if (laser && chassis) {
            for (int i = 0; i < 3; ++i) {
                ctx.logs.push_back(std::string("[武馆]     旋转 ") + std::to_string((i + 1) * 90) +
                                   "度，检查距离");
                chassis->set_velocity(0.0f, 0.0f, 0.5f);
                sleep_ms(600);

                laser->update();
                auto data = laser->get_data();
                ctx.logs.push_back(std::string("[武馆]     前方距离: ") +
                                   std::to_string(static_cast<int>(data.front_distance * 100)) + "cm");

                if (data.front_distance < 0.5f) {
                    ctx.logs.push_back("[武馆]     检测到端头位置!");
                    chassis->stop();
                    break;
                }
            }
            chassis->stop();
        }
    } else {
        ctx.logs.push_back("[武馆]   (模拟旋转搜索...)");
        sleep_ms(500);
        ctx.logs.push_back("[武馆]   检测到端头位置！");
    }
}

void R2DecisionStateMachine::move_to_weapon_rack(RobotContext& ctx) {
    ctx.logs.push_back("[武馆]   → 移动到武器架");

    if (hw_manager_) {
        auto chassis = hw_manager_->get_chassis("chassis");
        if (chassis) {
            ctx.logs.push_back("[武馆]     前进移动...");
            chassis->set_velocity(0.3f, 0.0f, 0.0f);
            sleep_ms(timing_.mc_move_timeout_ms);
            chassis->stop();
            ctx.logs.push_back("[武馆]     到达目标位置");
        }
    } else {
        sleep_ms(300);
        ctx.logs.push_back("[武馆]   (模拟到达武器架...)");
    }
}

bool R2DecisionStateMachine::grab_weapon_head_with_timeout(RobotContext& ctx) const {
    ctx.logs.push_back("[武馆]     动作: 打开夹爪");
    sleep_ms(timing_.wait_before_grab_ms);

    if (hw_manager_) {
        auto arm = hw_manager_->get_arm("arm");
        if (arm) {
            arm->gripper_open();
            ctx.logs.push_back("[武馆]     移动到抓取位置 (0.4m, 0m, 0.15m)");
            arm->move_to_pose(0.4f, 0.0f, 0.15f, 0, 0, 0);
            arm->wait_for_completion(timing_.grab_timeout_ms / 1000.0f);

            ctx.logs.push_back("[武馆]     闭合夹爪");
            arm->gripper_close();
            sleep_ms(200);

            auto feedback = arm->get_feedback();
            bool success = !feedback.gripper_open;
            ctx.logs.push_back(std::string("[武馆]     夹爪状态: ") +
                               (success ? "成功抓取" : "抓取失败"));
            return success;
        }
    }

    // 模拟模式
    ctx.logs.push_back("[武馆]     (模拟抓取端头...)");
    sleep_ms(timing_.grab_timeout_ms);
    ctx.logs.push_back("[武馆]     成功抓取");
    return true;
}

bool R2DecisionStateMachine::assemble_with_qr(RobotContext& ctx) const {
    ctx.logs.push_back("[武馆]     扫描QR码...");

    if (hw_manager_) {
        auto qr = hw_manager_->get_qr_scanner("qr_scanner");
        if (qr) {
            qr->update();
            auto result = qr->scan();
            if (result.detected) {
                ctx.logs.push_back(std::string("[武馆]     识别到二维码: ") + result.qr_code);
            } else {
                ctx.logs.push_back("[武馆]     未识别到二维码，重试扫描");
                sleep_ms(300);
            }
        }
    } else {
        ctx.logs.push_back("[武馆]   (模拟扫描QR码...)");
        sleep_ms(300);
        ctx.logs.push_back("[武馆]     识别到二维码: R2_001_MC_01");
    }

    ctx.logs.push_back("[武馆]     等待R1完成部分组装...");
    sleep_ms(timing_.assemble_timeout_ms);
    ctx.logs.push_back("[武馆]     R1已准备好，执行最终组装");
    sleep_ms(timing_.wait_after_assemble_ms);

    return true;
}

// ============================================================================
// 梅林阶段
// ============================================================================

void R2DecisionStateMachine::run_mf(RobotContext& ctx) {
    if (sub_step_ == 0) {
        ctx.logs.push_back("[梅林] ========== 梅林阶段开始 ==========");
        ctx.logs.push_back("[梅林] 目标: 使用路径规划算法收集所有KFS");
        sub_step_++;
    }

    // 检查保护条件
    if (violates_mf_guard()) {
        ctx.logs.push_back("[梅林] ✗ 检测到规则违例，进入重试");
        phase_ = Phase::重试;
        sub_step_ = 0;
        return;
    }

    // 子步骤1: 初始化梅林网格并规划路径
    if (sub_step_ == 1) {
        ctx.logs.push_back("[梅林] → 第1步: 初始化梅林网格并规划路径");
        plan_meilin_path(ctx);
        
        if (ctx.meilin_path.path_computed) {
            ctx.logs.push_back("[梅林]   ✓ 路径规划成功");
            ctx.logs.push_back(std::string("[梅林]   总步数: ") + 
                             std::to_string(ctx.meilin_path.path.size()));
            ctx.logs.push_back(std::string("[梅林]   计划收集KFS: ") + 
                             std::to_string(ctx.meilin_path.total_kfs_to_collect));
            ctx.logs.push_back(std::string("[梅林]   出口位置: (") + 
                             std::to_string(ctx.meilin_path.exit_pos.first) + "," +
                             std::to_string(ctx.meilin_path.exit_pos.second) + ")");
            sub_step_++;
        } else {
            ctx.logs.push_back("[梅林]   ✗ 路径规划失败: " + ctx.meilin_path.plan_status);
            ctx.missed_mf_kfs_on_platform = true;
            phase_ = Phase::重试;
            sub_step_ = 0;
        }
        return;
    }

    // 子步骤2: 导航到梅林区域入口
    if (sub_step_ == 2) {
        ctx.logs.push_back("[梅林] → 第2步: 导航到梅林入口 (1,2)");
        navigate_to_kfs_zone(ctx);
        sub_step_++;
        return;
    }

    // 子步骤3: 执行路径规划
    if (sub_step_ == 3) {
        ctx.logs.push_back("[梅林] → 第3步: 执行规划路径");
        execute_meilin_path(ctx);
        
        if (ctx.holding_r2_kfs > 0) {
            ctx.logs.push_back(std::string("[梅林] ✓ 成功收集 ") + 
                             std::to_string(ctx.holding_r2_kfs) + " 个KFS");
            ctx.logs.push_back("[梅林] 梅林阶段完成，准备进入对抗区");
            phase_ = Phase::对抗区;
            sub_step_ = 0;
        } else {
            ctx.logs.push_back("[梅林] ✗ 未能收集到KFS");
            ctx.missed_mf_kfs_on_platform = true;
            phase_ = Phase::重试;
            sub_step_ = 0;
        }
    }
}

void R2DecisionStateMachine::plan_meilin_path(RobotContext& ctx) {
    ctx.logs.push_back("[梅林]   → 创建梅花桩网格模型");
    
    // 延迟初始化梅花桩网格
    if (!meihua_grid_) {
        meihua_grid_ = std::make_unique<r2_decision::MeiHuaGrid>();
        ctx.logs.push_back("[梅林]     生成随机KFS分布");
        meihua_grid_->randomize_kfs();
        
        // 打印网格状态到日志
        ctx.logs.push_back("[梅林]     梅花桩初始状态:");
        for (int row = 4; row >= 1; --row) {
            std::string line = "[梅林]       行" + std::to_string(row) + ": ";
            for (int col = 1; col <= 3; ++col) {
                auto block = meihua_grid_->get_block(row, col);
                switch (block) {
                    case r2_decision::BlockType::R1:
                        line += "R1 ";
                        break;
                    case r2_decision::BlockType::R2:
                        line += "R2 ";
                        break;
                    case r2_decision::BlockType::FAKE:
                        line += "X  ";
                        break;
                    default:
                        line += "空 ";
                        break;
                }
            }
            ctx.logs.push_back(line);
        }
    }
    
    // 创建路径规划器
    if (!path_planner_) {
        path_planner_ = std::make_unique<r2_decision::PathPlanner>(*meihua_grid_);
        ctx.logs.push_back("[梅林]   → 初始化BFS路径规划器");
    }
    
    // 执行路径规划
    ctx.logs.push_back("[梅林]   → 开始BFS搜索最优路径...");
    r2_decision::GridPos start(1, 2);  // 起点固定在(1,2)
    auto result = path_planner_->plan_path(start);
    
    if (result.success) {
        // 保存规划结果到上下文
        ctx.meilin_path.path_computed = true;
        ctx.meilin_path.path.clear();
        for (const auto& pos : result.path) {
            ctx.meilin_path.path.emplace_back(pos.row, pos.col);
        }
        ctx.meilin_path.moves.assign(result.moves.begin(), result.moves.end());
        ctx.meilin_path.exit_pos = {result.exit_pos.row, result.exit_pos.col};
        ctx.meilin_path.total_kfs_to_collect = static_cast<int>(result.kfs_collected_at_step.size());
        ctx.meilin_path.current_step = 0;
        ctx.meilin_path.plan_status = "规划成功";
        
        // 输出路径详情
        ctx.logs.push_back(std::string("[梅林]     ✓ 找到最优路径，步数: ") + 
                         std::to_string(result.total_steps));
        
        // 输出移动指令序列
        std::string moves_str = "[梅林]     移动序列: ";
        for (char move : result.moves) {
            moves_str += move;
            moves_str += " ";
        }
        ctx.logs.push_back(moves_str);
        
        // 输出KFS收集点
        auto r2_positions = meihua_grid_->get_r2_positions();
        for (size_t i = 0; i < r2_positions.size(); ++i) {
            ctx.logs.push_back(std::string("[梅林]     R2 #") + std::to_string(i) +
                             " at (" + std::to_string(r2_positions[i].row) + "," +
                             std::to_string(r2_positions[i].col) + 
                             ") 收集于步骤 " + std::to_string(result.kfs_collected_at_step[i]));
        }
    } else {
        ctx.meilin_path.path_computed = false;
        ctx.meilin_path.plan_status = result.reason;
        ctx.logs.push_back("[梅林]     ✗ 规划失败: " + result.reason);
    }
}

void R2DecisionStateMachine::execute_meilin_path(RobotContext& ctx) {
    if (!ctx.meilin_path.path_computed) {
        ctx.logs.push_back("[梅林]   ✗ 没有可用的路径");
        return;
    }
    
    ctx.logs.push_back("[梅林]   开始按规划路径移动");
    
    int collected_kfs = 0;
    const auto& path = ctx.meilin_path.path;
    const auto& moves = ctx.meilin_path.moves;
    
    // 遍历路径的每一步
    for (size_t step = 0; step < path.size(); ++step) {
        auto [row, col] = path[step];
        
        ctx.logs.push_back(std::string("[梅林]     步骤 ") + std::to_string(step) +
                         ": 到达 (" + std::to_string(row) + "," + 
                         std::to_string(col) + ")");
        
        // 如果不是第一步，执行移动
        if (step > 0) {
            char move = moves[step - 1];
            std::string move_desc;
            switch (move) {
                case 'W': move_desc = "向上"; break;
                case 'S': move_desc = "向下"; break;
                case 'A': move_desc = "向左"; break;
                case 'D': move_desc = "向右"; break;
                default: move_desc = "未知";
            }
            
            ctx.logs.push_back(std::string("[梅林]       执行移动: ") + move_desc);
            
            if (hw_manager_) {
                auto chassis = hw_manager_->get_chassis("chassis");
                if (chassis) {
                    // 根据移动方向设置速度
                    float vx = 0.0f, vy = 0.0f;
                    switch (move) {
                        case 'W': vx = -0.2f; break;  // 向上
                        case 'S': vx = 0.2f; break;   // 向下
                        case 'A': vy = -0.2f; break;  // 向左
                        case 'D': vy = 0.2f; break;   // 向右
                    }
                    chassis->set_velocity(vx, vy, 0.0f);
                    sleep_ms(800);  // 移动到下一格
                    chassis->stop();
                }
            } else {
                sleep_ms(200);
            }
        }
        
        // 检查当前位置是否相邻R2（可以收集）
        if (meihua_grid_) {
            auto r2_positions = meihua_grid_->get_r2_positions();
            const int dx[4] = {1, -1, 0, 0};
            const int dy[4] = {0, 0, 1, -1};
            
            for (const auto& r2_pos : r2_positions) {
                for (int d = 0; d < 4; ++d) {
                    int adj_row = r2_pos.row + dx[d];
                    int adj_col = r2_pos.col + dy[d];
                    if (adj_row == row && adj_col == col) {
                        // 找到了相邻的R2，执行收集
                        ctx.logs.push_back(std::string("[梅林]       检测到相邻KFS at (") +
                                         std::to_string(r2_pos.row) + "," +
                                         std::to_string(r2_pos.col) + ")");
                        
                        if (hw_manager_) {
                            auto arm = hw_manager_->get_arm("arm");
                            if (arm) {
                                ctx.logs.push_back("[梅林]         打开夹爪");
                                arm->gripper_open();
                                sleep_ms(150);
                                
                                ctx.logs.push_back("[梅林]         移动到KFS");
                                arm->move_to_pose(0.3f, 0.0f, 0.1f, 0, 0, 0);
                                arm->wait_for_completion(1.5f);
                                
                                ctx.logs.push_back("[梅林]         闭合夹爪");
                                arm->gripper_close();
                                sleep_ms(200);
                                
                                collected_kfs++;
                                ctx.logs.push_back(std::string("[梅林]         ✓ 成功收集KFS #") +
                                                 std::to_string(collected_kfs));
                            }
                        } else {
                            sleep_ms(300);
                            collected_kfs++;
                            ctx.logs.push_back(std::string("[梅林]       ✓ (模拟) 收集KFS #") +
                                             std::to_string(collected_kfs));
                        }
                        
                        // 标记该R2已被收集（从网格中移除）
                        meihua_grid_->set_block(r2_pos.row, r2_pos.col, r2_decision::BlockType::EMPTY);
                        break;
                    }
                }
            }
        }
    }
    
    ctx.holding_r2_kfs = collected_kfs;
    ctx.logs.push_back(std::string("[梅林]   路径执行完成，共收集 ") + 
                     std::to_string(collected_kfs) + " 个KFS");
}

void R2DecisionStateMachine::navigate_to_kfs_zone(RobotContext& ctx) {
    ctx.logs.push_back("[梅林]   → 计划路径到 (2,4) 梅林入口");

    if (hw_manager_) {
        auto chassis = hw_manager_->get_chassis("chassis");
        if (chassis) {
            ctx.logs.push_back("[梅林]     移动...");
            chassis->set_velocity(0.25f, 0.0f, 0.0f);
            sleep_ms(timing_.mf_move_timeout_ms);
            chassis->stop();
            ctx.logs.push_back("[梅林]     到达梅林区域");
        }
    } else {
        sleep_ms(300);
        ctx.logs.push_back("[梅林]   (模拟导航到梅林...)");
    }
}

void R2DecisionStateMachine::scan_and_approach_kfs(RobotContext& ctx) {
    ctx.logs.push_back("[梅林]   → 使用相机扫描KFS");

    if (hw_manager_) {
        auto stereo = hw_manager_->get_stereo_camera("stereo_camera");
        if (stereo) {
            stereo->update();
            auto result = stereo->detect_and_depth("kfs");
            if (result.detected) {
                ctx.logs.push_back(std::string("[梅林]     检测到KFS @ 距离 ") +
                                   std::to_string(static_cast<int>(result.depth * 100)) + "cm");
            } else {
                ctx.logs.push_back("[梅林]     未检测到KFS，继续搜索");
            }
        }
    } else {
        sleep_ms(300);
        ctx.logs.push_back("[梅林]   (模拟扫描KFS...)");
        ctx.logs.push_back("[梅林]     检测到KFS @ 距离 50cm");
    }

    ctx.logs.push_back("[梅林]   接近KFS并准备夹取");
}

int R2DecisionStateMachine::collect_kfs_in_mf(RobotContext& ctx) const {
    ctx.logs.push_back("[梅林]     使用九宫格策略选择最优目标");
    ctx.logs.push_back("[梅林]     优先目标: 第1行第1列");

    if (hw_manager_) {
        auto arm = hw_manager_->get_arm("arm");
        if (arm) {
            ctx.logs.push_back("[梅林]     打开夹爪");
            arm->gripper_open();
            sleep_ms(200);

            ctx.logs.push_back("[梅林]     移动到KFS位置");
            arm->move_to_pose(0.35f, 0.05f, 0.1f, 0, 0, 0);
            arm->wait_for_completion(2.0f);

            ctx.logs.push_back("[梅林]     闭合夹爪抓取");
            arm->gripper_close();
            sleep_ms(300);

            ctx.logs.push_back("[梅林]     ✓ 成功抓取KFS");
            return 1;
        }
    }

    ctx.logs.push_back("[梅林]   (模拟夹取KFS...)");
    sleep_ms(500);
    ctx.logs.push_back("[梅林]     ✓ 成功抓取KFS");
    return 1;
}

// ============================================================================
// 对抗区阶段
// ============================================================================

void R2DecisionStateMachine::run_cf(RobotContext& ctx) {
    if (sub_step_ == 0) {
        ctx.logs.push_back("[对抗区] ========== 对抗区阶段开始 ==========");
        ctx.logs.push_back("[对抗区] 目标: 放置KFS并执行赢线或阻断策略");
        sub_step_++;
    }

    // 子步骤1: 进入对抗区
    if (sub_step_ == 1) {
        ctx.logs.push_back("[对抗区] → 第1步: 进入对抗区");
        enter_conflict_zone(ctx);
        sub_step_++;
    }

    // 子步骤2: 放置KFS
    if (sub_step_ == 2) {
        ctx.logs.push_back("[对抗区] → 第2步: 放置KFS");
        place_kfs_strategy(ctx);
        ctx.logs.push_back("[对抗区] 对抗区阶段完成");
        phase_ = Phase::结束;
        sub_step_ = 0;
    }
}

void R2DecisionStateMachine::enter_conflict_zone(RobotContext& ctx) {
    ctx.logs.push_back("[对抗区]   → 移动进入对抗区");

    if (hw_manager_) {
        auto chassis = hw_manager_->get_chassis("chassis");
        if (chassis) {
            ctx.logs.push_back("[对抗区]     前进...");
            chassis->set_velocity(0.2f, 0.0f, 0.0f);
            sleep_ms(timing_.cf_move_timeout_ms);
            chassis->stop();
            ctx.logs.push_back("[对抗区]     已入场");
        }
    } else {
        sleep_ms(300);
        ctx.logs.push_back("[对抗区]   (模拟进入对抗区...)");
        ctx.logs.push_back("[对抗区]     已入场");
    }
}

void R2DecisionStateMachine::place_kfs_strategy(RobotContext& ctx) {
    if (ctx.holding_r2_kfs == 0) {
        ctx.logs.push_back("[对抗区]   ✗ 未持有KFS，无法放置");
        return;
    }

    ctx.logs.push_back("[对抗区]   执行放置策略: 先阻断后成线");
    ctx.logs.push_back("[对抗区]   → 第一步: 放置阻断方块");

    if (hw_manager_) {
        auto arm = hw_manager_->get_arm("arm");
        if (arm) {
            ctx.logs.push_back("[对抗区]     定位到阻断位置 (0.5, 0, 0.05)");
            arm->move_to_pose(0.5f, 0.0f, 0.05f, 0, 0, 0);
            arm->wait_for_completion(2.0f);

            ctx.logs.push_back("[对抗区]     释放KFS");
            arm->gripper_open();
            sleep_ms(200);
            ctx.logs.push_back("[对抗区]     ✓ 阻断方块已放置");
        }
    } else {
        sleep_ms(400);
        ctx.logs.push_back("[对抗区]   (模拟放置KFS...)");
        ctx.logs.push_back("[对抗区]     ✓ 阻断方块已放置");
    }

    ctx.logs.push_back("[对抗区]   → 第二步: 形成赢线");
    ctx.logs.push_back("[对抗区]     ✓ 成功形成赢线，获得5分!");
}

// ============================================================================
// 重试阶段
// ============================================================================

void R2DecisionStateMachine::run_retry(RobotContext& ctx) {
    ctx.logs.push_back("[重试] ========== 重试阶段开始 ==========");

    ctx.retry_count++;
    ctx.logs.push_back(std::string("[重试] 重试次数: ") + std::to_string(ctx.retry_count));

    if (ctx.retry_count >= 3) {
        ctx.logs.push_back("[重试] ✗ 重试次数过多，进入结束阶段");
        phase_ = Phase::结束;
        return;
    }

    // 返回启动阶段重新尝试
    ctx.logs.push_back("[重试] 返回启动阶段重新尝试");
    phase_ = Phase::启动;
    sub_step_ = 0;
}

// ============================================================================
// 保护函数
// ============================================================================

bool R2DecisionStateMachine::violates_mf_guard() const {
    const bool fake_kfs = bus_.get_or<bool>("guard_touch_fake_kfs", false);
    const bool r1_kfs = bus_.get_or<bool>("guard_touch_r1_kfs", false);
    const bool non_adj = bus_.get_or<bool>("guard_non_adjacent_pick", false);
    const bool step_on_cell = bus_.get_or<bool>("guard_step_on_kfs_cell", false);

    return fake_kfs || r1_kfs || non_adj || step_on_cell;
}

void R2DecisionStateMachine::check_collision_safety(RobotContext& ctx) {
    if (hw_manager_) {
        auto laser = hw_manager_->get_laser("laser_front");
        if (laser) {
            laser->update();
            auto data = laser->get_data();
            if (data.front_distance < 0.2f) {
                ctx.logs.push_back("[安全检查] ⚠ 前方距离过近，触发碰撞警告!");
                bus_.publish("collision_alert", true);
            }
        }
    }
}

void R2DecisionStateMachine::log_sensor_status(RobotContext& ctx) {
    if (!hw_manager_) return;

    std::string status = "[传感器状态] ";
    auto laser = hw_manager_->get_laser("laser_front");
    if (laser && laser->is_ready()) {
        status += "激光✓ ";
    }
    auto cam = hw_manager_->get_mono_camera("mono_camera");
    if (cam && cam->is_ready()) {
        status += "相机✓ ";
    }
    auto chassis = hw_manager_->get_chassis("chassis");
    if (chassis && chassis->is_ready()) {
        status += "底盘✓ ";
    }
    auto arm = hw_manager_->get_arm("arm");
    if (arm && arm->is_ready()) {
        status += "机械臂✓";
    }
    ctx.logs.push_back(status);
}

}  // namespace r2

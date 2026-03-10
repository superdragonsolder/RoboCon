#pragma once

#include <string>
#include <vector>

namespace r2 {

// 梅林路径规划相关
struct MeiLinPathInfo {
    bool path_computed = false;           // 是否已计算路径
    std::vector<std::pair<int,int>> path; // 路径坐标 (row,col)
    std::vector<char> moves;              // 移动指令 W/A/S/D
    std::pair<int,int> exit_pos;          // 出口位置
    int total_kfs_to_collect = 0;         // 计划收集的KFS数量
    int current_step = 0;                 // 当前执行到第几步
    std::string plan_status;              // 规划状态信息
};

enum class LaunchMode {
    冷启动,
    返回重拼武器,
    返回梅林补拿KFS,
};

enum class Phase {
    启动,
    武馆,
    梅林,
    对抗区,
    重试,
    结束,
};

struct RobotContext {
    double robot_size_m = 0.8;
    bool use_relative_pose_in_mf = true;

    bool has_weapon_head = false;
    bool assembled_ok = false;
    bool qr_ok = false;
    bool r1_left_mc = false;
    bool last_mc_failed = false;

    int holding_r2_kfs = 0;
    bool missed_mf_kfs_on_platform = false;

    bool in_match = true;
    int retry_count = 0;

    // 梅林路径规划信息
    MeiLinPathInfo meilin_path;

    std::vector<std::string> logs;

    void log(const std::string& line) {
        logs.push_back(line);
    }
};

}  // namespace r2

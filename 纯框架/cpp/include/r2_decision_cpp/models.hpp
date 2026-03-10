#pragma once

#include <string>
#include <vector>

namespace r2 {

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

    std::vector<std::string> logs;

    void log(const std::string& line) {
        logs.push_back(line);
    }
};

}  // namespace r2

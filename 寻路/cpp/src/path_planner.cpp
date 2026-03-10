#include "r2_decision_cpp/path_planner.hpp"
#include <iostream>
#include <random>
#include <algorithm>
#include <queue>
#include <climits>
#include <cstring>

namespace r2_decision {

// ==================== MeiHuaGrid Implementation ====================

MeiHuaGrid::MeiHuaGrid() {
    // 初始化所有为EMPTY
    for (auto& row : block_) {
        row.fill(BlockType::EMPTY);
    }
    for (auto& row : known_fake_) {
        row.fill(false);
    }
    
    // 初始化高度数据（从原始代码）
    int heights[6][5] = {
        {0, 0,   0,   0,   0},
        {0, 400, 200, 400, 0},
        {0, 200, 400, 600, 0},
        {0, 400, 600, 400, 0},
        {0, 200, 400, 200, 0},
        {0, 0,   0,   0,   0}
    };
    
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 5; ++j) {
            height_[i][j] = heights[i][j];
        }
    }
}

bool MeiHuaGrid::is_forbidden_for_r1(int row, int col) const {
    // R1不能放在最中间的两个格子 (2,2) 和 (3,2)
    return (row == 2 && col == 2) || (row == 3 && col == 2);
}

bool MeiHuaGrid::is_forbidden_for_fake(int row, int col) const {
    // 假方块不能放在第4行
    return row == 4;
}

void MeiHuaGrid::randomize_kfs() {
    // 重置所有有效格子为EMPTY
    for (int i = 1; i <= 4; ++i) {
        for (int j = 1; j <= 3; ++j) {
            block_[i][j] = BlockType::EMPTY;
        }
    }
    
    // 收集所有有效位置
    std::vector<GridPos> all_positions;
    for (int i = 1; i <= 4; ++i) {
        for (int j = 1; j <= 3; ++j) {
            all_positions.emplace_back(i, j);
        }
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 1. 放置3个R1（不能在forbidden位置）
    std::vector<GridPos> candidates_r1;
    for (const auto& pos : all_positions) {
        if (!is_forbidden_for_r1(pos.row, pos.col)) {
            candidates_r1.push_back(pos);
        }
    }
    std::shuffle(candidates_r1.begin(), candidates_r1.end(), gen);
    int r1_count = std::min(3, static_cast<int>(candidates_r1.size()));
    for (int k = 0; k < r1_count; ++k) {
        block_[candidates_r1[k].row][candidates_r1[k].col] = BlockType::R1;
    }
    
    // 2. 收集剩余空位
    std::vector<GridPos> remaining;
    for (const auto& pos : all_positions) {
        if (block_[pos.row][pos.col] == BlockType::EMPTY) {
            remaining.push_back(pos);
        }
    }
    std::shuffle(remaining.begin(), remaining.end(), gen);
    
    // 3. 放置4个R2
    int r2_count = std::min(4, static_cast<int>(remaining.size()));
    for (int k = 0; k < r2_count; ++k) {
        block_[remaining[k].row][remaining[k].col] = BlockType::R2;
    }
    
    // 4. 放置1个FAKE（不能在第4行）
    if (remaining.size() > static_cast<size_t>(r2_count)) {
        for (size_t k = r2_count; k < remaining.size(); ++k) {
            if (!is_forbidden_for_fake(remaining[k].row, remaining[k].col)) {
                block_[remaining[k].row][remaining[k].col] = BlockType::FAKE;
                break;
            }
        }
    }
}

void MeiHuaGrid::set_block(int row, int col, BlockType type) {
    if (row >= 1 && row <= 4 && col >= 1 && col <= 3) {
        block_[row][col] = type;
    }
}

BlockType MeiHuaGrid::get_block(int row, int col) const {
    if (row >= 1 && row <= 4 && col >= 1 && col <= 3) {
        return block_[row][col];
    }
    return BlockType::EMPTY;
}

int MeiHuaGrid::get_height(int row, int col) const {
    if (row >= 0 && row < 6 && col >= 0 && col < 5) {
        return height_[5 - row][col];  // 注意坐标转换
    }
    return 0;
}

void MeiHuaGrid::mark_fake_known(int row, int col) {
    if (row >= 1 && row <= 4 && col >= 1 && col <= 3) {
        known_fake_[row][col] = true;
    }
}

bool MeiHuaGrid::is_fake_known(int row, int col) const {
    if (row >= 1 && row <= 4 && col >= 1 && col <= 3) {
        return known_fake_[row][col];
    }
    return false;
}

void MeiHuaGrid::print_grid() const {
    std::cout << "\n梅花桩网格状态 (行4->1, 列1->3):\n";
    for (int i = 4; i >= 1; --i) {
        std::cout << "行" << i << ": ";
        for (int j = 1; j <= 3; ++j) {
            switch (block_[i][j]) {
                case BlockType::EMPTY:
                    std::cout << "空  ";
                    break;
                case BlockType::R1:
                    std::cout << "R1 ";
                    break;
                case BlockType::R2:
                    std::cout << "R2 ";
                    break;
                case BlockType::FAKE:
                    std::cout << "X  ";
                    break;
            }
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
}

std::vector<GridPos> MeiHuaGrid::get_r2_positions() const {
    std::vector<GridPos> r2_list;
    for (int i = 1; i <= 4; ++i) {
        for (int j = 1; j <= 3; ++j) {
            if (block_[i][j] == BlockType::R2) {
                r2_list.emplace_back(i, j);
            }
        }
    }
    return r2_list;
}

// ==================== PathPlanner Implementation ====================

PathPlanner::PathPlanner(const MeiHuaGrid& grid) 
    : grid_(grid) {
    // 默认允许两个出口
    allowed_exits_.push_back(GridPos(4, 1));
    allowed_exits_.push_back(GridPos(4, 3));
}

void PathPlanner::set_exit_allowed(const GridPos& exit, bool allowed) {
    auto it = std::find(allowed_exits_.begin(), allowed_exits_.end(), exit);
    if (allowed && it == allowed_exits_.end()) {
        allowed_exits_.push_back(exit);
    } else if (!allowed && it != allowed_exits_.end()) {
        allowed_exits_.erase(it);
    }
}

int PathPlanner::compute_collection_mask(int row, int col, const std::vector<GridPos>& r2_list) const {
    int mask = 0;
    // 机器人在相邻格子时可以收集KFS（不是在KFS格子本身）
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    
    for (size_t k = 0; k < r2_list.size(); ++k) {
        const auto& r2 = r2_list[k];
        for (int d = 0; d < 4; ++d) {
            int adj_row = r2.row + dx[d];
            int adj_col = r2.col + dy[d];
            if (adj_row == row && adj_col == col) {
                mask |= (1 << k);
                break;
            }
        }
    }
    return mask;
}

PathPlanResult PathPlanner::plan_path(const GridPos& start) {
    PathPlanResult result;
    
    // 获取所有R2位置
    auto r2_list = grid_.get_r2_positions();
    int R = static_cast<int>(r2_list.size());
    
    if (R == 0) {
        result.reason = "没有R2方块需要收集";
        return result;
    }
    
    if (R > 16) {
        result.reason = "R2数量超过限制(16)";
        return result;
    }
    
    // 检查起点是否被假方块堵住
    if (grid_.get_block(start.row, start.col) == BlockType::FAKE) {
        result.reason = "起始位置被假方块堵住";
        return result;
    }
    
    int full_mask = (1 << R) - 1;
    
    // BFS状态空间: [row][col][mask]
    static bool visited[5][4][1 << 16];
    static std::tuple<int, int, int> prev_state[5][4][1 << 16];
    static char prev_move[5][4][1 << 16];
    
    std::memset(visited, false, sizeof(visited));
    
    std::queue<BFSState> q;
    int start_mask = compute_collection_mask(start.row, start.col, r2_list);
    
    visited[start.row][start.col][start_mask] = true;
    prev_state[start.row][start.col][start_mask] = std::make_tuple(-1, -1, -1);
    prev_move[start.row][start.col][start_mask] = 'X';
    q.push({start.row, start.col, start_mask, 0});
    
    // 跟踪每个出口的最佳状态
    std::vector<std::tuple<int, int, BFSState>> exit_best;  // depth, exit_idx, state
    
    while (!q.empty()) {
        auto state = q.front();
        q.pop();
        
        // 检查是否达到目标（收集所有R2）
        if (state.mask == full_mask) {
            for (size_t i = 0; i < allowed_exits_.size(); ++i) {
                if (state.row == allowed_exits_[i].row && 
                    state.col == allowed_exits_[i].col) {
                    exit_best.push_back(std::make_tuple(state.depth, i, state));
                }
            }
        }
        
        // 扩展邻居
        const int dx[4] = {-1, 0, 1, 0};
        const int dy[4] = {0, -1, 0, 1};
        const char moves[4] = {'W', 'A', 'S', 'D'};
        
        for (int d = 0; d < 4; ++d) {
            int new_row = state.row + dx[d];
            int new_col = state.col + dy[d];
            
            if (new_row < 1 || new_row > 4 || new_col < 1 || new_col > 3) {
                continue;
            }
            
            // 不能踩到假方块
            if (grid_.get_block(new_row, new_col) == BlockType::FAKE) {
                continue;
            }
            
            int new_mask = state.mask | compute_collection_mask(new_row, new_col, r2_list);
            
            if (!visited[new_row][new_col][new_mask]) {
                visited[new_row][new_col][new_mask] = true;
                prev_state[new_row][new_col][new_mask] = 
                    std::make_tuple(state.row, state.col, state.mask);
                prev_move[new_row][new_col][new_mask] = moves[d];
                q.push({new_row, new_col, new_mask, state.depth + 1});
            }
        }
    }
    
    // 检查是否有假方块阻塞某个出口
    for (auto& exit : allowed_exits_) {
        if (grid_.get_block(exit.row, exit.col) == BlockType::FAKE) {
            exit_best.erase(
                std::remove_if(exit_best.begin(), exit_best.end(),
                    [&](const auto& item) {
                        return std::get<2>(item).row == exit.row && 
                               std::get<2>(item).col == exit.col;
                    }),
                exit_best.end()
            );
        }
    }
    
    if (exit_best.empty()) {
        result.reason = "无法找到满足条件的路径（无法收集所有R2或到达出口）";
        return result;
    }
    
    // 选择步数最少的出口
    auto best = *std::min_element(exit_best.begin(), exit_best.end(),
        [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });
    
    BFSState goal_state = std::get<2>(best);
    
    // 重建路径
    std::vector<GridPos> path;
    int curr_row = goal_state.row;
    int curr_col = goal_state.col;
    int curr_mask = goal_state.mask;
    
    while (true) {
        path.emplace_back(curr_row, curr_col);
        auto prev = prev_state[curr_row][curr_col][curr_mask];
        int prev_row = std::get<0>(prev);
        
        if (prev_row == -1) break;
        
        curr_row = prev_row;
        curr_col = std::get<1>(prev);
        curr_mask = std::get<2>(prev);
    }
    
    std::reverse(path.begin(), path.end());
    
    // 生成移动指令
    std::vector<char> moves;
    for (size_t i = 1; i < path.size(); ++i) {
        int dr = path[i].row - path[i-1].row;
        int dc = path[i].col - path[i-1].col;
        
        if (dr == -1) moves.push_back('W');
        else if (dr == 1) moves.push_back('S');
        else if (dc == -1) moves.push_back('A');
        else if (dc == 1) moves.push_back('D');
    }
    
    // 计算每个R2何时被收集
    std::vector<int> collected_at(R, -1);
    for (size_t step = 0; step < path.size(); ++step) {
        int mask = compute_collection_mask(path[step].row, path[step].col, r2_list);
        for (int k = 0; k < R; ++k) {
            if (collected_at[k] == -1 && (mask & (1 << k))) {
                collected_at[k] = static_cast<int>(step);
            }
        }
    }
    
    result.success = true;
    result.path = path;
    result.moves = moves;
    result.exit_pos = GridPos(goal_state.row, goal_state.col);
    result.total_steps = static_cast<int>(path.size()) - 1;
    result.kfs_collected_at_step = collected_at;
    
    return result;
}

} // namespace r2_decision

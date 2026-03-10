#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>
#include <tuple>
#include <array>
#include <string>

namespace r2_decision {

// 梅花桩位置（行1-4，列1-3）
struct GridPos {
    int row;  // 1-4
    int col;  // 1-3
    
    GridPos(int r = 1, int c = 1) : row(r), col(c) {}
    bool operator==(const GridPos& other) const {
        return row == other.row && col == other.col;
    }
};

// 方块类型
enum class BlockType {
    EMPTY = -1,    // 空位
    R1 = 1,        // R1方块（对手机器人）
    R2 = 2,        // R2方块（需要收集的KFS）
    FAKE = 3       // 假方块（障碍物）
};

// 路径规划结果
struct PathPlanResult {
    bool success = false;                    // 是否找到路径
    std::vector<GridPos> path;               // 路径坐标序列
    std::vector<char> moves;                 // 移动指令序列 (W/A/S/D)
    GridPos exit_pos;                        // 出口位置
    int total_steps = 0;                     // 总步数
    std::vector<int> kfs_collected_at_step;  // 每个KFS在第几步被收集
    std::string reason;                      // 失败原因
};

// 梅花桩网格状态（4x3）
class MeiHuaGrid {
public:
    MeiHuaGrid();
    
    // 随机生成KFS分布
    // 规则：3个R1(不能放在中间两格), 4个R2, 1个FAKE(不能放在底部一行)
    void randomize_kfs();
    
    // 手动设置格子状态
    void set_block(int row, int col, BlockType type);
    BlockType get_block(int row, int col) const;
    
    // 获取高度（用于后续物理控制）
    int get_height(int row, int col) const;
    
    // 标记假方块为已知
    void mark_fake_known(int row, int col);
    bool is_fake_known(int row, int col) const;
    
    // 打印网格状态（调试用）
    void print_grid() const;
    
    // 获取所有R2位置
    std::vector<GridPos> get_r2_positions() const;
    
private:
    std::array<std::array<BlockType, 5>, 6> block_;  // 使用6x5以匹配原始索引
    std::array<std::array<int, 5>, 6> height_;       // 高度数据
    std::array<std::array<bool, 5>, 6> known_fake_;  // 已知的假方块
    
    bool is_forbidden_for_r1(int row, int col) const;
    bool is_forbidden_for_fake(int row, int col) const;
};

// BFS路径规划器
class PathPlanner {
public:
    explicit PathPlanner(const MeiHuaGrid& grid);
    
    // 从起点规划到收集所有R2并到达出口
    // start: 起始位置（默认1,2）
    PathPlanResult plan_path(const GridPos& start = GridPos(1, 2));
    
    // 设置是否允许使用某个出口
    void set_exit_allowed(const GridPos& exit, bool allowed);
    
private:
    const MeiHuaGrid& grid_;
    std::vector<GridPos> allowed_exits_;
    
    // BFS状态：(row, col, collected_mask)
    struct BFSState {
        int row;
        int col;
        int mask;  // 位掩码，表示已收集的R2
        int depth;
    };
    
    // 计算在某位置可以收集到的R2掩码
    int compute_collection_mask(int row, int col, const std::vector<GridPos>& r2_list) const;
    
    // 重建路径
    PathPlanResult reconstruct_path(
        const BFSState& goal_state,
        const std::vector<GridPos>& r2_list,
        const std::array<std::array<std::array<std::tuple<int,int,int>, 16>, 4>, 5>& prev_state,
        const std::array<std::array<std::array<char, 16>, 4>, 5>& prev_move) const;
};

} // namespace r2_decision

#endif // PATH_PLANNER_HPP

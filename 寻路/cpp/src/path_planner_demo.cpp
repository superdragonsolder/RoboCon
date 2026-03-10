#include "r2_decision_cpp/path_planner.hpp"
#include <iostream>
#include <locale>

using namespace r2_decision;

int main() {
    // 设置中文输出
    setlocale(LC_ALL, "");
    std::cout.imbue(std::locale(""));
    
    std::cout << "========================================\n";
    std::cout << "  R2 梅林路径规划算法演示程序\n";
    std::cout << "========================================\n\n";
    
    // 创建梅花桩网格
    MeiHuaGrid grid;
    
    std::cout << "1. 生成随机KFS分布...\n";
    grid.randomize_kfs();
    
    std::cout << "\n2. 梅花桩网格初始状态:\n";
    grid.print_grid();
    
    std::cout << "\n3. 检测到的R2方块位置:\n";
    auto r2_list = grid.get_r2_positions();
    for (size_t i = 0; i < r2_list.size(); ++i) {
        std::cout << "   R2 #" << i << ": (" << r2_list[i].row << "," 
                  << r2_list[i].col << ")" 
                  << " 高度: " << grid.get_height(r2_list[i].row, r2_list[i].col) << "mm\n";
    }
    
    // 创建路径规划器
    std::cout << "\n4. 初始化BFS路径规划器...\n";
    PathPlanner planner(grid);
    
    std::cout << "\n5. 开始路径规划 (起点: 1,2)...\n";
    GridPos start(1, 2);
    auto result = planner.plan_path(start);
    
    if (result.success) {
        std::cout << "\n✓ 路径规划成功!\n";
        std::cout << "-------------------------------------\n";
        std::cout << "总步数: " << result.total_steps << "\n";
        std::cout << "出口位置: (" << result.exit_pos.row << "," 
                  << result.exit_pos.col << ")\n";
        std::cout << "收集KFS数量: " << result.kfs_collected_at_step.size() << "\n";
        
        std::cout << "\n移动指令序列:\n   ";
        for (size_t i = 0; i < result.moves.size(); ++i) {
            char move = result.moves[i];
            switch (move) {
                case 'W': std::cout << "↑ "; break;
                case 'S': std::cout << "↓ "; break;
                case 'A': std::cout << "← "; break;
                case 'D': std::cout << "→ "; break;
                default: std::cout << "? "; break;
            }
            if ((i + 1) % 10 == 0) std::cout << "\n   ";
        }
        std::cout << "\n";
        
        std::cout << "\n路径坐标序列:\n";
        for (size_t i = 0; i < result.path.size(); ++i) {
            std::cout << "   步骤 " << i << ": (" 
                      << result.path[i].row << "," << result.path[i].col << ")";
            
            // 标记这一步收集了哪个KFS
            for (size_t k = 0; k < result.kfs_collected_at_step.size(); ++k) {
                if (result.kfs_collected_at_step[k] == static_cast<int>(i)) {
                    std::cout << " [收集 R2 #" << k << "]";
                }
            }
            std::cout << "\n";
        }
        
        std::cout << "\nKFS收集详情:\n";
        for (size_t k = 0; k < result.kfs_collected_at_step.size(); ++k) {
            std::cout << "   R2 #" << k << " at (" 
                      << r2_list[k].row << "," << r2_list[k].col 
                      << ") → 在步骤 " << result.kfs_collected_at_step[k] 
                      << " 被收集\n";
        }
        
        // 可视化路径
        std::cout << "\n6. 路径可视化 (逐步演示):\n";
        std::cout << "   图例: 我=当前位置  R2=目标  X=障碍  空=可走\n\n";
        
        for (size_t step = 0; step < result.path.size(); ++step) {
            std::cout << "   步骤 " << step << ":\n";
            for (int row = 4; row >= 1; --row) {
                std::cout << "     ";
                for (int col = 1; col <= 3; ++col) {
                    if (result.path[step].row == row && result.path[step].col == col) {
                        std::cout << "我 ";
                    } else {
                        auto block = grid.get_block(row, col);
                        switch (block) {
                            case BlockType::R1:
                                std::cout << "R1 ";
                                break;
                            case BlockType::R2:
                                std::cout << "R2 ";
                                break;
                            case BlockType::FAKE:
                                std::cout << "X  ";
                                break;
                            default:
                                std::cout << "·  ";
                                break;
                        }
                    }
                }
                std::cout << "\n";
            }
            
            // 显示移动指令
            if (step < result.moves.size()) {
                std::string move_desc;
                switch (result.moves[step]) {
                    case 'W': move_desc = "向上 ↑"; break;
                    case 'S': move_desc = "向下 ↓"; break;
                    case 'A': move_desc = "向左 ←"; break;
                    case 'D': move_desc = "向右 →"; break;
                }
                std::cout << "     下一步: " << move_desc << "\n";
            } else {
                std::cout << "     [到达目标]\n";
            }
            std::cout << "\n";
        }
        
    } else {
        std::cout << "\n✗ 路径规划失败!\n";
        std::cout << "原因: " << result.reason << "\n";
    }
    
    std::cout << "\n========================================\n";
    std::cout << "  演示完成\n";
    std::cout << "========================================\n";
    
    return 0;
}

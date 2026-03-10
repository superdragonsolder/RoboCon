#include "r2_decision_cpp/strategy.hpp"

#include <array>

namespace r2 {

int choose_best_cell(const BoardState& board, bool can_top) {
    static const std::array<std::array<int, 3>, 8> lines = {{
        {1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {1, 4, 7},
        {2, 5, 8}, {3, 6, 9}, {1, 5, 9}, {3, 5, 7},
    }};

    std::vector<int> empties;
    for (int i = 1; i <= 9; ++i) {
        if (!board.mine.count(i) && !board.enemy.count(i)) {
            empties.push_back(i);
        }
    }
    if (empties.empty()) {
        return -1;
    }

    for (const auto& line : lines) {
        int enemy_count = 0;
        std::vector<int> empty_cells;
        for (int c : line) {
            if (board.enemy.count(c)) {
                enemy_count++;
            } else if (!board.mine.count(c) && !board.enemy.count(c)) {
                empty_cells.push_back(c);
            }
        }
        if (enemy_count == 2 && empty_cells.size() == 1) {
            return empty_cells.front();
        }
    }

    for (const auto& line : lines) {
        int mine_count = 0;
        std::vector<int> empty_cells;
        for (int c : line) {
            if (board.mine.count(c)) {
                mine_count++;
            } else if (!board.mine.count(c) && !board.enemy.count(c)) {
                empty_cells.push_back(c);
            }
        }
        if (mine_count == 2 && empty_cells.size() == 1) {
            return empty_cells.front();
        }
    }

    const std::vector<int> priority = can_top
        ? std::vector<int>{5, 2, 8, 4, 6, 1, 3, 7, 9}
        : std::vector<int>{5, 4, 6, 2, 8, 1, 3, 7, 9};

    for (int p : priority) {
        for (int e : empties) {
            if (p == e) {
                return p;
            }
        }
    }
    return empties.front();
}

}  // namespace r2

#pragma once

#include <unordered_set>
#include <vector>

namespace r2 {

struct BoardState {
    std::unordered_set<int> mine;
    std::unordered_set<int> enemy;
};

int choose_best_cell(const BoardState& board, bool can_top);

}  // namespace r2

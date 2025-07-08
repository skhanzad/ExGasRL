#include "puzzle_state.h"
#include <sstream>
#include <algorithm>
#include <random>
#include <cmath>

namespace heuristic_search {

PuzzleState::PuzzleState(const Board& board)
    : board_(board), size_(static_cast<int>(std::sqrt(board.size()))) {
}

PuzzleState::PuzzleState(const Board& board, int size)
    : board_(board), size_(size) {
}

PuzzleState::Position PuzzleState::getEmptyPosition() const {
    for (int i = 0; i < static_cast<int>(board_.size()); ++i) {
        if (board_[i] == 0) {
            return {i % size_, i / size_};
        }
    }
    return {0, 0}; // Should not happen
}

bool PuzzleState::isSolved() const {
    for (int i = 0; i < static_cast<int>(board_.size()) - 1; ++i) {
        if (board_[i] != i + 1) {
            return false;
        }
    }
    return board_.back() == 0;
}

int PuzzleState::getTile(const Position& pos) const {
    if (isValidPosition(pos)) {
        return board_[pos.second * size_ + pos.first];
    }
    return -1;
}

void PuzzleState::setTile(const Position& pos, int value) {
    if (isValidPosition(pos)) {
        board_[pos.second * size_ + pos.first] = value;
    }
}

bool PuzzleState::isValidPosition(const Position& pos) const {
    return pos.first >= 0 && pos.first < size_ && 
           pos.second >= 0 && pos.second < size_;
}

std::vector<std::string> PuzzleState::getValidMoves() const {
    std::vector<std::string> moves;
    Position empty = getEmptyPosition();
    
    // Check all four directions
    std::vector<Position> directions = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
    std::vector<std::string> move_names = {"UP", "DOWN", "LEFT", "RIGHT"};
    
    for (size_t i = 0; i < directions.size(); ++i) {
        Position new_pos = {empty.first + directions[i].first, 
                           empty.second + directions[i].second};
        
        if (isValidPosition(new_pos)) {
            moves.push_back(move_names[i]);
        }
    }
    
    return moves;
}

PuzzleState PuzzleState::applyMove(const std::string& move) const {
    PuzzleState new_state = *this;
    Position empty = getEmptyPosition();
    
    Position offset = getMoveDirection(move);
    Position new_empty = {empty.first + offset.first, empty.second + offset.second};
    
    if (isValidPosition(new_empty)) {
        // Swap tiles
        int temp = new_state.getTile(new_empty);
        new_state.setTile(new_empty, 0);
        new_state.setTile(empty, temp);
    }
    
    return new_state;
}

std::string PuzzleState::toString() const {
    std::ostringstream oss;
    oss << size_ << ":";
    for (int tile : board_) {
        oss << tile << ",";
    }
    return oss.str();
}

PuzzleState PuzzleState::fromString(const std::string& str) {
    std::istringstream iss(str);
    int size;
    char colon;
    
    if (iss >> size >> colon) {
        Board board;
        int tile;
        char comma;
        
        while (iss >> tile >> comma) {
            board.push_back(tile);
        }
        
        return PuzzleState(board, size);
    }
    
    return PuzzleState({}, 3); // Default fallback
}

PuzzleState PuzzleState::createSolvedState(int size) {
    Board board(size * size);
    for (int i = 0; i < static_cast<int>(board.size()) - 1; ++i) {
        board[i] = i + 1;
    }
    board.back() = 0; // Empty tile at the end
    
    return PuzzleState(board, size);
}

PuzzleState PuzzleState::createRandomState(int size, int num_moves) {
    PuzzleState state = createSolvedState(size);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (int i = 0; i < num_moves; ++i) {
        auto valid_moves = state.getValidMoves();
        if (valid_moves.empty()) break;
        
        std::uniform_int_distribution<> dis(0, static_cast<int>(valid_moves.size()) - 1);
        int move_index = dis(gen);
        
        state = state.applyMove(valid_moves[move_index]);
    }
    
    return state;
}

bool PuzzleState::isSolvable() const {
    int inversions = getInversionCount();
    int empty_row = getEmptyRow();
    
    if (size_ % 2 == 1) {
        // Odd-sized puzzle: solvable if inversions are even
        return inversions % 2 == 0;
    } else {
        // Even-sized puzzle: solvable if (inversions + empty_row) is even
        return (inversions + empty_row) % 2 == 0;
    }
}

SearchAlgorithm::SuccessorFunction PuzzleState::getSuccessorFunction() {
    return [](const std::string& state_str) -> std::vector<std::pair<std::string, Node::CostType>> {
        PuzzleState state = PuzzleState::fromString(state_str);
        std::vector<std::pair<std::string, Node::CostType>> successors;
        
        auto valid_moves = state.getValidMoves();
        for (const auto& move : valid_moves) {
            PuzzleState successor = state.applyMove(move);
            successors.emplace_back(successor.toString(), 1.0);
        }
        
        return successors;
    };
}

SearchAlgorithm::HeuristicFunction PuzzleState::getManhattanHeuristic() {
    return [](const std::string& state_str, const std::string& goal_str) -> Node::CostType {
        PuzzleState state = PuzzleState::fromString(state_str);
        PuzzleState goal = PuzzleState::fromString(goal_str);
        
        return state.manhattanDistance(goal);
    };
}

SearchAlgorithm::HeuristicFunction PuzzleState::getMisplacedTilesHeuristic() {
    return [](const std::string& state_str, const std::string& goal_str) -> Node::CostType {
        PuzzleState state = PuzzleState::fromString(state_str);
        PuzzleState goal = PuzzleState::fromString(goal_str);
        
        return state.misplacedTiles(goal);
    };
}

SearchAlgorithm::HeuristicFunction PuzzleState::getLinearConflictHeuristic() {
    return [](const std::string& state_str, const std::string& goal_str) -> Node::CostType {
        PuzzleState state = PuzzleState::fromString(state_str);
        PuzzleState goal = PuzzleState::fromString(goal_str);
        
        return state.linearConflicts(goal);
    };
}

SearchAlgorithm::HeuristicFunction PuzzleState::getPatternDatabaseHeuristic() {
    // Simplified pattern database heuristic
    return [](const std::string& state_str, const std::string& goal_str) -> Node::CostType {
        PuzzleState state = PuzzleState::fromString(state_str);
        PuzzleState goal = PuzzleState::fromString(goal_str);
        
        // For now, just use Manhattan distance
        // A full pattern database would require precomputing lookup tables
        return state.manhattanDistance(goal);
    };
}

double PuzzleState::manhattanDistance(const PuzzleState& goal) const {
    double total_distance = 0.0;
    
    for (int tile = 1; tile < static_cast<int>(board_.size()); ++tile) {
        Position current_pos = getTilePosition(tile);
        Position goal_pos = goal.getTilePosition(tile);
        
        total_distance += std::abs(current_pos.first - goal_pos.first) + 
                         std::abs(current_pos.second - goal_pos.second);
    }
    
    return total_distance;
}

int PuzzleState::misplacedTiles(const PuzzleState& goal) const {
    int count = 0;
    
    for (int i = 0; i < static_cast<int>(board_.size()); ++i) {
        if (board_[i] != goal.board_[i]) {
            ++count;
        }
    }
    
    return count;
}

double PuzzleState::linearConflicts(const PuzzleState& goal) const {
    double conflicts = 0.0;
    
    // Check row conflicts
    for (int row = 0; row < size_; ++row) {
        conflicts += getRowConflicts(row, goal);
    }
    
    // Check column conflicts
    for (int col = 0; col < size_; ++col) {
        conflicts += getColumnConflicts(col, goal);
    }
    
    return manhattanDistance(goal) + 2 * conflicts;
}

int PuzzleState::getInversionCount() const {
    int inversions = 0;
    
    for (int i = 0; i < static_cast<int>(board_.size()); ++i) {
        if (board_[i] == 0) continue; // Skip empty tile
        
        for (int j = i + 1; j < static_cast<int>(board_.size()); ++j) {
            if (board_[j] == 0) continue; // Skip empty tile
            
            if (board_[i] > board_[j]) {
                ++inversions;
            }
        }
    }
    
    return inversions;
}

int PuzzleState::getEmptyRow() const {
    Position empty = getEmptyPosition();
    return size_ - 1 - empty.second; // Count from bottom
}

PuzzleState::Position PuzzleState::getTilePosition(int tile) const {
    for (int i = 0; i < static_cast<int>(board_.size()); ++i) {
        if (board_[i] == tile) {
            return {i % size_, i / size_};
        }
    }
    return {0, 0}; // Should not happen
}

PuzzleState::Position PuzzleState::getGoalPosition(int tile) const {
    if (tile == 0) {
        return {size_ - 1, size_ - 1}; // Empty tile at bottom-right
    }
    
    int pos = tile - 1; // 1-based to 0-based
    return {pos % size_, pos / size_};
}

bool PuzzleState::isValidMove(const std::string& move) const {
    auto valid_moves = getValidMoves();
    return std::find(valid_moves.begin(), valid_moves.end(), move) != valid_moves.end();
}

PuzzleState::Position PuzzleState::getMoveDirection(const std::string& move) const {
    if (move == "UP") return {0, -1};
    if (move == "DOWN") return {0, 1};
    if (move == "LEFT") return {-1, 0};
    if (move == "RIGHT") return {1, 0};
    return {0, 0};
}

double PuzzleState::getRowConflicts(int row, const PuzzleState& goal) const {
    double conflicts = 0.0;
    
    for (int col1 = 0; col1 < size_; ++col1) {
        for (int col2 = col1 + 1; col2 < size_; ++col2) {
            int tile1 = getTile({col1, row});
            int tile2 = getTile({col2, row});
            
            if (tile1 == 0 || tile2 == 0) continue; // Skip empty tile
            
            // Check if both tiles are in the same row in goal state
            Position goal_pos1 = goal.getTilePosition(tile1);
            Position goal_pos2 = goal.getTilePosition(tile2);
            
            if (goal_pos1.second == row && goal_pos2.second == row) {
                // Check if they conflict (tile1 should be to the left of tile2)
                if (goal_pos1.first < goal_pos2.first && col1 > col2) {
                    conflicts += 1.0;
                }
            }
        }
    }
    
    return conflicts;
}

double PuzzleState::getColumnConflicts(int col, const PuzzleState& goal) const {
    double conflicts = 0.0;
    
    for (int row1 = 0; row1 < size_; ++row1) {
        for (int row2 = row1 + 1; row2 < size_; ++row2) {
            int tile1 = getTile({col, row1});
            int tile2 = getTile({col, row2});
            
            if (tile1 == 0 || tile2 == 0) continue; // Skip empty tile
            
            // Check if both tiles are in the same column in goal state
            Position goal_pos1 = goal.getTilePosition(tile1);
            Position goal_pos2 = goal.getTilePosition(tile2);
            
            if (goal_pos1.first == col && goal_pos2.first == col) {
                // Check if they conflict (tile1 should be above tile2)
                if (goal_pos1.second < goal_pos2.second && row1 > row2) {
                    conflicts += 1.0;
                }
            }
        }
    }
    
    return conflicts;
}

} // namespace heuristic_search 
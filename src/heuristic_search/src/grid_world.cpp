#include "grid_world.h"
#include <sstream>
#include <fstream>
#include <random>
#include <iostream>

namespace heuristic_search {

GridWorld::GridWorld(int width, int height)
    : width_(width), height_(height), eight_directional_(false) {
    
    // Initialize grid
    grid_.resize(height, std::vector<int>(width, EMPTY));
    
    // Initialize movement costs
    movement_costs_.resize(4);
    movement_costs_[EMPTY] = 1.0;
    movement_costs_[WALL] = std::numeric_limits<double>::infinity();
    movement_costs_[WATER] = 2.0;
    movement_costs_[MOUNTAIN] = 3.0;
}

void GridWorld::setCell(int x, int y, CellType type) {
    if (isValidPosition(x, y)) {
        grid_[y][x] = static_cast<int>(type);
    }
}

GridWorld::CellType GridWorld::getCell(int x, int y) const {
    if (isValidPosition(x, y)) {
        return static_cast<CellType>(grid_[y][x]);
    }
    return WALL;
}

void GridWorld::setMovementCost(CellType type, double cost) {
    if (static_cast<size_t>(type) < movement_costs_.size()) {
        movement_costs_[static_cast<size_t>(type)] = cost;
    }
}

double GridWorld::getMovementCost(CellType type) const {
    if (static_cast<size_t>(type) < movement_costs_.size()) {
        return movement_costs_[static_cast<size_t>(type)];
    }
    return std::numeric_limits<double>::infinity();
}

bool GridWorld::isValidPosition(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridWorld::isWalkable(int x, int y) const {
    if (!isValidPosition(x, y)) {
        return false;
    }
    
    CellType cell = getCell(x, y);
    return cell != WALL && getMovementCost(cell) < std::numeric_limits<double>::infinity();
}

SearchAlgorithm::SuccessorFunction GridWorld::getSuccessorFunction() const {
    return [this](const std::string& state) -> std::vector<std::pair<std::string, Node::CostType>> {
        Position pos = stateToPosition(state);
        std::vector<std::pair<std::string, Node::CostType>> successors;
        
        auto neighbors = getNeighbors(pos);
        for (const auto& neighbor : neighbors) {
            double cost = getMovementCost(pos, neighbor);
            if (cost < std::numeric_limits<double>::infinity()) {
                successors.emplace_back(positionToState(neighbor), cost);
            }
        }
        
        return successors;
    };
}

SearchAlgorithm::HeuristicFunction GridWorld::getManhattanHeuristic() const {
    return [this](const std::string& state, const std::string& goal) -> Node::CostType {
        Position state_pos = stateToPosition(state);
        Position goal_pos = stateToPosition(goal);
        
        return std::abs(state_pos.first - goal_pos.first) + 
               std::abs(state_pos.second - goal_pos.second);
    };
}

SearchAlgorithm::HeuristicFunction GridWorld::getEuclideanHeuristic() const {
    return [this](const std::string& state, const std::string& goal) -> Node::CostType {
        Position state_pos = stateToPosition(state);
        Position goal_pos = stateToPosition(goal);
        
        double dx = state_pos.first - goal_pos.first;
        double dy = state_pos.second - goal_pos.second;
        
        return std::sqrt(dx * dx + dy * dy);
    };
}

std::string GridWorld::positionToState(const Position& pos) const {
    std::ostringstream oss;
    oss << pos.first << "," << pos.second;
    return oss.str();
}

GridWorld::Position GridWorld::stateToPosition(const std::string& state) const {
    std::istringstream iss(state);
    int x, y;
    char comma;
    
    if (iss >> x >> comma >> y) {
        return {x, y};
    }
    
    return {0, 0}; // Default fallback
}

std::vector<GridWorld::Position> GridWorld::getNeighbors(const Position& pos) const {
    std::vector<Position> neighbors;
    auto directions = getDirections();
    
    for (Direction dir : directions) {
        Position neighbor = moveInDirection(pos, dir);
        if (isWalkable(neighbor.first, neighbor.second)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

double GridWorld::getMovementCost(const Position& from, const Position& to) const {
    // Cost is based on the destination cell
    return getMovementCost(getCell(to.first, to.second));
}

bool GridWorld::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file >> width_ >> height_;
    grid_.resize(height_, std::vector<int>(width_));
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int cell_value;
            file >> cell_value;
            grid_[y][x] = cell_value;
        }
    }
    
    return true;
}

bool GridWorld::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    file << width_ << " " << height_ << std::endl;
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            file << grid_[y][x];
            if (x < width_ - 1) file << " ";
        }
        file << std::endl;
    }
    
    return true;
}

void GridWorld::createRandomGrid(double obstacle_ratio) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            double rand_val = dis(gen);
            
            if (rand_val < obstacle_ratio) {
                // Randomly choose obstacle type
                double obstacle_type = dis(gen);
                if (obstacle_type < 0.7) {
                    setCell(x, y, WALL);
                } else if (obstacle_type < 0.85) {
                    setCell(x, y, WATER);
                } else {
                    setCell(x, y, MOUNTAIN);
                }
            } else {
                setCell(x, y, EMPTY);
            }
        }
    }
}

void GridWorld::printGrid() const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            CellType cell = getCell(x, y);
            switch (cell) {
                case EMPTY: std::cout << ". "; break;
                case WALL: std::cout << "# "; break;
                case WATER: std::cout << "~ "; break;
                case MOUNTAIN: std::cout << "^ "; break;
                default: std::cout << "? "; break;
            }
        }
        std::cout << std::endl;
    }
}

std::vector<GridWorld::Direction> GridWorld::getDirections() const {
    if (eight_directional_) {
        return {UP, DOWN, LEFT, RIGHT, UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT};
    } else {
        return {UP, DOWN, LEFT, RIGHT};
    }
}

GridWorld::Position GridWorld::moveInDirection(const Position& pos, Direction dir) const {
    switch (dir) {
        case UP: return {pos.first, pos.second - 1};
        case DOWN: return {pos.first, pos.second + 1};
        case LEFT: return {pos.first - 1, pos.second};
        case RIGHT: return {pos.first + 1, pos.second};
        case UP_LEFT: return {pos.first - 1, pos.second - 1};
        case UP_RIGHT: return {pos.first + 1, pos.second - 1};
        case DOWN_LEFT: return {pos.first - 1, pos.second + 1};
        case DOWN_RIGHT: return {pos.first + 1, pos.second + 1};
        default: return pos;
    }
}

std::string GridWorld::getDirectionName(Direction dir) const {
    switch (dir) {
        case UP: return "UP";
        case DOWN: return "DOWN";
        case LEFT: return "LEFT";
        case RIGHT: return "RIGHT";
        case UP_LEFT: return "UP_LEFT";
        case UP_RIGHT: return "UP_RIGHT";
        case DOWN_LEFT: return "DOWN_LEFT";
        case DOWN_RIGHT: return "DOWN_RIGHT";
        default: return "UNKNOWN";
    }
}

} // namespace heuristic_search 
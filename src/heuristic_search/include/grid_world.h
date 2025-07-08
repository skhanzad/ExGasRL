#pragma once

#include "search_algorithm.h"
#include <vector>
#include <string>
#include <utility>

namespace heuristic_search {

/**
 * @brief GridWorld problem domain for pathfinding
 * 
 * This class provides a grid-based environment where agents can move
 * between cells. It includes obstacles, different movement costs,
 * and various movement patterns (4-directional, 8-directional).
 */
class GridWorld {
public:
    using Position = std::pair<int, int>;
    using Grid = std::vector<std::vector<int>>;
    
    /**
     * @brief Movement directions
     */
    enum Direction {
        UP, DOWN, LEFT, RIGHT,
        UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT
    };
    
    /**
     * @brief Cell types
     */
    enum CellType {
        EMPTY = 0,
        WALL = 1,
        WATER = 2,
        MOUNTAIN = 3
    };
    
    /**
     * @brief Constructor
     * @param width Grid width
     * @param height Grid height
     */
    GridWorld(int width, int height);
    
    /**
     * @brief Destructor
     */
    ~GridWorld() = default;
    
    /**
     * @brief Set cell type
     * @param x X coordinate
     * @param y Y coordinate
     * @param type Cell type
     */
    void setCell(int x, int y, CellType type);
    
    /**
     * @brief Get cell type
     * @param x X coordinate
     * @param y Y coordinate
     * @return Cell type
     */
    CellType getCell(int x, int y) const;
    
    /**
     * @brief Set movement cost for cell type
     * @param type Cell type
     * @param cost Movement cost
     */
    void setMovementCost(CellType type, double cost);
    
    /**
     * @brief Get movement cost for cell type
     * @param type Cell type
     * @return Movement cost
     */
    double getMovementCost(CellType type) const;
    
    /**
     * @brief Set movement pattern
     * @param eight_directional True for 8-directional, false for 4-directional
     */
    void setEightDirectional(bool eight_directional) { eight_directional_ = eight_directional; }
    
    /**
     * @brief Get movement pattern
     * @return True if 8-directional
     */
    bool isEightDirectional() const { return eight_directional_; }
    
    /**
     * @brief Check if position is valid
     * @param x X coordinate
     * @param y Y coordinate
     * @return True if valid
     */
    bool isValidPosition(int x, int y) const;
    
    /**
     * @brief Check if position is walkable
     * @param x X coordinate
     * @param y Y coordinate
     * @return True if walkable
     */
    bool isWalkable(int x, int y) const;
    
    /**
     * @brief Get grid width
     * @return Width
     */
    int getWidth() const { return width_; }
    
    /**
     * @brief Get grid height
     * @return Height
     */
    int getHeight() const { return height_; }
    
    /**
     * @brief Get successor function for this grid world
     * @return Successor function
     */
    SearchAlgorithm::SuccessorFunction getSuccessorFunction() const;
    
    /**
     * @brief Get Manhattan distance heuristic for this grid world
     * @return Heuristic function
     */
    SearchAlgorithm::HeuristicFunction getManhattanHeuristic() const;
    
    /**
     * @brief Get Euclidean distance heuristic for this grid world
     * @return Heuristic function
     */
    SearchAlgorithm::HeuristicFunction getEuclideanHeuristic() const;
    
    /**
     * @brief Convert position to state string
     * @param pos Position
     * @return State string
     */
    std::string positionToState(const Position& pos) const;
    
    /**
     * @brief Convert state string to position
     * @param state State string
     * @return Position
     */
    Position stateToPosition(const std::string& state) const;
    
    /**
     * @brief Get all valid neighbors of a position
     * @param pos Position
     * @return Vector of neighbor positions
     */
    std::vector<Position> getNeighbors(const Position& pos) const;
    
    /**
     * @brief Get movement cost between two adjacent positions
     * @param from Source position
     * @param to Destination position
     * @return Movement cost
     */
    double getMovementCost(const Position& from, const Position& to) const;
    
    /**
     * @brief Load grid from file
     * @param filename File path
     * @return True if successful
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief Save grid to file
     * @param filename File path
     * @return True if successful
     */
    bool saveToFile(const std::string& filename) const;
    
    /**
     * @brief Create random grid with obstacles
     * @param obstacle_ratio Ratio of obstacles (0.0 to 1.0)
     */
    void createRandomGrid(double obstacle_ratio);
    
    /**
     * @brief Print grid to console
     */
    void printGrid() const;

private:
    int width_;
    int height_;
    Grid grid_;
    std::vector<double> movement_costs_;
    bool eight_directional_;
    
    /**
     * @brief Get movement directions
     * @return Vector of directions
     */
    std::vector<Direction> getDirections() const;
    
    /**
     * @brief Get position after moving in direction
     * @param pos Current position
     * @param dir Direction
     * @return New position
     */
    Position moveInDirection(const Position& pos, Direction dir) const;
    
    /**
     * @brief Get direction name
     * @param dir Direction
     * @return Direction name
     */
    std::string getDirectionName(Direction dir) const;
};

} // namespace heuristic_search 
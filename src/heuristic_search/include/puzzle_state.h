#pragma once

#include "search_algorithm.h"
#include <vector>
#include <string>
#include <array>

namespace heuristic_search {

/**
 * @brief PuzzleState for sliding puzzle problems (e.g., 8-puzzle, 15-puzzle)
 * 
 * This class provides a state representation for sliding puzzle problems
 * where tiles can be moved into an empty space. It includes various
 * heuristic functions and successor generation.
 */
class PuzzleState {
public:
    using Board = std::vector<int>;
    using Position = std::pair<int, int>;
    
    /**
     * @brief Constructor for 8-puzzle (3x3)
     * @param board Initial board configuration
     */
    explicit PuzzleState(const Board& board);
    
    /**
     * @brief Constructor for n-puzzle
     * @param board Initial board configuration
     * @param size Board size (e.g., 3 for 3x3, 4 for 4x4)
     */
    PuzzleState(const Board& board, int size);
    
    /**
     * @brief Destructor
     */
    ~PuzzleState() = default;
    
    /**
     * @brief Get board configuration
     * @return Board as vector
     */
    const Board& getBoard() const { return board_; }
    
    /**
     * @brief Get board size
     * @return Board size (e.g., 3 for 3x3)
     */
    int getSize() const { return size_; }
    
    /**
     * @brief Get empty tile position
     * @return Position of empty tile (0)
     */
    Position getEmptyPosition() const;
    
    /**
     * @brief Check if puzzle is solved
     * @return True if solved
     */
    bool isSolved() const;
    
    /**
     * @brief Get tile at position
     * @param pos Position
     * @return Tile value
     */
    int getTile(const Position& pos) const;
    
    /**
     * @brief Set tile at position
     * @param pos Position
     * @param value Tile value
     */
    void setTile(const Position& pos, int value);
    
    /**
     * @brief Check if position is valid
     * @param pos Position
     * @return True if valid
     */
    bool isValidPosition(const Position& pos) const;
    
    /**
     * @brief Get valid moves from current state
     * @return Vector of valid moves
     */
    std::vector<std::string> getValidMoves() const;
    
    /**
     * @brief Apply move to state
     * @param move Move string
     * @return New state after move
     */
    PuzzleState applyMove(const std::string& move) const;
    
    /**
     * @brief Convert state to string representation
     * @return String representation
     */
    std::string toString() const;
    
    /**
     * @brief Convert string to state
     * @param str String representation
     * @return PuzzleState
     */
    static PuzzleState fromString(const std::string& str);
    
    /**
     * @brief Create solved state
     * @param size Board size
     * @return Solved puzzle state
     */
    static PuzzleState createSolvedState(int size);
    
    /**
     * @brief Create random solvable state
     * @param size Board size
     * @param num_moves Number of random moves to make
     * @return Random solvable state
     */
    static PuzzleState createRandomState(int size, int num_moves = 100);
    
    /**
     * @brief Check if state is solvable
     * @return True if solvable
     */
    bool isSolvable() const;
    
    /**
     * @brief Get successor function for puzzle
     * @return Successor function
     */
    static SearchAlgorithm::SuccessorFunction getSuccessorFunction();
    
    /**
     * @brief Get Manhattan distance heuristic for puzzle
     * @return Heuristic function
     */
    static SearchAlgorithm::HeuristicFunction getManhattanHeuristic();
    
    /**
     * @brief Get Misplaced tiles heuristic for puzzle
     * @return Heuristic function
     */
    static SearchAlgorithm::HeuristicFunction getMisplacedTilesHeuristic();
    
    /**
     * @brief Get Linear conflict heuristic for puzzle
     * @return Heuristic function
     */
    static SearchAlgorithm::HeuristicFunction getLinearConflictHeuristic();
    
    /**
     * @brief Get Pattern database heuristic for puzzle
     * @return Heuristic function
     */
    static SearchAlgorithm::HeuristicFunction getPatternDatabaseHeuristic();
    
    /**
     * @brief Calculate Manhattan distance to goal
     * @param goal Goal state
     * @return Manhattan distance
     */
    double manhattanDistance(const PuzzleState& goal) const;
    
    /**
     * @brief Calculate number of misplaced tiles
     * @param goal Goal state
     * @return Number of misplaced tiles
     */
    int misplacedTiles(const PuzzleState& goal) const;
    
    /**
     * @brief Calculate linear conflicts
     * @param goal Goal state
     * @return Linear conflict cost
     */
    double linearConflicts(const PuzzleState& goal) const;
    
    /**
     * @brief Get inversion count (for solvability check)
     * @return Number of inversions
     */
    int getInversionCount() const;
    
    /**
     * @brief Get row of empty tile (for solvability check)
     * @return Row of empty tile (0-indexed from bottom)
     */
    int getEmptyRow() const;

private:
    Board board_;
    int size_;
    
    /**
     * @brief Get position of tile
     * @param tile Tile value
     * @return Position of tile
     */
    Position getTilePosition(int tile) const;
    
    /**
     * @brief Get goal position of tile
     * @param tile Tile value
     * @return Goal position of tile
     */
    Position getGoalPosition(int tile) const;
    
    /**
     * @brief Check if move is valid
     * @param move Move string
     * @return True if valid
     */
    bool isValidMove(const std::string& move) const;
    
    /**
     * @brief Get move direction
     * @param move Move string
     * @return Direction as position offset
     */
    Position getMoveDirection(const std::string& move) const;
    
    /**
     * @brief Calculate linear conflicts in a row
     * @param row Row index
     * @param goal Goal state
     * @return Linear conflict cost for row
     */
    double getRowConflicts(int row, const PuzzleState& goal) const;
    
    /**
     * @brief Calculate linear conflicts in a column
     * @param col Column index
     * @param goal Goal state
     * @return Linear conflict cost for column
     */
    double getColumnConflicts(int col, const PuzzleState& goal) const;
};

} // namespace heuristic_search 
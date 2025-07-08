#include <iostream>
#include <cassert>
#include <cmath>
#include "astar.h"
#include "dijkstra.h"
#include "best_first.h"
#include "ida_star.h"
#include "grid_world.h"
#include "puzzle_state.h"

using namespace heuristic_search;

void testNode() {
    std::cout << "Testing Node class..." << std::endl;
    
    // Test basic node creation
    auto node1 = std::make_shared<Node>("test_state", 5.0, 3.0);
    assert(node1->getState() == "test_state");
    assert(node1->getGCost() == 5.0);
    assert(node1->getHCost() == 3.0);
    assert(node1->getFCost() == 8.0);
    
    // Test node equality
    auto node2 = std::make_shared<Node>("test_state", 1.0, 2.0);
    assert(node1->equals(*node2));
    
    // Test path reconstruction
    auto parent = std::make_shared<Node>("parent_state", 2.0, 1.0);
    node1->setParent(parent);
    node1->setAction("test_action");
    
    auto path = node1->getPath();
    assert(path.size() == 1);
    assert(path[0] == "test_action");
    
    std::cout << "Node tests passed!" << std::endl;
}

void testGridWorld() {
    std::cout << "Testing GridWorld..." << std::endl;
    
    GridWorld world(5, 5);
    
    // Test basic functionality
    assert(world.getWidth() == 5);
    assert(world.getHeight() == 5);
    assert(world.isValidPosition(0, 0));
    assert(world.isValidPosition(4, 4));
    assert(!world.isValidPosition(5, 5));
    
    // Test cell operations
    world.setCell(1, 1, GridWorld::WALL);
    assert(world.getCell(1, 1) == GridWorld::WALL);
    assert(!world.isWalkable(1, 1));
    
    // Test movement costs
    world.setMovementCost(GridWorld::WATER, 5.0);
    assert(world.getMovementCost(GridWorld::WATER) == 5.0);
    
    // Test state conversion
    auto pos = world.stateToPosition("2,3");
    assert(pos.first == 2 && pos.second == 3);
    
    auto state = world.positionToState({3, 2});
    assert(state == "3,2");
    
    // Test neighbors
    auto neighbors = world.getNeighbors({2, 2});
    assert(neighbors.size() == 4); // 4-directional by default
    
    world.setEightDirectional(true);
    neighbors = world.getNeighbors({2, 2});
    assert(neighbors.size() == 8); // 8-directional
    
    std::cout << "GridWorld tests passed!" << std::endl;
}

void testPuzzleState() {
    std::cout << "Testing PuzzleState..." << std::endl;
    
    // Test 8-puzzle creation
    std::vector<int> board = {1, 2, 3, 4, 0, 6, 7, 5, 8};
    PuzzleState puzzle(board, 3);
    
    assert(puzzle.getSize() == 3);
    assert(puzzle.getTile({1, 1}) == 0); // Empty tile
    assert(puzzle.getTile({0, 0}) == 1);
    
    // Test empty position
    auto empty_pos = puzzle.getEmptyPosition();
    assert(empty_pos.first == 1 && empty_pos.second == 1);
    
    // Test valid moves
    auto moves = puzzle.getValidMoves();
    assert(moves.size() == 4); // Should have 4 valid moves
    
    // Test move application
    auto new_state = puzzle.applyMove("UP");
    assert(new_state.getTile({1, 0}) == 0); // Empty moved up
    assert(new_state.getTile({1, 1}) == 4); // Tile moved down
    
    // Test solved state
    auto solved = PuzzleState::createSolvedState(3);
    assert(solved.isSolved());
    
    // Test solvability
    assert(puzzle.isSolvable());
    
    // Test string conversion
    auto str = puzzle.toString();
    auto reconstructed = PuzzleState::fromString(str);
    assert(reconstructed.getBoard() == puzzle.getBoard());
    
    std::cout << "PuzzleState tests passed!" << std::endl;
}

void testAStar() {
    std::cout << "Testing A* algorithm..." << std::endl;
    
    // Create a simple grid world
    GridWorld world(3, 3);
    world.setCell(1, 1, GridWorld::WALL); // Add obstacle
    
    auto successor_func = world.getSuccessorFunction();
    auto heuristic = world.getManhattanHeuristic();
    
    AStar astar(heuristic, successor_func);
    
    // Test pathfinding
    auto result = astar.search("0,0", "2,2");
    assert(result.success);
    assert(result.path_cost > 0);
    assert(result.nodes_expanded > 0);
    assert(result.nodes_generated > 0);
    
    // Test tie-breaking
    astar.setTieBreaking(true);
    auto result2 = astar.search("0,0", "2,2");
    assert(result2.success);
    
    // Test weighted A*
    astar.setWeight(1.5);
    auto result3 = astar.search("0,0", "2,2");
    assert(result3.success);
    
    std::cout << "A* tests passed!" << std::endl;
}

void testDijkstra() {
    std::cout << "Testing Dijkstra's algorithm..." << std::endl;
    
    GridWorld world(3, 3);
    auto successor_func = world.getSuccessorFunction();
    
    Dijkstra dijkstra(successor_func);
    
    // Test pathfinding
    auto result = dijkstra.search("0,0", "2,2");
    assert(result.success);
    assert(result.path_cost > 0);
    
    // Test finding all shortest paths
    auto all_paths = dijkstra.findAllShortestPaths("0,0");
    assert(all_paths.find("2,2") != all_paths.end());
    assert(all_paths["0,0"] == 0.0);
    
    // Test shortest path tree
    auto path_tree = dijkstra.getShortestPathTree("0,0");
    assert(path_tree.find("2,2") != path_tree.end());
    
    std::cout << "Dijkstra tests passed!" << std::endl;
}

void testBestFirst() {
    std::cout << "Testing Best-First Search..." << std::endl;
    
    GridWorld world(3, 3);
    auto successor_func = world.getSuccessorFunction();
    auto heuristic = world.getManhattanHeuristic();
    
    BestFirst best_first(heuristic, successor_func);
    
    // Test regular best-first search
    auto result = best_first.search("0,0", "2,2");
    assert(result.success);
    assert(result.path_cost > 0);
    
    // Test greedy best-first search
    best_first.setGreedy(true);
    auto result2 = best_first.search("0,0", "2,2");
    assert(result2.success);
    
    std::cout << "Best-First Search tests passed!" << std::endl;
}

void testIDAStar() {
    std::cout << "Testing IDA* algorithm..." << std::endl;
    
    GridWorld world(3, 3);
    auto successor_func = world.getSuccessorFunction();
    auto heuristic = world.getManhattanHeuristic();
    
    IDAStar ida_star(heuristic, successor_func);
    
    // Test pathfinding
    auto result = ida_star.search("0,0", "2,2");
    assert(result.success);
    assert(result.path_cost > 0);
    
    // Test threshold settings
    ida_star.setInitialThreshold(5.0);
    ida_star.setThresholdIncrement(2.0);
    
    auto result2 = ida_star.search("0,0", "2,2");
    assert(result2.success);
    
    std::cout << "IDA* tests passed!" << std::endl;
}

void testHeuristics() {
    std::cout << "Testing heuristic functions..." << std::endl;
    
    // Test Manhattan distance
    auto manhattan = heuristics::manhattanDistance("1,1", "3,3");
    assert(manhattan == 4.0);
    
    // Test Euclidean distance
    auto euclidean = heuristics::euclideanDistance("0,0", "3,4");
    assert(std::abs(euclidean - 5.0) < 1e-6);
    
    // Test zero heuristic
    auto zero = heuristics::zeroHeuristic("any", "state");
    assert(zero == 0.0);
    
    // Test Hamming distance
    auto hamming = heuristics::hammingDistance("hello", "world");
    assert(hamming == 4);
    
    std::cout << "Heuristic tests passed!" << std::endl;
}

void testPuzzleHeuristics() {
    std::cout << "Testing puzzle heuristics..." << std::endl;
    
    // Create test states
    std::vector<int> start_board = {1, 2, 3, 4, 0, 6, 7, 5, 8};
    std::vector<int> goal_board = {1, 2, 3, 4, 5, 6, 7, 8, 0};
    
    PuzzleState start_state(start_board, 3);
    PuzzleState goal_state(goal_board, 3);
    
    // Test Manhattan distance
    double manhattan_dist = start_state.manhattanDistance(goal_state);
    assert(manhattan_dist > 0);
    
    // Test misplaced tiles
    int misplaced = start_state.misplacedTiles(goal_state);
    assert(misplaced > 0);
    
    // Test linear conflicts
    double conflicts = start_state.linearConflicts(goal_state);
    assert(conflicts >= manhattan_dist);
    
    std::cout << "Puzzle heuristic tests passed!" << std::endl;
}

void runAllTests() {
    std::cout << "Running all tests..." << std::endl;
    std::cout << "=====================" << std::endl;
    
    try {
        testNode();
        testGridWorld();
        testPuzzleState();
        testHeuristics();
        testPuzzleHeuristics();
        testAStar();
        testDijkstra();
        testBestFirst();
        testIDAStar();
        
        std::cout << "\nAll tests passed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        throw;
    }
}

int main() {
    try {
        runAllTests();
        return 0;
    } catch (...) {
        return 1;
    }
} 
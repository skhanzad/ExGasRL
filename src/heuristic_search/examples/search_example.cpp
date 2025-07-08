#include <iostream>
#include <iomanip>
#include <chrono>
#include "astar.h"
#include "dijkstra.h"
#include "best_first.h"
#include "ida_star.h"
#include "grid_world.h"
#include "puzzle_state.h"

using namespace heuristic_search;

void printSearchResult(const SearchResult& result, const std::string& algorithm_name) {
    std::cout << "\n=== " << algorithm_name << " Results ===" << std::endl;
    std::cout << "Success: " << (result.success ? "Yes" : "No") << std::endl;
    
    if (result.success) {
        std::cout << "Path Cost: " << result.path_cost << std::endl;
        std::cout << "Path Length: " << result.path.size() << std::endl;
        std::cout << "Path: ";
        for (const auto& action : result.path) {
            std::cout << action << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "Nodes Expanded: " << result.nodes_expanded << std::endl;
    std::cout << "Nodes Generated: " << result.nodes_generated << std::endl;
    std::cout << "Execution Time: " << std::fixed << std::setprecision(3) 
              << result.execution_time_ms << " ms" << std::endl;
}

void gridWorldExample() {
    std::cout << "\n=== Grid World Pathfinding Example ===" << std::endl;
    
    // Create a 10x10 grid world
    GridWorld world(10, 10);
    
    // Add some obstacles
    for (int i = 0; i < 10; ++i) {
        world.setCell(3, i, GridWorld::WALL);
        world.setCell(7, i, GridWorld::WALL);
    }
    
    // Add some water
    for (int i = 2; i < 8; ++i) {
        world.setCell(i, 5, GridWorld::WATER);
    }
    
    // Add some mountains
    world.setCell(4, 2, GridWorld::MOUNTAIN);
    world.setCell(5, 2, GridWorld::MOUNTAIN);
    world.setCell(4, 3, GridWorld::MOUNTAIN);
    world.setCell(5, 3, GridWorld::MOUNTAIN);
    
    // Define start and goal positions
    std::string start_state = "0,0";
    std::string goal_state = "9,9";
    
    // Get successor and heuristic functions
    auto successor_func = world.getSuccessorFunction();
    auto manhattan_heuristic = world.getManhattanHeuristic();
    auto euclidean_heuristic = world.getEuclideanHeuristic();
    
    // Test A* with Manhattan distance
    AStar astar_manhattan(manhattan_heuristic, successor_func);
    auto result = astar_manhattan.search(start_state, goal_state);
    printSearchResult(result, "A* (Manhattan)");
    
    // Test A* with Euclidean distance
    AStar astar_euclidean(euclidean_heuristic, successor_func);
    result = astar_euclidean.search(start_state, goal_state);
    printSearchResult(result, "A* (Euclidean)");
    
    // Test Dijkstra's algorithm
    Dijkstra dijkstra(successor_func);
    result = dijkstra.search(start_state, goal_state);
    printSearchResult(result, "Dijkstra");
    
    // Test Best-First Search
    BestFirst best_first(manhattan_heuristic, successor_func);
    result = best_first.search(start_state, goal_state);
    printSearchResult(result, "Best-First Search");
    
    // Test Greedy Best-First Search
    BestFirst greedy_best_first(manhattan_heuristic, successor_func);
    greedy_best_first.setGreedy(true);
    result = greedy_best_first.search(start_state, goal_state);
    printSearchResult(result, "Greedy Best-First Search");
}

void puzzleExample() {
    std::cout << "\n=== 8-Puzzle Example ===" << std::endl;
    
    // Create a random 8-puzzle state
    PuzzleState start_state = PuzzleState::createRandomState(3, 20);
    PuzzleState goal_state = PuzzleState::createSolvedState(3);
    
    std::cout << "Start State:" << std::endl;
    std::cout << start_state.toString() << std::endl;
    std::cout << "Goal State:" << std::endl;
    std::cout << goal_state.toString() << std::endl;
    std::cout << "Solvable: " << (start_state.isSolvable() ? "Yes" : "No") << std::endl;
    
    // Get successor and heuristic functions
    auto successor_func = PuzzleState::getSuccessorFunction();
    auto manhattan_heuristic = PuzzleState::getManhattanHeuristic();
    auto misplaced_heuristic = PuzzleState::getMisplacedTilesHeuristic();
    auto linear_conflict_heuristic = PuzzleState::getLinearConflictHeuristic();
    
    // Test A* with Manhattan distance
    AStar astar_manhattan(manhattan_heuristic, successor_func);
    auto result = astar_manhattan.search(start_state.toString(), goal_state.toString());
    printSearchResult(result, "A* (Manhattan)");
    
    // Test A* with Misplaced Tiles
    AStar astar_misplaced(misplaced_heuristic, successor_func);
    result = astar_misplaced.search(start_state.toString(), goal_state.toString());
    printSearchResult(result, "A* (Misplaced Tiles)");
    
    // Test A* with Linear Conflicts
    AStar astar_linear(linear_conflict_heuristic, successor_func);
    result = astar_linear.search(start_state.toString(), goal_state.toString());
    printSearchResult(result, "A* (Linear Conflicts)");
    
    // Test IDA*
    IDAStar ida_star(manhattan_heuristic, successor_func);
    result = ida_star.search(start_state.toString(), goal_state.toString());
    printSearchResult(result, "IDA*");
}

void performanceComparison() {
    std::cout << "\n=== Performance Comparison ===" << std::endl;
    
    // Create a larger grid for performance testing
    GridWorld world(20, 20);
    world.createRandomGrid(0.3); // 30% obstacles
    
    std::string start_state = "0,0";
    std::string goal_state = "19,19";
    
    auto successor_func = world.getSuccessorFunction();
    auto manhattan_heuristic = world.getManhattanHeuristic();
    
    std::vector<std::pair<std::string, std::unique_ptr<SearchAlgorithm>>> algorithms;
    
    algorithms.emplace_back("A* (Manhattan)", 
                          std::make_unique<AStar>(manhattan_heuristic, successor_func));
    algorithms.emplace_back("Dijkstra", 
                          std::make_unique<Dijkstra>(successor_func));
    algorithms.emplace_back("Best-First", 
                          std::make_unique<BestFirst>(manhattan_heuristic, successor_func));
    algorithms.emplace_back("IDA*", 
                          std::make_unique<IDAStar>(manhattan_heuristic, successor_func));
    
    std::cout << std::setw(20) << "Algorithm" 
              << std::setw(15) << "Success" 
              << std::setw(15) << "Path Cost" 
              << std::setw(15) << "Nodes Expanded" 
              << std::setw(15) << "Time (ms)" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    for (const auto& [name, algorithm] : algorithms) {
        auto result = algorithm->search(start_state, goal_state);
        
        std::cout << std::setw(20) << name
                  << std::setw(15) << (result.success ? "Yes" : "No")
                  << std::setw(15) << (result.success ? std::to_string(result.path_cost) : "N/A")
                  << std::setw(15) << result.nodes_expanded
                  << std::setw(15) << std::fixed << std::setprecision(3) << result.execution_time_ms
                  << std::endl;
    }
}

int main() {
    std::cout << "Heuristic Search Framework Examples" << std::endl;
    std::cout << "===================================" << std::endl;
    
    try {
        // Grid World Example
        gridWorldExample();
        
        // Puzzle Example
        puzzleExample();
        
        // Performance Comparison
        performanceComparison();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\nExamples completed successfully!" << std::endl;
    return 0;
} 
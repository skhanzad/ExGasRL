#include "search_algorithm.h"
#include <chrono>
#include <sstream>
#include <cmath>

namespace heuristic_search {

SearchAlgorithm::SearchAlgorithm(HeuristicFunction heuristic, SuccessorFunction successor)
    : heuristic_(heuristic), successor_(successor), max_nodes_(1000000) {
}

SearchResult SearchAlgorithm::search(const std::string& start_state, const std::string& goal_state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    initializeSearch(start_state, goal_state);
    
    // Create start node
    auto start_node = std::make_shared<Node>(start_state, 0.0, heuristic_(start_state, goal_state));
    
    // Check if start is goal
    if (isGoal(start_node, goal_state)) {
        last_result_.success = true;
        last_result_.path = start_node->getPath();
        last_result_.path_cost = start_node->getPathCost();
        last_result_.nodes_expanded = 1;
        last_result_.nodes_generated = 1;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        last_result_.execution_time_ms = duration.count() / 1000.0;
        
        cleanupSearch();
        return last_result_;
    }
    
    // This is a base implementation - derived classes should override
    // For now, return failure
    last_result_.success = false;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    last_result_.execution_time_ms = duration.count() / 1000.0;
    
    cleanupSearch();
    return last_result_;
}

bool SearchAlgorithm::shouldContinue(const Node::NodePtr& current_node, const std::string& goal_state) {
    // Check if we've exceeded the maximum number of nodes
    if (last_result_.nodes_expanded >= max_nodes_) {
        return false;
    }
    
    return true;
}

bool SearchAlgorithm::isGoal(const Node::NodePtr& current_node, const std::string& goal_state) {
    return current_node->getState() == goal_state;
}

std::vector<Node::NodePtr> SearchAlgorithm::expandNode(const Node::NodePtr& node) {
    std::vector<Node::NodePtr> successors;
    
    if (!successor_) {
        return successors;
    }
    
    auto successors_data = successor_(node->getState());
    
    for (const auto& [state, cost] : successors_data) {
        auto successor = std::make_shared<Node>(state, node->getGCost() + cost, 0.0);
        successor->setParent(node);
        successors.push_back(successor);
    }
    
    return successors;
}

std::vector<std::string> SearchAlgorithm::reconstructPath(const Node::NodePtr& goal_node) {
    return goal_node->getPath();
}

void SearchAlgorithm::initializeSearch(const std::string& start_state, const std::string& goal_state) {
    last_result_ = SearchResult();
}

void SearchAlgorithm::cleanupSearch() {
    // Base implementation does nothing
    // Derived classes can override to clean up resources
}

void SearchAlgorithm::updateStatistics(size_t nodes_expanded, size_t nodes_generated, double execution_time) {
    last_result_.nodes_expanded = nodes_expanded;
    last_result_.nodes_generated = nodes_generated;
    last_result_.execution_time_ms = execution_time;
}

// Heuristic function implementations
namespace heuristics {

Node::CostType manhattanDistance(const std::string& state, const std::string& goal) {
    // Parse coordinates from strings like "x,y"
    std::istringstream state_stream(state);
    std::istringstream goal_stream(goal);
    
    int state_x, state_y, goal_x, goal_y;
    char comma;
    
    if (state_stream >> state_x >> comma >> state_y &&
        goal_stream >> goal_x >> comma >> goal_y) {
        return std::abs(state_x - goal_x) + std::abs(state_y - goal_y);
    }
    
    return 0.0;
}

Node::CostType euclideanDistance(const std::string& state, const std::string& goal) {
    // Parse coordinates from strings like "x,y"
    std::istringstream state_stream(state);
    std::istringstream goal_stream(goal);
    
    int state_x, state_y, goal_x, goal_y;
    char comma;
    
    if (state_stream >> state_x >> comma >> state_y &&
        goal_stream >> goal_x >> comma >> goal_y) {
        double dx = state_x - goal_x;
        double dy = state_y - goal_y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    return 0.0;
}

Node::CostType zeroHeuristic(const std::string& state, const std::string& goal) {
    return 0.0;
}

Node::CostType hammingDistance(const std::string& state, const std::string& goal) {
    if (state.length() != goal.length()) {
        return std::max(state.length(), goal.length());
    }
    
    int distance = 0;
    for (size_t i = 0; i < state.length(); ++i) {
        if (state[i] != goal[i]) {
            ++distance;
        }
    }
    
    return distance;
}

} // namespace heuristics

} // namespace heuristic_search 
#include "ida_star.h"
#include <unordered_set>
#include <algorithm>
#include <limits>

namespace heuristic_search {

IDAStar::IDAStar(HeuristicFunction heuristic, SuccessorFunction successor)
    : SearchAlgorithm(heuristic, successor), initial_threshold_(0.0), threshold_increment_(1.0) {
}

SearchResult IDAStar::search(const std::string& start_state, const std::string& goal_state) {
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
    
    // Initialize threshold
    Node::CostType threshold = initial_threshold_;
    if (threshold == 0.0) {
        threshold = start_node->getFCost();
    }
    
    size_t total_nodes_expanded = 0;
    size_t total_nodes_generated = 1;
    
    while (shouldContinue(start_node, goal_state)) {
        std::vector<Node::NodePtr> path;
        std::unordered_set<std::string> visited;
        
        // Perform depth-limited search
        Node::CostType next_threshold = depthLimitedSearch(start_node, goal_state, threshold, 
                                                          path, visited, last_result_);
        
        total_nodes_expanded += last_result_.nodes_expanded;
        total_nodes_generated += last_result_.nodes_generated;
        
        // If goal found, return result
        if (last_result_.success) {
            last_result_.nodes_expanded = total_nodes_expanded;
            last_result_.nodes_generated = total_nodes_generated;
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            last_result_.execution_time_ms = duration.count() / 1000.0;
            
            cleanupSearch();
            return last_result_;
        }
        
        // If no next threshold, search failed
        if (next_threshold < 0) {
            break;
        }
        
        // Update threshold for next iteration
        threshold = next_threshold;
    }
    
    // No path found
    last_result_.success = false;
    last_result_.nodes_expanded = total_nodes_expanded;
    last_result_.nodes_generated = total_nodes_generated;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    last_result_.execution_time_ms = duration.count() / 1000.0;
    
    cleanupSearch();
    return last_result_;
}

Node::CostType IDAStar::depthLimitedSearch(const Node::NodePtr& node,
                                          const std::string& goal_state,
                                          Node::CostType threshold,
                                          std::vector<Node::NodePtr>& path,
                                          std::unordered_set<std::string>& visited,
                                          SearchResult& result) {
    // Check if goal reached
    if (isGoal(node, goal_state)) {
        result.success = true;
        result.path = reconstructPath(node);
        result.path_cost = node->getPathCost();
        return -1; // Goal found
    }
    
    // Check if f-cost exceeds threshold
    if (node->getFCost() > threshold) {
        return node->getFCost(); // Return exceeded threshold
    }
    
    // Add node to path and visited set
    path.push_back(node);
    visited.insert(node->getState());
    
    Node::CostType min_exceeded = std::numeric_limits<Node::CostType>::max();
    
    // Expand node
    auto successors = expandNode(node);
    result.nodes_generated += successors.size();
    
    for (auto& successor : successors) {
        // Calculate h-cost for successor
        successor->setHCost(heuristic_(successor->getState(), goal_state));
        
        // Skip if already on current path (avoid cycles)
        if (isOnPath(successor, path)) {
            continue;
        }
        
        ++result.nodes_expanded;
        
        // Recursive depth-limited search
        Node::CostType exceeded = depthLimitedSearch(successor, goal_state, threshold, 
                                                   path, visited, result);
        
        if (exceeded < 0) {
            // Goal found
            return -1;
        }
        
        min_exceeded = std::min(min_exceeded, exceeded);
    }
    
    // Remove node from path (backtrack)
    path.pop_back();
    
    return min_exceeded;
}

bool IDAStar::isOnPath(const Node::NodePtr& node, const std::vector<Node::NodePtr>& path) {
    for (const auto& path_node : path) {
        if (node->getState() == path_node->getState()) {
            return true;
        }
    }
    return false;
}

Node::CostType IDAStar::calculateNextThreshold(Node::CostType min_exceeded) {
    if (min_exceeded == std::numeric_limits<Node::CostType>::max()) {
        return -1; // No next threshold
    }
    
    return min_exceeded + threshold_increment_;
}

} // namespace heuristic_search 
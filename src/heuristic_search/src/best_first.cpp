#include "best_first.h"
#include <unordered_set>
#include <algorithm>

namespace heuristic_search {

BestFirst::BestFirst(HeuristicFunction heuristic, SuccessorFunction successor)
    : SearchAlgorithm(heuristic, successor), greedy_(false) {
}

SearchResult BestFirst::search(const std::string& start_state, const std::string& goal_state) {
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
    
    // Initialize data structures
    PriorityQueue open_list{ BestFirstCompare(greedy_) };
    std::unordered_set<std::string> visited;
    
    // Add start node to open list
    open_list.push(start_node);
    visited.insert(start_state);
    
    size_t nodes_expanded = 0;
    size_t nodes_generated = 1;
    
    while (!open_list.empty() && shouldContinue(start_node, goal_state)) {
        auto current_node = open_list.top();
        open_list.pop();
        
        ++nodes_expanded;
        
        // Check if goal reached
        if (isGoal(current_node, goal_state)) {
            last_result_.success = true;
            last_result_.path = reconstructPath(current_node);
            last_result_.path_cost = current_node->getPathCost();
            last_result_.nodes_expanded = nodes_expanded;
            last_result_.nodes_generated = nodes_generated;
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            last_result_.execution_time_ms = duration.count() / 1000.0;
            
            cleanupSearch();
            return last_result_;
        }
        
        // Expand current node
        auto successors = expandNode(current_node);
        
        for (auto& successor : successors) {
            // Calculate h-cost for successor
            successor->setHCost(heuristic_(successor->getState(), goal_state));
            
            // Skip if already visited
            if (isVisited(successor, visited)) {
                continue;
            }
            
            // Add to open list and mark as visited
            open_list.push(successor);
            visited.insert(successor->getState());
            ++nodes_generated;
        }
    }
    
    // No path found
    last_result_.success = false;
    last_result_.nodes_expanded = nodes_expanded;
    last_result_.nodes_generated = nodes_generated;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    last_result_.execution_time_ms = duration.count() / 1000.0;
    
    cleanupSearch();
    return last_result_;
}

bool BestFirst::isVisited(const Node::NodePtr& node, 
                         const std::unordered_set<std::string>& visited) {
    return visited.find(node->getState()) != visited.end();
}

} // namespace heuristic_search 
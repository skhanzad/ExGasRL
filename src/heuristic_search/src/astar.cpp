#include "astar.h"
#include <unordered_set>
#include <algorithm>
#include <chrono>

namespace heuristic_search {

AStar::AStar(HeuristicFunction heuristic, SuccessorFunction successor)
    : SearchAlgorithm(heuristic, successor), tie_break_high_g_(false), weight_(1.0) {
}

SearchResult AStar::search(const std::string& start_state, const std::string& goal_state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    initializeSearch(start_state, goal_state);
    
    // Create start node
    auto start_node = std::make_shared<Node>(start_state, 0.0, 
                                           weight_ * heuristic_(start_state, goal_state));
    
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
    PriorityQueue open_list{AStarCompare(tie_break_high_g_)};
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, Node::CostType> g_scores;
    
    // Add start node to open list
    open_list.push(start_node);
    g_scores[start_state] = 0.0;
    
    size_t nodes_expanded = 0;
    size_t nodes_generated = 1;
    
    while (!open_list.empty() && shouldContinue(start_node, goal_state)) {
        auto current_node = open_list.top();
        open_list.pop();
        
        // Skip if already processed
        if (closed_set.find(current_node->getState()) != closed_set.end()) {
            continue;
        }
        
        // Add to closed set
        closed_set.insert(current_node->getState());
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
            successor->setHCost(weight_ * heuristic_(successor->getState(), goal_state));
            
            // Skip if already in closed set
            if (closed_set.find(successor->getState()) != closed_set.end()) {
                continue;
            }
            
            // Check if we found a better path
            if (hasBetterPath(successor, g_scores)) {
                updateGScore(successor, g_scores);
                open_list.push(successor);
                ++nodes_generated;
            }
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

bool AStar::hasBetterPath(const Node::NodePtr& node, 
                         const std::unordered_map<std::string, Node::CostType>& g_scores) {
    auto it = g_scores.find(node->getState());
    if (it == g_scores.end()) {
        return true; // New node
    }
    
    return node->getGCost() < it->second;
}

void AStar::updateGScore(const Node::NodePtr& node,
                        std::unordered_map<std::string, Node::CostType>& g_scores) {
    g_scores[node->getState()] = node->getGCost();
}

} // namespace heuristic_search 
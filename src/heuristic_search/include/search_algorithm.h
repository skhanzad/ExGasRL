#pragma once

#include "node.h"
#include <vector>
#include <memory>
#include <functional>
#include <unordered_set>
#include <queue>

namespace heuristic_search {

/**
 * @brief Structure to hold search results
 */
struct SearchResult {
    bool success;
    std::vector<std::string> path;
    Node::CostType path_cost;
    size_t nodes_expanded;
    size_t nodes_generated;
    double execution_time_ms;
    
    SearchResult() : success(false), path_cost(0.0), 
                    nodes_expanded(0), nodes_generated(0), 
                    execution_time_ms(0.0) {}
};

/**
 * @brief Base class for search algorithms
 * 
 * This class provides a common interface for all search algorithms
 * and implements common functionality like path reconstruction.
 */
class SearchAlgorithm {
public:
    using HeuristicFunction = std::function<Node::CostType(const std::string&, const std::string&)>;
    using SuccessorFunction = std::function<std::vector<std::pair<std::string, Node::CostType>>(const std::string&)>;
    
    /**
     * @brief Constructor
     * @param heuristic Heuristic function
     * @param successor Successor function
     */
    SearchAlgorithm(HeuristicFunction heuristic, SuccessorFunction successor);
    
    virtual ~SearchAlgorithm() = default;
    
    /**
     * @brief Perform search from start to goal
     * @param start_state Starting state
     * @param goal_state Goal state
     * @return Search result
     */
    virtual SearchResult search(const std::string& start_state, const std::string& goal_state);
    
    /**
     * @brief Set the heuristic function
     * @param heuristic New heuristic function
     */
    void setHeuristic(HeuristicFunction heuristic) { heuristic_ = heuristic; }
    
    /**
     * @brief Set the successor function
     * @param successor New successor function
     */
    void setSuccessor(SuccessorFunction successor) { successor_ = successor; }
    
    /**
     * @brief Get the heuristic function
     * @return Current heuristic function
     */
    HeuristicFunction getHeuristic() const { return heuristic_; }
    
    /**
     * @brief Get the successor function
     * @return Current successor function
     */
    SuccessorFunction getSuccessor() const { return successor_; }
    
    /**
     * @brief Set maximum number of nodes to expand
     * @param max_nodes Maximum nodes
     */
    void setMaxNodes(size_t max_nodes) { max_nodes_ = max_nodes; }
    
    /**
     * @brief Get maximum number of nodes to expand
     * @return Maximum nodes
     */
    size_t getMaxNodes() const { return max_nodes_; }
    
    /**
     * @brief Check if search should continue
     * @param current_node Current node being expanded
     * @param goal_state Goal state
     * @return True if search should continue
     */
    virtual bool shouldContinue(const Node::NodePtr& current_node, const std::string& goal_state);
    
    /**
     * @brief Check if goal is reached
     * @param current_node Current node
     * @param goal_state Goal state
     * @return True if goal is reached
     */
    virtual bool isGoal(const Node::NodePtr& current_node, const std::string& goal_state);
    
    /**
     * @brief Expand a node to get its successors
     * @param node Node to expand
     * @return Vector of successor nodes
     */
    virtual std::vector<Node::NodePtr> expandNode(const Node::NodePtr& node);
    
    /**
     * @brief Reconstruct path from goal node
     * @param goal_node Goal node
     * @return Vector of actions
     */
    std::vector<std::string> reconstructPath(const Node::NodePtr& goal_node);
    
    /**
     * @brief Get search statistics
     * @return Search result with statistics
     */
    SearchResult getStatistics() const { return last_result_; }

protected:
    HeuristicFunction heuristic_;
    SuccessorFunction successor_;
    size_t max_nodes_;
    SearchResult last_result_;
    
    /**
     * @brief Initialize search
     * @param start_state Starting state
     * @param goal_state Goal state
     */
    virtual void initializeSearch(const std::string& start_state, const std::string& goal_state);
    
    /**
     * @brief Clean up after search
     */
    virtual void cleanupSearch();
    
    /**
     * @brief Update search statistics
     * @param nodes_expanded Number of nodes expanded
     * @param nodes_generated Number of nodes generated
     * @param execution_time Execution time in milliseconds
     */
    void updateStatistics(size_t nodes_expanded, size_t nodes_generated, double execution_time);
};

/**
 * @brief Common heuristic functions
 */
namespace heuristics {
    
    /**
     * @brief Manhattan distance heuristic for grid problems
     * @param state Current state (format: "x,y")
     * @param goal Goal state (format: "x,y")
     * @return Manhattan distance
     */
    Node::CostType manhattanDistance(const std::string& state, const std::string& goal);
    
    /**
     * @brief Euclidean distance heuristic
     * @param state Current state (format: "x,y")
     * @param goal Goal state (format: "x,y")
     * @return Euclidean distance
     */
    Node::CostType euclideanDistance(const std::string& state, const std::string& goal);
    
    /**
     * @brief Zero heuristic (for Dijkstra's algorithm)
     * @param state Current state
     * @param goal Goal state
     * @return Always returns 0
     */
    Node::CostType zeroHeuristic(const std::string& state, const std::string& goal);
    
    /**
     * @brief Hamming distance for string states
     * @param state Current state
     * @param goal Goal state
     * @return Number of different characters
     */
    Node::CostType hammingDistance(const std::string& state, const std::string& goal);
}

} // namespace heuristic_search 
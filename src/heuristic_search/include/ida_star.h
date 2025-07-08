#pragma once

#include "search_algorithm.h"
#include <stack>
#include <unordered_set>
#include <chrono>

namespace heuristic_search {

/**
 * @brief IDA* (Iterative Deepening A*) search algorithm implementation
 * 
 * IDA* is a memory-efficient variant of A* that uses iterative deepening
 * with a threshold based on f-cost. It's useful when memory is limited
 * and the search space is large.
 */
class IDAStar : public SearchAlgorithm {
public:
    /**
     * @brief Constructor
     * @param heuristic Heuristic function
     * @param successor Successor function
     */
    IDAStar(HeuristicFunction heuristic, SuccessorFunction successor);
    
    /**
     * @brief Destructor
     */
    ~IDAStar() override = default;
    
    /**
     * @brief Perform IDA* search
     * @param start_state Starting state
     * @param goal_state Goal state
     * @return Search result
     */
    SearchResult search(const std::string& start_state, const std::string& goal_state) override;
    
    /**
     * @brief Set initial threshold
     * @param threshold Initial f-cost threshold
     */
    void setInitialThreshold(Node::CostType threshold) { initial_threshold_ = threshold; }
    
    /**
     * @brief Get initial threshold
     * @return Initial threshold value
     */
    Node::CostType getInitialThreshold() const { return initial_threshold_; }
    
    /**
     * @brief Set threshold increment strategy
     * @param increment Threshold increment value
     */
    void setThresholdIncrement(Node::CostType increment) { threshold_increment_ = increment; }
    
    /**
     * @brief Get threshold increment
     * @return Threshold increment value
     */
    Node::CostType getThresholdIncrement() const { return threshold_increment_; }

private:
    Node::CostType initial_threshold_;
    Node::CostType threshold_increment_;
    
    /**
     * @brief Perform depth-limited search
     * @param node Current node
     * @param goal_state Goal state
     * @param threshold Current f-cost threshold
     * @param path Current path
     * @param visited Set of visited states
     * @param result Search result to update
     * @return Next threshold if goal not found, -1 if goal found
     */
    Node::CostType depthLimitedSearch(const Node::NodePtr& node,
                                    const std::string& goal_state,
                                    Node::CostType threshold,
                                    std::vector<Node::NodePtr>& path,
                                    std::unordered_set<std::string>& visited,
                                    SearchResult& result);
    
    /**
     * @brief Check if node is on current path
     * @param node Node to check
     * @param path Current path
     * @return True if node is on path
     */
    bool isOnPath(const Node::NodePtr& node, const std::vector<Node::NodePtr>& path);
    
    /**
     * @brief Calculate next threshold
     * @param min_exceeded Minimum f-cost that exceeded current threshold
     * @return Next threshold value
     */
    Node::CostType calculateNextThreshold(Node::CostType min_exceeded);
};

} // namespace heuristic_search 
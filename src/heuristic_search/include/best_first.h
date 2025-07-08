#pragma once

#include "search_algorithm.h"
#include <queue>
#include <unordered_set>
#include <chrono>

namespace heuristic_search {

/**
 * @brief Best-First Search algorithm implementation
 * 
 * Best-First Search is an informed search algorithm that uses a heuristic
 * function to estimate the cost to the goal. It always expands the node
 * with the lowest heuristic value, regardless of the actual path cost.
 */
class BestFirst : public SearchAlgorithm {
public:
    /**
     * @brief Constructor
     * @param heuristic Heuristic function
     * @param successor Successor function
     */
    BestFirst(HeuristicFunction heuristic, SuccessorFunction successor);
    
    /**
     * @brief Destructor
     */
    ~BestFirst() override = default;
    
    /**
     * @brief Perform Best-First search
     * @param start_state Starting state
     * @param goal_state Goal state
     * @return Search result
     */
    SearchResult search(const std::string& start_state, const std::string& goal_state) override;
    
    /**
     * @brief Set whether to use greedy best-first search
     * @param greedy True for greedy (h-cost only), false for f-cost
     */
    void setGreedy(bool greedy) { greedy_ = greedy; }
    
    /**
     * @brief Get whether using greedy best-first search
     * @return True if greedy
     */
    bool isGreedy() const { return greedy_; }

private:
    bool greedy_;
    
    /**
     * @brief Custom comparison for priority queue
     */
    struct BestFirstCompare {
        bool greedy;
        
        BestFirstCompare(bool g) : greedy(g) {}
        
        bool operator()(const Node::NodePtr& a, const Node::NodePtr& b) const {
            if (greedy) {
                // Greedy best-first: use only h-cost
                return a->getHCost() > b->getHCost();
            } else {
                // Best-first: use f-cost (g + h)
                return a->getFCost() > b->getFCost();
            }
        }
    };
    
    using PriorityQueue = std::priority_queue<Node::NodePtr, 
                                             std::vector<Node::NodePtr>, 
                                             BestFirstCompare>;
    
    /**
     * @brief Check if node has been visited
     * @param node Node to check
     * @param visited Set of visited states
     * @return True if already visited
     */
    bool isVisited(const Node::NodePtr& node, 
                  const std::unordered_set<std::string>& visited);
};

} // namespace heuristic_search 
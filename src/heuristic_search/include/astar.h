#pragma once

#include "search_algorithm.h"
#include <queue>
#include <unordered_map>

namespace heuristic_search {

/**
 * @brief A* search algorithm implementation
 * 
 * A* is an informed search algorithm that uses a heuristic function
 * to estimate the cost from the current node to the goal. It combines
 * the actual cost from start (g-cost) with the heuristic estimate (h-cost)
 * to determine which node to expand next.
 */
class AStar : public SearchAlgorithm {
public:
    /**
     * @brief Constructor
     * @param heuristic Heuristic function
     * @param successor Successor function
     */
    AStar(HeuristicFunction heuristic, SuccessorFunction successor);
    
    /**
     * @brief Destructor
     */
    ~AStar() override = default;
    
    /**
     * @brief Perform A* search
     * @param start_state Starting state
     * @param goal_state Goal state
     * @return Search result
     */
    SearchResult search(const std::string& start_state, const std::string& goal_state) override;
    
    /**
     * @brief Set tie-breaking strategy
     * @param tie_break True for high g-cost tie-breaking, false for low g-cost
     */
    void setTieBreaking(bool tie_break) { tie_break_high_g_ = tie_break; }
    
    /**
     * @brief Get tie-breaking strategy
     * @return True if using high g-cost tie-breaking
     */
    bool getTieBreaking() const { return tie_break_high_g_; }
    
    /**
     * @brief Set weight for heuristic (for weighted A*)
     * @param weight Weight factor (1.0 for standard A*)
     */
    void setWeight(double weight) { weight_ = weight; }
    
    /**
     * @brief Get weight for heuristic
     * @return Weight factor
     */
    double getWeight() const { return weight_; }

private:
    bool tie_break_high_g_;
    double weight_;
    
    /**
     * @brief Custom comparison for priority queue with tie-breaking
     */
    struct AStarCompare {
        bool tie_break_high_g;
        
        AStarCompare(bool tie_break) : tie_break_high_g(tie_break) {}
        
        bool operator()(const Node::NodePtr& a, const Node::NodePtr& b) const {
            double f_a = a->getFCost();
            double f_b = b->getFCost();
            
            if (std::abs(f_a - f_b) < 1e-9) {
                // Tie-breaking based on g-cost
                if (tie_break_high_g) {
                    return a->getGCost() < b->getGCost();
                } else {
                    return a->getGCost() > b->getGCost();
                }
            }
            
            return f_a > f_b;
        }
    };
    
    using PriorityQueue = std::priority_queue<Node::NodePtr, 
                                             std::vector<Node::NodePtr>, 
                                             AStarCompare>;
    
    /**
     * @brief Check if a better path to a node exists
     * @param node Node to check
     * @param g_scores Map of g-scores
     * @return True if better path exists
     */
    bool hasBetterPath(const Node::NodePtr& node, 
                      const std::unordered_map<std::string, Node::CostType>& g_scores);
    
    /**
     * @brief Update g-score for a node
     * @param node Node to update
     * @param g_scores Map of g-scores
     */
    void updateGScore(const Node::NodePtr& node,
                     std::unordered_map<std::string, Node::CostType>& g_scores);
};

} // namespace heuristic_search 
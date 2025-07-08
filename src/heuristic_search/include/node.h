#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>

namespace heuristic_search {

/**
 * @brief Base class for nodes in search algorithms
 * 
 * This class provides a generic interface for nodes that can be used
 * in various search algorithms like A*, Dijkstra's, etc.
 */
class Node {
public:
    using NodePtr = std::shared_ptr<Node>;
    using CostType = double;
    
    /**
     * @brief Constructor
     * @param state The state represented by this node
     * @param g_cost The actual cost from start to this node
     * @param h_cost The heuristic cost from this node to goal
     */
    Node(const std::string& state, CostType g_cost = 0.0, CostType h_cost = 0.0);
    
    virtual ~Node() = default;
    
    /**
     * @brief Get the state string
     * @return State string
     */
    const std::string& getState() const { return state_; }
    
    /**
     * @brief Get the g-cost (actual cost from start)
     * @return G-cost value
     */
    CostType getGCost() const { return g_cost_; }
    
    /**
     * @brief Get the h-cost (heuristic cost to goal)
     * @return H-cost value
     */
    CostType getHCost() const { return h_cost_; }
    
    /**
     * @brief Get the f-cost (g + h)
     * @return F-cost value
     */
    CostType getFCost() const { return g_cost_ + h_cost_; }
    
    /**
     * @brief Set the g-cost
     * @param g_cost New g-cost value
     */
    void setGCost(CostType g_cost) { g_cost_ = g_cost; }
    
    /**
     * @brief Set the h-cost
     * @param h_cost New h-cost value
     */
    void setHCost(CostType h_cost) { h_cost_ = h_cost; }
    
    /**
     * @brief Get the parent node
     * @return Parent node pointer
     */
    NodePtr getParent() const { return parent_; }
    
    /**
     * @brief Set the parent node
     * @param parent Parent node pointer
     */
    void setParent(NodePtr parent) { parent_ = parent; }
    
    /**
     * @brief Get the action that led to this node
     * @return Action string
     */
    const std::string& getAction() const { return action_; }
    
    /**
     * @brief Set the action that led to this node
     * @param action Action string
     */
    void setAction(const std::string& action) { action_ = action; }
    
    /**
     * @brief Get the path from start to this node
     * @return Vector of actions
     */
    std::vector<std::string> getPath() const;
    
    /**
     * @brief Get the path cost from start to this node
     * @return Total path cost
     */
    CostType getPathCost() const { return g_cost_; }
    
    /**
     * @brief Check if this node equals another node
     * @param other Other node to compare with
     * @return True if nodes are equal
     */
    virtual bool equals(const Node& other) const;
    
    /**
     * @brief Get string representation of the node
     * @return String representation
     */
    virtual std::string toString() const;
    
    /**
     * @brief Create a copy of this node
     * @return New node with same state
     */
    virtual NodePtr clone() const;

protected:
    std::string state_;
    CostType g_cost_;
    CostType h_cost_;
    NodePtr parent_;
    std::string action_;
};

/**
 * @brief Comparison function for nodes (used in priority queues)
 */
struct NodeCompare {
    bool operator()(const Node::NodePtr& a, const Node::NodePtr& b) const {
        return a->getFCost() > b->getFCost();
    }
};

/**
 * @brief Hash function for nodes (used in hash sets)
 */
struct NodeHash {
    std::size_t operator()(const Node::NodePtr& node) const {
        return std::hash<std::string>{}(node->getState());
    }
};

/**
 * @brief Equality function for nodes (used in hash sets)
 */
struct NodeEqual {
    bool operator()(const Node::NodePtr& a, const Node::NodePtr& b) const {
        return a->equals(*b);
    }
};

} // namespace heuristic_search 
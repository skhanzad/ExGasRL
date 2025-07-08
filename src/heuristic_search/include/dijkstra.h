#pragma once

#include "search_algorithm.h"
#include <queue>
#include <unordered_map>

namespace heuristic_search {

/**
 * @brief Dijkstra's search algorithm implementation
 * 
 * Dijkstra's algorithm is an uninformed search algorithm that finds
 * the shortest path from a start node to all other nodes in a graph
 * with non-negative edge weights. It's equivalent to A* with a zero heuristic.
 */
class Dijkstra : public SearchAlgorithm {
public:
    /**
     * @brief Constructor
     * @param successor Successor function
     */
    explicit Dijkstra(SuccessorFunction successor);
    
    /**
     * @brief Destructor
     */
    ~Dijkstra() override = default;
    
    /**
     * @brief Perform Dijkstra's search
     * @param start_state Starting state
     * @param goal_state Goal state
     * @return Search result
     */
    SearchResult search(const std::string& start_state, const std::string& goal_state) override;
    
    /**
     * @brief Find shortest paths to all reachable nodes
     * @param start_state Starting state
     * @return Map of states to their shortest path costs
     */
    std::unordered_map<std::string, Node::CostType> findAllShortestPaths(const std::string& start_state);
    
    /**
     * @brief Get the shortest path tree
     * @param start_state Starting state
     * @return Map of states to their parent states
     */
    std::unordered_map<std::string, std::string> getShortestPathTree(const std::string& start_state);

private:
    /**
     * @brief Custom comparison for priority queue
     */
    struct DijkstraCompare {
        bool operator()(const Node::NodePtr& a, const Node::NodePtr& b) const {
            return a->getGCost() > b->getGCost();
        }
    };
    
    using PriorityQueue = std::priority_queue<Node::NodePtr, 
                                             std::vector<Node::NodePtr>, 
                                             DijkstraCompare>;
    
    /**
     * @brief Check if a better path to a node exists
     * @param node Node to check
     * @param distances Map of distances
     * @return True if better path exists
     */
    bool hasBetterPath(const Node::NodePtr& node, 
                      const std::unordered_map<std::string, Node::CostType>& distances);
    
    /**
     * @brief Update distance for a node
     * @param node Node to update
     * @param distances Map of distances
     */
    void updateDistance(const Node::NodePtr& node,
                       std::unordered_map<std::string, Node::CostType>& distances);
};

} // namespace heuristic_search 
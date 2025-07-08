#pragma once

#include "search_algorithm.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <functional>

namespace heuristic_search {

/**
 * @brief Neural Network Layer representation
 */
struct NeuralLayer {
    std::string layer_id;
    std::vector<std::string> node_ids;
    std::string layer_type;  // "input", "hidden", "output"
    int layer_index;
    
    NeuralLayer(const std::string& id, const std::string& type, int index)
        : layer_id(id), layer_type(type), layer_index(index) {}
};

/**
 * @brief Neural Network Connection representation
 */
struct NeuralConnection {
    std::string from_node;
    std::string to_node;
    double weight;
    bool is_active;
    
    NeuralConnection(const std::string& from, const std::string& to, double w, bool active = true)
        : from_node(from), to_node(to), weight(w), is_active(active) {}
};

/**
 * @brief Neural Network Pathfinding Problem
 * 
 * This class represents a neural network as a graph where nodes are neurons
 * and edges are connections with weights. The goal is to find paths from
 * input nodes to output nodes, optimizing for various objectives like:
 * - Minimum path length
 * - Maximum activation strength
 * - Minimum computational cost
 * - Path diversity
 */
class NeuralNetworkPath {
public:
    using NodeId = std::string;
    using LayerId = std::string;
    using PathObjective = std::string;
    
    /**
     * @brief Path objectives
     */
    enum Objective {
        MIN_LENGTH,           // Minimize number of nodes in path
        MAX_ACTIVATION,       // Maximize activation strength
        MIN_COST,            // Minimize computational cost
        MAX_DIVERSITY,       // Maximize path diversity
        BALANCED             // Balance between length and activation
    };
    
    /**
     * @brief Constructor
     * @param name Network name
     */
    explicit NeuralNetworkPath(const std::string& name = "neural_network");
    
    /**
     * @brief Destructor
     */
    ~NeuralNetworkPath() = default;
    
    /**
     * @brief Add a layer to the network
     * @param layer_id Layer identifier
     * @param layer_type Layer type ("input", "hidden", "output")
     * @param node_count Number of nodes in the layer
     * @return Layer index
     */
    int addLayer(const std::string& layer_id, const std::string& layer_type, int node_count);
    
    /**
     * @brief Add a connection between nodes
     * @param from_node Source node ID
     * @param to_node Target node ID
     * @param weight Connection weight
     * @param is_active Whether the connection is active
     */
    void addConnection(const std::string& from_node, const std::string& to_node, 
                      double weight, bool is_active = true);
    
    /**
     * @brief Get all nodes in the network
     * @return Vector of node IDs
     */
    std::vector<std::string> getAllNodes() const;
    
    /**
     * @brief Get input nodes
     * @return Vector of input node IDs
     */
    std::vector<std::string> getInputNodes() const;
    
    /**
     * @brief Get output nodes
     * @return Vector of output node IDs
     */
    std::vector<std::string> getOutputNodes() const;
    
    /**
     * @brief Get hidden nodes
     * @return Vector of hidden node IDs
     */
    std::vector<std::string> getHiddenNodes() const;
    
    /**
     * @brief Get connections from a node
     * @param node_id Node ID
     * @return Vector of connections
     */
    std::vector<NeuralConnection> getConnectionsFrom(const std::string& node_id) const;
    
    /**
     * @brief Get connections to a node
     * @param node_id Node ID
     * @return Vector of connections
     */
    std::vector<NeuralConnection> getConnectionsTo(const std::string& node_id) const;
    
    /**
     * @brief Get node layer
     * @param node_id Node ID
     * @return Layer index, -1 if not found
     */
    int getNodeLayer(const std::string& node_id) const;
    
    /**
     * @brief Get node type
     * @param node_id Node ID
     * @return Node type string
     */
    std::string getNodeType(const std::string& node_id) const;
    
    /**
     * @brief Check if node is input
     * @param node_id Node ID
     * @return True if input node
     */
    bool isInputNode(const std::string& node_id) const;
    
    /**
     * @brief Check if node is output
     * @param node_id Node ID
     * @return True if output node
     */
    bool isOutputNode(const std::string& node_id) const;
    
    /**
     * @brief Check if node is hidden
     * @param node_id Node ID
     * @return True if hidden node
     */
    bool isHiddenNode(const std::string& node_id) const;
    
    /**
     * @brief Get connection weight
     * @param from_node Source node
     * @param to_node Target node
     * @return Connection weight, 0.0 if no connection
     */
    double getConnectionWeight(const std::string& from_node, const std::string& to_node) const;
    
    /**
     * @brief Check if connection exists and is active
     * @param from_node Source node
     * @param to_node Target node
     * @return True if active connection exists
     */
    bool hasActiveConnection(const std::string& from_node, const std::string& to_node) const;
    
    /**
     * @brief Set path objective
     * @param objective Path optimization objective
     */
    void setObjective(Objective objective) { objective_ = objective; }
    
    /**
     * @brief Get current objective
     * @return Current objective
     */
    Objective getObjective() const { return objective_; }
    
    /**
     * @brief Get successor function for search algorithms
     * @return Successor function
     */
    SearchAlgorithm::SuccessorFunction getSuccessorFunction() const;
    
    /**
     * @brief Get heuristic function for search algorithms
     * @return Heuristic function
     */
    SearchAlgorithm::HeuristicFunction getHeuristicFunction() const;
    
    /**
     * @brief Get heuristic function for specific objective
     * @param objective Path objective
     * @return Heuristic function
     */
    SearchAlgorithm::HeuristicFunction getHeuristicFunction(Objective objective) const;
    
    /**
     * @brief Calculate path activation strength
     * @param path Sequence of node IDs
     * @return Activation strength
     */
    double calculatePathActivation(const std::vector<std::string>& path) const;
    
    /**
     * @brief Calculate path computational cost
     * @param path Sequence of node IDs
     * @return Computational cost
     */
    double calculatePathCost(const std::vector<std::string>& path) const;
    
    /**
     * @brief Calculate path diversity score
     * @param path Sequence of node IDs
     * @return Diversity score
     */
    double calculatePathDiversity(const std::vector<std::string>& path) const;
    
    /**
     * @brief Create a simple feedforward network
     * @param input_size Number of input nodes
     * @param hidden_sizes Vector of hidden layer sizes
     * @param output_size Number of output nodes
     * @param weight_range Range for random weights [min, max]
     */
    void createFeedforwardNetwork(int input_size, const std::vector<int>& hidden_sizes, 
                                 int output_size, const std::pair<double, double>& weight_range = {-1.0, 1.0});
    
    /**
     * @brief Create a recurrent network
     * @param input_size Number of input nodes
     * @param hidden_size Number of hidden nodes
     * @param output_size Number of output nodes
     * @param weight_range Range for random weights
     */
    void createRecurrentNetwork(int input_size, int hidden_size, int output_size,
                               const std::pair<double, double>& weight_range = {-1.0, 1.0});
    
    /**
     * @brief Load network from file
     * @param filename File path
     * @return True if successful
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief Save network to file
     * @param filename File path
     * @return True if successful
     */
    bool saveToFile(const std::string& filename) const;
    
    /**
     * @brief Print network structure
     */
    void printNetwork() const;
    
    /**
     * @brief Get network statistics
     * @return Map of statistics
     */
    std::unordered_map<std::string, int> getNetworkStats() const;

private:
    std::string network_name_;
    std::vector<NeuralLayer> layers_;
    std::vector<NeuralConnection> connections_;
    std::unordered_map<std::string, int> node_to_layer_;
    std::unordered_map<std::string, std::string> node_types_;
    std::unordered_map<std::string, std::vector<NeuralConnection>> node_connections_;
    Objective objective_;
    
    /**
     * @brief Generate node IDs for a layer
     * @param layer_id Layer identifier
     * @param node_count Number of nodes
     * @return Vector of node IDs
     */
    std::vector<std::string> generateNodeIds(const std::string& layer_id, int node_count) const;
    
    /**
     * @brief Calculate layer-based heuristic
     * @param current_node Current node
     * @param goal_node Goal node
     * @return Heuristic value
     */
    double layerBasedHeuristic(const std::string& current_node, const std::string& goal_node) const;
    
    /**
     * @brief Calculate activation-based heuristic
     * @param current_node Current node
     * @param goal_node Goal node
     * @return Heuristic value
     */
    double activationBasedHeuristic(const std::string& current_node, const std::string& goal_node) const;
    
    /**
     * @brief Calculate cost-based heuristic
     * @param current_node Current node
     * @param goal_node Goal node
     * @return Heuristic value
     */
    double costBasedHeuristic(const std::string& current_node, const std::string& goal_node) const;
    
    /**
     * @brief Calculate balanced heuristic
     * @param current_node Current node
     * @param goal_node Goal node
     * @return Heuristic value
     */
    double balancedHeuristic(const std::string& current_node, const std::string& goal_node) const;
    
    /**
     * @brief Get random weight in range
     * @param range Weight range
     * @return Random weight
     */
    double getRandomWeight(const std::pair<double, double>& range) const;
};

} // namespace heuristic_search 
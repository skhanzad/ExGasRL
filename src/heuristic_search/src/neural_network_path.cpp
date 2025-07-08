#include "neural_network_path.h"
#include <sstream>
#include <fstream>
#include <random>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace heuristic_search {

NeuralNetworkPath::NeuralNetworkPath(const std::string& name)
    : network_name_(name), objective_(MIN_LENGTH) {
}

int NeuralNetworkPath::addLayer(const std::string& layer_id, const std::string& layer_type, int node_count) {
    int layer_index = static_cast<int>(layers_.size());
    NeuralLayer layer(layer_id, layer_type, layer_index);
    
    // Generate node IDs for this layer
    layer.node_ids = generateNodeIds(layer_id, node_count);
    
    // Add layer
    layers_.push_back(layer);
    
    // Update node mappings
    for (const auto& node_id : layer.node_ids) {
        node_to_layer_[node_id] = layer_index;
        node_types_[node_id] = layer_type;
    }
    
    return layer_index;
}

void NeuralNetworkPath::addConnection(const std::string& from_node, const std::string& to_node, 
                                     double weight, bool is_active) {
    NeuralConnection connection(from_node, to_node, weight, is_active);
    connections_.push_back(connection);
    
    // Update node connections map
    node_connections_[from_node].push_back(connection);
}

std::vector<std::string> NeuralNetworkPath::getAllNodes() const {
    std::vector<std::string> nodes;
    for (const auto& layer : layers_) {
        nodes.insert(nodes.end(), layer.node_ids.begin(), layer.node_ids.end());
    }
    return nodes;
}

std::vector<std::string> NeuralNetworkPath::getInputNodes() const {
    std::vector<std::string> input_nodes;
    for (const auto& layer : layers_) {
        if (layer.layer_type == "input") {
            input_nodes.insert(input_nodes.end(), layer.node_ids.begin(), layer.node_ids.end());
        }
    }
    return input_nodes;
}

std::vector<std::string> NeuralNetworkPath::getOutputNodes() const {
    std::vector<std::string> output_nodes;
    for (const auto& layer : layers_) {
        if (layer.layer_type == "output") {
            output_nodes.insert(output_nodes.end(), layer.node_ids.begin(), layer.node_ids.end());
        }
    }
    return output_nodes;
}

std::vector<std::string> NeuralNetworkPath::getHiddenNodes() const {
    std::vector<std::string> hidden_nodes;
    for (const auto& layer : layers_) {
        if (layer.layer_type == "hidden") {
            hidden_nodes.insert(hidden_nodes.end(), layer.node_ids.begin(), layer.node_ids.end());
        }
    }
    return hidden_nodes;
}

std::vector<NeuralConnection> NeuralNetworkPath::getConnectionsFrom(const std::string& node_id) const {
    auto it = node_connections_.find(node_id);
    if (it != node_connections_.end()) {
        return it->second;
    }
    return {};
}

std::vector<NeuralConnection> NeuralNetworkPath::getConnectionsTo(const std::string& node_id) const {
    std::vector<NeuralConnection> connections_to;
    for (const auto& connection : connections_) {
        if (connection.to_node == node_id && connection.is_active) {
            connections_to.push_back(connection);
        }
    }
    return connections_to;
}

int NeuralNetworkPath::getNodeLayer(const std::string& node_id) const {
    auto it = node_to_layer_.find(node_id);
    return (it != node_to_layer_.end()) ? it->second : -1;
}

std::string NeuralNetworkPath::getNodeType(const std::string& node_id) const {
    auto it = node_types_.find(node_id);
    return (it != node_types_.end()) ? it->second : "unknown";
}

bool NeuralNetworkPath::isInputNode(const std::string& node_id) const {
    return getNodeType(node_id) == "input";
}

bool NeuralNetworkPath::isOutputNode(const std::string& node_id) const {
    return getNodeType(node_id) == "output";
}

bool NeuralNetworkPath::isHiddenNode(const std::string& node_id) const {
    return getNodeType(node_id) == "hidden";
}

double NeuralNetworkPath::getConnectionWeight(const std::string& from_node, const std::string& to_node) const {
    for (const auto& connection : connections_) {
        if (connection.from_node == from_node && connection.to_node == to_node && connection.is_active) {
            return connection.weight;
        }
    }
    return 0.0;
}

bool NeuralNetworkPath::hasActiveConnection(const std::string& from_node, const std::string& to_node) const {
    return getConnectionWeight(from_node, to_node) != 0.0;
}

SearchAlgorithm::SuccessorFunction NeuralNetworkPath::getSuccessorFunction() const {
    return [this](const std::string& node_id) -> std::vector<std::pair<std::string, Node::CostType>> {
        std::vector<std::pair<std::string, Node::CostType>> successors;
        
        auto connections = getConnectionsFrom(node_id);
        for (const auto& connection : connections) {
            if (connection.is_active) {
                // Cost depends on objective
                double cost = 1.0; // Default cost
                
                switch (objective_) {
                    case MIN_LENGTH:
                        cost = 1.0; // Unit cost for path length
                        break;
                    case MAX_ACTIVATION:
                        cost = -std::abs(connection.weight); // Negative for maximization
                        break;
                    case MIN_COST:
                        cost = std::abs(connection.weight); // Weight as cost
                        break;
                    case MAX_DIVERSITY:
                        cost = 1.0; // Unit cost, diversity calculated in heuristic
                        break;
                    case BALANCED:
                        cost = 1.0 + std::abs(connection.weight); // Combined cost
                        break;
                }
                
                successors.emplace_back(connection.to_node, cost);
            }
        }
        
        return successors;
    };
}

SearchAlgorithm::HeuristicFunction NeuralNetworkPath::getHeuristicFunction() const {
    return getHeuristicFunction(objective_);
}

SearchAlgorithm::HeuristicFunction NeuralNetworkPath::getHeuristicFunction(Objective objective) const {
    return [this, objective](const std::string& current_node, const std::string& goal_node) -> Node::CostType {
        switch (objective) {
            case MIN_LENGTH:
                return layerBasedHeuristic(current_node, goal_node);
            case MAX_ACTIVATION:
                return activationBasedHeuristic(current_node, goal_node);
            case MIN_COST:
                return costBasedHeuristic(current_node, goal_node);
            case MAX_DIVERSITY:
                return layerBasedHeuristic(current_node, goal_node); // Use layer-based for diversity
            case BALANCED:
                return balancedHeuristic(current_node, goal_node);
            default:
                return layerBasedHeuristic(current_node, goal_node);
        }
    };
}

double NeuralNetworkPath::calculatePathActivation(const std::vector<std::string>& path) const {
    if (path.size() < 2) return 0.0;
    
    double activation = 1.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double weight = getConnectionWeight(path[i], path[i + 1]);
        activation *= std::abs(weight);
    }
    return activation;
}

double NeuralNetworkPath::calculatePathCost(const std::vector<std::string>& path) const {
    if (path.size() < 2) return 0.0;
    
    double cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double weight = getConnectionWeight(path[i], path[i + 1]);
        cost += std::abs(weight);
    }
    return cost;
}

double NeuralNetworkPath::calculatePathDiversity(const std::vector<std::string>& path) const {
    if (path.empty()) return 0.0;
    
    std::unordered_set<std::string> unique_layers;
    for (const auto& node : path) {
        int layer = getNodeLayer(node);
        if (layer >= 0) {
            unique_layers.insert("layer_" + std::to_string(layer));
        }
    }
    
    return static_cast<double>(unique_layers.size()) / static_cast<double>(layers_.size());
}

void NeuralNetworkPath::createFeedforwardNetwork(int input_size, const std::vector<int>& hidden_sizes, 
                                                int output_size, const std::pair<double, double>& weight_range) {
    // Clear existing network
    layers_.clear();
    connections_.clear();
    node_to_layer_.clear();
    node_types_.clear();
    node_connections_.clear();
    
    // Add input layer
    int input_layer = addLayer("input", "input", input_size);
    
    // Add hidden layers
    std::vector<int> hidden_layers;
    for (size_t i = 0; i < hidden_sizes.size(); ++i) {
        int layer = addLayer("hidden_" + std::to_string(i), "hidden", hidden_sizes[i]);
        hidden_layers.push_back(layer);
    }
    
    // Add output layer
    int output_layer = addLayer("output", "output", output_size);
    
    // Connect layers
    std::vector<int> all_layers = {input_layer};
    all_layers.insert(all_layers.end(), hidden_layers.begin(), hidden_layers.end());
    all_layers.push_back(output_layer);
    
    // Create connections between consecutive layers
    for (size_t i = 0; i < all_layers.size() - 1; ++i) {
        const auto& from_layer = layers_[all_layers[i]];
        const auto& to_layer = layers_[all_layers[i + 1]];
        
        for (const auto& from_node : from_layer.node_ids) {
            for (const auto& to_node : to_layer.node_ids) {
                double weight = getRandomWeight(weight_range);
                addConnection(from_node, to_node, weight);
            }
        }
    }
}

void NeuralNetworkPath::createRecurrentNetwork(int input_size, int hidden_size, int output_size,
                                              const std::pair<double, double>& weight_range) {
    // Clear existing network
    layers_.clear();
    connections_.clear();
    node_to_layer_.clear();
    node_types_.clear();
    node_connections_.clear();
    
    // Add layers
    int input_layer = addLayer("input", "input", input_size);
    int hidden_layer = addLayer("hidden", "hidden", hidden_size);
    int output_layer = addLayer("output", "output", output_size);
    
    const auto& input_nodes = layers_[input_layer].node_ids;
    const auto& hidden_nodes = layers_[hidden_layer].node_ids;
    const auto& output_nodes = layers_[output_layer].node_ids;
    
    // Input to hidden connections
    for (const auto& input_node : input_nodes) {
        for (const auto& hidden_node : hidden_nodes) {
            double weight = getRandomWeight(weight_range);
            addConnection(input_node, hidden_node, weight);
        }
    }
    
    // Hidden to hidden recurrent connections
    for (const auto& hidden_node1 : hidden_nodes) {
        for (const auto& hidden_node2 : hidden_nodes) {
            double weight = getRandomWeight(weight_range);
            addConnection(hidden_node1, hidden_node2, weight);
        }
    }
    
    // Hidden to output connections
    for (const auto& hidden_node : hidden_nodes) {
        for (const auto& output_node : output_nodes) {
            double weight = getRandomWeight(weight_range);
            addConnection(hidden_node, output_node, weight);
        }
    }
}

bool NeuralNetworkPath::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Clear existing network
    layers_.clear();
    connections_.clear();
    node_to_layer_.clear();
    node_types_.clear();
    node_connections_.clear();
    
    std::string line;
    
    // Read network name
    if (std::getline(file, line)) {
        network_name_ = line;
    }
    
    // Read layers
    while (std::getline(file, line) && line != "CONNECTIONS") {
        std::istringstream iss(line);
        std::string layer_id, layer_type;
        int node_count;
        
        if (iss >> layer_id >> layer_type >> node_count) {
            addLayer(layer_id, layer_type, node_count);
        }
    }
    
    // Read connections
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string from_node, to_node;
        double weight;
        bool is_active;
        
        if (iss >> from_node >> to_node >> weight >> is_active) {
            addConnection(from_node, to_node, weight, is_active);
        }
    }
    
    return true;
}

bool NeuralNetworkPath::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Write network name
    file << network_name_ << std::endl;
    
    // Write layers
    for (const auto& layer : layers_) {
        file << layer.layer_id << " " << layer.layer_type << " " << layer.node_ids.size() << std::endl;
    }
    
    // Write connections
    file << "CONNECTIONS" << std::endl;
    for (const auto& connection : connections_) {
        file << connection.from_node << " " << connection.to_node << " " 
             << connection.weight << " " << connection.is_active << std::endl;
    }
    
    return true;
}

void NeuralNetworkPath::printNetwork() const {
    std::cout << "Neural Network: " << network_name_ << std::endl;
    std::cout << "========================" << std::endl;
    
    for (const auto& layer : layers_) {
        std::cout << "Layer " << layer.layer_index << " (" << layer.layer_type << "): ";
        for (const auto& node : layer.node_ids) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nConnections:" << std::endl;
    for (const auto& connection : connections_) {
        if (connection.is_active) {
            std::cout << connection.from_node << " -> " << connection.to_node 
                     << " (weight: " << connection.weight << ")" << std::endl;
        }
    }
}

std::unordered_map<std::string, int> NeuralNetworkPath::getNetworkStats() const {
    std::unordered_map<std::string, int> stats;
    
    stats["total_layers"] = static_cast<int>(layers_.size());
    stats["total_nodes"] = static_cast<int>(getAllNodes().size());
    stats["input_nodes"] = static_cast<int>(getInputNodes().size());
    stats["hidden_nodes"] = static_cast<int>(getHiddenNodes().size());
    stats["output_nodes"] = static_cast<int>(getOutputNodes().size());
    stats["total_connections"] = static_cast<int>(connections_.size());
    
    int active_connections = 0;
    for (const auto& connection : connections_) {
        if (connection.is_active) {
            ++active_connections;
        }
    }
    stats["active_connections"] = active_connections;
    
    return stats;
}

std::vector<std::string> NeuralNetworkPath::generateNodeIds(const std::string& layer_id, int node_count) const {
    std::vector<std::string> node_ids;
    for (int i = 0; i < node_count; ++i) {
        node_ids.push_back(layer_id + "_" + std::to_string(i));
    }
    return node_ids;
}

double NeuralNetworkPath::layerBasedHeuristic(const std::string& current_node, const std::string& goal_node) const {
    int current_layer = getNodeLayer(current_node);
    int goal_layer = getNodeLayer(goal_node);
    
    if (current_layer == -1 || goal_layer == -1) {
        return 0.0;
    }
    
    return std::abs(goal_layer - current_layer);
}

double NeuralNetworkPath::activationBasedHeuristic(const std::string& current_node, const std::string& goal_node) const {
    // Estimate activation strength based on layer distance and average weights
    double layer_distance = layerBasedHeuristic(current_node, goal_node);
    
    // Calculate average connection weight
    double total_weight = 0.0;
    int connection_count = 0;
    
    for (const auto& connection : connections_) {
        if (connection.is_active) {
            total_weight += std::abs(connection.weight);
            ++connection_count;
        }
    }
    
    double avg_weight = (connection_count > 0) ? total_weight / connection_count : 1.0;
    
    // Heuristic: layer distance * average weight
    return layer_distance * avg_weight;
}

double NeuralNetworkPath::costBasedHeuristic(const std::string& current_node, const std::string& goal_node) const {
    // Estimate computational cost based on layer distance
    double layer_distance = layerBasedHeuristic(current_node, goal_node);
    
    // Add cost for each layer that needs to be traversed
    double cost = layer_distance;
    
    // Add additional cost for hidden layers (more computation)
    int current_layer = getNodeLayer(current_node);
    int goal_layer = getNodeLayer(goal_node);
    
    for (int layer = current_layer; layer <= goal_layer; ++layer) {
        if (layer >= 0 && layer < static_cast<int>(layers_.size()) && 
            layers_[layer].layer_type == "hidden") {
            cost += 0.5; // Additional cost for hidden layers
        }
    }
    
    return cost;
}

double NeuralNetworkPath::balancedHeuristic(const std::string& current_node, const std::string& goal_node) const {
    // Combine layer distance and activation considerations
    double layer_h = layerBasedHeuristic(current_node, goal_node);
    double activation_h = activationBasedHeuristic(current_node, goal_node);
    
    // Weighted combination
    return 0.7 * layer_h + 0.3 * activation_h;
}

double NeuralNetworkPath::getRandomWeight(const std::pair<double, double>& range) const {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(range.first, range.second);
    return dis(gen);
}

} // namespace heuristic_search 
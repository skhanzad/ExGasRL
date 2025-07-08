#include <iostream>
#include <iomanip>
#include <vector>
#include "neural_network_path.h"
#include "astar.h"
#include "dijkstra.h"
#include "best_first.h"
#include "ida_star.h"

using namespace heuristic_search;

void printSearchResult(const SearchResult& result, const std::string& algorithm_name) {
    std::cout << "\n=== " << algorithm_name << " Results ===" << std::endl;
    std::cout << "Success: " << (result.success ? "Yes" : "No") << std::endl;
    
    if (result.success) {
        std::cout << "Path Cost: " << result.path_cost << std::endl;
        std::cout << "Path Length: " << result.path.size() << std::endl;
        std::cout << "Path: ";
        for (const auto& action : result.path) {
            std::cout << action << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "Nodes Expanded: " << result.nodes_expanded << std::endl;
    std::cout << "Nodes Generated: " << result.nodes_generated << std::endl;
    std::cout << "Execution Time: " << std::fixed << std::setprecision(3) 
              << result.execution_time_ms << " ms" << std::endl;
}

void testFeedforwardNetwork() {
    std::cout << "\n=== Feedforward Neural Network Pathfinding ===" << std::endl;
    
    // Create a feedforward network: 3 input -> 4 hidden -> 5 hidden -> 2 output
    NeuralNetworkPath network("feedforward_example");
    network.createFeedforwardNetwork(3, {4, 5}, 2, {-1.0, 1.0});
    
    // Print network structure
    network.printNetwork();
    
    // Get input and output nodes
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    
    std::cout << "\nInput nodes: ";
    for (const auto& node : input_nodes) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Output nodes: ";
    for (const auto& node : output_nodes) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
    
    // Test different objectives
    std::vector<std::pair<NeuralNetworkPath::Objective, std::string>> objectives = {
        {NeuralNetworkPath::MIN_LENGTH, "Minimum Path Length"},
        {NeuralNetworkPath::MAX_ACTIVATION, "Maximum Activation"},
        {NeuralNetworkPath::MIN_COST, "Minimum Computational Cost"},
        {NeuralNetworkPath::BALANCED, "Balanced Objective"}
    };
    
    for (const auto& [objective, name] : objectives) {
        std::cout << "\n--- Testing " << name << " ---" << std::endl;
        
        network.setObjective(objective);
        auto successor_func = network.getSuccessorFunction();
        auto heuristic_func = network.getHeuristicFunction();
        
        // Test A* algorithm
        AStar astar(heuristic_func, successor_func);
        auto result = astar.search(input_nodes[0], output_nodes[0]);
        
        if (result.success) {
            std::cout << "Path found from " << input_nodes[0] << " to " << output_nodes[0] << std::endl;
            
            // Calculate path metrics
            std::vector<std::string> path_nodes = {input_nodes[0]};
            for (const auto& action : result.path) {
                path_nodes.push_back(action);
            }
            
            double activation = network.calculatePathActivation(path_nodes);
            double cost = network.calculatePathCost(path_nodes);
            double diversity = network.calculatePathDiversity(path_nodes);
            
            std::cout << "Path Activation: " << std::fixed << std::setprecision(4) << activation << std::endl;
            std::cout << "Path Cost: " << cost << std::endl;
            std::cout << "Path Diversity: " << diversity << std::endl;
        } else {
            std::cout << "No path found!" << std::endl;
        }
        
        printSearchResult(result, "A* (" + name + ")");
    }
}

void testRecurrentNetwork() {
    std::cout << "\n=== Recurrent Neural Network Pathfinding ===" << std::endl;
    
    // Create a recurrent network: 2 input -> 3 hidden -> 1 output
    NeuralNetworkPath network("recurrent_example");
    network.createRecurrentNetwork(2, 3, 1, {-0.5, 0.5});
    
    // Print network structure
    network.printNetwork();
    
    // Get input and output nodes
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    
    std::cout << "\nTesting pathfinding in recurrent network..." << std::endl;
    
    // Test with different algorithms
    network.setObjective(NeuralNetworkPath::BALANCED);
    auto successor_func = network.getSuccessorFunction();
    auto heuristic_func = network.getHeuristicFunction();
    
    std::vector<std::pair<std::string, std::unique_ptr<SearchAlgorithm>>> algorithms;
    
    algorithms.emplace_back("A*", std::make_unique<AStar>(heuristic_func, successor_func));
    algorithms.emplace_back("Dijkstra", std::make_unique<Dijkstra>(successor_func));
    algorithms.emplace_back("Best-First", std::make_unique<BestFirst>(heuristic_func, successor_func));
    algorithms.emplace_back("IDA*", std::make_unique<IDAStar>(heuristic_func, successor_func));
    
    for (const auto& [name, algorithm] : algorithms) {
        std::cout << "\n--- Testing " << name << " ---" << std::endl;
        
        auto result = algorithm->search(input_nodes[0], output_nodes[0]);
        
        if (result.success) {
            std::vector<std::string> path_nodes = {input_nodes[0]};
            for (const auto& action : result.path) {
                path_nodes.push_back(action);
            }
            
            double activation = network.calculatePathActivation(path_nodes);
            double cost = network.calculatePathCost(path_nodes);
            
            std::cout << "Path found with activation: " << activation << ", cost: " << cost << std::endl;
        }
        
        printSearchResult(result, name);
    }
}

void testCustomNetwork() {
    std::cout << "\n=== Custom Neural Network Pathfinding ===" << std::endl;
    
    // Create a custom network manually
    NeuralNetworkPath network("custom_example");
    
    // Add layers
    int input_layer = network.addLayer("input", "input", 2);
    int hidden1_layer = network.addLayer("hidden1", "hidden", 3);
    int hidden2_layer = network.addLayer("hidden2", "hidden", 2);
    int output_layer = network.addLayer("output", "output", 1);
    
    // Add connections with specific weights
    network.addConnection("input_0", "hidden1_0", 0.8);
    network.addConnection("input_0", "hidden1_1", 0.3);
    network.addConnection("input_0", "hidden1_2", -0.5);
    network.addConnection("input_1", "hidden1_0", 0.2);
    network.addConnection("input_1", "hidden1_1", 0.9);
    network.addConnection("input_1", "hidden1_2", 0.1);
    
    network.addConnection("hidden1_0", "hidden2_0", 0.7);
    network.addConnection("hidden1_0", "hidden2_1", 0.4);
    network.addConnection("hidden1_1", "hidden2_0", -0.2);
    network.addConnection("hidden1_1", "hidden2_1", 0.6);
    network.addConnection("hidden1_2", "hidden2_0", 0.3);
    network.addConnection("hidden1_2", "hidden2_1", -0.8);
    
    network.addConnection("hidden2_0", "output_0", 0.9);
    network.addConnection("hidden2_1", "output_0", -0.4);
    
    // Print network
    network.printNetwork();
    
    // Test pathfinding with different objectives
    std::vector<NeuralNetworkPath::Objective> objectives = {
        NeuralNetworkPath::MIN_LENGTH,
        NeuralNetworkPath::MAX_ACTIVATION,
        NeuralNetworkPath::MIN_COST
    };
    
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    
    for (auto objective : objectives) {
        std::cout << "\n--- Testing Objective " << objective << " ---" << std::endl;
        
        network.setObjective(objective);
        auto successor_func = network.getSuccessorFunction();
        auto heuristic_func = network.getHeuristicFunction();
        
        AStar astar(heuristic_func, successor_func);
        auto result = astar.search(input_nodes[0], output_nodes[0]);
        
        if (result.success) {
            std::vector<std::string> path_nodes = {input_nodes[0]};
            for (const auto& action : result.path) {
                path_nodes.push_back(action);
            }
            
            double activation = network.calculatePathActivation(path_nodes);
            double cost = network.calculatePathCost(path_nodes);
            
            std::cout << "Path found!" << std::endl;
            std::cout << "Activation: " << activation << std::endl;
            std::cout << "Cost: " << cost << std::endl;
        }
        
        printSearchResult(result, "A* (Objective " + std::to_string(objective) + ")");
    }
}

void testNetworkAnalysis() {
    std::cout << "\n=== Neural Network Analysis ===" << std::endl;
    
    // Create a larger network for analysis
    NeuralNetworkPath network("analysis_example");
    network.createFeedforwardNetwork(5, {8, 6, 4}, 3, {-1.0, 1.0});
    
    // Get network statistics
    auto stats = network.getNetworkStats();
    
    std::cout << "Network Statistics:" << std::endl;
    for (const auto& [key, value] : stats) {
        std::cout << "  " << key << ": " << value << std::endl;
    }
    
    // Analyze all input-to-output paths
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    
    std::cout << "\nAnalyzing all input-to-output paths..." << std::endl;
    
    network.setObjective(NeuralNetworkPath::MAX_ACTIVATION);
    auto successor_func = network.getSuccessorFunction();
    auto heuristic_func = network.getHeuristicFunction();
    
    AStar astar(heuristic_func, successor_func);
    
    std::vector<double> activations;
    std::vector<double> costs;
    std::vector<int> path_lengths;
    
    for (const auto& input_node : input_nodes) {
        for (const auto& output_node : output_nodes) {
            auto result = astar.search(input_node, output_node);
            
            if (result.success) {
                std::vector<std::string> path_nodes = {input_node};
                for (const auto& action : result.path) {
                    path_nodes.push_back(action);
                }
                
                double activation = network.calculatePathActivation(path_nodes);
                double cost = network.calculatePathCost(path_nodes);
                
                activations.push_back(activation);
                costs.push_back(cost);
                path_lengths.push_back(static_cast<int>(result.path.size()));
                
                std::cout << "Path " << input_node << " -> " << output_node 
                         << ": activation=" << activation 
                         << ", cost=" << cost 
                         << ", length=" << result.path.size() << std::endl;
            }
        }
    }
    
    // Calculate statistics
    if (!activations.empty()) {
        double avg_activation = 0.0, avg_cost = 0.0, avg_length = 0.0;
        double max_activation = activations[0], min_activation = activations[0];
        
        for (size_t i = 0; i < activations.size(); ++i) {
            avg_activation += activations[i];
            avg_cost += costs[i];
            avg_length += path_lengths[i];
            
            max_activation = std::max(max_activation, activations[i]);
            min_activation = std::min(min_activation, activations[i]);
        }
        
        avg_activation /= activations.size();
        avg_cost /= costs.size();
        avg_length /= path_lengths.size();
        
        std::cout << "\nPath Analysis Summary:" << std::endl;
        std::cout << "  Average Activation: " << avg_activation << std::endl;
        std::cout << "  Max Activation: " << max_activation << std::endl;
        std::cout << "  Min Activation: " << min_activation << std::endl;
        std::cout << "  Average Cost: " << avg_cost << std::endl;
        std::cout << "  Average Path Length: " << avg_length << std::endl;
        std::cout << "  Total Paths Found: " << activations.size() << std::endl;
    }
}

int main() {
    std::cout << "Neural Network Pathfinding Examples" << std::endl;
    std::cout << "===================================" << std::endl;
    
    try {
        // Test feedforward network
        testFeedforwardNetwork();
        
        // Test recurrent network
        testRecurrentNetwork();
        
        // Test custom network
        testCustomNetwork();
        
        // Test network analysis
        testNetworkAnalysis();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\nNeural network pathfinding examples completed successfully!" << std::endl;
    return 0;
} 
#include <iostream>
#include <cassert>
#include <vector>
#include "neural_network_path.h"
#include "astar.h"
#include "dijkstra.h"

using namespace heuristic_search;

void testNetworkCreation() {
    std::cout << "Testing network creation..." << std::endl;
    
    NeuralNetworkPath network("test_network");
    
    // Test adding layers
    int input_layer = network.addLayer("input", "input", 3);
    int hidden_layer = network.addLayer("hidden", "hidden", 4);
    int output_layer = network.addLayer("output", "output", 2);
    
    assert(input_layer == 0);
    assert(hidden_layer == 1);
    assert(output_layer == 2);
    
    // Test node retrieval
    auto all_nodes = network.getAllNodes();
    assert(all_nodes.size() == 9); // 3 + 4 + 2
    
    auto input_nodes = network.getInputNodes();
    assert(input_nodes.size() == 3);
    
    auto output_nodes = network.getOutputNodes();
    assert(output_nodes.size() == 2);
    
    auto hidden_nodes = network.getHiddenNodes();
    assert(hidden_nodes.size() == 4);
    
    std::cout << "✓ Network creation tests passed" << std::endl;
}

void testConnections() {
    std::cout << "Testing connections..." << std::endl;
    
    NeuralNetworkPath network("test_network");
    
    // Add layers
    network.addLayer("input", "input", 2);
    network.addLayer("hidden", "hidden", 3);
    network.addLayer("output", "output", 1);
    
    // Add connections
    network.addConnection("input_0", "hidden_0", 0.5);
    network.addConnection("input_0", "hidden_1", 0.3);
    network.addConnection("input_1", "hidden_1", 0.7);
    network.addConnection("input_1", "hidden_2", 0.2);
    network.addConnection("hidden_0", "output_0", 0.8);
    network.addConnection("hidden_1", "output_0", 0.4);
    network.addConnection("hidden_2", "output_0", 0.6);
    
    // Test connection retrieval
    auto connections_from_input0 = network.getConnectionsFrom("input_0");
    assert(connections_from_input0.size() == 2);
    
    auto connections_to_output0 = network.getConnectionsTo("output_0");
    assert(connections_to_output0.size() == 3);
    
    // Test connection weights
    assert(network.getConnectionWeight("input_0", "hidden_0") == 0.5);
    assert(network.getConnectionWeight("input_0", "hidden_1") == 0.3);
    assert(network.getConnectionWeight("nonexistent", "node") == 0.0);
    
    // Test active connections
    assert(network.hasActiveConnection("input_0", "hidden_0"));
    assert(!network.hasActiveConnection("input_0", "hidden_2"));
    
    std::cout << "✓ Connection tests passed" << std::endl;
}

void testNodeProperties() {
    std::cout << "Testing node properties..." << std::endl;
    
    NeuralNetworkPath network("test_network");
    
    // Add layers
    network.addLayer("input", "input", 2);
    network.addLayer("hidden", "hidden", 3);
    network.addLayer("output", "output", 1);
    
    // Test node types
    assert(network.getNodeType("input_0") == "input");
    assert(network.getNodeType("hidden_1") == "hidden");
    assert(network.getNodeType("output_0") == "output");
    assert(network.getNodeType("nonexistent") == "unknown");
    
    // Test node layers
    assert(network.getNodeLayer("input_0") == 0);
    assert(network.getNodeLayer("hidden_1") == 1);
    assert(network.getNodeLayer("output_0") == 2);
    assert(network.getNodeLayer("nonexistent") == -1);
    
    // Test node type checks
    assert(network.isInputNode("input_0"));
    assert(!network.isInputNode("hidden_0"));
    
    assert(network.isHiddenNode("hidden_1"));
    assert(!network.isHiddenNode("input_0"));
    
    assert(network.isOutputNode("output_0"));
    assert(!network.isOutputNode("hidden_0"));
    
    std::cout << "✓ Node property tests passed" << std::endl;
}

void testPathCalculations() {
    std::cout << "Testing path calculations..." << std::endl;
    
    NeuralNetworkPath network("test_network");
    
    // Create a simple network
    network.addLayer("input", "input", 1);
    network.addLayer("hidden", "hidden", 2);
    network.addLayer("output", "output", 1);
    
    network.addConnection("input_0", "hidden_0", 0.5);
    network.addConnection("input_0", "hidden_1", 0.3);
    network.addConnection("hidden_0", "output_0", 0.8);
    network.addConnection("hidden_1", "output_0", 0.4);
    
    // Test path activation
    std::vector<std::string> path1 = {"input_0", "hidden_0", "output_0"};
    double activation1 = network.calculatePathActivation(path1);
    assert(activation1 == 0.5 * 0.8);
    
    // Test path cost
    double cost1 = network.calculatePathCost(path1);
    assert(cost1 == 0.5 + 0.8);
    
    // Test path diversity
    double diversity1 = network.calculatePathDiversity(path1);
    assert(diversity1 == 3.0 / 3.0); // All layers visited
    
    std::cout << "✓ Path calculation tests passed" << std::endl;
}

void testFeedforwardNetwork() {
    std::cout << "Testing feedforward network creation..." << std::endl;
    
    NeuralNetworkPath network("feedforward_test");
    network.createFeedforwardNetwork(2, {3, 4}, 1, {-1.0, 1.0});
    
    // Check structure
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    auto hidden_nodes = network.getHiddenNodes();
    
    assert(input_nodes.size() == 2);
    assert(output_nodes.size() == 1);
    assert(hidden_nodes.size() == 7); // 3 + 4
    
    // Check connections exist
    assert(network.hasActiveConnection("input_0", "hidden_0_0"));
    assert(network.hasActiveConnection("hidden_0_0", "hidden_1_0"));
    assert(network.hasActiveConnection("hidden_1_0", "output_0"));
    
    std::cout << "✓ Feedforward network tests passed" << std::endl;
}

void testRecurrentNetwork() {
    std::cout << "Testing recurrent network creation..." << std::endl;
    
    NeuralNetworkPath network("recurrent_test");
    network.createRecurrentNetwork(2, 3, 1, {-0.5, 0.5});
    
    // Check structure
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    auto hidden_nodes = network.getHiddenNodes();
    
    assert(input_nodes.size() == 2);
    assert(output_nodes.size() == 1);
    assert(hidden_nodes.size() == 3);
    
    // Check connections exist
    assert(network.hasActiveConnection("input_0", "hidden_0"));
    assert(network.hasActiveConnection("hidden_0", "hidden_1")); // Recurrent
    assert(network.hasActiveConnection("hidden_0", "output_0"));
    
    std::cout << "✓ Recurrent network tests passed" << std::endl;
}

void testSearchAlgorithms() {
    std::cout << "Testing search algorithms..." << std::endl;
    
    NeuralNetworkPath network("search_test");
    
    // Create a simple network for testing
    network.addLayer("input", "input", 1);
    network.addLayer("hidden", "hidden", 2);
    network.addLayer("output", "output", 1);
    
    network.addConnection("input_0", "hidden_0", 0.5);
    network.addConnection("input_0", "hidden_1", 0.3);
    network.addConnection("hidden_0", "output_0", 0.8);
    network.addConnection("hidden_1", "output_0", 0.4);
    
    auto input_nodes = network.getInputNodes();
    auto output_nodes = network.getOutputNodes();
    
    // Test different objectives
    std::vector<NeuralNetworkPath::Objective> objectives = {
        NeuralNetworkPath::MIN_LENGTH,
        NeuralNetworkPath::MAX_ACTIVATION,
        NeuralNetworkPath::MIN_COST,
        NeuralNetworkPath::BALANCED
    };
    
    for (auto objective : objectives) {
        network.setObjective(objective);
        auto successor_func = network.getSuccessorFunction();
        auto heuristic_func = network.getHeuristicFunction();
        
        // Test A*
        AStar astar(heuristic_func, successor_func);
        auto result = astar.search(input_nodes[0], output_nodes[0]);
        
        assert(result.success);
        assert(result.path.size() > 0);
        assert(result.nodes_expanded > 0);
        assert(result.nodes_generated > 0);
        
        // Test Dijkstra
        Dijkstra dijkstra(successor_func);
        auto dijkstra_result = dijkstra.search(input_nodes[0], output_nodes[0]);
        
        assert(dijkstra_result.success);
        assert(dijkstra_result.path.size() > 0);
    }
    
    std::cout << "✓ Search algorithm tests passed" << std::endl;
}

void testFileIO() {
    std::cout << "Testing file I/O..." << std::endl;
    
    NeuralNetworkPath network("file_test");
    
    // Create a network
    network.addLayer("input", "input", 2);
    network.addLayer("hidden", "hidden", 3);
    network.addLayer("output", "output", 1);
    
    network.addConnection("input_0", "hidden_0", 0.5);
    network.addConnection("input_1", "hidden_1", 0.7);
    network.addConnection("hidden_0", "output_0", 0.8);
    network.addConnection("hidden_1", "output_0", 0.4);
    network.addConnection("hidden_2", "output_0", 0.6);
    
    // Save to file
    bool save_success = network.saveToFile("test_network.txt");
    assert(save_success);
    
    // Load from file
    NeuralNetworkPath loaded_network("loaded_test");
    bool load_success = loaded_network.loadFromFile("test_network.txt");
    assert(load_success);
    
    // Verify loaded network
    auto original_nodes = network.getAllNodes();
    auto loaded_nodes = loaded_network.getAllNodes();
    assert(original_nodes.size() == loaded_nodes.size());
    
    // Check that connections are preserved
    assert(loaded_network.hasActiveConnection("input_0", "hidden_0"));
    assert(loaded_network.getConnectionWeight("input_0", "hidden_0") == 0.5);
    
    std::cout << "✓ File I/O tests passed" << std::endl;
}

void testNetworkStats() {
    std::cout << "Testing network statistics..." << std::endl;
    
    NeuralNetworkPath network("stats_test");
    network.createFeedforwardNetwork(3, {4, 5}, 2, {-1.0, 1.0});
    
    auto stats = network.getNetworkStats();
    
    assert(stats["total_layers"] == 4); // input + 2 hidden + output
    assert(stats["total_nodes"] == 14); // 3 + 4 + 5 + 2
    assert(stats["input_nodes"] == 3);
    assert(stats["hidden_nodes"] == 9); // 4 + 5
    assert(stats["output_nodes"] == 2);
    assert(stats["total_connections"] > 0);
    assert(stats["active_connections"] > 0);
    
    std::cout << "✓ Network statistics tests passed" << std::endl;
}

int main() {
    std::cout << "Running Neural Network Pathfinding Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        testNetworkCreation();
        testConnections();
        testNodeProperties();
        testPathCalculations();
        testFeedforwardNetwork();
        testRecurrentNetwork();
        testSearchAlgorithms();
        testFileIO();
        testNetworkStats();
        
        std::cout << "\n🎉 All neural network pathfinding tests passed!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Test failed with unknown exception" << std::endl;
        return 1;
    }
} 
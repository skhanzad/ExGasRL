# Heuristic Search Framework

A comprehensive C++ framework for implementing and comparing various heuristic search algorithms. This framework provides a flexible and extensible architecture for solving pathfinding and optimization problems.

## Features

### Search Algorithms
- **A*** - Optimal informed search with heuristic guidance
- **Dijkstra's Algorithm** - Optimal uninformed search for shortest paths
- **Best-First Search** - Informed search using heuristic values
- **IDA*** - Memory-efficient iterative deepening A*
- **Greedy Best-First Search** - Fast but potentially suboptimal search

### Problem Domains
- **GridWorld** - 2D grid-based pathfinding with obstacles and terrain costs
- **PuzzleState** - Sliding puzzle problems (8-puzzle, 15-puzzle, etc.)
- **NeuralNetworkPath** - Neural network pathfinding from input to output nodes

### Heuristic Functions
- Manhattan Distance
- Euclidean Distance
- Hamming Distance
- Misplaced Tiles (for puzzles)
- Linear Conflicts (for puzzles)
- Zero Heuristic (for Dijkstra's)

## Architecture

### Core Components

#### Node Class
```cpp
class Node {
    std::string state_;      // State representation
    double g_cost_;          // Cost from start to this node
    double h_cost_;          // Heuristic cost to goal
    NodePtr parent_;         // Parent node for path reconstruction
    std::string action_;     // Action that led to this node
};
```

#### SearchAlgorithm Base Class
```cpp
class SearchAlgorithm {
    HeuristicFunction heuristic_;    // Heuristic function
    SuccessorFunction successor_;    // Successor generation function
    size_t max_nodes_;              // Maximum nodes to expand
    
    virtual SearchResult search(const std::string& start, const std::string& goal);
};
```

#### SearchResult Structure
```cpp
struct SearchResult {
    bool success;                    // Whether a path was found
    std::vector<std::string> path;  // Sequence of actions
    double path_cost;               // Total path cost
    size_t nodes_expanded;          // Number of nodes expanded
    size_t nodes_generated;         // Number of nodes generated
    double execution_time_ms;       // Execution time in milliseconds
};
```

## Building the Framework

### Prerequisites
- CMake 3.16 or higher
- C++17 compatible compiler
- Make or Ninja build system

### Build Instructions
```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build the project
make

# Run tests
make test

# Run examples
./bin/search_example
./bin/neural_network_example
```

## Usage Examples

### Grid World Pathfinding

```cpp
#include "grid_world.h"
#include "astar.h"

// Create a grid world
GridWorld world(10, 10);

// Add obstacles
world.setCell(3, 3, GridWorld::WALL);
world.setCell(4, 4, GridWorld::WATER);

// Get functions for search
auto successor_func = world.getSuccessorFunction();
auto heuristic = world.getManhattanHeuristic();

// Create A* algorithm
AStar astar(heuristic, successor_func);

// Perform search
auto result = astar.search("0,0", "9,9");

if (result.success) {
    std::cout << "Path found! Cost: " << result.path_cost << std::endl;
    for (const auto& action : result.path) {
        std::cout << action << " ";
    }
}
```

### 8-Puzzle Solving

```cpp
#include "puzzle_state.h"
#include "astar.h"

// Create puzzle states
PuzzleState start_state = PuzzleState::createRandomState(3, 20);
PuzzleState goal_state = PuzzleState::createSolvedState(3);

// Get functions
auto successor_func = PuzzleState::getSuccessorFunction();
auto heuristic = PuzzleState::getManhattanHeuristic();

// Solve with A*
AStar astar(heuristic, successor_func);
auto result = astar.search(start_state.toString(), goal_state.toString());

if (result.success) {
    std::cout << "Puzzle solved in " << result.path.size() << " moves" << std::endl;
}
```

### Neural Network Pathfinding

```cpp
#include "neural_network_path.h"
#include "astar.h"

// Create a neural network
NeuralNetworkPath network("my_network");

// Create a feedforward network: 3 input -> 4 hidden -> 2 output
network.createFeedforwardNetwork(3, {4}, 2, {-1.0, 1.0});

// Set optimization objective
network.setObjective(NeuralNetworkPath::MAX_ACTIVATION);

// Get functions for search
auto successor_func = network.getSuccessorFunction();
auto heuristic_func = network.getHeuristicFunction();

// Create A* algorithm
AStar astar(heuristic_func, successor_func);

// Find path from input to output
auto input_nodes = network.getInputNodes();
auto output_nodes = network.getOutputNodes();
auto result = astar.search(input_nodes[0], output_nodes[0]);

if (result.success) {
    // Calculate path metrics
    std::vector<std::string> path_nodes = {input_nodes[0]};
    for (const auto& action : result.path) {
        path_nodes.push_back(action);
    }
    
    double activation = network.calculatePathActivation(path_nodes);
    double cost = network.calculatePathCost(path_nodes);
    
    std::cout << "Path found with activation: " << activation << std::endl;
    std::cout << "Path cost: " << cost << std::endl;
}
```

### Custom Problem Domain

```cpp
// Define your own successor function
auto my_successor = [](const std::string& state) -> std::vector<std::pair<std::string, double>> {
    // Generate successors for your problem
    return {{"successor1", 1.0}, {"successor2", 2.0}};
};

// Define your own heuristic function
auto my_heuristic = [](const std::string& state, const std::string& goal) -> double {
    // Estimate cost from state to goal
    return 0.0; // Replace with actual heuristic
};

// Use with any search algorithm
AStar astar(my_heuristic, my_successor);
auto result = astar.search("start", "goal");
```

## Algorithm Comparison

| Algorithm | Optimal | Complete | Memory | Time | Use Case |
|-----------|---------|----------|--------|------|----------|
| A* | Yes | Yes | O(b^d) | O(b^d) | General optimal search |
| Dijkstra | Yes | Yes | O(V) | O(V log V) | Shortest paths |
| Best-First | No | Yes | O(b^d) | O(b^d) | Fast approximate solutions |
| Greedy Best-First | No | No | O(b^d) | O(b^d) | Very fast, may not find solution |
| IDA* | Yes | Yes | O(d) | O(b^d) | Memory-constrained environments |

## Neural Network Pathfinding Features

### Network Types
- **Feedforward Networks**: Standard layered networks with forward connections
- **Recurrent Networks**: Networks with feedback connections
- **Custom Networks**: Manually defined layer structures and connections

### Optimization Objectives
- **MIN_LENGTH**: Find shortest path from input to output
- **MAX_ACTIVATION**: Find path with maximum activation strength
- **MIN_COST**: Find path with minimum computational cost
- **MAX_DIVERSITY**: Find path that visits diverse layers
- **BALANCED**: Balance between path length and activation

### Path Analysis
- **Activation Strength**: Product of connection weights along path
- **Computational Cost**: Sum of absolute connection weights
- **Path Diversity**: Ratio of unique layers visited to total layers
- **Network Statistics**: Comprehensive analysis of network structure

### File I/O Support
- Save/load network structures to/from text files
- Preserve layer structure, connections, and weights
- Support for network sharing and persistence

## Performance Considerations

### Memory Usage
- **A*** and **Best-First**: Store all expanded nodes in memory
- **IDA***: Only stores current path, very memory efficient
- **Dijkstra**: Stores all visited nodes with distances

### Time Complexity
- All algorithms: O(b^d) in worst case
- **A*** with good heuristic: Much better than uninformed search
- **IDA***: May re-expand nodes, but uses minimal memory

### Heuristic Quality
- **Admissible**: Never overestimates (guarantees optimality for A*)
- **Consistent**: Monotonic (A* never re-expands nodes)
- **Better heuristics**: Reduce search space significantly

## Extending the Framework

### Adding New Search Algorithms

1. Inherit from `SearchAlgorithm`
2. Override the `search` method
3. Implement your algorithm logic

```cpp
class MySearchAlgorithm : public SearchAlgorithm {
public:
    MySearchAlgorithm(HeuristicFunction h, SuccessorFunction s)
        : SearchAlgorithm(h, s) {}
    
    SearchResult search(const std::string& start, const std::string& goal) override {
        // Your algorithm implementation
    }
};
```

### Adding New Problem Domains

1. Create a class for your problem
2. Implement state representation
3. Provide successor and heuristic functions

```cpp
class MyProblem {
public:
    static SuccessorFunction getSuccessorFunction();
    static HeuristicFunction getHeuristic();
    std::string toString() const;
    static MyProblem fromString(const std::string& str);
};
```

### Adding New Heuristics

```cpp
// In heuristics namespace
Node::CostType myHeuristic(const std::string& state, const std::string& goal) {
    // Your heuristic implementation
    return estimated_cost;
}
```

## Testing

The framework includes comprehensive tests for all components:

```bash
# Run all tests
make test

# Run specific test
./bin/search_tests
```

Tests cover:
- Node operations and path reconstruction
- Grid world functionality
- Puzzle state operations
- Neural network pathfinding
- All search algorithms
- Heuristic functions
- Edge cases and error conditions

## Examples

The framework includes several example programs:

- **Grid World Pathfinding**: Demonstrates pathfinding in 2D grids
- **8-Puzzle Solving**: Shows how to solve sliding puzzles
- **Neural Network Pathfinding**: Demonstrates finding paths through neural networks
- **Performance Comparison**: Compares different algorithms
- **Custom Problems**: Shows how to implement new problem domains

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- Russell, S., & Norvig, P. (2010). Artificial Intelligence: A Modern Approach
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths
- Korf, R. E. (1985). Depth-first iterative-deepening: An optimal admissible tree search 
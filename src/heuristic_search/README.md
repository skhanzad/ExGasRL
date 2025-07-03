# Heuristic Search Algorithms in Rust

A comprehensive implementation of search algorithms including A*, Breadth-First Search, and Depth-First Search, designed for gas optimization and general problem-solving.

## Features

- **A* Search**: Optimal pathfinding with heuristic guidance
- **Breadth-First Search**: Complete search for shortest path
- **Depth-First Search**: Memory-efficient search for deep solutions
- **Multiple Heuristics**: Manhattan, Euclidean, Zero, and Gas Optimization
- **Extensible Framework**: Easy to add new algorithms and heuristics
- **Performance Tracking**: Detailed statistics on search performance

## Project Structure

```
src/
├── lib.rs              # Module declarations and exports
├── main.rs             # Demo and examples
├── search_node.rs      # Core data structures (State, SearchNode)
├── heuristics.rs       # Heuristic function implementations
├── base_search.rs      # Base search framework and traits
└── astar_search.rs     # A*, BFS, and DFS implementations
```

## Quick Start

### Building and Running

```bash
cd src/heuristic_search
cargo build
cargo run
```

### Basic Usage

```rust
use heuristic_search::{
    AStarSearch, SearchProblem, State, HeuristicFactory
};

// Create a problem
let problem = SearchProblem::new(
    start_state,
    goal_state,
    actions,
    step_cost_fn,
    successor_fn,
    goal_test_fn,
);

// Create heuristic
let heuristic = HeuristicFactory::create("manhattan", None);

// Run A* search
let mut astar = AStarSearch::new(problem, heuristic);
let result = astar.search(start_state, goal_state);

if result.success {
    println!("Found solution with cost: {}", result.total_cost);
    println!("Path: {:?}", result.path);
}
```

## Algorithms

### A* Search
- **Optimal**: Guarantees shortest path when heuristic is admissible
- **Efficient**: Uses priority queue with f-value (g + h)
- **Best for**: Pathfinding, optimization problems

### Breadth-First Search
- **Complete**: Guarantees finding solution if it exists
- **Optimal**: Finds shortest path in terms of steps
- **Best for**: Unweighted graphs, shortest path problems

### Depth-First Search
- **Memory Efficient**: Uses stack instead of queue
- **Fast**: May find solution quickly in deep search spaces
- **Best for**: Deep search spaces, constraint satisfaction

## Heuristics

### Manhattan Distance
```rust
let heuristic = HeuristicFactory::create("manhattan", None);
```
- Sum of absolute differences across all dimensions
- Admissible for grid-based problems

### Euclidean Distance
```rust
let heuristic = HeuristicFactory::create("euclidean", None);
```
- Square root of sum of squared differences
- Admissible for continuous spaces

### Gas Optimization
```rust
let mut params = HashMap::new();
params.insert("target_gas".to_string(), 50000.0);
let heuristic = HeuristicFactory::create("gas_optimization", Some(params));
```
- Specialized for gas optimization problems
- Penalizes higher gas usage

### Zero Heuristic
```rust
let heuristic = HeuristicFactory::create("zero", None);
```
- Returns 0 for all states
- Equivalent to uniform cost search

## Examples

### 8-Puzzle Problem
The classic sliding tile puzzle with 9 positions and one empty space.

```rust
let puzzle_problem = create_8_puzzle_problem();
let mut astar = AStarSearch::new(puzzle_problem, heuristic);
let result = astar.search(start_state, goal_state);
```

### Gas Optimization
Optimize smart contract gas usage through various techniques.

```rust
let gas_problem = create_gas_optimization_problem();
let gas_heuristic = HeuristicFactory::create("gas_optimization", Some(params));
let mut astar = AStarSearch::new(gas_problem, gas_heuristic);
let result = astar.search(start_state, goal_state);
```

## Performance

The implementation includes comprehensive performance tracking:

- **Nodes Expanded**: Number of nodes processed
- **Nodes Generated**: Total nodes created
- **Max Depth**: Maximum search depth reached
- **Path Length**: Number of steps in solution
- **Total Cost**: Cumulative cost of solution path

## Extending the Framework

### Adding New Heuristics

```rust
pub struct CustomHeuristic;

impl Heuristic for CustomHeuristic {
    fn evaluate(&self, state: &State, goal: &State) -> f64 {
        // Your heuristic logic here
        0.0
    }

    fn name(&self) -> &str {
        "custom"
    }
}
```

### Adding New Search Algorithms

```rust
pub struct CustomSearch {
    base: BaseSearch,
    // Additional fields
}

impl SearchAlgorithm for CustomSearch {
    fn search(&mut self, start: State, goal: State) -> SearchResult {
        // Your search logic here
        SearchResult::new()
    }

    fn get_stats(&self) -> &SearchStats {
        self.base.get_stats()
    }
}
```

## Dependencies

- `priority-queue`: For A* priority queue implementation
- `serde`: For serialization/deserialization support
- `serde_json`: For JSON serialization

## License

This project is part of the ExGasRL framework for gas optimization research. 
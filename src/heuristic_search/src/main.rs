use heuristic_search::{
    AStarSearch, BreadthFirstSearch, DepthFirstSearch, SearchAlgorithm, SearchProblem,
    State, HeuristicFactory,
};
use std::collections::HashMap;
use std::sync::Arc;
use heuristic_search::query_lunarlander_model;

/// Example: 8-puzzle problem
fn create_8_puzzle_problem() -> SearchProblem {
    // Start state: [1,2,3,4,0,6,7,5,8] (0 represents empty space)
    let mut start_state = State::new("start".to_string());
    start_state.data.insert("pos_0".to_string(), 1.0);
    start_state.data.insert("pos_1".to_string(), 2.0);
    start_state.data.insert("pos_2".to_string(), 3.0);
    start_state.data.insert("pos_3".to_string(), 4.0);
    start_state.data.insert("pos_4".to_string(), 0.0);
    start_state.data.insert("pos_5".to_string(), 6.0);
    start_state.data.insert("pos_6".to_string(), 7.0);
    start_state.data.insert("pos_7".to_string(), 5.0);
    start_state.data.insert("pos_8".to_string(), 8.0);

    // Goal state: [1,2,3,4,5,6,7,8,0]
    let mut goal_state = State::new("goal".to_string());
    goal_state.data.insert("pos_0".to_string(), 1.0);
    goal_state.data.insert("pos_1".to_string(), 2.0);
    goal_state.data.insert("pos_2".to_string(), 3.0);
    goal_state.data.insert("pos_3".to_string(), 4.0);
    goal_state.data.insert("pos_4".to_string(), 5.0);
    goal_state.data.insert("pos_5".to_string(), 6.0);
    goal_state.data.insert("pos_6".to_string(), 7.0);
    goal_state.data.insert("pos_7".to_string(), 8.0);
    goal_state.data.insert("pos_8".to_string(), 0.0);

    let actions = vec!["up".to_string(), "down".to_string(), "left".to_string(), "right".to_string()];

    let step_cost_fn = Arc::new(|_state: &State, _action: &str, _next_state: &State| -> f64 {
        1.0 // Uniform cost
    });

    let successor_fn = Arc::new(|state: &State, action: &str| -> Vec<(State, f64)> {
        let mut successors = Vec::new();
        
        // Find empty position (0)
        let mut empty_pos = 0;
        for i in 0..9 {
            if state.data.get(&format!("pos_{}", i)) == Some(&0.0) {
                empty_pos = i;
                break;
            }
        }

        // Generate valid moves
        let valid_moves = match action {
            "up" if empty_pos < 6 => Some(empty_pos + 3),
            "down" if empty_pos > 2 => Some(empty_pos - 3),
            "left" if empty_pos % 3 != 2 => Some(empty_pos + 1),
            "right" if empty_pos % 3 != 0 => Some(empty_pos - 1),
            _ => None,
        };

        if let Some(new_pos) = valid_moves {
            let mut new_state = state.clone();
            let value = state.data.get(&format!("pos_{}", new_pos)).unwrap();
            
            // Swap values
            new_state.data.insert(format!("pos_{}", empty_pos), *value);
            new_state.data.insert(format!("pos_{}", new_pos), 0.0);
            new_state.id = format!("state_{}", new_pos);
            
            successors.push((new_state, 1.0));
        }

        successors
    });

    let goal_test_fn = Arc::new(|state: &State, goal: &State| -> bool {
        for i in 0..9 {
            let state_val = state.data.get(&format!("pos_{}", i));
            let goal_val = goal.data.get(&format!("pos_{}", i));
            if state_val != goal_val {
                return false;
            }
        }
        true
    });

    SearchProblem::new(start_state, goal_state, actions, step_cost_fn, successor_fn, goal_test_fn)
}

/// Example: Gas optimization problem
fn create_gas_optimization_problem() -> SearchProblem {
    let mut start_state = State::new("high_gas".to_string());
    start_state.data.insert("gas_usage".to_string(), 100000.0);
    start_state.data.insert("complexity".to_string(), 10.0);

    let mut goal_state = State::new("optimized".to_string());
    goal_state.data.insert("gas_usage".to_string(), 50000.0);
    goal_state.data.insert("complexity".to_string(), 5.0);

    let actions = vec![
        "optimize_loop".to_string(),
        "reduce_storage".to_string(),
        "use_assembly".to_string(),
        "batch_operations".to_string(),
    ];

    let step_cost_fn = Arc::new(|state: &State, action: &str, next_state: &State| -> f64 {
        let current_gas = state.data.get("gas_usage").unwrap_or(&0.0);
        let next_gas = next_state.data.get("gas_usage").unwrap_or(&0.0);
        let improvement = current_gas - next_gas;
        
        // Cost is inverse to improvement (better improvements cost less)
        if improvement > 0.0 {
            1.0 / improvement
        } else {
            10.0 // Penalty for worsening
        }
    });

    let successor_fn = Arc::new(|state: &State, action: &str| -> Vec<(State, f64)> {
        let mut successors = Vec::new();
        let current_gas = state.data.get("gas_usage").unwrap_or(&100000.0);
        let current_complexity = state.data.get("complexity").unwrap_or(&10.0);

        let (new_gas, new_complexity) = match action {
            "optimize_loop" => (current_gas * 0.9, current_complexity * 0.95),
            "reduce_storage" => (current_gas * 0.85, current_complexity * 0.9),
            "use_assembly" => (current_gas * 0.7, current_complexity * 1.2),
            "batch_operations" => (current_gas * 0.8, current_complexity * 0.85),
            _ => (*current_gas, *current_complexity),
        };

        let mut new_state = state.clone();
        new_state.data.insert("gas_usage".to_string(), new_gas);
        new_state.data.insert("complexity".to_string(), new_complexity);
        new_state.id = format!("{}_{}", state.id, action);

        successors.push((new_state, 1.0));
        successors
    });

    let goal_test_fn = Arc::new(|state: &State, goal: &State| -> bool {
        let state_gas = state.data.get("gas_usage").unwrap_or(&f64::INFINITY);
        let goal_gas = goal.data.get("gas_usage").unwrap_or(&0.0);
        state_gas <= goal_gas
    });

    SearchProblem::new(start_state, goal_state, actions, step_cost_fn, successor_fn, goal_test_fn)
}

fn run_search_example(problem: SearchProblem, algorithm_name: &str) {
    println!("\n=== Running {} ===", algorithm_name);
    
    let heuristic = HeuristicFactory::create("manhattan", None);
    
    let mut algorithm: Box<dyn SearchAlgorithm> = match algorithm_name {
        "A*" => Box::new(AStarSearch::new(problem.clone(), heuristic)),
        "BFS" => Box::new(BreadthFirstSearch::new(problem.clone(), heuristic)),
        "DFS" => Box::new(DepthFirstSearch::new(problem.clone(), heuristic)),
        _ => {
            println!("Unknown algorithm: {}", algorithm_name);
            return;
        }
    };

    let start_state = problem.start_state.clone();
    let goal_state = problem.goal_state.clone();

    println!("Start state: {:?}", start_state);
    println!("Goal state: {:?}", goal_state);

    let result = algorithm.search(start_state, goal_state);

    println!("Result: {}", if result.success { "SUCCESS" } else { "FAILURE" });
    println!("Message: {}", result.message);
    println!("Total cost: {:.2}", result.total_cost);
    println!("Path length: {}", result.path.len());
    println!("Nodes expanded: {}", result.nodes_expanded);
    println!("Nodes generated: {}", result.nodes_generated);
    println!("Max depth: {}", result.max_depth);

    if result.success {
        println!("Path:");
        for (i, (action, state)) in result.path.iter().enumerate() {
            println!("  {}: {} -> {}", i + 1, action, state);
        }
    }
}

fn explain_search_path(states: &[Vec<f32>]) {
    for (i, state) in states.iter().enumerate() {
        let model_out = query_lunarlander_model(state);
        println!(
            "Step {}: State={:?}, Action={}, Q-values={:?}",
            i, state, model_out.action, model_out.q_values
        );
    }
}

fn main() {
    println!("🚀 Heuristic Search Algorithms Demo");
    println!("=====================================");

    // Example 1: 8-puzzle problem
    println!("\n📦 8-Puzzle Problem");
    let puzzle_problem = create_8_puzzle_problem();
    
    run_search_example(puzzle_problem.clone(), "A*");
    run_search_example(puzzle_problem.clone(), "BFS");
    run_search_example(puzzle_problem, "DFS");

    // Example 2: Gas optimization problem
    println!("\n⛽ Gas Optimization Problem");
    let gas_problem = create_gas_optimization_problem();
    
    // Use gas optimization heuristic for this problem
    let mut gas_heuristic_params = HashMap::new();
    gas_heuristic_params.insert("target_gas".to_string(), 50000.0);
    let gas_heuristic = HeuristicFactory::create("gas_optimization", Some(gas_heuristic_params));
    
    let mut astar_gas = AStarSearch::new(gas_problem.clone(), gas_heuristic);
    let result = astar_gas.search(gas_problem.start_state.clone(), gas_problem.goal_state.clone());
    
    println!("A* with Gas Optimization Heuristic:");
    println!("Result: {}", if result.success { "SUCCESS" } else { "FAILURE" });
    println!("Message: {}", result.message);
    println!("Total cost: {:.2}", result.total_cost);
    println!("Path length: {}", result.path.len());
    println!("Nodes expanded: {}", result.nodes_expanded);
    println!("Nodes generated: {}", result.nodes_generated);
    println!("Max depth: {}", result.max_depth);

    if result.success {
        println!("Optimization path:");
        for (i, (action, state)) in result.path.iter().enumerate() {
            println!("  {}: {} -> {}", i + 1, action, state);
        }
    }

    // Example: explain a dummy path
    let states = vec![
        vec![0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
    ];
    explain_search_path(&states);

    println!("\n✅ Demo completed!");
}

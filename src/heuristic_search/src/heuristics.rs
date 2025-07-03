use crate::search_node::State;
use std::collections::HashMap;

/// Trait for heuristic functions
pub trait Heuristic {
    fn evaluate(&self, state: &State, goal: &State) -> f64;
    fn name(&self) -> &str;
}

/// Manhattan distance heuristic
pub struct ManhattanHeuristic;

impl Heuristic for ManhattanHeuristic {
    fn evaluate(&self, state: &State, goal: &State) -> f64 {
        let mut distance = 0.0;
        
        for (key, goal_value) in &goal.data {
            if let Some(state_value) = state.data.get(key) {
                distance += (state_value - goal_value).abs();
            }
        }
        
        distance
    }

    fn name(&self) -> &str {
        "manhattan"
    }
}

/// Euclidean distance heuristic
pub struct EuclideanHeuristic;

impl Heuristic for EuclideanHeuristic {
    fn evaluate(&self, state: &State, goal: &State) -> f64 {
        let mut sum_squares = 0.0;
        
        for (key, goal_value) in &goal.data {
            if let Some(state_value) = state.data.get(key) {
                let diff = state_value - goal_value;
                sum_squares += diff * diff;
            }
        }
        
        sum_squares.sqrt()
    }

    fn name(&self) -> &str {
        "euclidean"
    }
}

/// Zero heuristic (for uniform cost search)
pub struct ZeroHeuristic;

impl Heuristic for ZeroHeuristic {
    fn evaluate(&self, _state: &State, _goal: &State) -> f64 {
        0.0
    }

    fn name(&self) -> &str {
        "zero"
    }
}

/// Gas optimization specific heuristic
pub struct GasOptimizationHeuristic {
    pub target_gas: f64,
}

impl GasOptimizationHeuristic {
    pub fn new(target_gas: f64) -> Self {
        Self { target_gas }
    }
}

impl Heuristic for GasOptimizationHeuristic {
    fn evaluate(&self, state: &State, _goal: &State) -> f64 {
        if let Some(current_gas) = state.data.get("gas_usage") {
            let diff = (current_gas - self.target_gas).abs();
            // Penalize higher gas usage more heavily
            if *current_gas > self.target_gas {
                diff * 2.0
            } else {
                diff
            }
        } else {
            f64::INFINITY
        }
    }

    fn name(&self) -> &str {
        "gas_optimization"
    }
}

/// Factory for creating heuristic functions
pub struct HeuristicFactory;

impl HeuristicFactory {
    pub fn create(heuristic_type: &str, params: Option<HashMap<String, f64>>) -> Box<dyn Heuristic> {
        match heuristic_type {
            "manhattan" => Box::new(ManhattanHeuristic),
            "euclidean" => Box::new(EuclideanHeuristic),
            "zero" => Box::new(ZeroHeuristic),
            "gas_optimization" => {
                let target_gas = params
                    .and_then(|p| p.get("target_gas").copied())
                    .unwrap_or(0.0);
                Box::new(GasOptimizationHeuristic::new(target_gas))
            }
            _ => Box::new(ZeroHeuristic), // Default to zero heuristic
        }
    }
} 
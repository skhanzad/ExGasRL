use crate::search_node::{SearchNode, State};
use crate::heuristics::Heuristic;
use std::fmt;
use std::sync::Arc;

/// Search result containing the solution path and statistics
#[derive(Debug, Clone)]
pub struct SearchResult {
    pub path: Vec<(String, String)>, // (action, state_id)
    pub total_cost: f64,
    pub nodes_expanded: usize,
    pub nodes_generated: usize,
    pub max_depth: usize,
    pub success: bool,
    pub message: String,
}

impl SearchResult {
    pub fn new() -> Self {
        Self {
            path: Vec::new(),
            total_cost: 0.0,
            nodes_expanded: 0,
            nodes_generated: 0,
            max_depth: 0,
            success: false,
            message: String::new(),
        }
    }

    pub fn success(path: Vec<(String, String)>, total_cost: f64, stats: SearchStats) -> Self {
        Self {
            path,
            total_cost,
            nodes_expanded: stats.nodes_expanded,
            nodes_generated: stats.nodes_generated,
            max_depth: stats.max_depth,
            success: true,
            message: "Search completed successfully".to_string(),
        }
    }

    pub fn failure(message: String, stats: SearchStats) -> Self {
        Self {
            path: Vec::new(),
            total_cost: f64::INFINITY,
            nodes_expanded: stats.nodes_expanded,
            nodes_generated: stats.nodes_generated,
            max_depth: stats.max_depth,
            success: false,
            message,
        }
    }
}

/// Search statistics
#[derive(Debug, Clone)]
pub struct SearchStats {
    pub nodes_expanded: usize,
    pub nodes_generated: usize,
    pub max_depth: usize,
}

impl SearchStats {
    pub fn new() -> Self {
        Self {
            nodes_expanded: 0,
            nodes_generated: 0,
            max_depth: 0,
        }
    }
}

/// Trait for search algorithms
pub trait SearchAlgorithm {
    fn search(&mut self, start: State, goal: State) -> SearchResult;
    fn get_stats(&self) -> &SearchStats;
}

/// Problem definition for search algorithms
pub struct SearchProblem {
    pub start_state: State,
    pub goal_state: State,
    pub actions: Vec<String>,
    pub step_cost_fn: Arc<dyn Fn(&State, &str, &State) -> f64 + Send + Sync>,
    pub successor_fn: Arc<dyn Fn(&State, &str) -> Vec<(State, f64)> + Send + Sync>,
    pub goal_test_fn: Arc<dyn Fn(&State, &State) -> bool + Send + Sync>,
}

impl Clone for SearchProblem {
    fn clone(&self) -> Self {
        Self {
            start_state: self.start_state.clone(),
            goal_state: self.goal_state.clone(),
            actions: self.actions.clone(),
            step_cost_fn: Arc::clone(&self.step_cost_fn),
            successor_fn: Arc::clone(&self.successor_fn),
            goal_test_fn: Arc::clone(&self.goal_test_fn),
        }
    }
}

impl SearchProblem {
    pub fn new(
        start_state: State,
        goal_state: State,
        actions: Vec<String>,
        step_cost_fn: Arc<dyn Fn(&State, &str, &State) -> f64 + Send + Sync>,
        successor_fn: Arc<dyn Fn(&State, &str) -> Vec<(State, f64)> + Send + Sync>,
        goal_test_fn: Arc<dyn Fn(&State, &State) -> bool + Send + Sync>,
    ) -> Self {
        Self {
            start_state,
            goal_state,
            actions,
            step_cost_fn,
            successor_fn,
            goal_test_fn,
        }
    }
}

/// Base search implementation with common functionality
pub struct BaseSearch {
    pub problem: SearchProblem,
    pub heuristic: Box<dyn Heuristic>,
    pub stats: SearchStats,
    pub max_iterations: Option<usize>,
    pub max_depth: Option<usize>,
}

impl BaseSearch {
    pub fn new(problem: SearchProblem, heuristic: Box<dyn Heuristic>) -> Self {
        Self {
            problem,
            heuristic,
            stats: SearchStats::new(),
            max_iterations: None,
            max_depth: None,
        }
    }

    pub fn with_limits(mut self, max_iterations: Option<usize>, max_depth: Option<usize>) -> Self {
        self.max_iterations = max_iterations;
        self.max_depth = max_depth;
        self
    }

    pub fn expand_node(&mut self, node: &SearchNode) -> Vec<SearchNode> {
        let mut successors = Vec::new();
        
        for action in &self.problem.actions {
            let successors_with_costs = (self.problem.successor_fn)(&node.state, action);
            
            for (successor_state, step_cost) in successors_with_costs {
                let mut successor_node = SearchNode::new(successor_state);
                successor_node.parent = Some(Box::new(node.clone()));
                successor_node.action = Some(action.clone());
                successor_node.path_cost = node.path_cost + step_cost;
                successor_node.depth = node.depth + 1;
                successor_node.heuristic_value = self.heuristic.evaluate(&successor_node.state, &self.problem.goal_state);
                
                successors.push(successor_node);
                self.stats.nodes_generated += 1;
            }
        }
        
        successors
    }

    pub fn is_goal(&self, state: &State) -> bool {
        (self.problem.goal_test_fn)(state, &self.problem.goal_state)
    }

    pub fn check_limits(&self, iterations: usize, depth: usize) -> bool {
        if let Some(max_iter) = self.max_iterations {
            if iterations >= max_iter {
                return false;
            }
        }
        
        if let Some(max_depth) = self.max_depth {
            if depth >= max_depth {
                return false;
            }
        }
        
        true
    }

    pub fn get_stats(&self) -> &SearchStats {
        &self.stats
    }
}

impl fmt::Display for BaseSearch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "BaseSearch(heuristic={}, max_iterations={:?}, max_depth={:?})",
            self.heuristic.name(),
            self.max_iterations,
            self.max_depth
        )
    }
} 
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;

/// Represents a state in the search space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct State {
    pub id: String,
    pub data: HashMap<String, f64>,
    pub metadata: HashMap<String, String>,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for State {}

impl std::hash::Hash for State {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl State {
    pub fn new(id: String) -> Self {
        Self {
            id,
            data: HashMap::new(),
            metadata: HashMap::new(),
        }
    }

    pub fn with_data(mut self, key: String, value: f64) -> Self {
        self.data.insert(key, value);
        self
    }

    pub fn with_metadata(mut self, key: String, value: String) -> Self {
        self.metadata.insert(key, value);
        self
    }
}

/// Represents a node in the search tree
#[derive(Debug, Clone)]
pub struct SearchNode {
    pub state: State,
    pub parent: Option<Box<SearchNode>>,
    pub action: Option<String>,
    pub path_cost: f64,
    pub depth: usize,
    pub heuristic_value: f64,
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.state == other.state
    }
}

impl Eq for SearchNode {}

impl std::hash::Hash for SearchNode {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.state.hash(state);
    }
}

impl SearchNode {
    pub fn new(state: State) -> Self {
        Self {
            state,
            parent: None,
            action: None,
            path_cost: 0.0,
            depth: 0,
            heuristic_value: 0.0,
        }
    }

    pub fn with_parent(mut self, parent: SearchNode, action: String, step_cost: f64) -> Self {
        self.parent = Some(Box::new(parent.clone()));
        self.action = Some(action);
        self.path_cost = parent.path_cost + step_cost;
        self.depth = parent.depth + 1;
        self
    }

    pub fn f_value(&self) -> f64 {
        self.path_cost + self.heuristic_value
    }

    pub fn get_path(&self) -> Vec<(String, String)> {
        let mut path = Vec::new();
        let mut current = self;
        
        while let Some(parent) = &current.parent {
            if let Some(action) = &current.action {
                path.push((action.clone(), current.state.id.clone()));
            }
            current = parent;
        }
        
        path.reverse();
        path
    }
}

impl fmt::Display for SearchNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Node(state={}, cost={:.2}, depth={}, f={:.2})",
            self.state.id,
            self.path_cost,
            self.depth,
            self.f_value()
        )
    }
} 
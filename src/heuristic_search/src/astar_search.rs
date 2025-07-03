use crate::search_node::{SearchNode, State};
use crate::base_search::{BaseSearch, SearchAlgorithm, SearchResult, SearchStats, SearchProblem};
use crate::heuristics::Heuristic;
use std::collections::{HashSet, BinaryHeap};
use std::cmp::Ordering;
use std::fmt;

/// Wrapper for f64 that implements Ord for priority queue
#[derive(Debug, Clone, Copy)]
struct F64Wrapper(f64);

impl PartialEq for F64Wrapper {
    fn eq(&self, other: &Self) -> bool {
        self.0.partial_cmp(&other.0) == Some(Ordering::Equal)
    }
}

impl Eq for F64Wrapper {}

impl PartialOrd for F64Wrapper {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl Ord for F64Wrapper {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

impl PartialEq for PriorityEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Eq for PriorityEntry {}

impl PartialOrd for PriorityEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // Reverse order for max heap (lowest f-value first)
        other.priority.partial_cmp(&self.priority)
    }
}

impl Ord for PriorityEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

/// Priority queue entry for A* search
#[derive(Debug, Clone)]
struct PriorityEntry {
    node: SearchNode,
    priority: F64Wrapper,
}

/// A* search algorithm implementation
pub struct AStarSearch {
    base: BaseSearch,
    visited: HashSet<String>,
}

impl AStarSearch {
    pub fn new(problem: SearchProblem, heuristic: Box<dyn Heuristic>) -> Self {
        Self {
            base: BaseSearch::new(problem, heuristic),
            visited: HashSet::new(),
        }
    }

    pub fn with_limits(mut self, max_iterations: Option<usize>, max_depth: Option<usize>) -> Self {
        self.base = self.base.with_limits(max_iterations, max_depth);
        self
    }

    pub fn reset(&mut self) {
        self.visited.clear();
        self.base.stats = SearchStats::new();
    }
}

impl SearchAlgorithm for AStarSearch {
    fn search(&mut self, start: State, goal: State) -> SearchResult {
        self.reset();
        
        // Initialize the start node
        let mut start_node = SearchNode::new(start);
        start_node.heuristic_value = self.base.heuristic.evaluate(&start_node.state, &goal);
        
        // Priority queue for A* (f-value as priority)
        let mut frontier: BinaryHeap<PriorityEntry> = BinaryHeap::new();
        frontier.push(PriorityEntry {
            node: start_node,
            priority: F64Wrapper(0.0),
        });
        
        // Track visited states to avoid cycles
        self.visited.clear();
        
        let mut iterations = 0;
        
        while !frontier.is_empty() {
            iterations += 1;
            
            // Check iteration limit
            if !self.base.check_limits(iterations, 0) {
                return SearchResult::failure(
                    format!("Search exceeded maximum iterations ({})", iterations),
                    self.base.stats.clone(),
                );
            }
            
            // Get the node with lowest f-value
            let current_entry = frontier.pop().unwrap();
            let current_node = current_entry.node;
            
            // Check if we've reached the goal
            if self.base.is_goal(&current_node.state) {
                let path = current_node.get_path();
                return SearchResult::success(path, current_node.path_cost, self.base.stats.clone());
            }
            
            // Mark as visited
            self.visited.insert(current_node.state.id.clone());
            self.base.stats.nodes_expanded += 1;
            
            // Update max depth
            if current_node.depth > self.base.stats.max_depth {
                self.base.stats.max_depth = current_node.depth;
            }
            
            // Expand the current node
            let successors = self.base.expand_node(&current_node);
            
            for successor in successors {
                // Skip if already visited
                if self.visited.contains(&successor.state.id) {
                    continue;
                }
                
                // Check depth limit
                if !self.base.check_limits(iterations, successor.depth) {
                    continue;
                }
                
                // Add to frontier with f-value as priority
                let f_value = successor.f_value();
                frontier.push(PriorityEntry {
                    node: successor,
                    priority: F64Wrapper(f_value),
                });
            }
        }
        
        SearchResult::failure(
            "No solution found - search space exhausted".to_string(),
            self.base.stats.clone(),
        )
    }

    fn get_stats(&self) -> &SearchStats {
        self.base.get_stats()
    }
}

impl fmt::Display for AStarSearch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "AStarSearch(heuristic={}, visited={}, stats={:?})",
            self.base.heuristic.name(),
            self.visited.len(),
            self.base.stats
        )
    }
}

/// Breadth-First Search implementation
pub struct BreadthFirstSearch {
    base: BaseSearch,
    visited: HashSet<String>,
}

impl BreadthFirstSearch {
    pub fn new(problem: SearchProblem, heuristic: Box<dyn Heuristic>) -> Self {
        Self {
            base: BaseSearch::new(problem, heuristic),
            visited: HashSet::new(),
        }
    }

    pub fn with_limits(mut self, max_iterations: Option<usize>, max_depth: Option<usize>) -> Self {
        self.base = self.base.with_limits(max_iterations, max_depth);
        self
    }
}

impl SearchAlgorithm for BreadthFirstSearch {
    fn search(&mut self, start: State, _goal: State) -> SearchResult {
        use std::collections::VecDeque;
        
        self.visited.clear();
        self.base.stats = SearchStats::new();
        
        let start_node = SearchNode::new(start);
        let mut frontier: VecDeque<SearchNode> = VecDeque::new();
        frontier.push_back(start_node);
        
        let mut iterations = 0;
        
        while !frontier.is_empty() {
            iterations += 1;
            
            if !self.base.check_limits(iterations, 0) {
                return SearchResult::failure(
                    format!("Search exceeded maximum iterations ({})", iterations),
                    self.base.stats.clone(),
                );
            }
            
            let current_node = frontier.pop_front().unwrap();
            
            if self.base.is_goal(&current_node.state) {
                let path = current_node.get_path();
                return SearchResult::success(path, current_node.path_cost, self.base.stats.clone());
            }
            
            self.visited.insert(current_node.state.id.clone());
            self.base.stats.nodes_expanded += 1;
            
            if current_node.depth > self.base.stats.max_depth {
                self.base.stats.max_depth = current_node.depth;
            }
            
            let successors = self.base.expand_node(&current_node);
            
            for successor in successors {
                if !self.visited.contains(&successor.state.id) && 
                   self.base.check_limits(iterations, successor.depth) {
                    frontier.push_back(successor);
                }
            }
        }
        
        SearchResult::failure(
            "No solution found - search space exhausted".to_string(),
            self.base.stats.clone(),
        )
    }

    fn get_stats(&self) -> &SearchStats {
        self.base.get_stats()
    }
}

/// Depth-First Search implementation
pub struct DepthFirstSearch {
    base: BaseSearch,
    visited: HashSet<String>,
}

impl DepthFirstSearch {
    pub fn new(problem: SearchProblem, heuristic: Box<dyn Heuristic>) -> Self {
        Self {
            base: BaseSearch::new(problem, heuristic),
            visited: HashSet::new(),
        }
    }

    pub fn with_limits(mut self, max_iterations: Option<usize>, max_depth: Option<usize>) -> Self {
        self.base = self.base.with_limits(max_iterations, max_depth);
        self
    }
}

impl SearchAlgorithm for DepthFirstSearch {
    fn search(&mut self, start: State, _goal: State) -> SearchResult {
        use std::collections::VecDeque;
        
        self.visited.clear();
        self.base.stats = SearchStats::new();
        
        let start_node = SearchNode::new(start);
        let mut frontier: VecDeque<SearchNode> = VecDeque::new();
        frontier.push_back(start_node);
        
        let mut iterations = 0;
        
        while !frontier.is_empty() {
            iterations += 1;
            
            if !self.base.check_limits(iterations, 0) {
                return SearchResult::failure(
                    format!("Search exceeded maximum iterations ({})", iterations),
                    self.base.stats.clone(),
                );
            }
            
            let current_node = frontier.pop_back().unwrap();
            
            if self.base.is_goal(&current_node.state) {
                let path = current_node.get_path();
                return SearchResult::success(path, current_node.path_cost, self.base.stats.clone());
            }
            
            self.visited.insert(current_node.state.id.clone());
            self.base.stats.nodes_expanded += 1;
            
            if current_node.depth > self.base.stats.max_depth {
                self.base.stats.max_depth = current_node.depth;
            }
            
            let successors = self.base.expand_node(&current_node);
            
            // Add successors in reverse order to maintain LIFO behavior
            for successor in successors.into_iter().rev() {
                if !self.visited.contains(&successor.state.id) && 
                   self.base.check_limits(iterations, successor.depth) {
                    frontier.push_back(successor);
                }
            }
        }
        
        SearchResult::failure(
            "No solution found - search space exhausted".to_string(),
            self.base.stats.clone(),
        )
    }

    fn get_stats(&self) -> &SearchStats {
        self.base.get_stats()
    }
} 
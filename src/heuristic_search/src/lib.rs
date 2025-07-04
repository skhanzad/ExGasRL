use pyo3::prelude::*;

pub mod base_search;
pub mod astar_search;
pub mod search_node;
pub mod heuristics;
pub mod py_model;

pub use base_search::*;
pub use astar_search::*;
pub use search_node::*;
pub use heuristics::*;
pub use py_model::*;

/// Python module entry point
#[pymodule]
fn heuristic_search(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(query_lunarlander_model_py, m)?)?;
    Ok(())
}

/// Python function wrapper for query_lunarlander_model
#[pyfunction]
fn query_lunarlander_model_py(state: Vec<f32>) -> PyResult<(usize, Vec<f32>)> {
    let result = py_model::query_lunarlander_model(&state);
    Ok((result.action, result.q_values))
} 
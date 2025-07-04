use pyo3::prelude::*;
use pyo3::types::PyModule;

pub struct ModelOutput {
    pub action: usize,
    pub q_values: Vec<f32>,
}

pub fn query_lunarlander_model(state: &[f32]) -> ModelOutput {
    Python::with_gil(|py| {
        let xai_mod = PyModule::import_bound(py, "src.models.lunarlander_xai").expect("Import failed");
        let result = xai_mod
            .getattr("predict")
            .expect("No predict")
            .call1((state.to_vec(),))
            .expect("Call failed");
        let (action, q_values): (usize, Vec<f32>) = result.extract().expect("Extract failed");
        ModelOutput { action, q_values }
    })
} 
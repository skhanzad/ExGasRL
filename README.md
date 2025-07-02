# Gas Optimization XAI Framework with Heuristic Search

An explainable AI framework that uses heuristic search algorithms to optimize gas consumption in Solidity smart contracts.

## Features

- **Heuristic Search Algorithms**: A*, beam search, genetic algorithms, simulated annealing
- **Explainable AI**: Search path visualization, decision explanations, heuristic analysis
- **Gas Optimization**: Automated Solidity contract optimization with measurable improvements
- **Comprehensive Evaluation**: Performance metrics, explainability measures, benchmarking

## Quick Start

1. **Installation**:
   ```bash
   pip install -e .
   ```

2. **Run optimization**:
   ```bash
   python scripts/optimize_contract.py --contract path/to/contract.sol
   ```

3. **Generate explanations**:
   ```bash
   python scripts/generate_explanations.py --results results/optimization_run_001
   ```

## Architecture

The framework consists of:

- **Heuristic Search Engine**: Multiple search algorithms for optimization space exploration
- **Gas Environment**: Solidity contract analysis and gas calculation
- **Explainability Module**: Multi-layered explanation generation
- **Evaluation Suite**: Comprehensive metrics and benchmarking

## Usage Examples

See the `notebooks/` directory for detailed examples and tutorials.

## Contributing

Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

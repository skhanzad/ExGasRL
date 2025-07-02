#!/usr/bin/env python3
"""
Gas Optimization XAI-RL Framework Project Generator
Generates the complete folder structure and template files for an explainable AI
reinforcement learning framework focused on Solidity gas optimization.
"""

import os
import json
from pathlib import Path

def create_project_structure():
    """Create the complete project directory structure."""
    
    structure = {
        "gas_optimization_xai_rl": {
            "src": {
                "agents": {
                    "__init__.py": "",
                    "base_agent.py": "",
                    "ppo_agent.py": "",
                    "dqn_agent.py": "",
                    "gas_optimizer_agent.py": ""
                },
                "environment": {
                    "__init__.py": "",
                    "solidity_env.py": "",
                    "gas_calculator.py": "",
                    "contract_parser.py": "",
                    "optimization_env.py": ""
                },
                "heuristic_search": {
                    "__init__.py": "",
                    "base_search.py": "",
                    "a_star_search.py": "",
                    "beam_search.py": "",
                    "genetic_algorithm.py": "",
                    "simulated_annealing.py": "",
                    "hill_climbing.py": "",
                    "search_heuristics.py": ""
                },
                "explainability": {
                    "__init__.py": "",
                    "base_explainer.py": "",
                    "search_path_explainer.py": "",
                    "heuristic_explainer.py": "",
                    "decision_tree_explainer.py": "",
                    "rule_based_explainer.py": "",
                    "counterfactual_explainer.py": "",
                    "visualization.py": ""
                },
                "models": {
                    "__init__.py": "",
                    "gas_prediction_model.py": "",
                    "optimization_state.py": "",
                    "search_node.py": "",
                    "heuristic_functions.py": ""
                },
                "utils": {
                    "__init__.py": "",
                    "solidity_parser.py": "",
                    "gas_metrics.py": "",
                    "data_loader.py": "",
                    "config.py": "",
                    "logger.py": ""
                },
                "evaluation": {
                    "__init__.py": "",
                    "metrics.py": "",
                    "benchmarks.py": "",
                    "explainability_metrics.py": "",
                    "search_performance.py": ""
                },
                "__init__.py": ""
            },
            "data": {
                "contracts": {
                    "original": {},
                    "optimized": {},
                    "benchmarks": {}
                },
                "training": {},
                "results": {},
                "explanations": {}
            },
            "tests": {
                "__init__.py": "",
                "test_agents.py": "",
                "test_environment.py": "",
                "test_heuristic_search.py": "",
                "test_explainability.py": "",
                "test_models.py": "",
                "test_utils.py": ""
            },
            "notebooks": {
                "01_data_exploration.ipynb": "",
                "02_heuristic_search_training.ipynb": "",
                "03_explainability_analysis.ipynb": "",
                "04_search_path_visualization.ipynb": "",
                "05_results_comparison.ipynb": ""
            },
            "configs": {
                "search_config.yaml": "",
                "environment_config.yaml": "",
                "heuristics_config.yaml": "",
                "explainability_config.yaml": ""
            },
            "scripts": {
                "run_search.py": "",
                "evaluate_search.py": "",
                "generate_explanations.py": "",
                "optimize_contract.py": "",
                "benchmark_heuristics.py": ""
            },
            "docs": {
                "README.md": "",
                "API_REFERENCE.md": "",
                "TUTORIAL.md": "",
                "ARCHITECTURE.md": ""
            },
            "requirements.txt": "",
            "setup.py": "",
            "README.md": "",
            ".gitignore": ""
        }
    }
    
    def create_structure(base_path, structure_dict):
        """Recursively create directory structure."""
        for name, content in structure_dict.items():
            path = base_path / name
            if isinstance(content, dict):
                path.mkdir(parents=True, exist_ok=True)
                create_structure(path, content)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                if not path.exists():
                    path.touch()
    
    # Create the project structure
    base_dir = Path(".")
    create_structure(base_dir, structure)
    print("✅ Project structure created successfully!")

def generate_template_files():
    """Generate template files with boilerplate code."""
    
    templates = {
        "requirements.txt": """# Core dependencies
numpy>=1.21.0
pandas>=1.3.0
matplotlib>=3.4.0
seaborn>=0.11.0
plotly>=5.0.0
networkx>=2.6.0
scipy>=1.7.0
scikit-learn>=1.0.0

# Solidity and blockchain
py-solc-x>=1.1.0
web3>=5.20.0
eth-brownie>=1.16.0

# Heuristic search and optimization
deap>=1.3.0
pymoo>=0.5.0

# Explainability
shap>=0.40.0
lime>=0.2.0

# Visualization and reporting
dash>=2.0.0
dash-bootstrap-components>=1.0.0
jupyter>=1.0.0
ipywidgets>=7.6.0

# Testing and development
pytest>=6.2.0
pytest-cov>=2.12.0
black>=21.6.0
flake8>=3.9.0
mypy>=0.910

# Configuration
pyyaml>=5.4.0
python-dotenv>=0.19.0
""",

        "setup.py": """from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="gas-optimization-xai-search",
    version="0.1.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="Explainable AI framework with heuristic search for Solidity gas optimization",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/gas-optimization-xai-search",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": ["pytest", "pytest-cov", "black", "flake8", "mypy"],
        "docs": ["sphinx", "sphinx-rtd-theme"],
    },
)
""",

        ".gitignore": """# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
*.egg-info/
.installed.cfg
*.egg

# Virtual environments
.env
.venv
env/
venv/
ENV/
env.bak/
venv.bak/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# Jupyter Notebook
.ipynb_checkpoints

# Data and results
data/results/
data/training/cache/
*.log
*.pkl
*.h5

# OS
.DS_Store
Thumbs.db

# Solidity compilation artifacts
build/
cache/
artifacts/
""",

        "README.md": """# Gas Optimization XAI Framework with Heuristic Search

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
"""
    }
    
    # Write template files
    for filename, content in templates.items():
        file_path = Path("gas_optimization_xai_rl") / filename
        if filename in ["requirements.txt", "setup.py", ".gitignore"]:
            file_path = Path("gas_optimization_xai_rl") / filename
        else:
            file_path = Path("gas_optimization_xai_rl") / filename
            
        file_path.parent.mkdir(parents=True, exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(content)
    
    print("✅ Template files generated successfully!")

if __name__ == "__main__":
    print("🚀 Generating Gas Optimization XAI Framework with Heuristic Search...")
    create_project_structure()
    generate_template_files()
    print("🎉 Project generation complete!")
    print("\nNext steps:")
    print("1. cd gas_optimization_xai_rl")
    print("2. python -m venv venv")
    print("3. source venv/bin/activate  # On Windows: venv\\Scripts\\activate")
    print("4. pip install -e .")
    print("5. Start developing!")
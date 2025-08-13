# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TeraSim is an open-source traffic simulation platform designed for naturalistic and adversarial testing of autonomous vehicles (AVs). It enables high-speed, AI-driven testing environment generation to expose AVs to both routine and rare, high-risk driving conditions. Built upon SUMO (Simulation of Urban MObility), TeraSim extends its capabilities for specialized AV testing.

## Common Development Commands

### Environment Setup
```bash
# Install with Poetry (required)
conda create -n terasim python=3.10
conda activate terasim
poetry install
```

### Testing
```bash
# Run tests with coverage
poetry run pytest
# or manually: pytest -v --cov=terasim --cov-report=term-missing

# Run specific test
poetry run pytest tests/test_physics.py::test_dummy
```

### Code Quality
```bash
# Format code
poetry run black .

# Sort imports
poetry run isort .

# Lint code
poetry run flake8

# Type checking
poetry run mypy terasim/
```

### Running Simulations
```bash
# Run main simulation experiments
python run_experiments.py  # Uses examples/scenarios/police_pullover_case.yaml by default

# Run debug mode with GUI
python run_experiments_debug.py  # Uses examples/scenarios/cutin.yaml with GUI enabled

# Start TeraSim service
python run_service.py  # Starts FastAPI service on port 8000

# Run example simulation
cd examples/scripts/
python example.py

# With GUI enabled (modify gui_flag=True in example.py)
python example.py
```

## Architecture Overview

### Core Components

**Simulator (`terasim/simulator.py`)**: Central orchestrator that manages SUMO integration, synchronization, and simulation lifecycle. Handles initialization, step-by-step execution, and cleanup.

**Environment System (`terasim/envs/`)**: Abstract base classes for defining testing environments:
- `BaseEnv`: Abstract base with lifecycle hooks (`on_start`, `on_step`, `on_stop`)
- `EnvTemplate`: Concrete implementation for standard testing scenarios
- Environments control simulation flow and define test conditions

**Agent System (`terasim/agent/`)**: Core abstraction for all simulation entities:
- `Agent`: Base class with sensor-decision-controller architecture
- `Vehicle`: Specialized agent for vehicles with SUMO integration
- `VehicleList`: Collection management for multiple vehicles

**Vehicle Architecture (`terasim/vehicle/`)**: Modular vehicle system:
- **Sensors** (`sensors/`): Gather environment information (ego, local perception)
- **Decision Models** (`decision_models/`): AI/behavior logic (IDM, SUMO models)
- **Controllers** (`controllers/`): Actuate decisions (high-efficiency, SUMO integration)
- **Factory Pattern** (`factories/`): Create configured vehicle instances

**Pipeline System (`terasim/pipeline.py`)**: Execution framework for ordered, prioritized operations during simulation steps.

### Key Design Patterns

1. **Factory Pattern**: `VehicleFactory` creates vehicles with customizable sensor-decision-controller combinations
2. **Observer Pattern**: Agents observe environment through sensors and react via decision models
3. **Strategy Pattern**: Interchangeable decision models and controllers for different behaviors
4. **Pipeline Pattern**: Ordered execution of simulation steps with priority-based scheduling

### Configuration System

- **Core Configs**: System-level configurations in `configs/` directory
  - `base.yaml`: Base configuration settings
  - `environment.yaml`: Environment parameters
  - `av_config.yaml`: Autonomous vehicle configuration
  - `adversities/`: Adversarial behavior templates
- **Scenario Examples**: Concrete scenarios in `examples/scenarios/`
- **SUMO Integration**: Uses `.sumocfg`, `.net.xml`, and `.rou.xml` files for network and traffic definition
- **Map Examples**: `examples/maps/` contains various test maps including Mcity, 3LaneHighway, and Town10
- **Programmatic Config**: Python-based configuration through factory classes and environment setup

## Development Notes

### SUMO Integration
- Uses SUMO 1.23.1 with TraCI for real-time control
- Supports both GUI and headless modes
- Network files define road topology, traffic lights, and routing

### Agent Lifecycle
1. **Creation**: Factory creates agent with sensors, decision model, controller
2. **Initialization**: Agent registers with simulator and SUMO
3. **Simulation Loop**: Sensor observation → decision making → controller execution
4. **Cleanup**: Agent removal and resource cleanup

### Testing Strategy
- Unit tests in `tests/` directory
- Integration tests in `tests/integration/` directory
- Physics simulation testing with dummy models
- Integration tests through example scenarios
- Coverage reporting with pytest-cov

### Dependencies
- **Core**: SUMO suite (eclipse-sumo, traci, libsumo, sumolib)
- **ML/Math**: numpy, scipy for numerical computations
- **Utilities**: attrs for dataclasses, bidict for bidirectional mappings
- **Geospatial**: pyproj for coordinate transformations, rtree for spatial indexing

## Common Development Tasks

### Adding New Vehicle Behavior
1. Implement `AgentDecisionModel` in `terasim/vehicle/decision_models/`
2. Create factory method in custom `VehicleFactory`
3. Test with example scenario

### Creating Custom Environment
1. Extend `BaseEnv` and implement abstract methods
2. Define simulation termination conditions in `on_step`
3. Use with `Simulator.bind_env()`

### Adding New Sensor Type
1. Implement `AgentSensor` interface
2. Add to vehicle factory sensor list
3. Access data in decision model via `agent.sensors`
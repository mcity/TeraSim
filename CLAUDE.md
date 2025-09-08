# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TeraSim is an open-source traffic simulation platform designed for naturalistic and adversarial testing of autonomous vehicles (AVs). It enables high-speed, AI-driven testing environment generation to expose AVs to both routine and rare, high-risk driving conditions. Built upon SUMO (Simulation of Urban MObility), TeraSim extends its capabilities for specialized AV testing.

This is a monorepo containing multiple Python packages managed with uv workspace. The project includes the core simulation engine, NDE-NADE algorithms for naturalistic and adversarial environments, a FastAPI service for integration, environment generation tools, data processing utilities, and visualization components.

## Common Development Commands

### Environment Setup
```bash
# Quick setup (recommended)
git clone https://github.com/mcity/TeraSim.git
cd TeraSim
./setup_environment.sh  # Automated setup script

# Manual setup with uv (if needed)
conda create -n terasim python=3.10
conda activate terasim
uv sync  # Install all workspace dependencies
```

### Testing
```bash
# Run all tests with coverage
uv run pytest
# or: make test

# Run specific test suites
make test-core         # Core simulation tests
make test-envgen      # Environment generation tests
make test-service     # Service API tests
make test-integration # Integration tests
make test-fast        # Skip slow tests

# Run specific test file
uv run pytest tests/test_core/test_physics.py::test_dummy

# With coverage HTML report
uv run pytest --cov=terasim --cov-report=html
```

### Code Quality
```bash
# Format code
uv run black packages/
# or: make format

# Sort imports
uv run isort packages/

# Lint code (ruff is preferred over flake8)
uv run ruff check packages/
# or: make lint

# Type checking
uv run mypy packages/terasim/
```

### Running Simulations
```bash
# Run main simulation with HTTP API (default: police pullover scenario)
python run_experiments.py

# Run debug mode with GUI (cutin scenario)
python run_experiments_debug.py

# Start TeraSim FastAPI service
python run_service.py  # Starts on port 8000

# Run basic simulation example
cd examples/scripts/
python example.py  # Set gui_flag=True in script for GUI mode

# Run with custom scenario
python run_experiments.py --config examples/scenarios/urban_construction_ann_arbor.yaml
```

## Architecture Overview

### Monorepo Structure
```
TeraSim/
├── packages/               # Python packages (workspace members)
│   ├── terasim/           # Core simulation engine
│   ├── terasim-nde-nade/  # NDE-NADE algorithms for naturalistic/adversarial environments
│   ├── terasim-service/   # FastAPI service for integration
│   ├── terasim-envgen/    # Environment generation tools
│   ├── terasim-datazoo/   # Data processing utilities
│   └── terasim-vis/       # Visualization tools
├── examples/              # Example simulations and scenarios
├── configs/               # Configuration files
├── tests/                 # Test suites
└── sumo/                  # SUMO source code (if present)
```

### Core Components

**Simulator (`packages/terasim/terasim/simulator.py`)**: Central orchestrator managing SUMO integration, synchronization, and simulation lifecycle. Key methods:
- `__init__()`: Configure SUMO, GUI, output paths, traffic scale
- `bind_env()`: Attach environment to simulator
- `start()`: Initialize SUMO and agents
- `run()`: Execute simulation steps
- `close()`: Clean up resources

**Environment System (`packages/terasim/terasim/envs/`)**: Testing environment abstractions:
- `BaseEnv`: Abstract base with lifecycle hooks (`on_start`, `on_step`, `on_stop`)
- `EnvTemplate`: Standard testing scenario implementation
- Environments control simulation flow, agent creation, and termination conditions

**Agent Architecture**: Modular sensor-decision-controller design:
- **Agent** (`agent/agent.py`): Base class for all entities
- **Vehicle** (`vehicle/vehicle.py`): Vehicle-specific agent
- **Sensors** (`vehicle/sensors/`): EgoSensor, LocalSensor for perception
- **Decision Models** (`vehicle/decision_models/`): IDMModel, SUMOModel for behavior
- **Controllers** (`vehicle/controllers/`): HighEfficiencyController, SUMOMoveController for actuation
- **Factories** (`vehicle/factories/`): VehicleFactory for creating configured vehicles

**Pipeline System (`terasim/pipeline.py`)**: Ordered execution framework with priority-based scheduling for simulation steps.

### Service Architecture

**TeraSim Service (`packages/terasim-service/`)**: RESTful API for integration:
- FastAPI-based service for external simulator integration
- Supports CARLA co-simulation
- Real-time visualization with Dash
- Message passing for agent states and commands

### Key Design Patterns

1. **Factory Pattern**: Vehicle and agent creation with customizable components
2. **Observer Pattern**: Sensors observe environment, decision models react
3. **Strategy Pattern**: Interchangeable decision models and controllers
4. **Pipeline Pattern**: Ordered, prioritized execution of simulation steps

## Development Notes

### SUMO Integration
- SUMO 1.23.1 required (installed automatically)
- TraCI for real-time control, libsumo for performance
- Supports GUI (`sumo-gui`) and headless (`sumo`) modes
- Network files: `.net.xml` (topology), `.rou.xml` (routes), `.sumocfg` (config)

### Agent Lifecycle
1. **Creation**: Factory creates agent with configured components
2. **Registration**: Agent registers with SUMO via simulator
3. **Execution Loop**: `sense()` → `decide()` → `control()` each step
4. **Cleanup**: Proper removal from SUMO and simulator

### Testing Strategy
- Unit tests per package in `tests/test_*/`
- Integration tests in `tests/test_integration/`
- Markers: `@pytest.mark.slow`, `@pytest.mark.requires_sumo`
- Coverage target: >80% for core modules

### Package Dependencies
- **Core**: eclipse-sumo==1.23.1, numpy==1.26.4, scipy, attrs, bidict
- **Service**: fastapi, uvicorn, redis, dash
- **NDE-NADE**: Cython extensions for performance
- **Visualization**: matplotlib, plotly, folium

## Common Development Tasks

### Adding New Vehicle Behavior
1. Create decision model in `packages/terasim/terasim/vehicle/decision_models/`
2. Inherit from `AgentDecisionModel`, implement `decide()` method
3. Create custom `VehicleFactory` with new model
4. Test with scenario in `examples/scenarios/`

### Creating Custom Environment
1. Extend `BaseEnv` in `packages/terasim/terasim/envs/`
2. Implement `on_start()`, `on_step()`, `on_stop()` methods
3. Define termination conditions and metrics
4. Bind to simulator with `sim.bind_env(env)`

### Adding Adversarial Behavior
1. Create adversity in `packages/terasim-nde-nade/terasim_nde_nade/adversity/`
2. Define trigger conditions and behavior modifications
3. Register in adversity configuration
4. Test with NADE environment

### Running with CARLA Co-simulation
1. Start CARLA server
2. Configure co-simulation in service settings
3. Use `terasim-service` with cosim plugin
4. Exchange states via API endpoints
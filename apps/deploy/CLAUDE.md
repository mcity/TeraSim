# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is TeraSim-Deploy, a containerized deployment platform for TeraSim, an autonomous driving simulation environment. The project consists of three main components:

1. **TeraSim** - Core simulation engine for AV testing with SUMO traffic simulation
2. **TeraSim-Service** - FastAPI-based HTTP service providing REST API interface
3. **TeraSim-NDE-NADE** - Neural differential equations component for enhanced simulation

## Common Development Commands

### Environment Setup
```bash
# Download and setup all repositories
bash scripts/download_repo.sh

# Setup environment
bash scripts/setup_environment.sh

# Setup individual components with Poetry (from root)
cd ../.. && poetry install

# Prerequisites: Redis server must be running
sudo systemctl start redis-server
```

### Running the Application
```bash
# From project root:
# Start the main service (runs on port 8000)
python run_service.py

# Run experiments
python run_experiments.py

# Run debug experiments
python run_experiments_debug.py

# Kill service if port 8000 is in use
kill -9 $(lsof -t -i:8000)
```

### Docker Operations
```bash
# Build all components (CPU version)
./scripts/build.sh --version=1.0.0

# Build with GPU support
./scripts/build.sh --version=1.0.0 --gpu

# Build specific components
./scripts/build.sh --version=1.0.0 --components=terasim_service,terasim_nde_nade

# Start services
docker-compose -f docker/docker-compose.yml up -d

# TeraSim-CARLA co-simulation
docker-compose -f docker/docker-compose-terasim-carla-cosim.yml up -d
docker-compose -f docker/docker-compose-terasim-carla-cosim-offscreen.yml up -d
```

### Testing and Quality Assurance
```bash
# Run tests for each component
cd TeraSim && poetry run pytest
cd TeraSim-Service && poetry run pytest
cd TeraSim-NDE-NADE && poetry run pytest

# Code formatting and linting
cd TeraSim && poetry run black . && poetry run isort . && poetry run flake8
cd TeraSim-Service && poetry run black . && poetry run isort . && poetry run flake8

# Type checking
cd TeraSim && poetry run mypy terasim/
cd TeraSim-Service && poetry run mypy terasim_service/
```

## Architecture Overview

### Core Components
- **TeraSim**: Physics-based simulation engine with SUMO integration for traffic simulation
- **TeraSim-Service**: FastAPI service providing HTTP endpoints for simulation control
- **TeraSim-NDE-NADE**: Neural differential equations for advanced scenario generation

### Key Directories (from project root)
- `/configs/` - Core system configuration files
- `/examples/scenarios/` - Example simulation scenarios
- `/examples/maps/` - Map files for different test environments
- `apps/deploy/docker/` - Docker configuration files for deployment
- `apps/deploy/scripts/` - Deployment and setup scripts

### API Architecture
The service exposes REST endpoints for:
- Simulation lifecycle management (start, stop, tick)
- Real-time state monitoring
- AV route planning and control
- Result retrieval and analysis

### Configuration System
- Uses YAML files for simulation configuration
- Supports multiple environment types (Mcity, Town10, Austin)
- Configurable traffic scenarios and AV behaviors

## Development Notes

- The project uses Poetry for dependency management across all components
- Redis is required for inter-component communication
- CARLA integration available for advanced co-simulation scenarios
- All components follow consistent code style (Black, isort, flake8)
- Type checking enforced with mypy
- Test coverage reporting with pytest-cov

## Common Issues

- Port 8000 conflicts: Use `kill -9 $(lsof -t -i:8000)` to clean up
- Redis connection issues: Ensure Redis is running on port 6379
- CARLA integration requires specific version 0.9.15
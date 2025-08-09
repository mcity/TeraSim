# TeraSim-Deploy

A deployment platform for TeraSim, an autonomous vehicle simulation environment with integrated traffic simulation capabilities.

## Overview

TeraSim-Deploy enables you to:
- Run local simulations with visualization for debugging autonomous vehicle scenarios
- Deploy TeraSim as a FastAPI service for remote access
- Visualize simulation results through a web interface on port (5050 by default)
- Test various traffic scenarios defined in YAML configuration files

## Environment Setup

### Prerequisites
- Conda environment manager
- GCC compiler
- Redis server (for inter-component communication)

### Installation

1. Create and activate a conda environment:
```bash
conda create -n terasim python=3.10
conda activate terasim
```

2. Install GCC if not already available:
```bash
# Ubuntu/Debian
sudo apt-get install gcc

# macOS
brew install gcc
```

3. Run the environment setup script (must be executed after GCC installation):
```bash
bash setup_environment.sh
```

This script will automatically install TeraSim and all its dependencies.

## Usage

### Local Simulation with Visualization

Use `run_experiments_debug.py` to run simulations locally with SUMO GUI visualization:

```bash
python run_experiments_debug.py
```

This script:
- Runs simulations with GUI enabled for visual debugging
- Allows you to observe autonomous vehicle behavior in real-time
- By default runs the `stalled_vehicle_disappear_front_vehicle.yaml` scenario

To run a different scenario, modify line 62 in `run_experiments_debug.py`:
```python
yaml_files = [Path("your_scenario.yaml")]
```

### Available Scenarios

The repository includes several pre-defined traffic accident scenarios as YAML files in the root directory:

- `construction_zone.yaml` - Construction zone scenario
- `construction_zone_with_truck.yaml` - Construction zone with truck obstacle
- `cutin.yaml` - Vehicle cut-in scenario
- `police_pullover_case.yaml` - Police pullover scenario
- `stalled_vehicle.yaml` - Stalled vehicle scenario
- `stalled_vehicle_disappear_front_vehicle.yaml` - Disappearing stalled vehicle scenario

Additional scenarios are available in the `config_yamls/` directory.

### Deploy as FastAPI Service

To deploy TeraSim as a web service:

```bash
python terasim_service_main.py
```

This starts:
- FastAPI service on port 8000
- Visualization debugging service on port 5050

### Using the API Service

Once the service is running, you can interact with it using the REST API. The file `terasim_request.rest` provides example requests:

```http
# Start a simulation with visualization
POST http://localhost:8000/start_simulation?enable_viz=true&viz_port=8050&viz_update_freq=1
{
    "config_file": "stalled_vehicle_disappear_front_vehicle.yaml",
    "auto_run": false
}

# Get simulation status
GET http://localhost:8000/simulation_status/{simulation_id}

# Advance simulation by one tick
POST http://localhost:8000/simulation_tick/{simulation_id}

# Get simulation state
GET http://localhost:8000/simulation/{simulation_id}/state

# Stop simulation
POST http://localhost:8000/simulation_control/{simulation_id}
{
    "command": "stop"
}
```

### Visualization Service

The visualization service runs on port 5050 and provides:
- Real-time visualization of simulation state
- Vehicle trajectories and positions
- Traffic flow analysis
- Debugging interface for scenario development

Access the visualization at: `http://localhost:5050`

## Project Structure

```
TeraSim-Deploy/
├── TeraSim/                   # Core simulation engine
├── TeraSim-Service/           # FastAPI service wrapper
├── TeraSim-NDE-NADE/          # Neural differential equations component
├── *.yaml                     # Scenario definition files (root directory)
├── config_yamls/              # Additional scenario configurations
├── run_experiments_debug.py   # Local visualization runner
├── terasim_service_main.py    # FastAPI service launcher
├── terasim_request.rest       # API usage examples
└── setup_environment.sh       # Environment setup script
```

## Troubleshooting

1. **Port already in use**: If port 8000 is occupied:
   ```bash
   kill -9 $(lsof -t -i:8000)
   ```

2. **Redis connection error**: Ensure Redis is running:
   ```bash
   sudo systemctl start redis-server
   ```

3. **SUMO not found**: The setup script should install SUMO automatically. If issues persist, install manually:
   ```bash
   pip install eclipse-sumo==1.23.1 libsumo==1.23.1 traci==1.23.1 sumolib==1.23.1
   ```

## Development

To create custom scenarios:
1. Copy an existing YAML file as a template
2. Modify vehicle positions, routes, and behaviors
3. Test locally with `run_experiments_debug.py`
4. Deploy via the FastAPI service for production use

## Notes

- Ensure all dependencies are properly installed before running the service
- The service must be running before sending test requests
- Check logs for any potential errors during execution

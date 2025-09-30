# TeraSim-Cosmos Bridge Package

TeraSim-Cosmos is a bridge package that converts TeraSim traffic simulation data (SUMO map and FCD) into NVIDIA Cosmos-Drive compatible inputs for autonomous vehicle world model training and video generation. This package enables seamless integration between TeraSim simulations and NVIDIA's Cosmos-Drive platform.

This package is adapted from [Cosmos-Drive-Dreams toolkits](https://github.com/nv-tlabs/Cosmos-Drive-Dreams/tree/main/cosmos-drive-dreams-toolkits) and extended with TeraSim-specific functionality for seamless integration with TeraSim's simulation outputs.

## Overview

TeraSim-Cosmos provides:

* **TeraSim-to-Cosmos-Drive Conversion**: Convert TeraSim simulation outputs (SUMO map and FCD files) into RDS-HQ format fully compatible with NVIDIA Cosmos-Drive platform
* **HD Map Video Generation**: Generate high-quality HD map videos for world model training and autonomous vehicle simulation
* **Street View Integration**: Retrieve and analyze real-world street view images to enhance simulation realism and provide textual descriptions
* **Multi-Camera Support**: Process data from multiple camera viewpoints (f-theta and pinhole camera models) matching Cosmos-Drive requirements

The package enables direct integration with NVIDIA Cosmos-Drive by generating properly formatted HD map videos and associated metadata that can be seamlessly used for world model training and video generation.

## Key Components

### Core Conversion Pipeline (`TeraSimToCosmosConverter`)
Main converter class that:
- Processes TeraSim FCD (Floating Car Data) and SUMO map files
- Extracts vehicle trajectories and collision information
- Converts data to WebDataset (WDS) format compatible with Cosmos-Drive
- Renders HD map videos and sensor data for world model training

### Data Processing Modules
- `convert_terasim_to_rds_hq.py`: Converts TeraSim data to RDS-HQ format for Cosmos-Drive compatibility
- `street_view_analysis.py`: Integrates street view imagery and generates textual descriptions
- `render_from_rds_hq.py`: Renders HD map videos and depth visualizations for Cosmos-Drive input

### Visualization Tools
- `visualize_rds_hq.py`: Interactive 3D visualization using Viser
- `terasim_vis.py`: TeraSim-specific visualization utilities

### Utilities (`utils/`)
- Camera models (f-theta, pinhole)
- Animation and bounding box utilities
- Point cloud processing
- WebDataset utilities

## Integration with TeraSim and Cosmos-Drive

This package seamlessly integrates with TeraSim's output format, reading:
- **FCD files** (`fcd_all.xml`): Vehicle trajectory and state information
- **Map files** (`map.net.xml`): Road network topology from SUMO
- Monitor files (`monitor.json`): Collision and event records (optional)

The converted outputs are fully compatible with NVIDIA Cosmos-Drive platform and can be directly used for:
- **World Model Training**: HD map videos for autonomous vehicle simulation
- **Video Generation**: High-quality driving scenario videos
- **Scene Understanding**: Multi-camera view rendering with depth information


## Quick Start

### Installation and Usage

Install the package:
```bash
pip install -e packages/terasim-cosmos
```

Convert TeraSim simulation outputs to Cosmos-Drive compatible inputs:

```python
from terasim_cosmos import TeraSimToCosmosConverter

# Create converter from configuration file
converter = TeraSimToCosmosConverter.from_config_file('configs/converter/scenario.yaml')

# Run conversion with street view retrieval
converter.convert(streetview_retrieval=True)
```

### Configuration File

Create a YAML configuration file:

```yaml
path_to_output: "path/to/output"
path_to_fcd: "path/to/fcd_all.xml"
path_to_map: "path/to/map.net.xml"
camera_setting_name: "default"  # or "waymo"
vehicle_id: "vehicle_001"
time_start: 460.0
time_end: 464.0
agent_clip_distance: 30.0
map_clip_distance: 100.0
```

### Command Line Usage

```bash
python convert_terasim_to_cosmos.py --path_to_config configs/converter/scenario.yaml --streetview_retrieval True
```

### Parameters

- **path_to_output**: Directory containing TeraSim simulation outputs
- **path_to_fcd**: Path to the FCD (Floating Car Data) XML file
- **path_to_map**: Path to the SUMO network map XML file
- **camera_setting_name**: Camera configuration ("default" for RDS-HQ, "waymo" for Waymo-style)
- **vehicle_id**: Specific vehicle to track (None for auto-detection from collision records)
- **timestep_start/end**: Time window to process (-1 for last timestep)

### Output Structure

After conversion, the output directory will contain:

```
<output_directory>/
├── wds/                    # WebDataset format data
├── render/                 # Rendered visualizations
│   └── <vehicle_id>/
│       ├── hdmap/         # HD map renderings
│       └── lidar/         # LiDAR visualizations
├── streetview/            # Retrieved street view images
└── monitor.json           # Collision/event records
```

## Installation

### Prerequisites
- TeraSim installation (see main TeraSim README)
- NVIDIA GPU (required for LiDAR rendering)
- Python 3.10+

### Setup
```bash
# From the TeraSim root directory
cd packages/terasim-cosmos
pip install -e .
```

### Additional Dependencies
For Waymo Open Dataset support:
```bash
pip install waymo-open-dataset-tf-2-11-0==1.6.1
```

## Acknowledgments

This package is adapted from the [Cosmos-Drive-Dreams toolkits](https://github.com/nv-tlabs/Cosmos-Drive-Dreams/tree/main/cosmos-drive-dreams-toolkits) developed by NVIDIA. The original toolkits provide rendering and conversion capabilities for RDS-HQ datasets and Waymo Open Dataset. We have extended these tools with:

- TeraSim-specific data conversion (`terasim_to_cosmos_input.py`, `convert_terasim_to_rds_hq.py`)
- Integration with TeraSim's FCD and SUMO map formats
- Street view analysis for TeraSim scenarios
- Automated vehicle tracking from collision records

## Citation

When using TeraSim-World in your research, please cite both TeraSim and the original Cosmos work:

## License

This subrepository follows the licensing terms of the [Cosmos-Drive-Dreams toolkits](https://github.com/nv-tlabs/Cosmos-Drive-Dreams) and is subject to their license agreement. This package will be synchronously maintained in the Cosmos-Drive-Dreams repository.

For the broader TeraSim project license, see the LICENSE file in the TeraSim root directory.

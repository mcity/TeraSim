# TeraSim-World

TeraSim-World is a bridge package that converts TeraSim traffic simulation data into world model inputs compatible with NVIDIA Cosmos Transfer1 models. This package enables the generation of photorealistic driving scenarios from TeraSim simulations for advanced autonomous vehicle testing and validation.

This package is adapted from [Cosmos-Drive-Dreams toolkits](https://github.com/nv-tlabs/Cosmos-Drive-Dreams/tree/main/cosmos-drive-dreams-toolkits) and extended with TeraSim-specific functionality for seamless integration with TeraSim's simulation outputs.

## Overview

TeraSim-World provides:

* **TeraSim-to-Cosmos Conversion**: Convert TeraSim simulation outputs (FCD files, map data) into RDS-HQ format compatible with Cosmos Transfer1 models
* **HD Map Rendering**: Generate HD map visualizations from simulation data
* **Street View Integration**: Retrieve and analyze real-world street view images to enhance simulation realism
* **Multi-Camera Support**: Process data from multiple camera viewpoints (f-theta and pinhole camera models)

The main entry point is `terasim_to_cosmos_input.py`, which orchestrates the entire conversion pipeline from TeraSim outputs to Cosmos-compatible inputs.

## Key Components

### Core Conversion Pipeline (`terasim_to_cosmos_input.py`)
Main function that:
- Processes TeraSim FCD (Floating Car Data) and map files
- Extracts vehicle trajectories and collision information
- Converts data to WebDataset (WDS) format
- Renders HD maps and sensor data for world model input

### Data Processing Modules
- `convert_terasim_to_rds_hq.py`: Converts TeraSim data to RDS-HQ format
- `convert_waymo_to_rds_hq.py`: Processes Waymo Open Dataset
- `street_view_analysis.py`: Integrates street view imagery and descriptions
- `render_from_rds_hq.py`: Renders HD maps and LiDAR visualizations

### Visualization Tools
- `visualize_rds_hq.py`: Interactive 3D visualization using Viser
- `terasim_vis.py`: TeraSim-specific visualization utilities

### Utilities (`utils/`)
- Camera models (f-theta, pinhole)
- Animation and bounding box utilities
- Point cloud processing
- WebDataset utilities

## Integration with TeraSim

This package seamlessly integrates with TeraSim's output format, reading:
- **FCD files** (`fcd_all.xml`): Vehicle trajectory and state information
- **Map files** (`map.net.xml`): Road network topology from SUMO
- **Monitor files** (`monitor.json`): Collision and event records

The converted outputs can then be used with NVIDIA Cosmos Transfer1 models to generate photorealistic driving scenarios based on TeraSim simulations.


## Quick Start

### Using TeraSim Data

To convert TeraSim simulation outputs to Cosmos-compatible inputs:

```python
from pathlib import Path
from terasim_to_cosmos_input import terasim_to_cosmos_input

# Define paths to TeraSim output
path_to_output = Path("/path/to/terasim/output")
path_to_fcd = path_to_output / "fcd_all.xml"
path_to_map = path_to_output / "map.net.xml"

# Convert to Cosmos input
terasim_to_cosmos_input(
    path_to_output=path_to_output,
    path_to_fcd=path_to_fcd,
    path_to_map=path_to_map,
    camera_setting_name="default",  # or "waymo"
    vehicle_id=None,  # Auto-detect from monitor.json
    timestep_start=0,
    timestep_end=100
)
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
cd packages/terasim-world
conda env create -f environment.yaml
conda activate cosmos-av-toolkits
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

See LICENSE file in the TeraSim root directory. The original Cosmos-Drive-Dreams toolkits are subject to their own licensing terms.

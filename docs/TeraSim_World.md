# TeraSim-World

<div align="center">
<img src="figure/TeraSim_World.png" height="600px">
</div>

**TeraSim-World** automatically synthesizes geographically grounded, safety-critical data for End-to-End autonomous driving **anywhere in the world**. See 📄 [paper](https://arxiv.org/abs/2503.03629) and 🌐 [videos](https://wjiawei.com/terasim-world-web/) for details.



## Quick Start

### 1. Installation

```bash
# Clone the repository
git clone https://github.com/mcity/TeraSim.git
cd TeraSim

# Run automated setup (installs all components including TeraSim-World)
./setup_environment.sh

# Activate environment
conda activate terasim

# Configure API keys in .env file
# Required: OPENAI_API_KEY and GOOGLE_MAPS_API_KEY
```

### 2. Generate Experiments

```bash
python scripts/generate_experiments.py \
    --lat 42.331936167160165 \
    --lon -83.70812725301244 \
    --bbox 500 \
    --output generated_experiments \
    --name ann_arbor_four_way_stop
```

**Key Parameters:**
- `--lat`: Latitude of the center point for scenario generation
- `--lon`: Longitude of the center point (required with --lat)
- `--bbox`: Size of bounding box in meters (default: 500)
- `--output`: Output directory for generated experiment files (default: generated_experiments)
- `--name`: Name for the scenario (auto-generated if not provided)

### 3. Run Experiments (Debug Mode)

Run simulations with visual debugging and real-time monitoring:

```bash
python scripts/run_experiments_debug.py --config path/to/your/config.yaml
```

Example configurations are available in `configs/simulation/example.yaml`.

### 4. Convert to Cosmos-Drive Format

Transform TeraSim outputs into NVIDIA Cosmos-Drive compatible data:

```bash
python scripts/convert_terasim_to_cosmos.py \
    --path_to_output /path/to/output \
    --path_to_fcd /path/to/fcd_all.xml \
    --path_to_map /path/to/map.net.xml \
    --vehicle_id vehicle_001 \
    --time_start 460.0 \
    --time_end 464.0 \
    --streetview_retrieval
```

**Key Parameters:**
- `--path_to_output`: Output directory for converted data
- `--path_to_fcd`: Path to TeraSim FCD (Floating Car Data) XML file
- `--path_to_map`: Path to SUMO network map XML file
- `--vehicle_id`: ID of the ego vehicle to track
- `--time_start/--time_end`: Time window in seconds for conversion
- `--streetview_retrieval`: Enable real-world street view image integration

### 5. Use with Cosmos-Drive

The converted output includes:
- **WebDataset Format**: Compatible with Cosmos-Drive data format
- **HD Map Videos**: Multi-view HDMap videos
- **Street View Images**: Multi-view street view images from Google Maps
- **Text Prompts**: Scene descriptions from street view analysis

Import the data into [Cosmos-Drive-Dreams](https://github.com/nv-tlabs/Cosmos-Drive-Dreams) for multi-view video generation.


## Citation

```bibtex
@article{wang2025terasim-world,
  author    = {Wang, Jiawei and Sun, Haowei and Yan, Xintao and Feng, Shuo and Gao, Jun and Liu, Henry},
  title     = {TeraSim-World: Worldwide Safety-Critical Data Synthesis for End-to-End Autonomous Driving},
  journal   = {arXiv preprint arXiv:2509.13164},
  year      = {2025},
}
```
# TeraSim-Cosmos GPU Docker Guide

This guide covers GPU-accelerated Docker deployment for TeraSim-Cosmos, enabling high-performance world model training and video generation with NVIDIA Cosmos-Drive integration.

## Prerequisites

### Hardware Requirements
- **NVIDIA GPU** with CUDA 11.8+ support (required for LiDAR rendering and ML inference)
- Minimum 16GB RAM (32GB recommended)
- 50GB+ free disk space

### Software Requirements
- Docker Engine 19.03+
- Docker Compose 1.28+ (with GPU support)
- NVIDIA Container Toolkit

### Installing NVIDIA Container Toolkit

#### Ubuntu/Debian
```bash
# Add NVIDIA package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify installation
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

#### Other Systems
See official guide: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

---

## Quick Start

### Build and Run

```bash
# Navigate to terasim-cosmos directory
cd packages/terasim-cosmos

# Build the GPU-enabled image
docker-compose -f docker-compose.gpu.yml up -d --build

# Enter the container
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos bash

# Verify GPU is available
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
python -c "import tensorflow as tf; print(f'GPU devices: {tf.config.list_physical_devices(\"GPU\")}')"
```

### Convert TeraSim Simulation to Cosmos-Drive Format

```bash
# Inside the container
cd /app/TeraSim

# Run conversion with street view retrieval
python packages/terasim-cosmos/terasim_cosmos/converter.py \
    --path_to_config configs/converter/scenario.yaml \
    --streetview_retrieval True
```

---

## Usage Examples

### Example 1: Convert Simulation Output

```python
from terasim_cosmos import TeraSimToCosmosConverter

# Create converter from configuration
converter = TeraSimToCosmosConverter.from_config_file(
    'configs/converter/scenario.yaml'
)

# Run conversion
converter.convert(streetview_retrieval=True)
```

### Example 2: Render HD Map Video

```bash
# Inside container
python packages/terasim-cosmos/terasim_cosmos/render_from_rds_hq.py \
    --input_path outputs/scenario_001/wds \
    --output_path outputs/scenario_001/render \
    --camera_setting default
```

### Example 3: Visualize RDS-HQ Data

```bash
# Start interactive 3D visualization
python packages/terasim-cosmos/terasim_cosmos/visualize_rds_hq.py \
    --data_path outputs/scenario_001/wds \
    --port 8080
```

---

## Configuration

### Configuration File Format

Create a YAML configuration file (e.g., `configs/converter/my_scenario.yaml`):

```yaml
# Paths
path_to_output: "outputs/my_scenario"
path_to_fcd: "outputs/simulation_run/fcd_all.xml"
path_to_map: "examples/maps/Mcity/map.net.xml"

# Camera settings
camera_setting_name: "default"  # Options: "default", "waymo"

# Vehicle tracking
vehicle_id: null  # Set to specific ID, or null for auto-detection from collisions

# Time window
time_start: 460.0  # Start time in seconds
time_end: 464.0    # End time in seconds

# Clipping distances
agent_clip_distance: 30.0   # Distance to include surrounding agents (meters)
map_clip_distance: 100.0    # Distance to include map elements (meters)
```

### Environment Variables

Configure in `docker-compose.gpu.yml` or set manually:

```bash
# GPU visibility
export NVIDIA_VISIBLE_DEVICES=all

# TensorFlow GPU memory growth (prevents OOM errors)
export TF_FORCE_GPU_ALLOW_GROWTH=true

# PyTorch CUDA settings
export CUDA_VISIBLE_DEVICES=0  # Use first GPU only
```

---

## Common Operations

### Running Jupyter Notebook

```bash
# Start Jupyter inside container
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos \
    jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root

# Access at: http://localhost:8888
```

### Batch Processing Multiple Scenarios

```bash
# Create a batch processing script
cat > batch_convert.sh << 'EOF'
#!/bin/bash
for config in configs/converter/*.yaml; do
    echo "Processing: $config"
    python packages/terasim-cosmos/terasim_cosmos/converter.py \
        --path_to_config "$config"
done
EOF

chmod +x batch_convert.sh
./batch_convert.sh
```

### Monitor GPU Usage

```bash
# Watch GPU utilization in real-time
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos \
    watch -n 1 nvidia-smi

# Or from host machine
watch -n 1 nvidia-smi
```

---

## Performance Optimization

### GPU Memory Management

```python
# For TensorFlow - enable memory growth
import tensorflow as tf
gpus = tf.config.list_physical_devices('GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)

# For PyTorch - clear cache periodically
import torch
torch.cuda.empty_cache()
```

### Parallel Processing

```python
# Use Ray for parallel processing
import ray
from terasim_cosmos import TeraSimToCosmosConverter

ray.init(num_gpus=1)

@ray.remote(num_gpus=0.25)  # Use 1/4 GPU per task
def convert_scenario(config_path):
    converter = TeraSimToCosmosConverter.from_config_file(config_path)
    converter.convert()
    return config_path

# Process multiple scenarios in parallel
futures = [convert_scenario.remote(f"configs/converter/scenario_{i}.yaml")
           for i in range(4)]
results = ray.get(futures)
```

---

## Troubleshooting

### GPU Not Detected

```bash
# Check NVIDIA driver
nvidia-smi

# Check Docker GPU support
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# Check container GPU access
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos nvidia-smi
```

### CUDA Out of Memory

```bash
# Reduce batch size in your code
# Enable TensorFlow memory growth
export TF_FORCE_GPU_ALLOW_GROWTH=true

# Or set explicit memory limit
export TF_GPU_MEMORY_FRACTION=0.8  # Use 80% of GPU memory
```

### Import Errors for waymo_open_dataset

```bash
# Reinstall Waymo dataset package
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos \
    pip install --force-reinstall waymo-open-dataset-tf-2-11-0==1.6.1
```

### Slow Rendering Performance

```bash
# Check if GPU is actually being used
python -c "import torch; torch.cuda.is_available() and print('GPU OK')"

# Enable CUDA optimization
export CUDA_LAUNCH_BLOCKING=0
export TORCH_CUDNN_V8_API_ENABLED=1
```

---

## Output Structure

After conversion, your output directory will contain:

```
outputs/my_scenario/
├── wds/                          # WebDataset format (Cosmos-Drive compatible)
│   ├── 000000.tar                # Compressed data shards
│   ├── 000001.tar
│   └── ...
├── render/                       # Rendered visualizations
│   └── vehicle_001/
│       ├── hdmap/               # HD map renderings
│       │   ├── frame_0000.png
│       │   ├── frame_0001.png
│       │   └── video.mp4
│       └── lidar/               # LiDAR visualizations
│           ├── frame_0000.png
│           └── video.mp4
├── streetview/                  # Retrieved street view images
│   ├── location_001.jpg
│   └── descriptions.json
└── monitor.json                 # Collision/event records
```

---

## Integration with NVIDIA Cosmos-Drive

The output from TeraSim-Cosmos is directly compatible with NVIDIA Cosmos-Drive platform:

```bash
# Export WebDataset for Cosmos-Drive training
cosmos-drive-train \
    --data_path outputs/my_scenario/wds \
    --output_path models/world_model \
    --config cosmos_config.yaml

# Generate videos using trained model
cosmos-drive-generate \
    --model_path models/world_model \
    --prompt_path outputs/my_scenario/wds/000000.tar \
    --output_path generated_videos/
```

See [NVIDIA Cosmos-Drive documentation](https://developer.nvidia.com/cosmos) for details.

---

## Differences from Base Dockerfile

| Feature | Base Dockerfile | GPU Dockerfile |
|---------|----------------|----------------|
| Base Image | Ubuntu 22.04 | NVIDIA CUDA 11.8 + cuDNN 8 |
| PyTorch | CPU-only | CUDA 11.8 |
| TensorFlow | CPU-only | GPU-enabled |
| GPU Libraries | ❌ | ✅ (CUDA, cuDNN, NCCL) |
| Waymo Dataset | ❌ | ✅ |
| Rendering | Software | GPU-accelerated |
| Image Size | ~3-4 GB | ~8-10 GB |
| terasim-cosmos | ⚠️ Limited | ✅ Full support |

---

## Resource Requirements

### Minimum Configuration
- GPU: NVIDIA GTX 1080 or equivalent (8GB VRAM)
- RAM: 16GB
- Disk: 50GB
- CPU: 4 cores

### Recommended Configuration
- GPU: NVIDIA RTX 3090 or A100 (24GB+ VRAM)
- RAM: 32GB
- Disk: 100GB SSD
- CPU: 8+ cores

---

## Additional Resources

- **TeraSim-Cosmos Package**: See [README.md](README.md)
- **NVIDIA Cosmos-Drive**: https://developer.nvidia.com/cosmos
- **Waymo Open Dataset**: https://waymo.com/open/
- **TeraSim Main Documentation**: [../../README.md](../../README.md)

---

## License

This Docker configuration follows the licensing of TeraSim-Cosmos package. See [../../LICENSE](../../LICENSE) for details.
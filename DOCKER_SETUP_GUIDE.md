# TeraSim Docker Setup Guide

## Overview

TeraSim now provides two Docker deployment options:

1. **Base Image**: Lightweight deployment for core TeraSim functionality (simulation, planning, control testing)
2. **GPU Image**: Full-featured deployment for TeraSim-Cosmos with NVIDIA GPU support (world model training, video generation)

---

## Quick Decision Guide

### Use **Base Docker Image** if you need:
- ✅ Traffic simulation with SUMO
- ✅ Naturalistic and adversarial environment generation (NDE-NADE)
- ✅ Planning and control testing
- ✅ FastAPI service for integration
- ✅ Basic visualization tools
- ✅ Smaller image size (~2-3 GB)
- ✅ No GPU required

### Use **GPU Docker Image** if you need:
- ✅ TeraSim-Cosmos functionality
- ✅ NVIDIA Cosmos-Drive integration
- ✅ HD map video generation
- ✅ LiDAR rendering
- ✅ ML model training/inference
- ✅ Street view analysis
- ⚠️ Requires NVIDIA GPU
- ⚠️ Larger image size (~8-10 GB)

---

## Option 1: Base Docker Image

### Quick Start

```bash
# From TeraSim root directory
docker-compose up -d --build
docker-compose exec terasim bash

# Inside container - run simulation
python run_experiments.py
```

### What's Included
- Core TeraSim packages:
  - `terasim` - Core simulation engine
  - `terasim-nde-nade` - Naturalistic & adversarial environments
  - `terasim-service` - FastAPI service
  - `terasim-envgen` - Environment generation
  - `terasim-datazoo` - Data processing
  - `terasim-vis` - Visualization tools
- SUMO 1.23.1 traffic simulator
- Redis for service components
- Development tools (pytest, black, ruff, mypy)

### What's NOT Included
- ❌ `terasim-cosmos` (requires GPU)
- ❌ PyTorch/TensorFlow with GPU support
- ❌ Waymo Open Dataset tools
- ❌ GPU-accelerated rendering

### Documentation
See [README_DOCKER.md](README_DOCKER.md)

---

## Option 2: GPU Docker Image (TeraSim-Cosmos)

### Prerequisites

#### Hardware
- NVIDIA GPU with CUDA 11.8+ support
- 16GB+ RAM (32GB recommended)
- 50GB+ free disk space

#### Software
Install NVIDIA Container Toolkit:

```bash
# Ubuntu/Debian
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test installation
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### Quick Start

```bash
# Navigate to terasim-cosmos directory
cd packages/terasim-cosmos

# Build and start GPU container
docker-compose -f docker-compose.gpu.yml up -d --build

# Enter container
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos bash

# Inside container - verify GPU
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"

# Run conversion
python packages/terasim-cosmos/terasim_cosmos/converter.py \
    --path_to_config configs/converter/scenario.yaml \
    --streetview_retrieval True
```

### What's Included
- **Everything from Base Image**, PLUS:
- `terasim-cosmos` package
- PyTorch with CUDA 11.8
- TensorFlow with GPU support
- Waymo Open Dataset
- GPU-accelerated rendering libraries
- Additional ML tools (transformers, ray)
- Jupyter Notebook

### Documentation
See [packages/terasim-cosmos/README_DOCKER_GPU.md](packages/terasim-cosmos/README_DOCKER_GPU.md)

---

## Comparison Table

| Feature | Base Image | GPU Image |
|---------|------------|-----------|
| **Base OS** | Ubuntu 22.04 | Ubuntu 22.04 + CUDA 11.8 |
| **Image Size** | ~2-3 GB | ~8-10 GB |
| **Build Time** | 10-15 min | 30-60 min |
| **GPU Required** | No | Yes (NVIDIA) |
| **RAM Required** | 4GB+ | 16GB+ |
| **Disk Space** | 10GB+ | 50GB+ |
| | | |
| **Core Packages** | | |
| terasim | ✅ | ✅ |
| terasim-nde-nade | ✅ | ✅ |
| terasim-service | ✅ | ✅ |
| terasim-envgen | ✅ | ✅ |
| terasim-datazoo | ✅ | ✅ |
| terasim-vis | ✅ | ✅ |
| terasim-cosmos | ❌ | ✅ |
| | | |
| **ML Libraries** | | |
| PyTorch | ❌ | ✅ (CUDA 11.8) |
| TensorFlow | ❌ | ✅ (GPU) |
| Transformers | ❌ | ✅ |
| Ray | ❌ | ✅ |
| | | |
| **Capabilities** | | |
| Traffic simulation | ✅ | ✅ |
| NDE-NADE testing | ✅ | ✅ |
| FastAPI service | ✅ | ✅ |
| Cosmos-Drive integration | ❌ | ✅ |
| HD map video gen | ❌ | ✅ |
| LiDAR rendering | ❌ | ✅ |
| Street view analysis | ❌ | ✅ |
| GPU acceleration | ❌ | ✅ |

---

## Common Workflows

### Workflow 1: Basic Simulation Testing (Use Base Image)

```bash
# Start base container
docker-compose up -d
docker-compose exec terasim bash

# Run simulation
python run_experiments.py

# Run tests
pytest

# View outputs
ls outputs/
```

### Workflow 2: Cosmos-Drive Video Generation (Use GPU Image)

```bash
# Start GPU container
cd packages/terasim-cosmos
docker-compose -f docker-compose.gpu.yml up -d
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos bash

# Step 1: Run TeraSim simulation (or use existing outputs)
cd /app/TeraSim
python run_experiments.py

# Step 2: Convert to Cosmos format
python packages/terasim-cosmos/terasim_cosmos/converter.py \
    --path_to_config configs/converter/my_scenario.yaml

# Step 3: View outputs
ls outputs/my_scenario/render/
```

### Workflow 3: Development with Live Code Changes

```bash
# For base image - mount source code
docker run -it --rm \
    -v $(pwd)/packages:/app/TeraSim/packages \
    -v $(pwd)/outputs:/app/TeraSim/outputs \
    terasim:latest bash

# For GPU image - edit docker-compose.gpu.yml to add volume:
# volumes:
#   - ../../packages/terasim-cosmos:/app/TeraSim/packages/terasim-cosmos
```

---

## Troubleshooting

### Issue: "libproj25 not found" during build
**Solution**: This has been fixed. Update to latest Dockerfile (uses libproj22).

### Issue: "terasim_cosmos import error" in base image
**Solution**: This is expected. Use GPU image for terasim-cosmos functionality.

### Issue: GPU not detected in GPU container
**Solution**:
```bash
# Verify NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# Check GPU visibility in container
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos nvidia-smi
```

### Issue: CUDA out of memory
**Solution**:
```bash
# Set memory growth for TensorFlow
export TF_FORCE_GPU_ALLOW_GROWTH=true

# Reduce batch sizes in your code
```

---

## File Structure

```
TeraSim/
├── Dockerfile                           # Base image (no GPU)
├── docker-compose.yml                   # Base image compose
├── README_DOCKER.md                     # Base image docs
├── DOCKER_SETUP_GUIDE.md               # This file
│
├── packages/terasim-cosmos/
│   ├── Dockerfile.gpu                  # GPU-enabled image
│   ├── docker-compose.gpu.yml          # GPU image compose
│   └── README_DOCKER_GPU.md            # GPU image docs
│
├── outputs/                            # Simulation outputs (mounted)
├── logs/                               # Log files (mounted)
└── configs/                            # Configuration files
    └── converter/                      # Cosmos converter configs
```

---

## Next Steps

### For Base Image Users
1. Read [README_DOCKER.md](README_DOCKER.md)
2. Run example simulations
3. Explore NDE-NADE scenarios
4. Test FastAPI service integration

### For GPU Image Users
1. Install NVIDIA Container Toolkit
2. Read [packages/terasim-cosmos/README_DOCKER_GPU.md](packages/terasim-cosmos/README_DOCKER_GPU.md)
3. Run TeraSim simulation
4. Convert outputs to Cosmos-Drive format
5. Generate HD map videos
6. Integrate with NVIDIA Cosmos platform

---

## Support

- **Main Documentation**: [README.md](README.md)
- **Development Guide**: [CLAUDE.md](CLAUDE.md)
- **GitHub Issues**: https://github.com/mcity/TeraSim/issues
- **Discussions**: https://github.com/mcity/TeraSim/discussions

---

## License

- **TeraSim Core**: Apache 2.0 License
- **TeraSim Visualization**: MIT License

See [LICENSE](LICENSE) for details.
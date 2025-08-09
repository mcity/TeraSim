# TeraSim Deploy

TeraSim Deploy provides containerized deployment solutions for the TeraSim autonomous driving simulation platform. This repository orchestrates the deployment of various TeraSim components using Docker.

## Components

TeraSim Deploy integrates the following components:
- `terasim`: Core simulation engine
- `terasim_service`: HTTP service interface
- `terasim_nde_nade`: Neural differential equations component
<!-- - `terasim_data_zoo`: Data management utilities
- `terasim_gpt`: GPT-based simulation enhancement -->

## Prerequisites

- Docker >= 20.10
- Docker Compose >= 2.0
- NVIDIA Container Toolkit (for GPU support)
- 16GB+ RAM recommended
- 50GB+ free disk space

## Quick Start

1. Clone the repository:
```bash
git clone https://github.com/mcity/TeraSim-Deploy.git
cd TeraSim-Deploy
bash download_repo.sh # download TeraSim, TeraSim-NDE-NADE, TeraSim-Service
```

2. Build images:
```bash
# For CPU version
./scripts/build.sh --version=1.0.0

# # For GPU version
# ./scripts/build.sh --version=1.0.0 --gpu

# # Build specific components
# ./scripts/build.sh --version=1.0.0 --gpu --components=terasim_service,terasim_nde_nade
```

3. Start services:
```bash
# Start all services
docker-compose -f docker/docker-compose.yml up -d

# Stop all services
docker-compose -f docker/docker-compose.yml down
```

4. Verify deployment:
```bash
curl http://localhost:8000/health
```
If the service is running and healthy, you might see:
```json
{"status": "healthy"}
```

## Advanced Usage

1. TeraSim-Carla co-simulation in Mcity
We have prepared two pre-built Docker images, one containing the TeraSim-related packages and the other containing the CARLA-related packages. Both of them are public and you can access them through [link1](https://gallery.ecr.aws/x0u0p9b6/terasim/service) and [link2](https://gallery.ecr.aws/x0u0p9b6/mcity-digital-twin). You can directly use these images to run TeraSim and CARLA in a co-simulation environment.

```bash
# Start TeraSim and CARLA co-simulation with CARLA visualization
docker-compose -f docker/docker-compose-terasim-carla-cosim.yml up -d

# Start TeraSim and CARLA co-simulation without visualization
docker-compose -f docker/docker-compose-terasim-carla-cosim-offscreen.yml up -d

# Stop all services
docker-compose -f docker/docker-compose-terasim-carla-cosim.yml down
docker-compose -f docker/docker-compose-terasim-carla-cosim-offscreen.yml down
```


## Directory Structure

```
TeraSim-Deploy/
├── docker/                                         # Docker configuration
│   ├── base/                                       # Base image configuration
│   │   ├── Dockerfile.base.cpu
│   │   ├── Dockerfile.base.gpu
│   │   └── requirements.*.txt
│   ├── components/                                 # Component-specific Dockerfiles
│   │   ├── terasim/
│   │   ├── terasim_service/
│   │   └── ...
│   └── docker-compose.yml
├── examples/                                       # Example files
│   ├── maps/                                       # Maps
│   │   ├── Mcity_safetest/                         # Mcity map
│   │   └── ...
|   ├── simulation_Mcity_carla_config.yaml          # Simulation configs
│   └── ...                                         
└── scripts/                                        # Utility scripts
    └── build.sh                                    # Build script
```

## Configuration

### Environment Variables

Key configuration options:
- `VERSION`: Version of TeraSim components
- `ENVIRONMENT`: Deployment environment (development/production)
- `CUDA_VISIBLE_DEVICES`: GPU device selection

### Component Configuration

Each component can be configured through:
- Environment variables
- Configuration files in `config/` directory
- Docker Compose overrides

## Health Monitoring

- Service health endpoint: `http://localhost:8000/health`
- Container health checks are configured in docker-compose.yml

## Scaling

```bash
# Scale specific services
docker compose up -d --scale terasim_service=3
```

## Troubleshooting

1. Image building issues:
```bash
# Check build logs
./scripts/build.sh --version=1.0.0 --gpu 2>&1 | tee build.log
```

2. Container issues:
```bash
# Check container logs
docker compose logs -f [service-name]
```

3. GPU support:
```bash
# Verify GPU access
docker run --rm --gpus all nvidia/cuda:12.1.1-base nvidia-smi
```

## Support

For deployment-related issues:
- GitHub Issues: <repository-url>/issues
- Documentation: <docs-url>

## License

[License Type] - See LICENSE file for details 
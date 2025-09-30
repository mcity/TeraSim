# TeraSim Docker Deployment Guide

## Overview

This Dockerfile provides a complete, production-ready environment for TeraSim based on **Ubuntu 22.04** and **Python 3.10**. It includes all necessary dependencies and tools for running autonomous vehicle simulations.

### What's Included

- ✅ **Compilers**: gcc, g++, build-essential
- ✅ **SUMO 1.23.1**: Traffic simulation engine (installed via pip)
- ✅ **SUMO Tools**: Full toolchain cloned to `~/.terasim/deps/sumo`
- ✅ **Redis**: Required for service components
- ✅ **Python Dependencies**: All packages including Cython extensions
- ✅ **Optimized Build**: Multi-stage Docker build (~2-3 GB final image)

---

## Quick Start

### Using Docker Compose (Recommended)

```bash
# Build and start the container
docker-compose up -d --build

# Enter the container
docker-compose exec terasim bash

# Run simulation inside container
python run_experiments.py

# Stop the container
docker-compose down
```

### Using Docker CLI

```bash
# Build the image
docker build -t terasim:latest .

# Run container in interactive mode
docker run -it --name terasim \
  -p 8000:8000 \
  -p 6379:6379 \
  -v $(pwd)/outputs:/app/TeraSim/outputs \
  -v $(pwd)/logs:/app/TeraSim/logs \
  terasim:latest

# Inside the container, start Redis (if needed)
redis-server --daemonize yes

# Run your simulation
python run_experiments.py
```

---

## Common Operations

### Running Different Simulations

```bash
# Standard simulation (police pullover scenario)
docker-compose exec terasim python run_experiments.py

# Debug mode (cutin scenario, GUI required)
docker-compose exec terasim python run_experiments_debug.py

# Start FastAPI service
docker-compose exec terasim python run_service.py

# Run example scripts
docker-compose exec terasim python examples/scripts/example.py
```

### Running Tests

```bash
# Run all tests
docker-compose exec terasim pytest

# Run specific test suite
docker-compose exec terasim pytest tests/test_core/

# Generate HTML coverage report
docker-compose exec terasim pytest --cov=terasim --cov-report=html

# Run only fast tests (skip slow integration tests)
docker-compose exec terasim pytest -m "not slow"
```

### Code Quality Tools

```bash
# Format code with Black
docker-compose exec terasim black packages/

# Lint with Ruff
docker-compose exec terasim ruff check packages/

# Type checking with mypy
docker-compose exec terasim mypy packages/terasim/

# Sort imports with isort
docker-compose exec terasim isort packages/
```

---

## Configuration

### Data Persistence

The Docker Compose configuration automatically mounts volumes for data persistence:

- `./outputs` → Simulation output files
- `./logs` → Log files

Your data persists on the host machine even after container removal.

### Port Mappings

- **8000**: TeraSim FastAPI service endpoint
- **6379**: Redis database

### Environment Variables

The following environment variables are pre-configured in the container:

```bash
SUMO_HOME=/root/.terasim/deps/sumo
PATH=$SUMO_HOME/bin:$PATH
PYTHONUNBUFFERED=1
```

### Resource Limits

Default resource limits in `docker-compose.yml`:

```yaml
deploy:
  resources:
    limits:
      cpus: '4'
      memory: 8G
    reservations:
      cpus: '2'
      memory: 4G
```

Adjust these values based on your simulation requirements.

---

## Customization

### Changing Startup Command

Edit the `command` field in `docker-compose.yml`:

```yaml
# Auto-start simulation on container launch
command: sh -c "redis-server --daemonize yes && python run_experiments.py"

# Auto-start FastAPI service
command: sh -c "redis-server --daemonize yes && python run_service.py"

# Keep container running with bash (default)
command: sh -c "redis-server --daemonize yes && /bin/bash"
```

### Using Separate Redis Container

Uncomment the Redis service in `docker-compose.yml`:

```yaml
services:
  redis:
    image: redis:7-alpine
    container_name: terasim-redis
    ports:
      - "6379:6379"
    volumes:
      - redis-data:/data
    command: redis-server --appendonly yes

volumes:
  redis-data:
```

Then update the TeraSim service to link to Redis:

```yaml
services:
  terasim:
    # ... other config ...
    depends_on:
      - redis
    environment:
      - REDIS_HOST=redis
```

---

## Troubleshooting

### Build Failures

```bash
# Clean Docker cache and rebuild
docker-compose build --no-cache

# Check Docker disk space
docker system df

# Prune unused resources
docker system prune -a
```

### Redis Connection Issues

```bash
# Check Redis status inside container
docker-compose exec terasim redis-cli ping

# Manually start Redis
docker-compose exec terasim redis-server --daemonize yes

# View Redis logs
docker-compose exec terasim redis-cli INFO
```

### SUMO_HOME Not Set

```bash
# Verify environment variable
docker-compose exec terasim echo $SUMO_HOME

# Manually set if needed
docker-compose exec terasim bash -c "export SUMO_HOME=/root/.terasim/deps/sumo"

# Check SUMO installation
docker-compose exec terasim sumo --version
```

### Permission Issues with Mounted Volumes

```bash
# On host machine, adjust permissions
chmod -R 777 outputs logs

# Or change ownership to your user
sudo chown -R $USER:$USER outputs logs
```

### Container Won't Start

```bash
# View container logs
docker-compose logs terasim

# Check specific error messages
docker-compose logs --tail=50 terasim

# Restart container
docker-compose restart terasim
```

### Python Import Errors

```bash
# Verify package installation inside container
docker-compose exec terasim python -c "import terasim; print(terasim.__version__)"

# Reinstall packages if needed
docker-compose exec terasim pip install -e packages/terasim
```

---

## Development Workflow

### Making Code Changes

1. Edit code on your host machine
2. Rebuild the container to apply changes:

```bash
docker-compose up -d --build
```

Alternatively, mount your source code as a volume for live editing:

```yaml
services:
  terasim:
    volumes:
      - ./packages:/app/TeraSim/packages
```

### Debugging Inside Container

```bash
# Enter container with bash
docker-compose exec terasim bash

# Run Python interactively
docker-compose exec terasim python

# Use IPython for better debugging
docker-compose exec terasim pip install ipython
docker-compose exec terasim ipython
```

### Running GUI Applications

For SUMO GUI (`sumo-gui`) to work, you need X11 forwarding:

```yaml
services:
  terasim:
    environment:
      - DISPLAY=$DISPLAY
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
```

On the host:

```bash
# Allow X11 connections
xhost +local:docker

# Run container
docker-compose up -d
```

---

## Image Information

- **Base Image**: `ubuntu:22.04`
- **Python Version**: `3.10`
- **SUMO Version**: `1.23.1`
- **Build Type**: Multi-stage (optimized)
- **Estimated Size**: 2-3 GB
- **Note**: Base image excludes `terasim-cosmos` (requires GPU support)

### TeraSim-Cosmos GPU Image

For full TeraSim-Cosmos functionality with GPU support, use the dedicated GPU Dockerfile:

```bash
# Navigate to terasim-cosmos package
cd packages/terasim-cosmos

# Build and run GPU-enabled image
docker-compose -f docker-compose.gpu.yml up -d --build

# Enter container
docker-compose -f docker-compose.gpu.yml exec terasim-cosmos bash
```

See [packages/terasim-cosmos/README_DOCKER_GPU.md](packages/terasim-cosmos/README_DOCKER_GPU.md) for detailed GPU setup instructions.

---

## Performance Tips

1. **Use BuildKit**: Enable faster builds with Docker BuildKit:
   ```bash
   DOCKER_BUILDKIT=1 docker-compose build
   ```

2. **Layer Caching**: Order Dockerfile commands from least to most frequently changed

3. **Parallel Builds**: Build with multiple CPU cores:
   ```bash
   docker-compose build --parallel
   ```

4. **Resource Allocation**: Increase Docker Desktop resources in Settings → Resources

---

## Support

For issues, questions, or contributions:

- **GitHub Issues**: [https://github.com/mcity/TeraSim/issues](https://github.com/mcity/TeraSim/issues)
- **Discussions**: [https://github.com/mcity/TeraSim/discussions](https://github.com/mcity/TeraSim/discussions)
- **Documentation**: See main [README.md](README.md) and [CLAUDE.md](CLAUDE.md)

---

## License

- **TeraSim Core**: Apache 2.0 License
- **TeraSim Visualization Tools**: MIT License

See [LICENSE](LICENSE) for details.
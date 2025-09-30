# TeraSim Docker Image
# Base: Ubuntu 22.04 with Python 3.10
# Includes: gcc, g++, SUMO, Redis, and all TeraSim dependencies

FROM ubuntu:22.04 AS builder

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    gcc \
    g++ \
    make \
    cmake \
    build-essential \
    # Python 3.10
    python3.10 \
    python3.10-dev \
    python3-pip \
    python3.10-venv \
    # Version control
    git \
    # Redis
    redis-server \
    # Spatial libraries (for rtree, shapely, pyproj)
    libspatialindex-dev \
    libgeos-dev \
    libproj-dev \
    # XML processing (for lxml)
    libxml2-dev \
    libxslt1-dev \
    # Other utilities
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.10 as default python3
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1

# Upgrade pip
RUN python3 -m pip install --upgrade pip setuptools wheel

# Set working directory
WORKDIR /app/TeraSim

# Copy project files
COPY . .

# Clone SUMO tools repository for SUMO_HOME
RUN mkdir -p /root/.terasim/deps && \
    git clone --depth 1 https://github.com/eclipse/sumo.git /root/.terasim/deps/sumo && \
    echo "SUMO_HOME=/root/.terasim/deps/sumo" > /root/.terasim/deps/.sumo_home

# Set SUMO_HOME environment variable
ENV SUMO_HOME=/root/.terasim/deps/sumo
ENV PATH="${SUMO_HOME}/bin:${PATH}"

# Install Python packages in editable mode
# Note: terasim-cosmos is excluded from base image (requires GPU support)
# For terasim-cosmos, use packages/terasim-cosmos/Dockerfile.gpu instead
RUN pip install -e packages/terasim && \
    pip install -e packages/terasim-nde-nade && \
    pip install -e packages/terasim-service && \
    pip install -e packages/terasim-envgen && \
    pip install -e packages/terasim-datazoo && \
    pip install -e packages/terasim-vis

# Install development dependencies
RUN pip install \
    pytest>=7.4.0 \
    pytest-cov>=4.1.0 \
    black>=23.7.0 \
    ruff>=0.1.0 \
    mypy>=1.5.1 \
    isort>=5.12.0

# Build Cython extensions for NDE-NADE
RUN cd packages/terasim-nde-nade && \
    python setup.py build_ext --inplace && \
    cd ../..

# Create output directories
RUN mkdir -p outputs logs

# Verify installation
RUN python3 -c "import terasim; print('✅ TeraSim core imported successfully')" && \
    python3 -c "import terasim_nde_nade; print('✅ TeraSim NDE-NADE imported successfully')" && \
    python3 -c "import terasim_service; print('✅ TeraSim Service imported successfully')" && \
    python3 -c "import terasim_vis; print('✅ TeraSim Visualization imported successfully')"

# === Runtime Stage ===
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies only
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    git \
    redis-server \
    libspatialindex-c6 \
    libgeos-c1v5 \
    libproj22 \
    libxml2 \
    libxslt1.1 \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.10 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1

# Copy installed packages and SUMO from builder
COPY --from=builder /usr/local/lib/python3.10 /usr/local/lib/python3.10
COPY --from=builder /usr/local/bin /usr/local/bin
COPY --from=builder /root/.terasim /root/.terasim
COPY --from=builder /app/TeraSim /app/TeraSim

# Set working directory
WORKDIR /app/TeraSim

# Set environment variables
ENV SUMO_HOME=/root/.terasim/deps/sumo
ENV PATH="${SUMO_HOME}/bin:${PATH}"
ENV PYTHONUNBUFFERED=1

# Expose ports
# 8000: TeraSim FastAPI service
# 6379: Redis
EXPOSE 8000 6379

# Default command: start bash shell
CMD ["/bin/bash"]

# Alternative entry points (uncomment as needed):
# Run simulation with GUI disabled
# CMD ["python3", "run_experiments.py"]

# Start TeraSim service
# CMD ["python3", "run_service.py"]

# Start Redis and then run simulation
# CMD ["sh", "-c", "redis-server --daemonize yes && python3 run_experiments.py"]
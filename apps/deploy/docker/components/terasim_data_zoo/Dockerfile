# Build arguments
ARG BASE_IMAGE=terasim_base
ARG VERSION=latest
ARG VARIANT=cpu

# Use the specified base image
FROM ${BASE_IMAGE}:${VERSION}-${VARIANT}

# Copy TeraSim-Data-Zoo package
COPY TeraSim-Data-Zoo/terasim_data_zoo /app/terasim_data_zoo
COPY TeraSim-Data-Zoo/pyproject.toml TeraSim-Data-Zoo/poetry.lock /app/

# Install poetry if not already installed
RUN pip install --no-cache-dir poetry

# Install package
WORKDIR /app
RUN poetry config virtualenvs.create false \
    && poetry install --no-dev --no-interaction --no-ansi

# Create data directory
RUN mkdir -p /app/data

# Set environment variables
ENV PYTHONPATH=/app:${PYTHONPATH} \
    TERASIM_DATA_DIR=/app/data

VOLUME ["/app/data"] 
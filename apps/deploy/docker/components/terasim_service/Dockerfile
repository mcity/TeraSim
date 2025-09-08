# Build arguments
ARG BASE_IMAGE=terasim_nde_nade
ARG VERSION=latest
ARG VARIANT=cpu

# Use the specified base image
FROM ${BASE_IMAGE}:${VERSION}-${VARIANT}

# Copy TeraSim-Service package
USER root
COPY TeraSim-Service /app/TeraSim-Service
RUN chown -R terasim:terasim /app/TeraSim-Service

# Install TeraSim-Service package
USER root
WORKDIR /app/TeraSim-Service
RUN rm -rf poetry.lock
RUN poetry install

# Install Redis
RUN apt-get update && apt-get install -y redis

# Expose service port
EXPOSE 8000
EXPOSE 2000

# Set environment variables
ENV PYTHONPATH=/app:${PYTHONPATH}

# Set the default command
USER terasim

WORKDIR /app/TeraSim-Service
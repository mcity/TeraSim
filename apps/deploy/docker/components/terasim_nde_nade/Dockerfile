# Build arguments
ARG BASE_IMAGE=terasim_core
ARG VERSION=latest
ARG VARIANT=cpu

# Use the specified base image
FROM ${BASE_IMAGE}:${VERSION}-${VARIANT}

# Copy TeraSim-NDE-NADE package
COPY TeraSim-NDE-NADE /app/TeraSim-NDE-NADE

# Install package
USER root
WORKDIR /app/TeraSim-NDE-NADE
RUN rm -rf poetry.lock
RUN poetry install

# Set environment variables
ENV PYTHONPATH=/app:${PYTHONPATH}

USER terasim

WORKDIR /app
# Build arguments
ARG BASE_IMAGE=terasim_base
ARG VERSION=latest
ARG VARIANT=cpu

# Use the specified base image
FROM ${BASE_IMAGE}:${VERSION}-${VARIANT}

# Copy TeraSim core package
COPY TeraSim /app/TeraSim

# Install TeraSim package
USER root
WORKDIR /app/TeraSim
RUN rm -rf poetry.lock
RUN poetry install

# Set environment variables
ENV PYTHONPATH=/app:${PYTHONPATH}

USER terasim

WORKDIR /app
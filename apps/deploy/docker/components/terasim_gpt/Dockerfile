# Build arguments
ARG BASE_IMAGE=terasim_base
ARG VERSION=latest
ARG VARIANT=cpu

# Use the specified base image
FROM ${BASE_IMAGE}:${VERSION}-${VARIANT}

# Copy TeraSim-GPT package
COPY TeraSim-GPT/terasim_gpt /app/terasim_gpt
COPY TeraSim-GPT/pyproject.toml TeraSim-GPT/poetry.lock /app/

# Install poetry if not already installed
RUN pip install --no-cache-dir poetry

# Install package
WORKDIR /app
RUN poetry config virtualenvs.create false \
    && poetry install --no-dev --no-interaction --no-ansi

# Create models directory
RUN mkdir -p /app/models

# Set environment variables
ENV PYTHONPATH=/app:${PYTHONPATH} \
    TERASIM_MODELS_DIR=/app/models

VOLUME ["/app/models"] 
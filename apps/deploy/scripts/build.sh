#!/bin/bash

# Define variables
DOCKER_BASE_DIR="docker/base"
DOCKER_COMPONENTS_DIR="docker/components"
VERSION="1.0.0"
CUDA_VERSION="12.1.1"

# Parse arguments
USE_GPU=false
COMPONENTS=()

# Function to check if an array contains a value
contains() {
    local n=$#
    local value=${!n}
    for ((i=1;i < $#;i++)) {
        if [ "${!i}" == "${value}" ]; then
            return 0
        fi
    }
    return 1
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --gpu)
            USE_GPU=true
            shift
            ;;
        --version=*)
            VERSION="${1#*=}"
            shift
            ;;
        --components=*)
            IFS=',' read -r -a COMPONENTS <<< "${1#*=}"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# If no components specified, build all
if [ ${#COMPONENTS[@]} -eq 0 ]; then
    # COMPONENTS=("terasim" "terasim_nde_nade" "terasim_data_zoo" "terasim_gpt" "terasim_service")
    COMPONENTS=("terasim" "terasim_nde_nade" "terasim_service")
fi

# Build base image
VARIANT=$([ "$USE_GPU" = true ] && echo "gpu" || echo "cpu")
BASE_TAG="terasim_base"
CORE_TAG="terasim_core"
NADE_TAG="terasim_nde_nade"
SERVICE_TAG="terasim_service"


echo "Building ${VARIANT} version of base image..."
docker build -f ${DOCKER_BASE_DIR}/Dockerfile.base.${VARIANT} \
    -t ${BASE_TAG}:${VERSION}-${VARIANT} \
    --build-arg CUDA_VERSION=${CUDA_VERSION} \
    ${DOCKER_BASE_DIR}

# Always build terasim if it's a dependency for other components
if contains "${COMPONENTS[@]}" "terasim_nde_nade" || \
   contains "${COMPONENTS[@]}" "terasim_data_zoo" || \
   contains "${COMPONENTS[@]}" "terasim_gpt" || \
   contains "${COMPONENTS[@]}" "terasim_service" || \
   contains "${COMPONENTS[@]}" "terasim"; then
    
    echo "Building terasim (core) image..."
    docker build -f ${DOCKER_COMPONENTS_DIR}/terasim/Dockerfile \
        -t ${CORE_TAG}:${VERSION}-${VARIANT} \
        --build-arg BASE_IMAGE=${BASE_TAG} \
        --build-arg VERSION=${VERSION} \
        --build-arg VARIANT=${VARIANT} \
        .
fi

# Build NDE-NADE if requested (high priority)
if contains "${COMPONENTS[@]}" "terasim_nde_nade"; then
    echo "Building terasim_nde_nade image..."
    docker build -f ${DOCKER_COMPONENTS_DIR}/terasim_nde_nade/Dockerfile \
        -t ${NADE_TAG}:${VERSION}-${VARIANT} \
        --build-arg BASE_IMAGE=${CORE_TAG} \
        --build-arg VERSION=${VERSION} \
        --build-arg VARIANT=${VARIANT} \
        .
fi

# Build other components if requested
for comp in "terasim_data_zoo" "terasim_gpt" "terasim_service"; do
    if contains "${COMPONENTS[@]}" "${comp}"; then
        echo "Building ${comp} image..."
        docker build -f ${DOCKER_COMPONENTS_DIR}/${comp}/Dockerfile \
            -t ${comp}:${VERSION}-${VARIANT} \
            --build-arg BASE_IMAGE=${NADE_TAG} \
            --build-arg VERSION=${VERSION} \
            --build-arg VARIANT=${VARIANT} \
            .
    fi
done 
#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:x11"
CONTAINER_NAME="turtlebot4_sim_dev"

# Check if container exists
if docker ps -a --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container ${CONTAINER_NAME} exists. Using docker exec..."
    
    # Check if container is running
    if docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
        echo "Container is running. Executing bash..."
        docker exec -it \
        -e DISPLAY=$DISPLAY \
        -w /workspace/solver \
        "$CONTAINER_NAME" \
        bash
    else
        echo "Container is stopped. Starting container..."
        docker start "$CONTAINER_NAME"
        echo "Executing bash..."
        docker exec -it \
        -e DISPLAY=$DISPLAY \
        -w /workspace/solver \
        "$CONTAINER_NAME" \
        bash
    fi
else
    echo "Container ${CONTAINER_NAME} does not exist. Creating new container..."
    # Check if GPU is available
    GPU_ARG=""
    GPU_ENV_ARGS=""
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        GPU_ARG="--gpus all"
        GPU_ENV_ARGS="-e __NV_PRIME_RENDER_OFFLOAD=1 \
-e __GLX_VENDOR_LIBRARY_NAME=nvidia \
-e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute"
    fi
    docker run --rm -it \
    $GPU_ARG \
    --ipc host \
    --name "$CONTAINER_NAME" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    $GPU_ENV_ARGS \
    --device /dev/dri \
    -v "$(pwd)/solver:/workspace/solver" \
    -w /workspace/solver \
    "$IMAGE" \
    bash
fi

#!/usr/bin/env bash
set -e
cd ../..

IMAGE="turtlebot4:vnc"
CONTAINER_NAME="turtlebot4_sim_dev"

# Check if GPU is available
GPU_ARG=""
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    GPU_ARG="--gpus all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute"
fi

# Check if container exists
if docker ps -a --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container ${CONTAINER_NAME} exists. Using docker exec..."
    
    # Check if container is running
    if docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
        echo "Container is running. Executing bash..."
        docker exec -it \
        -w /workspace/solver \
        "$CONTAINER_NAME" \
        bash
    else
        echo "Container is stopped. Starting container..."
        docker start "$CONTAINER_NAME"
        echo "Executing bash..."
        docker exec -it \
        -w /workspace/solver \
        "$CONTAINER_NAME" \
        bash -lc "pgrep -x Xvnc >/dev/null 2>&1 || Xvnc :0 -geometry 1920x1080 -localhost no -SecurityTypes None & sleep 3 && export DISPLAY=:0 && pgrep -x xfce4-session >/dev/null 2>&1 || exec /usr/bin/startxfce4 > /dev/null 2>&1 & bash"
    fi
else
    echo "Container ${CONTAINER_NAME} does not exist. Creating new container..."
    docker run --rm -it \
    $GPU_ARG \
    --name "$CONTAINER_NAME" \
    -p 5900:5900 \
    -v "$(pwd)/solver:/workspace/solver" \
    -w /workspace/solver \
    "$IMAGE" \
    bash -lc "pgrep -x Xvnc >/dev/null 2>&1 || Xvnc :0 -geometry 1920x1080 -localhost no -SecurityTypes None & sleep 3 && export DISPLAY=:0 && pgrep -x xfce4-session >/dev/null 2>&1 || exec /usr/bin/startxfce4 > /dev/null 2>&1 & bash"
fi 
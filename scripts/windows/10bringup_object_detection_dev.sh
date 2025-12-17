#!/usr/bin/env bash
set -e

cd ../..

IMAGE="turtlebot4:semantic_slam"

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
--name turtlebot4_object_detection_dev \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
$GPU_ENV_ARGS \
-v "$(pwd)/object_detection:/workspace/object_detection" \
-w /workspace/object_detection \
"$IMAGE"
#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:semantic_slam-vnc"


# Check if GPU is available
GPU_ARG=""
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    GPU_ARG="--gpus all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute"
fi

docker run --rm -it \
$GPU_ARG \
--name turtlebot4_object_detection_dev \
-p 5901:5900 \
-v "$(pwd)/object_detection:/workspace/object_detection" \
-w /workspace/object_detection \
"$IMAGE"

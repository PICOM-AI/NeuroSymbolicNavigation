#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:slam"

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
--name turtlebot4_slam \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
$GPU_ENV_ARGS \
--device /dev/dri \
-v "$(pwd)/automotive:/ws" \
-v "/media/legion/WData/esteri4/20250904_111219/20250904_111219:/ws/20250904_111219" \
-w /ws \
"$IMAGE"






#!/usr/bin/env bash
set -e

cd ../..

IMAGE="turtlebot4:semantic_slam"

# Check if GPU is available
GPU_ARG=""
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    GPU_ARG="--gpus all"
fi

docker run --rm -it \
$GPU_ARG \
--ipc host \
--name turtlebot4_semantic_slam \
"$IMAGE" \
bash -lc 'exec ros2 launch semantic_slam semantic_slam.launch.py'







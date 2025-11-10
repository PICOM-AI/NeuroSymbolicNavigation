#!/usr/bin/env bash
set -e

cd ../..

IMAGE="turtlebot4:semantic_slam-vnc"

docker run --rm -it \
--gpus all \
--ipc host \
--name turtlebot4_semantic_slam \
"$IMAGE" \
bash -lc 'exec ros2 launch semantic_slam semantic_slam.launch.py'







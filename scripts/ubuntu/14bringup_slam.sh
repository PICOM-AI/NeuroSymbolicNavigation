#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:slam"

docker run --rm -it \
--gpus all \
--ipc host \
--name turtlebot4_slam \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e __NV_PRIME_RENDER_OFFLOAD=1 \
-e __GLX_VENDOR_LIBRARY_NAME=nvidia \
-e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
--device /dev/dri \
-v "$(pwd)/automotive:/ws" \
-w /ws \
"$IMAGE"






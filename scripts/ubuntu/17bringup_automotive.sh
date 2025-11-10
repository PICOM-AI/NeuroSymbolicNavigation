#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:automotive"

docker run --rm -it \
--gpus all \
--ipc host \
--name turtlebot4_automotive \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e __NV_PRIME_RENDER_OFFLOAD=1 \
-e __GLX_VENDOR_LIBRARY_NAME=nvidia \
-e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
--device /dev/dri \
-v /media/legion/WData/helsinki/actual_record_jatkasari/lidar_container/:/data/:ro \
-v /home/legion/workspace/gazebo/coursefull/NeuroSymbolicNavigation/automotive:/root/ \
-w /root \
"$IMAGE"






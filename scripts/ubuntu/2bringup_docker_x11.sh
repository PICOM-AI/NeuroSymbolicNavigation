#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:x11"

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
--name turtlebot4_sim \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
$GPU_ENV_ARGS \
--device /dev/dri \
-v "$(pwd)/solver:/workspace/solver" \
"$IMAGE" \
bash -lc 'exec ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze slam:=true nav:=true rviz:=true'

#bash -lc 'exec ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py localization:=true world:=maze map:=maze/maze.yaml slam:=true rviz:=true'


#bash -lc 'exec ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze nav:=true rviz:=true'




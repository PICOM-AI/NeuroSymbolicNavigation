#!/usr/bin/env bash
set -e
cd ../..

IMAGE="turtlebot4:vnc"

ARCH=$(uname -m)
GPU_ARG=""

if [ "$ARCH" = "x86_64" ]; then
    GPU_ARG="--gpus all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute"
fi

docker run --rm -it \
$GPU_ARG \
--name turtlebot4_sim_dev \
-p 5900:5900 \
-v "$(pwd)/solver:/workspace/solver" \
-w /workspace/solver \
"$IMAGE" \
bash -lc "pgrep -x Xvnc >/dev/null 2>&1 || Xvnc :0 -geometry 1920x1080 -localhost no -SecurityTypes None & sleep 3 && export DISPLAY=:0 && pgrep -x xfce4-session >/dev/null 2>&1 || exec /usr/bin/startxfce4 > /dev/null 2>&1 & bash" 
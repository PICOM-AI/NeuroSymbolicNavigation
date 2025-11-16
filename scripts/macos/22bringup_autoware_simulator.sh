 cd ../..
 docker build -t autoware_sim:vnc -f Dockerfile.autoware_sim_vnc .

#!/usr/bin/env bash
set -e

cd ../..

IMAGE="autoware_sim:vnc"


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
--name autoware_sim \
-p 5900:5900 \
-v $PWD/hdmap/map_files:/data/hdmap \
"$IMAGE"

 cd ../..
 xhost +
 
 # Check if GPU is available
 GPU_ARG=""
 if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
     GPU_ARG="--gpus=all"
 fi
 
 docker run -it -v $PWD/hdmap/map_files:/data/hdmap \
 $GPU_ARG \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/dev:/dev" \
    --name autoware_sim \
 --rm ghcr.io/autowarefoundation/autoware:universe-cuda
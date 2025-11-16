 cd ../..
 docker run -it -v $PWD/hdmap/map_files:/data/hdmap \
 --gpus=all \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/dev:/dev" \
 --rm ghcr.io/autowarefoundation/autoware:universe-cuda
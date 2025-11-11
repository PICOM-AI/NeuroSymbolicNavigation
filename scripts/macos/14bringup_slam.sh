#!/usr/bin/env bash
set -e
xhost +

cd ../..

IMAGE="turtlebot4:slam"


docker run --rm -it \
--name turtlebot4_slam \
-p 5900:5900 \
-v "$(pwd)/automotive:/ws" \
-w /ws \
"$IMAGE"






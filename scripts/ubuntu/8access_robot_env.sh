# !/bin/bash
set -e

# Replace horizon, radius, and lp map as needed

docker exec -it turtlebot4_sim bash -lc "source /opt/ros/humble/setup.bash &&
cd /workspace/solver && bash"
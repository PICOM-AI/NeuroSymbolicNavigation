# !/bin/bash
set -e

# Replace horizon, radius, and lp map as needed

docker exec -it turtlebot4_sim bash -c "source /opt/ros/humble/setup.bash &&
cd /workspace/solver && 
python3 plot.py --file maze0_4.lp"
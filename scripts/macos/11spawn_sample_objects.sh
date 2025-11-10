# !/bin/bash
set -e

# Replace horizon, radius, and lp map as needed

docker exec -it turtlebot4_sim bash -c "source /opt/ros/humble/setup.bash &&
cd /workspace/solver && 
ros2 run ros_gz_sim create -world maze -file person_walking/model.sdf -name person1 -x 2 -y 0 -z 0.1"

docker exec -it turtlebot4_sim bash -c "source /opt/ros/humble/setup.bash &&
cd /workspace/solver && 
ros2 run ros_gz_sim create -world maze -file fire_hydrant/model.sdf -name firehydrant1 -x 2 -y 1 -z -0.05"

docker exec -it turtlebot4_sim bash -c "source /opt/ros/humble/setup.bash &&
cd /workspace/solver && 
ros2 run ros_gz_sim create -world maze -file construction_cone/model.sdf -name constructioncone1 -x 2 -y -1 -z 0.1"
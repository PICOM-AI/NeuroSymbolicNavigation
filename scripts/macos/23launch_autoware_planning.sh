docker exec -it autoware_sim \
  bash -c "source /opt/ros/humble/setup.bash && \
         source /opt/autoware/setup.bash && \
         ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/data/hdmap vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"
#!/bin/bash

# # aws_common ROS package used in kinesis is outdated and wants aws credentials
# # in /home/robomaker//.aws/config location
mkdir -p /root/.aws/
ln -s ${AWS_CONFIG_FILE} /root/.aws/credentials

# Sourcing all ROS packages
source /opt/ros/melodic/setup.bash
source /opt/amazon/install/setup.bash
export GAZEBO_MODEL_PATH=/opt/amazon/install/deepracer_simulation_environment/share/deepracer_simulation_environment/
echo "Executing simulation-entrypoint.sh script"

printenv
exec "${@:1}"

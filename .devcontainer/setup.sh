#!/bin/bash

set -x

source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ros/$ROS_DISTRO/local_setup.bash


cd ./driving_swarm_infrastructure
rosdep update
rosdep install --from-paths src/ --ignore-src -y
colcon build

cd ..

cd ./introduction_to_robotics_tutorial
colcon build

# make all .bash files executable
chmod +x ./install/*.bash


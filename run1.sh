#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
. /usr/share/gazebo/setup.sh
. ~/ros2_ws/install/setup.bash
echo "Launching application, please wait!"
ros2 launch hagen_gazebo1 hagen.launch.py world:=hagen_city.world



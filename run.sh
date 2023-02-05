#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
. /usr/share/gazebo/setup.sh
. ~/ros2_ws/install/setup.bash
echo "Launching application, please wait!"
ros2 launch hagen_gazebo hagen.launch.py world:=hagen_empty.world 





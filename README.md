# Autonomous Mobile Robots

This repository contains an implementation of the Autonomous Mobile Robots course for ROCV master's program at Innopolis University. The course is instructed by [Geesara Prathap](https://github.com/GPrathap). So, this repository contains the course material besides my solutions for the assignments. In addition, I developed a PD controller for the differential drive robot. 

The course contents includes:
- Motion control (Kinematics, control, and dubins path planning).
- Estimation (Kalman filter, extended kalman filter, particle filter).
- Localization (Monte carlo, and ekf localization).

## Setup

1. Install at least one simulator:
   [Gazebo](http://gazebosim.org/tutorials?cat=install) 

2. Install the appropriate ROS 2 version as instructed:
   [here](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

3. Clone the repository:
    
       mkdir -p ~/ros2_ws/src
       cd ~/ros2_ws/src
       git clone https://github.com/GPrathap/autonomous_mobile_robots.git

4. Install dependencies:

       cd ~/ros2_ws
       rosdep install --from-paths src --ignore-src -r -y

5. Build and install:

       cd ~/ros2_ws
       colcon build

## Run

### Gazebo-classic

If you had Gazebo installed when compiling Hagen's packages, Gazebo support should be enabled.

1. Setup environment variables (the order is important):

       . /usr/share/gazebo/setup.sh
       . ~/ws/install/setup.bash

   > *Tip*: If the command `ros2 pkg list | grep hagen_gazebo` comes up empty after setting up the environment, 
     Gazebo support wasn't correctly setup.

2. Launch Hagen in a city (this will take some time to download models):

       ros2 launch hagen_gazebo hagen.launch.py world:=hagen_city.world

3. Launch Hagen in an empty world:

       ros2 launch hagen_gazebo hagen.launch.py world:=hagen_empty.world
        

To avoid these steps, in your terminal, naviagate to the repository directory and make the file 'run.sh' executable, then run it to start the simulation directly.
    
       cd ~/ros2_ws/src/autonomous_mobile_robots
       chmod +x run.sh
       ./run.sh



Acknowledgement
   https://github.com/chapulina/dolly





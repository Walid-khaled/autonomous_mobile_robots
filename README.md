# Autonomous Mobile Robots

This repository contains an implementation of the Autonomous Mobile Robots course for ROCV master's program at Innopolis University. The course is instructed by [Geesara Prathap](https://github.com/GPrathap). So, this repository contains the course material besides my solutions for the assignments. In addition, I developed PID, LQR controllers for a differential drive robot. Trajectory-tracking error model was developed for applying MPC controller.

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
       git clone https://github.com/Walid-khaled/autonomous_mobile_robots.git

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
       . ~/ros2_ws/install/setup.bash

   > *Tip*: If the command `ros2 pkg list | grep hagen_gazebo` comes up empty after setting up the environment, 
     Gazebo support wasn't correctly setup.

2. Launch Hagen in a city (this will take some time to download models):

       ros2 launch hagen_gazebo hagen.launch.py world:=hagen_city.world

3. Launch Hagen in an empty world:

       ros2 launch hagen_gazebo hagen.launch.py world:=hagen_empty.world
        
   To avoid these steps, in your terminal, naviagate to the repository directory and make the file **"run.sh"** executable, then run it to start the simulation directly.
    
       cd ~/ros2_ws/src/autonomous_mobile_robots
       chmod +x run.sh
       ./run.sh

4. In a new terminal:

       . ~/ros2_ws/install/setup.bash
       ros2 run hagen_control desired_controller
       desired_controller should be replaced with one of these executables (PID, LQR, MPC)

5. To reset the simulation:

       ros2 service call /reset_simulation std_srvs/srv/Empty

## Main results

### Control Strategies:
In **"hagen_control/hagen_control/hagen_control_strategy.py"**, specify one of the following controller at the **main()** and **timer_callback()** functions:
- Control to reference pose
- Control to reference pose via an intermediate point
- Control to reference pose via an intermediate direction
- Reference path control

After the simulation finish, a plot will be genereated to visualize the odometery data with respect to the actual one. 
For example this is the output for **reference path control**.


![Figure_1](https://user-images.githubusercontent.com/90580636/205742442-8d85d9fe-d796-46a2-90dc-d46950d02255.png)
<!-- <p float="left">
    <img src="https://user-images.githubusercontent.com/90580636/205742442-8d85d9fe-d796-46a2-90dc-d46950d02255.png" width="600" height="450" />
</p> -->

As shown, the odom data diverge by time when reaching the path points without any feedback from the odometery data.

### PD Controller

A PD controller was developed to incorporate the odometery data as feedback. It is implemented in 2 stages; reaching the target position and correcting the orientation. 

https://user-images.githubusercontent.com/90580636/205747639-0930f718-3cef-40f2-9913-4f9b7f47750d.mp4

![Figure_1](https://user-images.githubusercontent.com/90580636/205747519-f6d9cff2-db78-448a-bbf1-0c6f8e797446.png)

<p align="center">
    <img src="https://user-images.githubusercontent.com/90580636/205748797-a46efa4d-4f1b-4f1b-a161-5048ac3e13c6.png"/>
</p>
 
### PID Controller

Adding the integral term has affected the removed the steady state error and improved the response. With the parameter tuning, you can obtain better results. Check **"hagen_control/hagen_control/diff_drive_PD_control.py"**.

https://user-images.githubusercontent.com/90580636/206735154-6048563d-6747-4807-ac38-20395af411c1.mp4

<p align="center">
    <img src="https://user-images.githubusercontent.com/90580636/206735209-ba78dfee-7d1d-4d6b-aa9d-0103e8b11dcf.png"/>
</p>

### Linear-Quadratic Controller (LQR)
Two different approaches were developed to solve the LQR problem. Check the formulation and the explanation in the notebooks in **"hagen_control/hagen_control/LQR/lqr-01.ipynb"**.
However the approach of the notebook **"hagen_control/hagen_control/LQR/lqr-02.ipynb"** was adopted as it showed a better performance. This is because 

- K is recalculated at each timestamp unlike the previous approach.
- The Discrete Algebraic Riccati Equation (DARE) is solved for each state (timestamp) using dynamic programming, instead of solving it once as in the previous approach.
    
So a ros node was developed for LQR implementation, and below are the results. Check **"hagen_control/hagen_control/diff_drive_LQR.py"**.

https://user-images.githubusercontent.com/90580636/210401939-4b83276f-4775-44e6-bb96-566fc8fa1b1c.mp4

![Figure_1](https://user-images.githubusercontent.com/90580636/210408071-8ee96c72-1bf6-4ef9-bd04-9a9fc999f658.png)

Some notes:
- There is an advantage of the LQR controller over the previous PID controller. In LQR, there is no need to control the position and the orientation in 2 separate controllers as we did in PID. In PID, we controlled position and after reaching the position, another controller was applied to correct the orientation which makes a shift in position again. But in LQR, only one controller corrects the position and orientation simultaneously until reaching the desired state (x,y, yaw).
- However, I noted that changing the desired state requires to tweak the Q and R matrices again, which is extremely time consuming if it is done manually.  
### Model Predictive Controller (MPC)
Trajectory tracking error model was developed. Check the formulation and the explanation in the reportin **"hagen_control/hagen_control/MPC/MPC_Trajectory_Tracking_Error_Model.pdf"** and the MATLAB script **"hagen_control/hagen_control/MPC/MPC.m"**.

Ros node was developed for MPC implementation, and below are the results. Check **"hagen_control/hagen_control/diff_drive_MPC.py"**.

When simulating the robot motion, the following response was obtained:
![Figure_1](https://user-images.githubusercontent.com/90580636/221671016-8aa50393-030c-4531-b38f-202a48c66991.png)

However, when using the odometry data to update the pose, the following result was obtained:
![Figure_1](https://user-images.githubusercontent.com/90580636/221677411-ac8371b2-05f8-4d3c-8417-8a50840e8ae7.png)


### EKF Localization

![Figure_1](https://user-images.githubusercontent.com/90580636/205748556-8123b8fe-563d-4638-9a0a-b209e3661b0b.png)

## Acknowledgement

https://github.com/GPrathap/autonomous_mobile_robots

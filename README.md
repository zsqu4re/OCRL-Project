# CMU_16745 Project
Use LQR as controller for FOC_legged_wheel robot

## Instruction
1. Download the git:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:zsqu4re/OCRL-Project.git
cd ..
catkin_make
source devel/setup.bash
```

2. Install the gazebo_ros_control:
```
sudo apt update
sudo apt install ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers
```

3. Run the Gazebo Simulator
```
roscore
roslaunch foc_urdf gazebo_control.launch
```

4. IMU data
```
rostopic echo /imu/data
```

## Issue
~~1. Read the IMU date of the robot~~
2. Read the data from gazebo for control input
3. LQR controller...

# CMU 16745 OCRL Project
Simulation about FOC_legged_wheel Robot

## Instrction:
create a workspace for this project
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:zsqu4re/OCRL-Project.git
cd ..
catkin_make
source devel/setup.bash
```

Run the Gazebo simulator
```
roscore
roslaunch foc_urdf gazebo_urdf.launch
```

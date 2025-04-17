<<<<<<< HEAD
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

sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-joint-trajectory-controller
=======
## FOC_Wheel_Leg Robot LQR + PID Controller

### instruction
下载到本地的workspace
```
mkdir -p/xxx_ws
git clone -b MaoZao git@github.com:zsqu4re/OCRL-Project.git
catkin_init_workspace
catkin_make
source devel/setup.bash
```
运行gazebo和controller
```
roslaunch balance_robot_controller balance_robot.launch
```

### 问题
我不知道为啥站不起来，而且rostopic中没有每个关节的信息，但是我echo了joint_states/effort可以看到每个关节都会有力的输入
我初步猜想是urdf文件中定义的每个关节的扭矩方向和我balance_robot_controller.cpp文件中的输出不一样，这个可能需要再查一下，
可嫩还需要检查一个这个文件中UpState的关节定义是否正确
同时可能需要修改一下机器人的初始位置例如L0的长度等等
可能PID方面的内容还是需要再调整一下
>>>>>>> origin/MaoZao

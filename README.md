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

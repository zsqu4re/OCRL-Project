创建目录结构:
cd ~/catkin_ws/src
mkdir -p lqr_controller/include/lqr_controller
mkdir -p lqr_controller/src
mkdir -p lqr_controller/config
mkdir -p lqr_controller/launch
mkdir -p lqr_controller/scripts

确保Python脚本有执行权限:
chmod +x ~/catkin_ws/src/lqr_controller/scripts/test_controller.py
chmod +x ~/catkin_ws/src/lqr_controller/scripts/test_gazebo_controller.py

编译包:
cd ~/catkin_ws
catkin_make
source devel/setup.bash


2. 使用方法
使用主启动文件（推荐）
# 在Gazebo中仿真
roslaunch lqr_controller foc_balance_control.launch

# 连接到真实机器人
roslaunch lqr_controller foc_balance_control.launch sim:=false

# 运行测试序列
roslaunch lqr_controller foc_balance_control.launch test:=true

# 不使用图形界面
roslaunch lqr_controller foc_balance_control.launch gui:=false

# 启用调试模式
roslaunch lqr_controller foc_balance_control.launch debug:=true

单独启动控制器
如果你想更精细地控制每个组件，可以单独启动控制器

# 只启动控制器节点
roslaunch lqr_controller lqr_vmc_controller.launch

# 在Gazebo中运行
roslaunch lqr_controller lqr_vmc_gazebo.launch

手动测试
你也可以手动发送控制命令进行测试：

# 发送目标状态（站立位置）
rostopic pub /desired_state std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.08, 0.0, 0.0, 0.0]"

# 发送目标状态（前倾）
rostopic pub /desired_state std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.08, 0.0, 0.0, 0.0]"

# 观察当前状态
rostopic echo /lqr_vmc_controller/state
// lqr_controller.cpp
#include "lqr_controller/lqr_controller.h"

LQRController::LQRController(ros::NodeHandle& nh) : nh_(nh) {
  // 初始化状态向量
  current_state_ = Eigen::VectorXd::Zero(6);
  desired_state_ = Eigen::VectorXd::Zero(6);
  
  // 从参数服务器加载参数
  nh_.param("control_rate", control_rate_, 200.0);  // 默认200Hz
  nh_.param("current_L0", current_L0_, 0.08);      // 默认L0=0.08m
  
  // 设置发布器和订阅器
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, 
                                   &LQRController::jointStateCallback, this);
  joint_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_commands", 10);
  
  // 新增：添加目标状态订阅者
  desired_state_sub_ = nh_.subscribe("/desired_state", 10,
                                    &LQRController::desiredStateCallback, this);
  
  // 启动控制循环
  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate_), 
                                   &LQRController::controlLoop, this);
                                   
  ROS_INFO("LQR controller initialized with L0 = %f", current_L0_);
}

void LQRController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // 将关节状态转换为状态向量
  // 假设msg包含了所有我们需要的状态信息（位置、速度）
  // 这里需要根据你的机器人具体结构来调整
  
  if (msg->position.size() >= 3 && msg->velocity.size() >= 3) {
    current_state_(0) = msg->position[0];  // theta
    current_state_(1) = msg->velocity[0];  // theta_dot
    current_state_(2) = msg->position[1];  // x
    current_state_(3) = msg->velocity[1];  // x_dot
    current_state_(4) = msg->position[2];  // phi
    current_state_(5) = msg->velocity[2];  // phi_dot
  }
}

// 新增：目标状态回调函数
void LQRController::desiredStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() >= 6) {
    desired_state_(0) = msg->data[0];  // theta
    desired_state_(1) = msg->data[1];  // theta_dot
    desired_state_(2) = msg->data[2];  // x
    desired_state_(3) = msg->data[3];  // x_dot
    desired_state_(4) = msg->data[4];  // phi
    desired_state_(5) = msg->data[5];  // phi_dot
    
    ROS_INFO("收到新的目标状态: [%f, %f, %f, %f, %f, %f]", 
             msg->data[0], msg->data[1], msg->data[2],
             msg->data[3], msg->data[4], msg->data[5]);
  } else {
    ROS_WARN("收到的目标状态数组长度不足 (%zu < 6)", msg->data.size());
  }
}

void LQRController::controlLoop(const ros::TimerEvent& event) {
  // 计算状态偏差
  Eigen::VectorXd state_error = current_state_ - desired_state_;
  
  // 根据当前L0值计算K矩阵
  Eigen::MatrixXd K = calculateK(current_L0_);
  
  // 计算控制输入 u = -K * state_error
  Eigen::VectorXd u = -K * state_error;
  
  // 发布控制命令
  std_msgs::Float64MultiArray cmd_msg;
  cmd_msg.data.push_back(u(0));  // T
  cmd_msg.data.push_back(u(1));  // Tp
  
  joint_cmd_pub_.publish(cmd_msg);
  
  // 添加调试日志，便于观察控制器工作状态
  ROS_DEBUG("当前状态: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
           current_state_(0), current_state_(1), current_state_(2),
           current_state_(3), current_state_(4), current_state_(5));
  ROS_DEBUG("目标状态: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
           desired_state_(0), desired_state_(1), desired_state_(2),
           desired_state_(3), desired_state_(4), desired_state_(5));
  ROS_DEBUG("控制输出: [%.3f, %.3f]", u(0), u(1));
}

Eigen::MatrixXd LQRController::calculateK(double L0) {
  // 根据MATLAB生成的多项式计算K矩阵
  Eigen::MatrixXd K(2, 6);
  
  double L0_2 = L0 * L0;
  double L0_3 = L0_2 * L0;
  
  // 根据myfile.m中的多项式系数计算K矩阵
  // K(0,0) - 第一行第一列
  K(0,0) = L0*(-3.21009594468645)-L0_2*1.881836058678146e+1+L0_3*2.837054639938506e+1-3.117217124460946e-2;
  
  // K(0,1) - 第一行第二列
  K(0,1) = L0*2.583148717002317e-2+L0_2*3.944223462903662e-1-L0_3*1.404764581659955-5.450506854508016e-3;
  
  // ... 填充其余K矩阵元素
  K(0,2) = L0*(-2.882754412918012e-1)-L0_2*4.923395347159518+L0_3*5.56114756200345-2.646917326353769e-3;
  K(0,3) = L0*7.533814531496149e-3+L0_2*7.101183505093456e-3-L0_3*3.244118479986221e-2+5.281644432392987e-6;
  K(0,4) = L0*(-1.195174408517629)-L0_2*8.585011416045143+L0_3*2.065972323608874e+1+7.704551835264361e-3;
  K(0,5) = L0*(-7.041557158977539e-2)+L0_2*4.118314970194606e-1-L0_3*9.55960281117106e-1+1.385760875423502e-2;
  
  K(1,0) = L0*(-9.830415514310217e-1)-L0_2*8.036583568014665+L0_3*1.806962507871544e+1-9.285061412466452e-3;
  K(1,1) = L0*3.258533064120575e-2-L0_2*1.308289511283231e-1+L0_3*2.329011192435436e-1+1.563506575445632e-3;
  K(1,2) = L0*(-3.595603866986543e+1)+L0_2*1.871124169373346e+2-L0_3*4.016884300029781e+2+3.881513133213613;
  K(1,3) = L0*1.4753086893779-L0_2*9.287636817603351+L0_3*2.188622591057195e+1+4.388328301724943;
  K(1,4) = L0*(-1.316008380499702)+L0_2*6.9594192207579-L0_3*1.507036590307988e+1+1.439766201813577e-1;
  K(1,5) = L0*3.285811617407983e-2-L0_2*2.056802904580541e-1+L0_3*4.837303212192992e-1+1.625717525892759e-1;
  
  return K;
}
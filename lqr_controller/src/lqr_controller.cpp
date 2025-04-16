// lqr_controller.cpp
#include "lqr_controller/lqr_controller.h"
#include <ros/ros.h>

LQRController::LQRController(ros::NodeHandle& nh) : nh_(nh), vmc_(nh), state_initialized_(true) {
  // 初始化状态向量
  current_state_ = Eigen::VectorXd::Zero(6);
  desired_state_ = Eigen::VectorXd::Zero(6);
  
  // 初始化关节状态
  phi1_ = 0.0;
  phi4_ = 0.0;
  dphi1_ = 0.0;
  dphi4_ = 0.0;
  l0_ = 0.0;
  phi0_ = 0.0;
  dl0_ = 0.0;
  dphi0_ = 0.0;
  
  // 从参数服务器加载参数
  nh_.param("control_rate", control_rate_, 100.0);  // 默认100Hz
  nh_.param("current_L0", current_L0_, 0.08);      // 默认L0=0.08m
  nh_.param("controller_topic", controller_topic_, std::string("effort_controllers"));

  // Store joint names in the map using the URDF names as keys.
  const std::string joint01_left = "joint01_left";
  const std::string joint01_right = "joint01_right";
  const std::string joint04_left = "joint04_left";
  const std::string joint04_right = "joint04_right";
  const std::string joint_tire_left = "joint_tire_left";
  const std::string joint_tire_right = "joint_tire_right";

  joint_names_["joint01_left"] = joint01_left;
  joint_names_["joint01_right"] = joint01_right;
  joint_names_["joint04_left"] = joint04_left;
  joint_names_["joint04_right"] = joint04_right;
  joint_names_["joint_tire_left"] = joint_tire_left;
  joint_names_["joint_tire_right"] = joint_tire_right;

  
  // 设置发布器和订阅器
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, 
                                  &LQRController::jointStateCallback, this);
  
  // Replace the single publisher with individual publishers for each joint
  joint01_left_pub_ = nh_.advertise<std_msgs::Float64>("/joint01_left_effort_controller/command", 10);
  joint01_right_pub_ = nh_.advertise<std_msgs::Float64>("/joint01_right_effort_controller/command", 10);
  joint04_left_pub_ = nh_.advertise<std_msgs::Float64>("/joint04_left_effort_controller/command", 10);
  joint04_right_pub_ = nh_.advertise<std_msgs::Float64>("/joint04_right_effort_controller/command", 10);
  joint_tire_left_pub_ = nh_.advertise<std_msgs::Float64>("/joint_tire_left_effort_controller/command", 10);
  joint_tire_right_pub_ = nh_.advertise<std_msgs::Float64>("/joint_tire_right_effort_controller/command", 10);
  
  // 添加目标状态订阅者
  desired_state_sub_ = nh_.subscribe("/desired_state", 10,
                                   &LQRController::desiredStateCallback, this);
  
  // 初始化目标状态
  nh_.param("default_state/theta", desired_state_(0), 0.0);
  nh_.param("default_state/theta_dot", desired_state_(1), 0.0);
  nh_.param("default_state/l0", desired_state_(2), 0.08);
  nh_.param("default_state/l0_dot", desired_state_(3), 0.0);
  nh_.param("default_state/phi", desired_state_(4), 0.0);
  nh_.param("default_state/phi_dot", desired_state_(5), 0.0);
  
  // 启动控制循环
  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate_), 
                                  &LQRController::controlLoop, this);
                                  
  ROS_INFO("LQR controller with VMC integration initialized with L0 = %.3f, control rate = %.1f Hz", 
           current_L0_, control_rate_);
  ROS_INFO("Using joints: %s, %s, %s, %s", 
            joint_names_["joint01_left"].c_str(), joint_names_["joint01_right"].c_str(),
            joint_names_["joint04_left"].c_str(), joint_names_["joint04_right"].c_str());
  ROS_INFO("send controll topic to: %s", controller_topic_.c_str());
}

int LQRController::findJointIndex(const std::vector<std::string>& joint_names, const std::string& target_joint) {
  for (size_t i = 0; i < joint_names.size(); i++) {
    if (joint_names[i] == target_joint) {
      return static_cast<int>(i);
    }
  }
  return -1;  // 未找到
}

void LQRController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // 更新关节状态和系统状态
  updateRobotState(msg);
}

void LQRController::updateRobotState(const sensor_msgs::JointState::ConstPtr& msg) {
  // 查找必要的关节索引
  // int front_left_idx = findJointIndex(msg->name, joint_names_["front_left"]);
  // int front_right_idx = findJointIndex(msg->name, joint_names_["front_right"]);
  // int rear_left_idx = findJointIndex(msg->name, joint_names_["rear_left"]);
  // int rear_right_idx = findJointIndex(msg->name, joint_names_["rear_right"]);
  int joint01_left_idx = findJointIndex(msg->name, joint_names_["joint01_left"]);
  int joint01_right_idx = findJointIndex(msg->name, joint_names_["joint01_right"]);
  int joint04_left_idx = findJointIndex(msg->name, joint_names_["joint04_left"]);
  int joint04_right_idx = findJointIndex(msg->name, joint_names_["joint04_right"]);

  
  // 检查是否找到了所有必要的关节
  if (joint01_left_idx < 0 || joint04_left_idx < 0) {
    ROS_WARN_THROTTLE(1.0, "Could not find joints %s or %s", 
                    joint_names_["joint01_left"].c_str(), joint_names_["joint04_right"].c_str());
    return;
  }

  // 如果找不到后腿电机关节，使用前腿电机关节代替（简化实现）
  // if (joint04_left_idx < 0 && joint01_right_idx < 0) {
  //   ROS_WARN_THROTTLE(1.0, "Could not find joints %s or %s, using front joints values", 
  //                   joint_names_["joint04_left"].c_str(), joint_names_["joint01_right"].c_str());
  //   joint04_left_idx = joint01_left_idx;
  //   joint01_right_idx = joint01_right_idx;
  // }
  
  // 检查关节状态数据有效性
  if (joint01_left_idx >= static_cast<int>(msg->position.size()) || 
      joint04_left_idx >= static_cast<int>(msg->position.size()) ||
      joint01_left_idx >= static_cast<int>(msg->velocity.size()) || 
      joint04_left_idx >= static_cast<int>(msg->velocity.size())) {
    ROS_WARN_THROTTLE(1.0, "Joint state message does not contain required position/velocity data");
    return;
  }
  
  // 更新关节角度和速度（我们用左右前腿作为虚拟模型控制的输入）
  phi1_ = msg->position[joint01_left_idx];
  phi4_ = msg->position[joint04_left_idx];
  dphi1_ = msg->velocity[joint01_left_idx];
  dphi4_ = msg->velocity[joint04_left_idx];
  
  // 使用VMC接口计算腿部状态
  try {
    Eigen::Vector2d leg_pos = vmc_.calculateLegPosition(phi1_, phi4_);
    l0_ = leg_pos(0);
    phi0_ = leg_pos(1);
    
    Eigen::Vector2d leg_spd = vmc_.calculateLegSpeed(dphi1_, dphi4_, phi1_, phi4_);
    dl0_ = leg_spd(0);
    dphi0_ = leg_spd(1);
    
    // 检查计算结果是否有效
    if (std::isnan(l0_) || std::isnan(phi0_) || std::isnan(dl0_) || std::isnan(dphi0_)) {
      ROS_WARN("VMC calculation returned NaN values");
      return;
    }
    
    // 更新状态向量用于LQR控制
    current_state_(0) = phi0_;        // theta（角度）
    current_state_(1) = dphi0_;       // theta_dot（角速度）
    current_state_(2) = l0_;          // x（位置）
    current_state_(3) = dl0_;         // x_dot（速度）
    current_state_(4) = (phi1_ + phi4_) / 2.0;  // phi（姿态角）
    current_state_(5) = (dphi1_ + dphi4_) / 2.0;  // phi_dot（姿态角速度）
    
    // 更新当前L0值，用于计算LQR增益
    current_L0_ = l0_;
    
    // 标记状态已初始化
    state_initialized_ = true;
    
    ROS_DEBUG("update state: l0=%.3f, phi0=%.3f, dl0=%.3f, dphi0=%.3f", 
              l0_, phi0_, dl0_, dphi0_);
  }
  catch (const std::exception& e) {
    ROS_ERROR("Exception in VMC calculation: %s", e.what());
  }
}

void LQRController::desiredStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() >= 6) {
    desired_state_(0) = msg->data[0];  // theta（腿角度）
    desired_state_(1) = msg->data[1];  // theta_dot（腿角速度）
    desired_state_(2) = msg->data[2];  // l0（腿长度）
    desired_state_(3) = msg->data[3];  // l0_dot（腿长度变化率）
    desired_state_(4) = msg->data[4];  // phi（机身姿态角）
    desired_state_(5) = msg->data[5];  // phi_dot（机身姿态角速度）
    
    ROS_INFO("New goal state: [theta=%.3f, dtheta=%.3f, l0=%.3f, dl0=%.3f, phi=%.3f, dphi=%.3f]", 
             msg->data[0], msg->data[1], msg->data[2],
             msg->data[3], msg->data[4], msg->data[5]);
  } else {
    ROS_WARN("length not enough (%zu < 6)", msg->data.size());
  }
}

void LQRController::controlLoop(const ros::TimerEvent& event) {
  // If state is not yet initialized, skip this cycle
  if (!state_initialized_) {
    ROS_WARN_THROTTLE(1.0, "Waiting for joint state initialization...");
    return;
  }
  
  // Calculate state error
  Eigen::VectorXd state_error = current_state_ - desired_state_;
  
  // Calculate K matrix based on current L0 value
  Eigen::MatrixXd K = calculateK(current_L0_);
  
  // Calculate LQR control force and torque u = -K * state_error
  Eigen::VectorXd u = -K * state_error;
  double F = u(0);  // Force
  double Tp = u(1); // Torque
  
  // Limit maximum force and torque to avoid instability
  double max_force, max_torque, max_motor_torque;
  nh_.param("max_force", max_force, 20.0);
  nh_.param("max_torque", max_torque, 5.0);
  nh_.param("max_motor_torque", max_motor_torque, 10.0);
  
  if (std::abs(F) > max_force) {
    F = F > 0 ? max_force : -max_force;
    ROS_WARN_THROTTLE(1.0, "Force limited to %.3f", F);
  }
  
  if (std::abs(Tp) > max_torque) {
    Tp = Tp > 0 ? max_torque : -max_torque;
    ROS_WARN_THROTTLE(1.0, "Torque limited to %.3f", Tp);
  }
  
  // Use VMC to convert force and torque to joint torques
  Eigen::Vector2d motor_torques;
  try {
    motor_torques = vmc_.convertToMotorTorques(F, Tp, phi1_, phi4_);
    
    // Check if calculation results are valid
    if (std::isnan(motor_torques(0)) || std::isnan(motor_torques(1))) {
      ROS_WARN("VMC returned NaN, using zero torques");
      motor_torques << 0.0, 0.0;
    }
    
    // Limit motor torque magnitude
    if (std::abs(motor_torques(0)) > max_motor_torque) {
      motor_torques(0) = motor_torques(0) > 0 ? max_motor_torque : -max_motor_torque;
    }
    
    if (std::abs(motor_torques(1)) > max_motor_torque) {
      motor_torques(1) = motor_torques(1) > 0 ? max_motor_torque : -max_motor_torque;
    }
  }
  catch (const std::exception& e) {
    ROS_ERROR("VMC error: %s", e.what());
    motor_torques << 0.0, 0.0;  // Use zero torque on error
  }
  
  
  // 发布控制命令到各个关节的effort controller
  std_msgs::Float64 cmd;
    
  // 给前腿马达设置力矩
  cmd.data = motor_torques(0);
  joint01_left_pub_.publish(cmd);

  cmd.data = motor_torques(1);
  joint01_right_pub_.publish(cmd);

  // 给后腿马达设置相同的力矩，以保持平衡
  cmd.data = motor_torques(0);
  joint04_left_pub_.publish(cmd);

  cmd.data = motor_torques(1);
  joint04_right_pub_.publish(cmd);

  // 轮胎关节不施加扭矩
  cmd.data = 0.0;
  joint_tire_left_pub_.publish(cmd);
  joint_tire_right_pub_.publish(cmd);
  
  // 添加调试日志，便于观察控制器工作状态
  ROS_DEBUG("Current State: [theta=%.3f, dtheta=%.3f, l0=%.3f, dl0=%.3f, phi=%.3f, dphi=%.3f]", 
    current_state_(0), current_state_(1), current_state_(2),
    current_state_(3), current_state_(4), current_state_(5));
  ROS_DEBUG("Goal state: [theta=%.3f, dtheta=%.3f, l0=%.3f, dl0=%.3f, phi=%.3f, dphi=%.3f]", 
      desired_state_(0), desired_state_(1), desired_state_(2),
      desired_state_(3), desired_state_(4), desired_state_(5));
  ROS_DEBUG("LQR output: F=%.3f, Tp=%.3f", F, Tp);
  ROS_DEBUG("Motor torques: T1=%.3f, T2=%.3f", motor_torques(0), motor_torques(1));
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
  
  // 填充其余K矩阵元素
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
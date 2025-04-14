// lqr_controller.h
#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>

class LQRController {
public:
  LQRController(ros::NodeHandle& nh);
  ~LQRController() {}

private:
  // 读取机器人状态
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  
  // 读取目标状态
  void desiredStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  
  // 控制循环
  void controlLoop(const ros::TimerEvent& event);
  
  // 根据L0值计算K矩阵
  Eigen::MatrixXd calculateK(double L0);

  // ROS接口
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber desired_state_sub_;  // 新增
  ros::Publisher joint_cmd_pub_;
  ros::Timer control_timer_;
  
  // 状态变量
  Eigen::VectorXd current_state_;  // [theta, theta_dot, x, x_dot, phi, phi_dot]
  Eigen::VectorXd desired_state_;  // 期望状态
  
  // 控制参数
  double control_rate_;  // 控制频率(Hz)
  double current_L0_;    // 当前的L0值
};

#endif // LQR_CONTROLLER_H
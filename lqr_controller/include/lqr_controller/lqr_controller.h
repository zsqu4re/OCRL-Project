// lqr_controller.h
#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <map>
#include <string>
#include "lqr_controller/vmc_interface.h"

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
  
  // 更新机器人状态（从关节状态计算l0, phi0等）
  void updateRobotState(const sensor_msgs::JointState::ConstPtr& msg);
  
  // 查找关节在关节数组中的索引
  int findJointIndex(const std::vector<std::string>& joint_names, const std::string& target_joint);

  // ROS接口
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber desired_state_sub_;  // 新增
  ros::Publisher joint_cmd_pub_;
  ros::Timer control_timer_;
  
  // VMC接口
  VMCInterface vmc_;
  
  // 状态变量
  Eigen::VectorXd current_state_;  // [theta, theta_dot, x, x_dot, phi, phi_dot]
  Eigen::VectorXd desired_state_;  // 期望状态
  
  // 关节名称
  std::map<std::string, std::string> joint_names_;
  
  // 关节状态
  double phi1_;       // 左马达角度
  double phi4_;       // 右马达角度
  double dphi1_;      // 左马达角速度
  double dphi4_;      // 右马达角速度
  double l0_;         // 腿长度
  double phi0_;       // 腿角度
  double dl0_;        // 腿长度变化率
  double dphi0_;      // 腿角度变化率
  
  // 控制参数
  double control_rate_;  // 控制频率(Hz)
  double current_L0_;    // 当前的L0值
  std::string controller_topic_; // 控制器发布话题
  
  // 控制器状态
  bool state_initialized_;  // 状态是否已初始化
};

#endif // LQR_CONTROLLER_H
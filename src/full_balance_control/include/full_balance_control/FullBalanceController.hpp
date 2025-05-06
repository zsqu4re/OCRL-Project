#pragma once

#include <controller_interface/controller.h>
// #include <hardware_interface/effort_joint_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

namespace full_balance_control
{

class FullBalanceController
  : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  FullBalanceController() = default;
  ~FullBalanceController() override = default;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override {}
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override {}

private:
  // joints
  hardware_interface::JointHandle left_wheel_, right_wheel_;

  // IMU subscriber & state
  ros::Subscriber imu_sub_;
  double roll_{0}, pitch_{0}, roll_rate_{0}, pitch_rate_{0};

  // PD gains
  double Krp_, Krd_, Kpp_, Kpd_;

  // callback
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
};

}  // namespace full_balance_control

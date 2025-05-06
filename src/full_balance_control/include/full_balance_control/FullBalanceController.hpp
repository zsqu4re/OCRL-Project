#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/effort_joint_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

namespace full_balance_control
{

class FullBalanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  FullBalanceController();
  ~FullBalanceController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  // Joints
  hardware_interface::JointHandle left_wheel_joint_;
  hardware_interface::JointHandle right_wheel_joint_;

  // IMU
  ros::Subscriber imu_sub_;
  double roll_, roll_rate_;
  double pitch_, pitch_rate_;

  // Gains
  double Krp_, Krd_, Kpp_, Kpd_;

  // IMU callback
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
};

} // namespace

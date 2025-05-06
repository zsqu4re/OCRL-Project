#include "full_balance_control/FullBalanceController.hpp"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

namespace full_balance_control
{

bool FullBalanceController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle& nh)
{
  // get handles
  left_wheel_  = hw->getHandle("left_wheel_joint");
  right_wheel_ = hw->getHandle("right_wheel_joint");

  // subscribe IMU
  imu_sub_ = nh.subscribe("imu/data", 1, &FullBalanceController::imuCallback, this);

  // load gains
  nh.param("Krp", Krp_, 50.0);
  nh.param("Krd", Krd_, 5.0);
  nh.param("Kpp", Kpp_, 100.0);
  nh.param("Kpd", Kpd_, 10.0);

  return true;
}

void FullBalanceController::update(const ros::Time&, const ros::Duration&)
{
  // PD on roll & pitch
  double roll_ctrl  = Krp_ * (-roll_)  + Krd_ * (-roll_rate_);
  double pitch_ctrl = Kpp_ * (-pitch_) + Kpd_ * (-pitch_rate_);

  // mix into wheel efforts
  left_wheel_.setCommand( pitch_ctrl + roll_ctrl );
  right_wheel_.setCommand( pitch_ctrl - roll_ctrl );
}

void FullBalanceController::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  // quaternion → RPY
  tf::Quaternion q{ msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w };
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  roll_      = roll;
  pitch_     = pitch;
  roll_rate_ = msg->angular_velocity.x;
  pitch_rate_= msg->angular_velocity.y;
}

}  // namespace full_balance_control

PLUGINLIB_EXPORT_CLASS(full_balance_control::FullBalanceController,
                       controller_interface::ControllerBase)

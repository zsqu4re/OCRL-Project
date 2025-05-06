#include <pluginlib/class_list_macros.h>
#include "full_balance_control/FullBalanceController.hpp"
#include <tf/transform_datatypes.h>

namespace full_balance_control
{

FullBalanceController::FullBalanceController() {}
FullBalanceController::~FullBalanceController() {}

bool FullBalanceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
  left_wheel_joint_ = hw->getHandle("left_wheel_joint");
  right_wheel_joint_ = hw->getHandle("right_wheel_joint");

  imu_sub_ = nh.subscribe("imu/data", 1, &FullBalanceController::imuCallback, this);

  nh.param("Krp", Krp_, 50.0);
  nh.param("Krd", Krd_, 5.0);
  nh.param("Kpp", Kpp_, 100.0);
  nh.param("Kpd", Kpd_, 10.0);

  roll_ = pitch_ = roll_rate_ = pitch_rate_ = 0.0;

  return true;
}

void FullBalanceController::starting(const ros::Time& time)
{
}

void FullBalanceController::update(const ros::Time& time, const ros::Duration& period)
{
  double roll_control = Krp_ * (-roll_) + Krd_ * (-roll_rate_);
  double pitch_control = Kpp_ * (-pitch_) + Kpd_ * (-pitch_rate_);

  double left_effort = pitch_control + roll_control;
  double right_effort = pitch_control - roll_control;

  left_wheel_joint_.setCommand(left_effort);
  right_wheel_joint_.setCommand(right_effort);
}

void FullBalanceController::stopping(const ros::Time& time)
{
}

void FullBalanceController::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  tf::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  roll_ = roll;
  pitch_ = pitch;
  roll_rate_ = msg->angular_velocity.x;
  pitch_rate_ = msg->angular_velocity.y;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(full_balance_control::FullBalanceController, controller_interface::ControllerBase)

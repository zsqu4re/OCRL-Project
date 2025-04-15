// lqr_controller_node.cpp
#include <ros/ros.h>
#include "lqr_controller/lqr_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_controller_node");
  ros::NodeHandle nh;
  
  LQRController controller(nh);
  
  ros::spin();
  
  return 0;
}
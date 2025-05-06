cmake_minimum_required(VERSION 3.0.2)
project(full_balance_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES full_balance_controller
  CATKIN_DEPENDS controller_interface hardware_interface pluginlib realtime_tools roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(full_balance_controller
  src/FullBalanceController.cpp
)

target_link_libraries(full_balance_controller ${catkin_LIBRARIES})

install(TARGETS full_balance_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


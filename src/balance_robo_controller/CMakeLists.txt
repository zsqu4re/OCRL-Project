cmake_minimum_required(VERSION 3.0.2)
project(balance_robot_controller)

## 查找项目依赖的包
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
)


## 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS controller_interface hardware_interface pluginlib roscpp sensor_msgs std_msgs geometry_msgs tf
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 声明C++库并添加源文件
add_library(${PROJECT_NAME}
  src/balance_robot_controller.cpp
  src/leg_pos.cpp
  src/leg_spd.cpp
  src/leg_conv.cpp
  src/lqr_k.cpp
  src/pid.cpp
)

## 添加依赖
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## 安装目标
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## 安装插件描述文件
install(FILES balance_robot_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



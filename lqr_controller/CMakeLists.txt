cmake_minimum_required(VERSION 3.0.2)
project(lqr_controller)

## 使用C++11标准
add_compile_options(-std=c++11)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  sensor_msgs
  std_msgs
)

## 系统依赖项
find_package(Eigen3 REQUIRED)

## catkin特定配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lqr_controller
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

###########
## 构建 ##
###########

## 指定头文件的额外位置
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 声明C++库
add_library(${PROJECT_NAME}
  src/vmc_interface.cpp
  src/lqr_controller.cpp
)

## 声明C++可执行文件
add_executable(lqr_controller_node 
  src/lqr_controller_node.cpp
)

## 指定库链接的目标
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(lqr_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## 安装 ##
#############

## 标记可执行文件和库进行安装
install(TARGETS ${PROJECT_NAME} lqr_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 标记头文件安装
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## 标记其他文件安装(如启动文件和配置文件)
install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 标记可执行Python脚本
catkin_install_python(PROGRAMS
  scripts/test_controller.py
  scripts/test_gazebo_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
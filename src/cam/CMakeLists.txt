cmake_minimum_required(VERSION 3.0.2)
project(cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/camera_error_publisher.py  # 确保你的Python脚本放在项目src目录下
  scripts/cam_target_distpub.py  # 确保你的Python脚本放在项目src目录下
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
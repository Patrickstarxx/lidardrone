cmake_minimum_required(VERSION 3.0.2)
project(cam)

find_package(Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  OpenCV
  cv_bridge
)


catkin_package(
  #INCLUDE_DIRS include
  #CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	)

install(PROGRAMS
  scripts/camera_error_publisher.py  # 确保你的Python脚本放在项目src目录下
  scripts/cam_target_distpub.py  # 确保你的Python脚本放在项目src目录下
  scripts/aruco_det.py
  scripts/aruco_py.py
  scripts/aruco_img.py
  scripts/aruco_img_multi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(aruco_detect src/aruco_detect.cpp)
target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})


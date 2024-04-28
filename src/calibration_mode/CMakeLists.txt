cmake_minimum_required(VERSION 3.0.2)
project(calibration_mode)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp 
) 

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(calibration_mode src/calibration.cpp src/workingSpaceTF.cpp)

target_link_libraries(
  calibration_mode
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
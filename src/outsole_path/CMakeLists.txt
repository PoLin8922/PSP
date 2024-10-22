cmake_minimum_required(VERSION 3.0.2)
project(outsole_path)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath=/usr/lib/gcc/x86_64-linux-gnu/9")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  get_outpath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Open3D REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs std_srvs
) 

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)


# add_executable(outsole_path src/outsole_path.cpp src/workingSpaceTF.cpp src/planning.cpp src/pointcloud_process.cpp)

# target_link_libraries(
#   outsole_path
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${Open3D_LIBRARIES}
# )

catkin_install_python(PROGRAMS
  src/outsole_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(bringup_driver)

## Compile as C++11, supported in ROS Kinetic and newer
set( CMAKE_BUILD_TYPE Debug )
add_compile_options(-std=c++11 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  amr_msgs
  amr_common
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
 # LIBRARIES
#  LIBRARIES usbcannodemessage_runtime
 CATKIN_DEPENDS roscpp  std_msgs geometry_msgs amr_common
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${catkin_LIB_DIRS} 
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
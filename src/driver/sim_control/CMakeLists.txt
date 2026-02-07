cmake_minimum_required(VERSION 3.0.2)
project(sim_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common
  controller_manager
  gazebo_ros
  gazebo_ros_control
  # ros_control
  # ros_controllers
  rospy
  std_msgs
  cmake_modules
  tf
  eigen_conversions
)


find_package(Eigen3 REQUIRED)
# find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_control
#  CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control ros_control ros_controllers rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY 
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
 scripts/key_ctrl.py
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

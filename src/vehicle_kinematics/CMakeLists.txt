cmake_minimum_required(VERSION 3.0.2)
project(vehicle_kinematics)

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
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES
#  LIBRARIES usbcannodemessage_runtime
 CATKIN_DEPENDS roscpp  std_msgs geometry_msgs
#  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  #${catkin_LIB_DIRS} 
)

add_executable(vehicle_kinematics_node  src/vehicle_kinematics_node.cpp
 src/single_servo.cpp 
 src/dual.cpp
 src/tinyxml2.cpp src/omv.cpp)
target_link_libraries(vehicle_kinematics_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(vehicle_kinematics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_cmd_node
 src/test_cmd.cpp )
target_link_libraries(test_cmd_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(test_cmd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS test_cmd_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS vehicle_kinematics_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
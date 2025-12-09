cmake_minimum_required(VERSION 3.0.2)
project(example)

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
  INCLUDE_DIRS include
 # LIBRARIES
#  LIBRARIES usbcannodemessage_runtime
 CATKIN_DEPENDS roscpp  std_msgs geometry_msgs amr_common
#  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  #${catkin_LIB_DIRS} 
)


add_executable(action_cilent_test
 src/action_cilent_test.cc src/track_path_client.h)
target_link_libraries(action_cilent_test ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(action_cilent_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bypass_obstacle
 src/bypass_obstacle.cc src/track_path_client.h)
target_link_libraries(bypass_obstacle ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(bypass_obstacle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS bypass_obstacle
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS action_cilent_test
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
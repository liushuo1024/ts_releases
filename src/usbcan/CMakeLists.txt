cmake_minimum_required(VERSION 2.8.3)
project(usbcan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  common
  can_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES
#  LIBRARIES usbcannodemessage_runtime
 CATKIN_DEPENDS roscpp std_msgs can_msgs
#  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  #${catkin_LIB_DIRS} lib
)

link_directories(
  ${catkin_LIB_DIRS} lib
)


add_executable(like_usbcannode  src/likecan.cpp)
target_link_libraries(like_usbcannode ${catkin_LIBRARIES} libCanCmd.so ${Boost_LIBRARIES})
add_dependencies(like_usbcannode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(orangepi_cannode  src/orangepi_can.cpp)
target_link_libraries(orangepi_cannode ${catkin_LIBRARIES} )
add_dependencies(orangepi_cannode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS like_usbcannode 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS orangepi_cannode 
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
install(FILES lib/libCanCmd.so
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
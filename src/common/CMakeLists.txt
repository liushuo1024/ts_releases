cmake_minimum_required(VERSION 3.0.2)
project(common)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_message_files(
   FILES
   can.msg
  #  Battery.msg
   omv_cmd.msg
   omv_servo_cmd.msg
   omv_servo_encoder.msg
   servo_omv_cmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS 
 # LIBRARIES
#  LIBRARIES usbcannodemessage_runtime
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime 
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${catkin_LIB_DIRS} 
)

include_directories(
 ${catkin_LIB_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS} 
)


install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
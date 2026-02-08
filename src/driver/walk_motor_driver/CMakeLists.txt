cmake_minimum_required(VERSION 3.0.2)
project(walk_motor_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  can_msgs
  common
  message_generation
)

# 生成额外依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  can_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    can_msgs
    common
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# 可执行文件
add_executable(walk_motor_driver_node src/walk_motor_driver.cpp)

target_link_libraries(walk_motor_driver_node
  ${catkin_LIBRARIES}
)

# 安装
install(TARGETS walk_motor_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

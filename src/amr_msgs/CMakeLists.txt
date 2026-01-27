cmake_minimum_required(VERSION 2.8.3)
project(amr_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

cmake_policy(SET CMP0046 OLD)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
)

# find_package(catkin REQUIRED genjava)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  
  # 与调度通讯相关的msg (update_event request_response)
  step.msg
  ## update_event
  task_msg/update_event/update_event.msg
  ## request_response
  task_msg/request_response/task_response.msg # 请求应答包(统一)
  ### agv --> ds (agv发起请求, 调度应答)
  task_msg/request_response/agv_ds/agv_info_response.msg
  task_msg/request_response/agv_ds/battery_info_response.msg
  task_msg/request_response/agv_ds/task_finish_request.msg
  task_msg/request_response/agv_ds/finish_relocation_request.msg
  ### ds <--> agv (调度发起请求, agv应答)
  task_msg/request_response/ds_agv/agv_info_request.msg
  task_msg/request_response/ds_agv/battery_info_request.msg
  task_msg/request_response/ds_agv/motor_para_request.msg
  task_msg/request_response/ds_agv/task_request.msg
  task_msg/request_response/ds_agv/task_calibration_request.msg
  task_msg/request_response/ds_agv/task_get_map_request.msg
  task_msg/request_response/ds_agv/task_control_request.msg
  task_msg/request_response/ds_agv/task_control_audio_request.msg
  task_msg/request_response/ds_agv/task_tts_request.msg
  
  # 监控msg
  diagnostic_msg/node_diagnostic.msg
  diagnostic_msg/fault_report.msg
  error_state.msg
  error_info.msg
  
  # 设备msg
  device_msg/curtis_feedback.msg
  device_msg/io_feedback.msg
  device_msg/audio_feedback.msg
  device_msg/single_line_laser_feedback.msg
  device_msg/gyro_rion_feedback.msg
  device_msg/battery_feedback.msg
  device_msg/motor_motec_feedback.msg
  device_msg/motor_syntro_feedback.msg
  device_msg/pgv_r2100_feedback.msg
  device_msg/olm100_feedback.msg
  device_msg/pgv100_feedback.msg
  device_msg/rfid_feedback.msg
  device_msg/sick_mls_feedback.msg
  device_msg/rfid_feedback.msg
  device_msg/bmmsk34_encoder_feedback.msg
  device_msg/ultarsonic_feedback.msg
  device_msg/ks103_feedback.msg
  device_msg/valve_feedback.msg
  device_msg/linde_kob_feedback.msg
  device_msg/hikvs_feedback.msg
  device_msg/hinson_mls_feedback.msg
  device_msg/manipulator_feedback.msg
  device_msg/weighing_feedback.msg
  device_msg/dongle_feedback.msg
  device_msg/wit61_motion_feedback.msg
  device_msg/multi_pedestrian_pose.msg
  device_msg/pedestrian_detect_feedback.msg
  device_msg/AprilTagDetectionArray.msg
  
  # 硬件抽象msg (action、io_state、move、safety)
  ## action
  hal_msg/action/forklift_action.msg
  hal_msg/action/jack_up_action.msg
  hal_msg/action/roller_action.msg
  hal_msg/action/led_control.msg
  hal_msg/action/audio_control.msg
  ## io_state
  hal_msg/io_state/jack_up_io_state.msg
  hal_msg/io_state/roller_io_state.msg
  hal_msg/io_state/pallet_fork_io_state.msg
  
  ## move
  hal_msg/move/move_cmd.msg
  hal_msg/move/move_feedback.msg
  ## safety
  hal_msg/safety/safety_io_state.msg
  hal_msg/safety/safety_setting.msg
  hal_msg/safety/safety_state.msg

  ## 电池信息
  hal_msg/battery.msg
  ## 电源信息
  hal_msg/power_supply.msg 
  ## 载货状态
  hal_msg/load_state.msg
  ## 手动模式
  hal_msg/manual.msg
  ## 急停信号
  hal_msg/emergency.msg
  ## teleop模式
  hal_msg/teleop_mode.msg
  ## 充电继电器
  hal_msg/charge_do_state.msg
  ## 磁条导航数据
  hal_msg/mls_navigation.msg
  ## 库位托盘检测
  hal_msg/pallet_detect_state.msg
  ## 掉货检测
  weighting_loaded.msg
  ## 夹抱机构信息
  hal_msg/clamp_state.msg
  ## 防撞条复位
  hal_msg/bump_reset.msg
  ## 防撞条复位


  # 视觉类消息
  
  ## 前向相机障碍物距离信息
  visual_msg/front_camera_distance.msg
  ## 托盘车卸货信息
  visual_msg/pallet_stablizer.msg
  ## 托盘车整定信息
  visual_msg/pallet_center_feedback.msg
  ## 请求april_tag
  visual_msg/request_pallet_center.msg
  ## apriltag整定信息
  visual_msg/april_tag_pallet_stablizer.msg
  ## 请求april_tag
  visual_msg/request_april_tag.msg
  ## 请求货物尺寸信息
  visual_msg/request_measure_size.msg
  ## 货物尺寸信息
  visual_msg/measure_size_feedback.msg
  ## 请求检查叉腿后方超板
  visual_msg/request_pallet_back_limit.msg
  ## 反馈叉腿后方超板
  visual_msg/pallet_back_limit_feedback.msg

  visual_msg/tag_pose_to_cam_with_id.msg

  ## 放货区域检测信息
  detect_msg/feedback/unload_detect_feedback.msg
  detect_msg/request/request_unload_detect.msg
  
  # 调试相关
  robot_pose.msg
  reflector.msg
  reflector_list.msg
  pose_deviation.msg
  pose_deviation_test.msg
  linde_mode_debug.msg
  pallet_nomove_info.msg
  adjust_up.msg

  # apriltag相关
  AprilTagDetection.msg

  test_scan_data.msg
  # carto landmark
  LandmarkEntry.msg
  LandmarkList.msg

  # 标定参数
  motion_calibration.msg
  motion_calibration_cmd.msg
  test_move_cmd.msg
  test_move_feedback.msg
  match_mark_num.msg
  pallet_act_info.msg
  qr_calibration.msg

  # 定位相关
  slam_pose_deviation.msg
  re_location.msg

  # 加密数据
  dongle_to_dispatch.msg

  # 保护参数
  stop_protect.msg
  # 系统初始化
  update_init_pose.msg

  # 库位查询
  stock.msg
  storage.msg

  # 行人检测
  pose3D.msg
  pose2D.msg
)
  
## Generate services in the 'srv' folder
add_service_files(
  FILES
  agv_query_dispatch.srv
  switch_map.srv
  query_storage_info.srv
  calibrate_visual_angle.srv
  calibrate_visual_background.srv
  reset_bmmsk34_height.srv
  reset_tda04_weight.srv
  reset_weight.srv
  # 更新carto初始位置
  update_carto_init_pose.srv
  # 更新单机任务
  update_single_task.srv
  # 更新參數 都是空文件,只是利用 service 机制
  config/update_action_config.srv
  config/update_navigation_config.srv
  config/update_avoid_config.srv
  config/update_avoid_area_config.srv
  config/update_logic_config.srv
  config/update_localizer_config.srv
  config/update_embedded_config.srv
  config/update_realsense_config.srv
  config/update_storage_config.srv
  
  
  # 调用carto
  carto/stop_ref_pub.srv
  carto/save_carto_map.srv
  carto/update_carto_map.srv
  carto/start_carto_with_pure_localizer.srv
  carto/start_carto_with_mapping.srv
  carto/finish_trajectory.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  agv_action.action
  openloop.action
  track_path.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  actionlib_msgs
  amr_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime geometry_msgs std_msgs
#  INCLUDE_DIRS include
#  LIBRARIES amr_common
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/amr_common.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/amr_common_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   homeMap.json
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_amr_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

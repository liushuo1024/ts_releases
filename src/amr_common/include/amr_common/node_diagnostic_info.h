/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_NODE_DIAGNOSTIC_INFO_H_
#define amr_COMMON_INCLUDE_amr_COMMON_NODE_DIAGNOSTIC_INFO_H_
#include <amr_msgs/node_diagnostic.h>

namespace amr_diagnostic {

// 节点监控枚举
enum class CommunicateNodeStatus : uint16_t {
  /*------------INFO 0~299 ------------*/
  NORMAL = 1000,

  /*------------WARN 300~599 ------------*/
  WARN = 1300,
  MSG_TYPE_ABNORMAL = WARN + 1,

  /*------------ERROR 600~999 ------------*/
  ERROR = 1600,
  CONFIG_ERROR = ERROR + 1,
  TIMEOUT = ERROR + 2,

};

enum class DecisionMakerNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 2000,

  /*------------WARN 300~599 ------------*/
  WARN = 2300,
  TASK_SEQUENCE_NUMBER_ERROR = WARN + 1,
  TASK_SEQUENCE_ID_ERROR = WARN + 2,
  TASK_TYPE_ERROR = WARN + 3,

  /*------------ERROR 600~999 ------------*/
  ERROR = 2600,
  CONFIG_ERROR = ERROR + 1,
  TASK_ORDER_ID_ERROR = ERROR + 2,
  DONGLE_DIED = ERROR + 3,
  LOST_GOODS = ERROR + 4

};

enum class EmbeddedNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 3000,

  /*------------WARN 300~599 ------------*/
  WARN = 3300,
  CAN_0_SEND_ERROR = WARN + 1,
  CAN_1_SEND_ERROR = WARN + 2,
  CAN_RECEIVE_TIME_OUT = WARN + 3,
  CAN_CYCLE_TIME_OUT = WARN + 4,

  /*------------ERROR 600~999 ------------*/
  ERROR = 3600,
  CONFIG_ERROR = ERROR + 1,
  CAN_INIT_ERROR = ERROR + 2,
  AGV_HARDWARE_INIT_ERROR = ERROR + 3,
  DRIVER_NOT_READY = ERROR + 4

};

enum class LocalizerNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 4000,

  /*------------WARN 300~599 ------------*/
  WARN = 4300,
  GET_LANDMARK_MAP_ERROR = WARN + 1,
  INIT_FAILED = WARN + 2,
  ILLEGAL_QR_TAG_ERROR = WARN + 3,
  DOWN_QR_CAMERA_ERROR = WARN + 4,
  GET_CALIBRATION_MAP_ERROR = WARN + 5,
  UP_QR_CAMERA_ERROR = WARN + 6,
  LOCALIZER_INIT_POSE_ERROR = WARN + 7,
  ENVIROMENT_MATCH_WARING = WARN + 8,

  /*------------ERROR 600~999 ------------*/
  ERROR = 4600,
  CONFIG_ERROR = ERROR + 1,
  LASER_MESSAGE_EMPTY = ERROR + 2,
  STATIC_LOCALIZER_ERROR = ERROR + 3,
  GET_VELOCITY_ERROR = ERROR + 4,
  SLAM_POSE_ERROR = ERROR + 5,

};

enum class CalibrationNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 5000,

  /*------------WARN 300~599 ------------*/
  WARN = 5300,
  GET_LANDMARK_MAP_ERROR = WARN + 1,
  ENVIROMENT_MATCH_WARING = WARN + 2,

  /*------------ERROR 600~999 ------------*/
  ERROR = 5600,
  CONFIG_ERROR = ERROR + 1,
  LASER_MESSAGE_EMPTY = ERROR + 2

};

enum class NavigationNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 6000,

  /*------------WARN 300~599 ------------*/
  WARN = 6300,
  DOWN_TAG_LOSS = WARN + 1,
  UP_TAG_LOSS = WARN + 2,
  STABILIZE_TIMEOUT = WARN + 3,
  POWERON_INIT_TIME_OUT = WARN + 4,
  PALLET_LIMIT_ERROR = WARN + 5,

  /*------------ERROR 600~999 ------------*/
  ERROR = 6600,
  CONFIG_ERROR = ERROR + 1,
  OUT_OF_THE_ROUTE = ERROR + 2,
  DOCKING_TIMEOUT = ERROR + 3,
  SHELF_LOSS = ERROR + 4
};

enum class AvoidNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 7000,

  /*------------WARN 300~599 ------------*/
  WARN = 7300,
  BUMP_ERROR = WARN + 1,
  PEDERSTRAIN_WARING = WARN + 2,
  AVOID_TIME_OUT = WARN + 3,

  /*------------ERROR 600~999 ------------*/
  ERROR = 7600,
  CONFIG_ERROR = ERROR + 1,
  AVOID_MAP_ERROR = ERROR + 2,
  DATA_TIME_OUT = ERROR + 3
};

enum class StorageNodeStatus : uint16_t {
  /*------------INFO 0~299 ------------*/
  NORMAL = 8000,

  /*------------WARN 300~599 ------------*/
  WARN = 8300,

  /*------------ERROR 600~999 ------------*/
  ERROR = 8600,
  CONFIG_ERROR = ERROR + 1

};

enum class VisualNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 9000,

  /*------------WARN 300~599 ------------*/
  WARN = 9300,

  /*------------ERROR 600~999 ------------*/
  ERROR = 9600,
  CONFIG_ERROR = ERROR + 1,
  LACK_OF_ANGLE = ERROR + 2,
  LACK_OF_BACKGROUND = ERROR + 3,
  FAIL_INIT = ERROR + 4

};

enum class ActionNodeStatus : uint16_t {

  /*------------INFO 0~299 ------------*/
  NORMAL = 10000,

  /*------------WARN 300~599 ------------*/
  WARN = 10300,
  EXCEPTION_NO_PALLET_ERR = WARN + 1,
  PALLET_DETECT_ERROR = WARN + 2,
  PALLET_LIMIT_ERROR = WARN + 3,

  /*------------ERROR 600~999 ------------*/
  ERROR = 10600,
  CONFIG_ERROR = ERROR + 1,
  ACTION_TYPE_ERR = ERROR + 2,
  BAD_PARAMETER_ERR = ERROR + 3,
  ACTION_TIME_OUT = ERROR + 4,
  PALLET_DOWN_ERR = ERROR + 5,
  PALLET_ROTATE_ERR = ERROR + 6,
  CHARGE_ERR = ERROR + 7,
  PALLET_ZERO_ERR = ERROR + 8,
  PALLET_MOVE_ERR = ERROR + 9,
  UP_WEIGHT_CHECK_ERROR = ERROR + 10,
  DOWN_WEIGHT_CHECK_ERROR = ERROR + 11,
  SHELF_LOSS = ERROR + 12

};

struct diagnostic {
  uint32_t time_stamp;
  uint16_t status;
};

}  // namespace amr_diagnostic

#endif  // amr_COMMON_INCLUDE_amr_COMMON_NODE_DIAGNOSTIC_INFO_H_

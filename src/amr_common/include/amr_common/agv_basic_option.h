/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_AGV_BASIC_OPTION_H_
#define amr_COMMON_INCLUDE_amr_COMMON_AGV_BASIC_OPTION_H_

#include <string>
#include <vector>
struct UnicycleModelOption {
  float wheel_track;                // 轮间距 m
  float wheel_diameter;             // 轮子直径 m
  float move_motor_reduce_ratio;    // 移动电机减速比 系数
  float lift_motor_reduce_ratio;    // 顶升电机减速比
  float lift_screw_lead;            // 顶升结构导程
  float lift_max_height;            // 顶升电机最大顶升高度
  float rotate_motor_reduce_ratio;  // 旋转电机减速比
  int lift_mechanism;    // 升降机构:1-丝杆 2-剪刀叉 3-IO气缸
  bool use_lift;         // 启用顶升
  bool lift_motor_mode;  // 顶升电机工作模式：0-速度 1:绝对位置
  bool use_rotate;       // 启用旋转
  bool use_up_camera;    // 启用向上电机
  bool use_down_camera;  // 启用向下电机
  bool use_io_laser_avoid;  // 启用激光IO雷达避障
};

struct BicycleModelOption {
  float wheel_base;  // 轴距 m
  float max_wheel_base;
  float min_wheel_base;
  float eccentric_distance;  // 舵轮距过后轮中心垂线的距离(偏心距) m
  float eccentric_angle;  // 舵轮(偏心角度) 弧度
  float reduction_ratio;  // 辅助减速比
};

struct MoveModelControl {
  // 传感器屏蔽相关参数
  bool use_fork_avod;
  double fork_avoid_dist;
  bool fork_io_positive; // 叉尖光电是否高电平触发
  // 惯性补偿相关参数
  bool use_inertia_compensation;
  double velocity_coefficient;
  double omega_coefficient;
  // 手动控制相关参数
  float pallet_up_speed;
  float pallet_down_speed;
  float pallet_rotate_speed;
  float forward_speed;
  float backward_speed;
  float rotate_left_speed;
  float rotate_right_speed;
  uint8_t led_type;
  uint8_t audio_type;
  bool enter_low_power;
  bool enable_charge;
  //电机选装方向定义
  float set_left_sign;
  float set_right_sign;
  float feedback_left_sign;
  float feedback_right_sign;
  float rotate_motor_sign;
  float lift_motor_sign;
};

struct SensorTransform {
  double translation_x;
  double translation_y;
  double rotation;
};

typedef std::map<std::string, SensorTransform> SensorTFData;

struct LaserSensorInstallOption {
  double laser_offset_x;
  double laser_offset_y;
  double laser_offset_theta;
};
struct UltrasonicSensorOption {
  int32_t type;
  int32_t num;
};

struct QrCameraInfo {
  int32_t up_camera_type;
  int32_t down_camera_type;
};

struct SensorInstallOption {
  LaserSensorInstallOption laser_sensor_install_option;
};

struct MechanicalOption {
  int model_type;
  UnicycleModelOption unicycle_model_option;
  BicycleModelOption bicycle_modle_option;
  // TODO(@someone) 后续统一所有tf
  SensorInstallOption sensor_install_option;
  QrCameraInfo qr_camera_info;
};

struct LandmarkMapOption {
  std::string http_get_landmark_map_url;
  std::string http_update_landmark_map_url;
  int http_landmark_map_port;
  int current_reflector_map_id;
  int current_reflector_section_id;
  int current_qr_map_id;
  int current_qr_section_id;
  int current_rfid_map_id;
  int current_rfid_section_id;
};

struct SlamMapOption {
  std::string http_get_slam_pgm_url;
  std::string http_get_slam_yaml_url;
  std::string http_get_slam_pbstream_url;
};

#endif  // amr_COMMON_INCLUDE_amr_COMMON_AGV_BASIC_OPTION_H_

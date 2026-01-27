/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_amr_CONFIG_HELPER_H_
#define amr_COMMON_INCLUDE_amr_COMMON_amr_CONFIG_HELPER_H_
#include <pwd.h>
#include <ros/package.h>
#include <unistd.h>

#include <map>
#include <string>

#include "amr_common/agv_basic_option.h"
#include "amr_common/amr_enum_type.h"
#include "amr_common/log_porting.h"
#include "amr_common/util/json11.h"
#include "amr_common/util/local_service.h"
#include "amr_common/util/remote_service.h"
#include "amr_common/util/singleton.h"
#include "amr_msgs/update_action_config.h"
#include "amr_msgs/update_avoid_config.h"
#include "amr_msgs/update_embedded_config.h"
#include "amr_msgs/update_localizer_config.h"
#include "amr_msgs/update_logic_config.h"
#include "amr_msgs/update_navigation_config.h"

namespace amr_config {
enum class ConfigType : uint8_t {
  NONE = 0,
  AGV_BASIC_CONFIG = 1,
  LOGIC_CONFIG = 2,
  LOCALIZER_CONFIG = 3,
  NAVIGATION_CONFIG = 4,
  ACTION_CONFIG = 5,
  AVOID_CONFIG = 6,
  EMBEDDED_CONFIG = 7,
  DEVICE_TABLE = 8,
  AVOID_AREA = 9,
  LOCALIZER_LANDMARK = 10,
  SINGLE_TASK = 11,
  LAST_ROBOT_POSE = 12,
  VISUAL_CONFIG = 13,
  STORAGE_CONFIG = 14,
  GLOBAL_MAP = 15,
  // 二维码标定
  QR_CALIBRATION_MAP = 16,
  TF_CONFIG = 17,
  // 用来测试代码
  TEST_CONFIG = 0xFF,
  // 测试二维码标定
  TEST_QR_CALIBRATE = 0xFE
};

constexpr char kAgvBasicConfigPath[] = "agv_basic_config.json";
constexpr char kLogicConfigPath[] = "logic_config.json";
constexpr char kLocalizerConfigPath[] = "localizer_config.json";
constexpr char kLastRobotPoseConfigPath[] = "last_robot_pose_config.json";
constexpr char kLocalizerLandmarkConfigPath[] = "landmark.json";
constexpr char kCalibrationConfigPath[] = "calibration_map.json";
constexpr char kGlobalMapPath[] = "global_map.json";
constexpr char kNavigationConfigPath[] = "navigation_config.json";
constexpr char kActionConfigPath[] = "action_config.json";
constexpr char kAvoidConfigPath[] = "avoid_config.json";
constexpr char kEmbeddedConfigPath[] = "embedded_config.json";
constexpr char kDeviceTablePath[] = "device_table.json";
constexpr char kAvoidAreaPath[] = "avoid_area.json";
constexpr char kSingleTaskPath[] = "single_task.json";
constexpr char kVisualConfigPath[] = "visual_config.json";
constexpr char kStorageConfigPath[] = "storage_config.json";
constexpr char kTfConfigPath[] = "tf_config.json";

constexpr char kTestConfigPath[] = "test.json";
constexpr char kTestQrCalibratePath[] = "landmark_revise.json";

constexpr char kUpdateLogicConfigService[] = "update_logic_config";
constexpr char kUpdateLocalizerConfigService[] = "update_localizer_config";
constexpr char kUpdateStorageConfigService[] = "update_storage_config";
constexpr char kUpdateNavigationConfigService[] = "update_navigation_config";
constexpr char kUpdateVisualConfigService[] = "update_visual_config";
constexpr char kUpdateActionConfigService[] = "update_action_config";
constexpr char kUpdateAvoidConfigService[] = "update_avoid_config";
constexpr char kUpdateAvoidAreaConfigService[] = "update_avoid_area_config";
constexpr char kUpdateEmbeddedConfigService[] = "update_embedded_config";
constexpr char kUpdateSingleTaskService[] = "update_single_task";
constexpr char kCalibrateVisualAngleService[] = "calibrate_visual_angle";
constexpr char kCalibrateVisualBackgroundService[] =
    "calibrate_visual_background";

}  // namespace amr_config

// 读取 json 文件 加载配置
class BasicConfigHelper {
 public:
  ~BasicConfigHelper() {}

  // 加载 json 文件获取配置
  bool LoadConfig();

  // 动态更新后保存到本地
  bool SaveLocal(amr_config::ConfigType type, std::string);

  // 调度读取配置
  std::string GetConfig(amr_config::ConfigType type);

  inline const AgvType agv_type() { return static_cast<AgvType>(agv_type_); }

  inline const std::string server_ip() { return server_ip_; }

  inline const int server_port() { return server_port_; }

  inline const int local_port() { return local_port_; }

  inline const int communicate_method() { return communicate_method_; }

  inline const std::string project_topic() { return project_topic_; }

  inline const LandmarkMapOption landmark_map_option() {
    return landmark_map_option_;
  }

  inline const SlamMapOption slam_map_option() { return slam_map_option_; }

  inline const MechanicalOption mechanical_option() {
    return mechanical_option_;
  }

  inline const SensorTFData sensor_tf_data() { return sensor_tf_data_; }

  inline const std::string logic_config_file_name() {
    return logic_config_file_name_;
  }

  inline const std::string navigation_config_file_name() {
    return navigation_config_file_name_;
  }

  inline const std::string action_config_file_name() {
    return action_config_file_name_;
  }

  inline const std::string embedded_config_file_name() {
    return embedded_config_file_name_;
  }

  std::string GetUserName() {
    uid_t userid;
    struct passwd* pwd;
    userid = getuid();
    pwd = getpwuid(userid);
    return pwd->pw_name;
  }

  std::string GetConfigPath() { return user_path_; }
  // 获取指定类型文件路径接口
  std::string GetTargetConfigPath(const amr_config::ConfigType& type) {
    try {
      return config_map_.at(type);
    } catch (std::out_of_range& ex) {
      LOG_ERROR_STREAM(ex.what());
    }
  }

  std::string GetBasicConfigPath() { return user_path_ + "basic_config/"; }
  std::string GetSlamConfigPath() { return user_path_ + "slam_config/"; }
  std::string GetMapPath() { return user_path_ + "map/"; }

 private:
  // 单例实现
  DECLARE_SINGLETON(BasicConfigHelper);
  BasicConfigHelper()
      : has_been_loaded_(false),
        agv_type_(0),
        server_port_(0),
        current_map_id_(0),
        current_section_id_(0),
        http_landmark_map_port_(0) {
    config_map_[amr_config::ConfigType::AGV_BASIC_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kAgvBasicConfigPath);
    config_map_[amr_config::ConfigType::LOCALIZER_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kLocalizerConfigPath);
    config_map_[amr_config::ConfigType::LOCALIZER_LANDMARK] =
        GetMapPath() + std::string(amr_config::kLocalizerLandmarkConfigPath);
    config_map_[amr_config::ConfigType::GLOBAL_MAP] =
        GetMapPath() + std::string(amr_config::kGlobalMapPath);
    config_map_[amr_config::ConfigType::LAST_ROBOT_POSE] =
        GetBasicConfigPath() +
        std::string(amr_config::kLastRobotPoseConfigPath);
    config_map_[amr_config::ConfigType::LOGIC_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kLogicConfigPath);
    config_map_[amr_config::ConfigType::NAVIGATION_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kNavigationConfigPath);
    config_map_[amr_config::ConfigType::ACTION_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kActionConfigPath);
    config_map_[amr_config::ConfigType::AVOID_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kAvoidConfigPath);
    config_map_[amr_config::ConfigType::EMBEDDED_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kEmbeddedConfigPath);
    config_map_[amr_config::ConfigType::DEVICE_TABLE] =
        GetBasicConfigPath() + std::string(amr_config::kDeviceTablePath);
    config_map_[amr_config::ConfigType::AVOID_AREA] =
        GetBasicConfigPath() + std::string(amr_config::kAvoidAreaPath);
    config_map_[amr_config::ConfigType::SINGLE_TASK] =
        GetBasicConfigPath() + std::string(amr_config::kSingleTaskPath);
    config_map_[amr_config::ConfigType::STORAGE_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kStorageConfigPath);
    config_map_[amr_config::ConfigType::TF_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kTfConfigPath);

    config_map_[amr_config::ConfigType::TEST_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kTestConfigPath);
    config_map_[amr_config::ConfigType::VISUAL_CONFIG] =
        GetBasicConfigPath() + std::string(amr_config::kVisualConfigPath);
    config_map_[amr_config::ConfigType::TEST_QR_CALIBRATE] =
        GetBasicConfigPath() + std::string(amr_config::kTestQrCalibratePath);

    config_map_[amr_config::ConfigType::QR_CALIBRATION_MAP] =
        GetMapPath() + std::string(amr_config::kCalibrationConfigPath);

    LoadConfig();
  }

  bool has_been_loaded_;

  int agv_type_;
  std::string server_ip_;
  int server_port_;
  int local_port_;
  int communicate_method_;
  std::string project_topic_;
  int current_map_id_;
  int current_section_id_;
  std::string http_get_landmark_map_url_;
  std::string http_update_landmark_map_url_;
  int http_landmark_map_port_;
  LandmarkMapOption landmark_map_option_;
  SlamMapOption slam_map_option_;
  MechanicalOption mechanical_option_;
  std::string logic_config_file_name_;
  std::string navigation_config_file_name_;
  std::string action_config_file_name_;
  std::string embedded_config_file_name_;

  SensorTFData sensor_tf_data_;
  std::map<amr_config::ConfigType, std::string> config_map_;

  // config文件 所在目录的前缀
  std::string user_path_ = "/home/" + GetUserName() + "/config/";
};

#endif  // amr_COMMON_INCLUDE_amr_COMMON_amr_CONFIG_HELPER_H_

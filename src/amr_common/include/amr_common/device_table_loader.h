/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_DEVICE_TABLE_LOADER_H_
#define amr_COMMON_INCLUDE_amr_COMMON_DEVICE_TABLE_LOADER_H_
#include <ros/package.h>

#include <map>
#include <string>
#include <vector>

#include "amr_common/amr_enum_type.h"
#include "amr_common/log_porting.h"
#include "amr_common/util/json11.h"
#include "amr_common/util/singleton.h"

enum Cantype {
  CAN_NONE = 0,
  CAN_ZLG = 1,
  CAN_EMUC = 2,
  CAN_LIKE = 3,
};

struct ChildDeviceParam {
  int node_id;
  int channel;
  int mode;
};

struct CanParam {
  int type;
  int channel;
  int can0_baudrate;
  int can1_baudrate;
};

struct SerialParam {
  std::string port;
  int baudrate;
};

typedef std::map<int, SerialParam> SerialParamMap;

// <child name, device param>
using DeviceConfig = std::map<std::string, ChildDeviceParam>;

// 读取 设备表 文件 加载设备配置
class DeviceTableLoader {
 public:
  ~DeviceTableLoader() {}

  // 加载 json 文件获取配置
  bool LoadConfig(const std::string& json_str);

  inline const DeviceConfig& GetConfig(std::string device_name) {
    return device_config_map_.at(device_name);
  }

  inline const CanParam GetCanParam() { return can_param_; }

  inline const SerialParamMap GetSerialParamMap() { return serial_map_param_; }

  inline const std::map<std::string, DeviceConfig>& GetDeviceList() {
    return device_config_map_;
  }

  inline const std::vector<std::string>& GetDiagnosticDeviceList() {
    return device_diagnostic_list_;
  }

  inline const std::map<std::string, unsigned char> GetIoMapping(
      std::string io_board) {
    std::map<std::string, unsigned char> io_map;
    try {
      io_map = io_config_.at(io_board);
    } catch (std::out_of_range& err) {
      LOG_ERROR("invalid io port");
      LOG_ERROR_STREAM(err.what()
                       << " file: " << __FILE__ << " line: " << __LINE__);
      return {};
    }
    return io_map;
  }

 private:
  // 单例实现
  DECLARE_SINGLETON(DeviceTableLoader);
  DeviceTableLoader() {}

  // <parent name, install device>
  std::map<std::string, DeviceConfig> device_config_map_;
  CanParam can_param_;
  SerialParamMap serial_map_param_;
  std::vector<std::string> device_diagnostic_list_;
  std::map<std::string, std::map<std::string, unsigned char>> io_config_;
};

#endif  // amr_COMMON_INCLUDE_amr_COMMON_DEVICE_TABLE_LOADER_H_

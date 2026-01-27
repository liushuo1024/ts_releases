/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#include "amr_common/device_table_loader.h"

bool DeviceTableLoader::LoadConfig(const std::string &json_str) {
  // 检测文件格式是否错误
  std::string err;
  const auto &j_config = json11::Json::parse(json_str, err);
  if (!err.empty()) {
    LOG_WARN("json file data formats is error.");
    return false;
  }

  LOG_WARN_COND(j_config["device_list"].array_items().empty(),
                "device list is empty !");

  LOG_DEBUG_STREAM("embedded device list contains "
                   << j_config["device_list"].array_items().size()
                   << " deivce(s).");

  try {
    // can配置
    can_param_.type = j_config["can_type"].int_value();
    auto &obj = j_config["can_channel"];
    if(obj.is_number()) {
      can_param_.channel = obj.int_value();
    } else {
      can_param_.channel = 3;
    }
    auto &&can_param = j_config["channel_baudrate"].object_items();
    auto itcb = can_param.cbegin();
    can_param_.can0_baudrate = itcb->second.int_value();
    LOG_DEBUG_STREAM(itcb->first << ":" << itcb->second.int_value());
    can_param_.can1_baudrate = (++itcb)->second.int_value();
    LOG_DEBUG_STREAM(itcb->first << ":" << itcb->second.int_value());

    // serial配置
    auto &&serials = j_config["serial"].array_items();
    for (const auto &serial_cfg : serials) {
      SerialParam serial;
      serial.port = serial_cfg["port"].string_value();
      serial.baudrate = serial_cfg["baudrate"].int_value();
      serial_map_param_[serial_cfg["channel"].int_value()] = serial;
      LOG_INFO_STREAM("[serial]:  channel= "
                      << serial_cfg["channel"].int_value()
                      << " port = " << serial_cfg["port"].string_value()
                      << " baudrate = " << serial_cfg["baudrate"].int_value());
    }

    // 放入 IO 映射表
    LOG_DEBUG("-------------- io config --------------");
    auto &&j_io_configs = j_config["io_config"].array_items();
    for (const auto &j_io_config : j_io_configs) {
      std::string board_info = j_io_config["io_board_info"].string_value();
      LOG_DEBUG_STREAM(board_info << ": ");
      auto &j_io_map = j_io_config["io_map"].object_items();
      auto itr = j_io_map.cbegin();
      std::map<std::string, unsigned char> io_map;
      while (itr != j_io_map.cend()) {
        io_map[itr->first] = itr->second.int_value();
        LOG_DEBUG_STREAM(itr->first << ": " << itr->second.int_value());
        ++itr;
      }
      io_config_[board_info] = io_map;
    }

    LOG_DEBUG("-------------- device list --------------");
    auto &&deivce_list = j_config["device_list"].array_items();
    for (const auto &iter_dev : deivce_list) {
      auto &&install_device = iter_dev["install_device"].array_items();
      DeviceConfig ins_dev;
      for (const auto &iter_ins_dev : install_device) {
        ChildDeviceParam param;
        param.node_id = iter_ins_dev["device_param"]
                            .object_items()
                            .at("node_id")
                            .int_value();
        param.channel = iter_ins_dev["device_param"]
                            .object_items()
                            .at("channel")
                            .int_value();
        try {
          param.mode = iter_ins_dev["device_param"]
                           .object_items()
                           .at("mode")
                           .int_value();
        } catch (std::out_of_range &ex) {
          param.mode = 0;
        }
        ins_dev.emplace(iter_ins_dev["child_name"].string_value(), param);
        device_diagnostic_list_.emplace_back(
            iter_ins_dev["child_name"].string_value());
        LOG_DEBUG_STREAM(iter_ins_dev["child_name"].string_value());
        LOG_DEBUG_STREAM("node_id: " << param.node_id);
        LOG_DEBUG_STREAM("channel: " << param.channel);
      }
      device_config_map_.emplace(iter_dev["name"].string_value(), ins_dev);
      LOG_DEBUG_STREAM(iter_dev["name"].string_value());
    }
  } catch (const std::exception &e) {
    LOG_WARN_STREAM(e.what());
    return false;
  }

  return true;
}

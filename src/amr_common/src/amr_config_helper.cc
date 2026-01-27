/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#include <amr_common/amr_config_helper.h>

bool BasicConfigHelper::LoadConfig() {
  if (has_been_loaded_) {
    return true;
  }

  std::string config_json_str;
  std::string err;
  try {
    config_json_str = util::LocalService::GetStringFromFile(
        config_map_.at(amr_config::ConfigType::LOCALIZER_CONFIG));
  } catch (std::out_of_range &err) {
    LOG_WARN("invalid config file");
    LOG_WARN_STREAM(err.what()
                    << " file: " << __FILE__ << " line: " << __LINE__);
    return false;
  }
  const auto &j_config_localizer = json11::Json::parse(config_json_str, err);
  // 检测文件格式是否错误
  if (!err.empty()) {
    LOG_WARN("localizer_config.json file data formats is error.");
    LOG_ERROR_STREAM("config error info: " << err);
    return false;
  }
  try {
    LOG_INFO("---------------load localizer config---------------");
    auto &&qroption = j_config_localizer["qroption"]["type"].object_items();
    mechanical_option_.qr_camera_info.up_camera_type =
        qroption.at("up_type").number_value();
    LOG_INFO_STREAM(
        "up_camera_type: " << mechanical_option_.qr_camera_info.up_camera_type);
    mechanical_option_.qr_camera_info.down_camera_type =
        qroption.at("down_type").number_value();
    LOG_INFO_STREAM("down_camera_type: "
                    << mechanical_option_.qr_camera_info.down_camera_type);
  } catch (const std::exception &e) {
    LOG_WARN_STREAM(e.what());
    LOG_ERROR_STREAM("load agv localizer config failed.");
    return false;
  }

  try {
    config_json_str = util::LocalService::GetStringFromFile(
        config_map_.at(amr_config::ConfigType::TF_CONFIG));
  } catch (std::out_of_range &err) {
    LOG_WARN("invalid config file");
    LOG_WARN_STREAM(err.what()
                    << " file: " << __FILE__ << " line: " << __LINE__);
    return false;
  }
  // 检测文件格式是否错误
  const auto &j_tf_config = json11::Json::parse(config_json_str, err);
  if (!err.empty()) {
    LOG_WARN("tf json file data formats is error.");
    LOG_ERROR_STREAM("config error info: " << err);
    return false;
  }
  try {
    LOG_INFO("---------------load tf config---------------");
    auto &&tf = j_tf_config["sensor_tf"].array_items();
    for (auto &sensor : tf) {
      SensorTransform stf;
      stf.translation_x =
          sensor["tf_value"].object_items().at("x").number_value();
      stf.translation_y =
          sensor["tf_value"].object_items().at("y").number_value();
      stf.rotation =
          sensor["tf_value"].object_items().at("theta").number_value();
      sensor_tf_data_.emplace(sensor["frame_id"].string_value(), stf);
      //   LOG_INFO_STREAM(sensor["frame_id"].string_value());
      //   LOG_INFO_STREAM("x: " << stf.translation_x << ", y:" <<
      //   stf.translation_y
      //                          << ", theta: " << stf.rotation);
    }
  } catch (const std::out_of_range &e) {
    LOG_WARN_STREAM(e.what());
    LOG_ERROR_STREAM("load agv tf config failed.");
    return false;
  }

  try {
    config_json_str = util::LocalService::GetStringFromFile(
        config_map_.at(amr_config::ConfigType::AGV_BASIC_CONFIG));
  } catch (std::out_of_range &err) {
    LOG_WARN("invalid config file");
    LOG_WARN_STREAM(err.what()
                    << " file: " << __FILE__ << " line: " << __LINE__);
    return false;
  }
  // 检测文件格式是否错误
  const auto &j_config = json11::Json::parse(config_json_str, err);
  if (!err.empty()) {
    LOG_WARN("agv basic config json file data formats is error.");
    LOG_ERROR_STREAM("config error info: " << err);
    return false;
  }
  try {
    LOG_INFO("---------------load agv config---------------");
    // 车型配置
    agv_type_ = j_config["agv_type"].int_value();
    LOG_INFO_STREAM("agv type: " << agv_type_);
    server_ip_ = j_config["server_ip"].string_value();
    LOG_INFO_STREAM("server_ip: " << server_ip_);
    server_port_ = j_config["server_port"].int_value();
    LOG_INFO_STREAM("server_port: " << server_port_);

    local_port_ = j_config["local_port"].int_value();
    LOG_INFO_STREAM("local_port: " << local_port_);

    if (j_config.has_shape({std::make_pair(std::string("communicate_method"),
                                           json11::Json::Type::NUMBER)},
                           err))
      communicate_method_ = j_config["communicate_method"].int_value();
    else
      communicate_method_ = 1;
    LOG_INFO_STREAM("communicate_method: " << communicate_method_);

    if (j_config.has_shape({std::make_pair(std::string("project_topic"),
                                           json11::Json::Type::STRING)},
                           err)) {
      project_topic_ = j_config["project_topic"].string_value();
      LOG_INFO_STREAM("project_topic: " << project_topic_);
    }

    auto &&landmark_map = j_config["landmark_map"].object_items();
    landmark_map_option_.http_get_landmark_map_url =
        landmark_map.at("http_get_landmark_map_url").string_value();
    LOG_INFO_STREAM("http_get_landmark_map_url: "
                    << landmark_map_option_.http_get_landmark_map_url);
    landmark_map_option_.http_update_landmark_map_url =
        landmark_map.at("http_update_landmark_map_url").string_value();
    LOG_INFO_STREAM("http_update_landmark_map_url: "
                    << landmark_map_option_.http_update_landmark_map_url);
    landmark_map_option_.http_landmark_map_port =
        landmark_map.at("http_landmark_map_port").int_value();
    LOG_INFO_STREAM("http_landmark_map_port: "
                    << landmark_map_option_.http_landmark_map_port);
    landmark_map_option_.current_reflector_map_id =
        landmark_map.at("current_reflector_map_id").int_value();
    LOG_INFO_STREAM("current_reflector_map_id: "
                    << landmark_map_option_.current_reflector_map_id);
    landmark_map_option_.current_reflector_section_id =
        landmark_map.at("current_reflector_section_id").int_value();
    LOG_INFO_STREAM("current_reflector_section_id: "
                    << landmark_map_option_.current_reflector_section_id);
    landmark_map_option_.current_qr_map_id =
        landmark_map.at("current_qr_map_id").int_value();
    LOG_INFO_STREAM(
        "current_qr_map_id: " << landmark_map_option_.current_qr_map_id);
    landmark_map_option_.current_qr_section_id =
        landmark_map.at("current_qr_section_id").int_value();
    LOG_INFO_STREAM("current_qr_section_id: "
                    << landmark_map_option_.current_qr_section_id);
    landmark_map_option_.current_rfid_map_id =
        landmark_map.at("current_rfid_map_id").int_value();
    LOG_INFO_STREAM(
        "current_rfid_map_id: " << landmark_map_option_.current_rfid_map_id);
    landmark_map_option_.current_rfid_section_id =
        landmark_map.at("current_rfid_section_id").int_value();
    LOG_INFO_STREAM("current_rfid_section_id: "
                    << landmark_map_option_.current_rfid_section_id);
    slam_map_option_.http_get_slam_pgm_url =
        landmark_map.at("http_get_slam_pgm_url").string_value();
    LOG_INFO_STREAM(
        "http_get_slam_pgm_url: " << slam_map_option_.http_get_slam_pgm_url);
    slam_map_option_.http_get_slam_yaml_url =
        landmark_map.at("http_get_slam_yaml_url").string_value();
    LOG_INFO_STREAM(
        "http_get_slam_yaml_url: " << slam_map_option_.http_get_slam_yaml_url);
    slam_map_option_.http_get_slam_pbstream_url =
        landmark_map.at("http_get_slam_pbstream_url").string_value();
    LOG_INFO_STREAM("http_get_slam_pbstream_url: "
                    << slam_map_option_.http_get_slam_pbstream_url);

    auto &&mechanical_param = j_config["mechanical_param"].object_items();
    mechanical_option_.model_type =
        mechanical_param.at("model_type").int_value();
    LOG_INFO_STREAM("model type: " << mechanical_option_.model_type);

    auto &&bicycle_model = mechanical_param.at("bicycle_model").object_items();
    mechanical_option_.bicycle_modle_option.wheel_base =
        bicycle_model.at("wheel_base").number_value();
    LOG_INFO_STREAM(
        "wheel_base: " << mechanical_option_.bicycle_modle_option.wheel_base);
    mechanical_option_.bicycle_modle_option.max_wheel_base =
        bicycle_model.at("max_wheel_base").number_value();
    LOG_INFO_STREAM("max_wheel_base: "
                    << mechanical_option_.bicycle_modle_option.max_wheel_base);
    mechanical_option_.bicycle_modle_option.min_wheel_base =
        bicycle_model.at("min_wheel_base").number_value();
    LOG_INFO_STREAM("min_wheel_base: "
                    << mechanical_option_.bicycle_modle_option.min_wheel_base);
    mechanical_option_.bicycle_modle_option.eccentric_distance =
        bicycle_model.at("eccentric_distance").number_value();
    LOG_INFO_STREAM(
        "eccentric_distance: "
        << mechanical_option_.bicycle_modle_option.eccentric_distance);

    mechanical_option_.bicycle_modle_option.eccentric_angle =
        bicycle_model.at("eccentric_angle").number_value();
    LOG_INFO_STREAM("eccentric_angle: "
                    << mechanical_option_.bicycle_modle_option.eccentric_angle);
    mechanical_option_.bicycle_modle_option.reduction_ratio =
        bicycle_model.at("reduction_ratio").number_value();
    LOG_INFO_STREAM("reduction_ratio: "
                    << mechanical_option_.bicycle_modle_option.reduction_ratio);

    auto &&unicycle_model =
        mechanical_param.at("unicycle_model").object_items();
    mechanical_option_.unicycle_model_option.wheel_track =
        unicycle_model.at("wheel_track").number_value();
    LOG_INFO_STREAM("wheel_track: "
                    << mechanical_option_.unicycle_model_option.wheel_track);
    mechanical_option_.unicycle_model_option.wheel_diameter =
        unicycle_model.at("wheel_diameter").number_value();
    LOG_INFO_STREAM("wheel_diameter: "
                    << mechanical_option_.unicycle_model_option.wheel_diameter);
    mechanical_option_.unicycle_model_option.move_motor_reduce_ratio =
        unicycle_model.at("move_motor_reduce_ratio").number_value();
    LOG_INFO_STREAM(
        "move_motor_reduce_ratio: "
        << mechanical_option_.unicycle_model_option.move_motor_reduce_ratio);
    mechanical_option_.unicycle_model_option.lift_motor_reduce_ratio =
        unicycle_model.at("lift_motor_reduce_ratio").number_value();
    LOG_INFO_STREAM(
        "lift_motor_reduce_ratio: "
        << mechanical_option_.unicycle_model_option.lift_motor_reduce_ratio);
    mechanical_option_.unicycle_model_option.lift_screw_lead =
        unicycle_model.at("lift_screw_lead").number_value();
    LOG_INFO_STREAM(
        "lift_screw_lead: "
        << mechanical_option_.unicycle_model_option.lift_screw_lead);
    mechanical_option_.unicycle_model_option.lift_max_height =
        unicycle_model.at("lift_max_height").number_value();
    LOG_INFO_STREAM(
        "lift_max_height: "
        << mechanical_option_.unicycle_model_option.lift_max_height);
    mechanical_option_.unicycle_model_option.rotate_motor_reduce_ratio =
        unicycle_model.at("rotate_motor_reduce_ratio").number_value();
    LOG_INFO_STREAM(
        "rotate_motor_reduce_ratio: "
        << mechanical_option_.unicycle_model_option.rotate_motor_reduce_ratio);

    mechanical_option_.unicycle_model_option.lift_mechanism =
        unicycle_model.at("lift_mechanism").number_value();
    LOG_INFO_STREAM("lift_mechanism: "
                    << mechanical_option_.unicycle_model_option.lift_mechanism);

    mechanical_option_.unicycle_model_option.use_lift =
        unicycle_model.at("use_lift").bool_value();
    LOG_INFO_STREAM(
        "use_lift: " << mechanical_option_.unicycle_model_option.use_lift);

    mechanical_option_.unicycle_model_option.lift_motor_mode =
        unicycle_model.at("lift_motor_mode").number_value();
    LOG_INFO_STREAM(
        "lift_motor_mode: "
        << mechanical_option_.unicycle_model_option.lift_motor_mode);

    mechanical_option_.unicycle_model_option.use_rotate =
        unicycle_model.at("use_rotate").bool_value();
    LOG_INFO_STREAM(
        "use_rotate: " << mechanical_option_.unicycle_model_option.use_rotate);

    mechanical_option_.unicycle_model_option.use_up_camera =
        unicycle_model.at("use_up_camera").bool_value();
    LOG_INFO_STREAM("use_up_camera: "
                    << mechanical_option_.unicycle_model_option.use_up_camera);

    mechanical_option_.unicycle_model_option.use_down_camera =
        unicycle_model.at("use_down_camera").bool_value();
    LOG_INFO_STREAM(
        "use_down_camera: "
        << mechanical_option_.unicycle_model_option.use_down_camera);

    mechanical_option_.unicycle_model_option.use_io_laser_avoid =
        unicycle_model.at("use_io_laser_avoid").bool_value();
    LOG_INFO_STREAM(
        "use_io_laser_avoid: "
        << mechanical_option_.unicycle_model_option.use_io_laser_avoid);

    // auto &&sensot_install =
    //     mechanical_param.at("sensor_install").object_items();
    // auto &&laser_sensor_install =
    //     sensot_install.at("laser_sensor_install").object_items();
    // mechanical_option_.sensor_install_option.laser_sensor_install_option
    //     .laser_offset_x =
    //     laser_sensor_install.at("laser_offset_x").number_value();
    // LOG_INFO_STREAM(
    //     "laser_offset_x: " << mechanical_option_.sensor_install_option
    //                               .laser_sensor_install_option.laser_offset_x);
    // mechanical_option_.sensor_install_option.laser_sensor_install_option
    //     .laser_offset_y =
    //     laser_sensor_install.at("laser_offset_y").number_value();
    // LOG_INFO_STREAM(
    //     "laser_offset_y: " << mechanical_option_.sensor_install_option
    //                               .laser_sensor_install_option.laser_offset_y);
    // mechanical_option_.sensor_install_option.laser_sensor_install_option
    //     .laser_offset_theta =
    //     laser_sensor_install.at("laser_offset_theta").number_value();
    // LOG_INFO_STREAM("laser_offset_theta: "
    //                 << mechanical_option_.sensor_install_option
    //                        .laser_sensor_install_option.laser_offset_theta);
  } catch (const std::exception &e) {
    LOG_WARN_STREAM(e.what());
    LOG_ERROR_STREAM("load agv basic config failed.");
    return false;
  }

  has_been_loaded_ = true;
  return true;
}

bool BasicConfigHelper::SaveLocal(amr_config::ConfigType type,
                                  std::string data) {
  return util::LocalService::SaveStringToFile(config_map_.at(type), data);
}

std::string BasicConfigHelper::GetConfig(amr_config::ConfigType type) {
  std::string config_json_str;
  config_json_str = util::LocalService::GetStringFromFile(config_map_.at(type));
  std::string file_name = config_map_.at(type);
  LOG_INFO_STREAM("GetConfig path:" << file_name);
  if (config_json_str.empty()) {
    LOG_ERROR_STREAM(file_name << " is empty.");
    return "";
  }
  return config_json_str;
}

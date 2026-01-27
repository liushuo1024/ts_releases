/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_amr_TOPIC_NAME_H_
#define amr_COMMON_INCLUDE_amr_COMMON_amr_TOPIC_NAME_H_
#include <amr_msgs/node_diagnostic.h>

#include "amr_msgs/adjust_up.h"

constexpr uint32_t kTopicSendCacheSize = 10;
constexpr uint32_t kTopicReciveCacheSize = 2;

// actionlib
constexpr char kActionClientName[] = "actionserver";
constexpr char kTrackPathClientName[] = "track_path";
constexpr char kOpenloopClientName[] = "openloop";

// TODO(@someone) 以后将所有服务名常量移步于此
// service
namespace amr_services {

constexpr char kStartCartoWithPureLocalizerService[] =
    "start_carto_with_pure_localizer_srv";
constexpr char kStartCartoWithMappingService[] = "start_carto_with_mapping_srv";
constexpr char kSaveCartoMapService[] = "save_carto_map_srv";
constexpr char kUpdateCartoMapService[] = "update_carto_map_srv";
constexpr char kFinishCartoService[] = "finish_carto_srv";
constexpr char kFinishTrajectory[] = "finish_trajectory_srv";
constexpr char kResetHeight[] = "reset_height";
constexpr char kResetLateral[] = "reset_lateral";
constexpr char kResetTda04Weight[] = "reset_tda04_weight";
constexpr char kResetWeight[] = "reset_weight";

constexpr char kAgvSendDispatchService[] = "agv_send_dispatch_srv";
constexpr char kAgvQueryDispatchService[] = "agv_query_dispatch_srv";

}  // namespace amr_services

// TODO(@someone) 以后将所有话题名常量移步于此
namespace amr_topic {
// ros标准
constexpr char kOdomTopic[] = "odom";
constexpr char kImuTopic[] = "imu";
constexpr char kCombinedOdomopic[] = "combined_odom";

// 节点监控相关
constexpr char kCommunicateDiagnosticTopic[] = "communicate_node_diagnostic";
constexpr char kEmbeddedDiagnosticTopic[] = "embedded_node_diagnostic";
constexpr char kLocalizerDiagnosticTopic[] = "localizer_node_diagnostic";
constexpr char kReflectorDiagnosticTopic[] = "reflector_node_diagnostic";
constexpr char kCalibrationDiagnosticTopic[] = "calibration_node_diagnostic";
constexpr char kNavigationDiagnosticTopic[] = "navigation_node_diagnostic";
constexpr char kDecisionDiagnosticTopic[] = "decision_node_diagnostic";
constexpr char kActionDiagnosticTopic[] = "action_node_diagnostic";
constexpr char kAvoidDiagnosticTopic[] = "avoid_node_diagnostic";
constexpr char kStorageDiagnosticTopic[] = "storage_node_diagnostic";
constexpr char kVisaulDiagnosticTopic[] = "visual_node_diagnostic";
constexpr char kFaultReportTopic[] = "fault_report";

// 移动相关:
constexpr char kMoveCmdTopic[] = "move_cmd";
constexpr char kMoveFeedbackTopic[] = "move_feedback";

// 安全避障相关
constexpr char kSafetySettingTopic[] = "safety_setting";
constexpr char kSafetyStateTopic[] = "safety_state";
constexpr char kSafetyIoStateTopic[] = "safety_io_state";
constexpr char kStopProtectTopic[] = "stop_protect";
constexpr char kFrontCameraDistTopic[] = "front_camera_distance";
constexpr char kFrontLeftCameraDistTopic[] = "front_left_camera_distance";
constexpr char kFrontRightCameraDistTopic[] = "front_right_camera_distance";
constexpr char kPedestrianDetectTopic[] = "pedestrian_pose";
constexpr char kMultiPedestrianDetectTopic[] = "multi_pedestrian_pose";

// 动作相关
constexpr char kForkLiftActionTopic[] = "fork_lift_action";
constexpr char kJackUpActionTopic[] = "jack_up_action";
constexpr char kRollerActionTopic[] = "roller_action";

// 抽象IO状态
constexpr char kJackUpIoTopic[] = "jack_up_io_state";
constexpr char kRollerIoTopic[] = "roller_io_state";
constexpr char kPalletForkIoTopic[] = "pallet_fork_io_state";
constexpr char kChargeDoStateTopic[] = "charge_do_state";
constexpr char kBumpResetTopic[] = "bump_reset_state";

// 抽象数据
constexpr char kForkPalletStablizerTopic[] = "fork_pallet_stablizer";
constexpr char kRightForkPalletStablizerTopic[] = "right_fork_pallet_stablizer";
constexpr char kAprilTagPalletStablizerTopic[] = "april_tag_pallet_stablizer";
constexpr char kPalletCenterTopic[] = "pallet_center_feedback";
constexpr char kMeasureSizeTopic[] = "measure_size_feedback";
constexpr char kUnloadDetectTopic[] = "unload_detect_feedback";
constexpr char kPowerSupplyTopic[] = "power_supply";
constexpr char kManualTopic[] = "manual";
constexpr char kEmergencyTopic[] = "emergency";
constexpr char kBatteryTopic[] = "battery";
constexpr char kLoadStateTopic[] = "load_state";
constexpr char kLedControlTopic[] = "led_control";
constexpr char kAudioControlTopic[] = "audio_control";
constexpr char kUltarsonicTopic[] = "ultarsonic";
constexpr char kKs103Topic[] = "ks103";
constexpr char kHinsonForwardMlsNavidataTopic[] = "forward_navidata";
constexpr char kHinsonBackwardMlsNavidataTopic[] = "backward_navidata";
constexpr char kPalletActInfoTopic[] = "pallet_act_info";
constexpr char kPalletDetectStateTopic[] = "pallet_detect_state";
constexpr char kDongleAliveStateTopic[] = "dongle_alive_state";
constexpr char kWeightTopic[] = "weight";

constexpr char kClampStateTopic[] = "clamp_state";

constexpr char kIODongleTopic[] = "io_dongle";
constexpr char kDongleToDispatchTopic[] = "dongle_to_diapatch";

// 视觉检测叉腿后方超板
constexpr char kPalletBackLimitFeedbackTopic[] = "pallet_back_limit_feedback";
constexpr char kRequestPalletBackLimitTopic[] = "request_pallet_back_limit";

constexpr char kWeightDropCheckTopic[] = "weighting_loaded";
constexpr char kWit61MotionTopic[] = "wit61_motion";
constexpr char kLivoxLidarTopic[] = "/livox/lidar";
constexpr char kLivoxDataTopic[] = "livox_data";

// 任务相关
constexpr char kUpdateEventTopic[] = "update_event";
constexpr char kTaskResponseTopic[] = "task_response";
constexpr char kTaskFinishRequestTopic[] = "task_finish_request";
constexpr char kTaskRequestTopic[] = "task_request";
constexpr char kTaskFinishResponseTopic[] = "task_finish_response";
constexpr char kTaskControlRequestTopic[] = "task_control_request";
constexpr char kTaskCalibrationRequestTopic[] = "task_calibration_request";
constexpr char kTaskGetMapRequestTopic[] = "task_get_map_request";

// 调度控制相关
constexpr char kTaskControlAudioRequestTopic[] = "task_control_audio_request";
constexpr char kTaskMotorParaRequestTopic[] = "task_motor_para_request";

constexpr char kTaskTTSRequestTopic[] = "task_tts_request";

constexpr char kBatteryInfoRequestTopic[] = "battery_info_request";
constexpr char kAgvInfoRequestTopic[] = "agv_info_request";
constexpr char kBatteryInfoResponseTopic[] = "battery_info_response";
constexpr char kAgvInfoResponseTopic[] = "agv_info_response";

// 定位相关
constexpr char kFinishRelocationRequestTopic[] = "finish_relocation_request";
constexpr char kSlamPoseDeviationTopic[] = "slam_pose_deviation";
constexpr char kReLocationTopic[] = "re_location";

// 放货检测请求
constexpr char kRequestUnloadDetectTopic[] = "request_unload_detect";

// 整定相关
constexpr char kRequestAprilTagTopic[] = "request_april_tag";

constexpr char kRequestPalletCenterTopic[] = "request_pallet_center";

constexpr char kRequestMeasureSizeTopic[] = "request_measure_size";

// 库位相关
constexpr char kStorageInfoTopic[] = "storage_info";

// 调试相关
constexpr char kTeleopModeTopic[] = "teleop_mode";
constexpr char kDynamicAddReflectorTopic[] = "dynamic_add_reflector";
constexpr char kMatchMarkNumTopic[] = "match_mark_num";
constexpr char kPoseDeviationTopic[] = "pose_deviation";
constexpr char kLaserScanTopic[] = "scan";
constexpr char kAvoidLaser0Topic[] = "avoid_laser_0";
constexpr char kAvoidLaser1Topic[] = "avoid_laser_1";
constexpr char kAvoidLaser2Topic[] = "avoid_laser_2";
constexpr char kAvoidLaser3Topic[] = "avoid_laser_3";
constexpr char kAvoidLaser4Topic[] = "avoid_laser_4";
constexpr char kAvoidLaser5Topic[] = "avoid_laser_5";
constexpr char kQrCalibration[] = "qr_calibration";
constexpr char kActionPalletNomoveTopic[] = "pallet_nomove_info";
constexpr char kAdjustUpTopic[] = "adjust_up";

constexpr char kSyncInitPose[] = "update_init_pose";
constexpr char kReflectorDisplay[] = "reflector_display";
constexpr char kAllReflectorDisplay[] = "allreflector_display";
}  // namespace amr_topic

// 设备相关(TODO)
namespace device_name_list {

constexpr char* kLinDeKobDriverName = const_cast<char*>("linde_kob");
constexpr char* kValveDriverName = const_cast<char*>("valve");
constexpr char* kCurtisDriverName = const_cast<char*>("curtis");
constexpr char* kReachForkliftCurtisDriverName =
    const_cast<char*>("reach_forklift_curtis");
constexpr char* kAudioDriverName = const_cast<char*>("audio");
constexpr char* kIODriverName = const_cast<char*>("io");
constexpr char* kGpioDriverName = const_cast<char*>("gpio");
constexpr char* kKs104DriverName = const_cast<char*>("ks104");
constexpr char* kWeightTda04DriverName = const_cast<char*>("tda04_driver");
constexpr char* kHKweightDriveName = const_cast<char*>("hk_weight_driver");
constexpr char* kDongleDriverName = const_cast<char*>("dongle");
constexpr char* kSingleLineLaserDriverName =
    const_cast<char*>("single_line_laser");
constexpr char* kPgvR2100DriverName = const_cast<char*>("pgv_r2100");
constexpr char* kGyroRionDriverName = const_cast<char*>("gyro_rion");
constexpr char* kSerialDriverName = const_cast<char*>("serial");
constexpr char* kGyroMemsplusDriverName = const_cast<char*>("gyro_memsplus");
constexpr char* kSickMLSDriverName = const_cast<char*>("sick_mls_sensor");
constexpr char* kLowPowerDriverName = const_cast<char*>("low_power");
constexpr char* kRfidDriverName = const_cast<char*>("rfid");
constexpr char* kChargingDriverName = const_cast<char*>("charging");
constexpr char* kBatteryRebotDriverName =
    const_cast<char*>("battery_rebot_monitor");
constexpr char* kBatteryZLDriverName = const_cast<char*>("battery_zl_monitor");
constexpr char* kBatteryYFDriverName = const_cast<char*>("battery_yf_monitor");
constexpr char* kBatteryYF2DriverName =
    const_cast<char*>("battery_yf_2_monitor");
constexpr char* kBatteryMMDriverName = const_cast<char*>("battery_mm_monitor");
constexpr char* kBatteryLindeDriverName =
    const_cast<char*>("battery_linde_monitor");
constexpr char* kBatteryBaoeDriverName =
    const_cast<char*>("battery_baoe_monitor");
constexpr char* kBatteryLKDriverName =
    const_cast<char*>("battery_lonking_monitor");
constexpr char* kSyntronMotorDriverName = const_cast<char*>("syntron_motor");
constexpr char* kMotecMotorDriverName = const_cast<char*>("motec_motor");
constexpr char* kPGV100DriverName = const_cast<char*>("pgv100");
constexpr char* kLicheEncoderDriverName = const_cast<char*>("liche_encoder");
constexpr char* kBmmsk34EncoderDriverName =
    const_cast<char*>("bmmsk34_encoder");
constexpr char* kWit905DriveName = const_cast<char*>("wit905_driver");
constexpr char* kUltarsonicDriverName = const_cast<char*>("ultarsonic");
constexpr char* kHikvsDriverName = const_cast<char*>("hikvs");
constexpr char* kHinsonMlsDriverName = const_cast<char*>("hinson_mls");
constexpr char* kKS103DriverName = const_cast<char*>("ks103");
constexpr char* kWeighingDriverName = const_cast<char*>("weighing");
constexpr char* KJoystickDriverName = const_cast<char*>("joy_stick");
constexpr char* KIoblueoneDriverName = const_cast<char*>("io_blueone");

}  // namespace device_name_list

namespace install_device_name_list {
constexpr char* kWit905DriveName = const_cast<char*>("wit905_driver");
constexpr char* kLinDeKobInstallDriverName = const_cast<char*>("linde_kob");
constexpr char* kValveInstallDriverName = const_cast<char*>("valve");
constexpr char* kCurtisInstallDriverName =
    const_cast<char*>("curtis_1220c_1232e");
constexpr char* kReachForkliftCurtisInstallDriverName =
    const_cast<char*>("curtis_1236e_1222");
constexpr char* kAudioInstallDriverName = const_cast<char*>("audio");
constexpr char* kIOInstallDriverName = const_cast<char*>("io");
constexpr char* kIOObstacleInstallDriverName = const_cast<char*>("io_obstacle");
constexpr char* kIODongleInstallDriverName = const_cast<char*>("io_dongle");
constexpr char* kIoRemoteControlInstallDriverName =
    const_cast<char*>("io_remote_control");
constexpr char* kIoSigleOutInstallDriverName =
    const_cast<char*>("io_single_out");
constexpr char* kIo2InstallDriverName = const_cast<char*>("io2");
constexpr char* kGpioInstallDriverName = const_cast<char*>("gpio");
constexpr char* kLeftSingleLineLaserInstallDriverName =
    const_cast<char*>("left_single_line_laser");
constexpr char* kRightSingleLineLaserInstallDriverName =
    const_cast<char*>("right_single_line_laser");
constexpr char* kPgvR2100InstallDriverName = const_cast<char*>("pgv_r2100");
constexpr char* kGyroRionInstallDriverName = const_cast<char*>("gyro_rion");
constexpr char* kGyroMemsplusInstallDriverName =
    const_cast<char*>("gyro_memsplus");
constexpr char* kSickMLSInstallDriverName =
    const_cast<char*>("sick_mls_sensor");
constexpr char* kLowPowerInstallDriverName = const_cast<char*>("low_power");
constexpr char* kRfidInstallDriverName = const_cast<char*>("rfid");
constexpr char* kChargingInstallDriverName = const_cast<char*>("charging");
constexpr char* kPGV100InstallDriverName = const_cast<char*>("pgv100");
constexpr char* kRS232InstallDriverName = const_cast<char*>("rs232");
constexpr char* kRS232GyroRionInstallDriverName =
    const_cast<char*>("rs232_gyro_rion");
constexpr char* kRS485InstallDriverName = const_cast<char*>("rs485");
constexpr char* kRS485OLM100InstallDriverName =
    const_cast<char*>("rs485_olm100");
constexpr char* kRS485PGV100InstallDriverName =
    const_cast<char*>("rs485_pgv100");
constexpr char* kLeft1UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_1");
constexpr char* kLeft2UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_2");
constexpr char* kForward1UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_3");
constexpr char* kForward2UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_4");
constexpr char* kRight1UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_5");
constexpr char* kRight2UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_6");
constexpr char* kBack1UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_7");
constexpr char* kBack2UltrasonicInstallDriverName =
    const_cast<char*>("ultrasonic_8");
constexpr char* kWeightTda04InstallDriverName =
    const_cast<char*>("weight_tda04_driver");
constexpr char* kHKweightInstallDriverName =
    const_cast<char*>("hk_weight_driver");
constexpr char* kHightEncoderInstallDriverName =
    const_cast<char*>("height_encoder");
constexpr char* kLateralEncoderInstallDriverName =
    const_cast<char*>("lateral_encoder");
constexpr char* kVerticalEncoderInstallDriverName =
    const_cast<char*>("vertical_encoder");
constexpr char* kMotecLeftMotorInstallDriverName =
    const_cast<char*>("motec_left_move_motor");
constexpr char* kMotecRightMotorInstallDriverName =
    const_cast<char*>("motec_right_move_motor");
constexpr char* kSyntronLeftMotorInstallDriverName =
    const_cast<char*>("syntron_left_move_motor");
constexpr char* kSyntronRightMotorInstallDriverName =
    const_cast<char*>("syntron_right_move_motor");
constexpr char* kSyntronLiftMotorInstallDriverName =
    const_cast<char*>("syntron_lift_motor");
constexpr char* kSyntronRotateMotorInstallDriverName =
    const_cast<char*>("syntron_rotate_motor");
constexpr char* kSyntronClampLeftInstallDriverName =
    const_cast<char*>("syntron_clamp_left_motor");
constexpr char* kSyntronClampRightInstallDriverName =
    const_cast<char*>("syntron_clamp_right_motor");
constexpr char* kSingleLineLaserInstallDriverName =
    const_cast<char*>("single_line_laser");
constexpr char* kBatteryInstallDriverName =
    const_cast<char*>("battery_feedback");
constexpr char* kBatteryRebotInstallDriverName =
    const_cast<char*>("battery_rebot_monitor");
constexpr char* kBatteryZLInstallDriverName =
    const_cast<char*>("battery_zl_monitor");
constexpr char* kBatteryYFInstallDriverName =
    const_cast<char*>("battery_yf_monitor");
constexpr char* kBatteryYF2InstallDriverName =
    const_cast<char*>("battery_yf_2_monitor");
constexpr char* kBatteryMMInstallDriverName =
    const_cast<char*>("battery_mm_monitor");
constexpr char* kBatteryLindeInstallDriverName =
    const_cast<char*>("battery_linde_monitor");
constexpr char* kBatteryBaoeInstallDriverName =
    const_cast<char*>("battery_baoe_monitor");
constexpr char* kBatteryLKInstallDriverName =
    const_cast<char*>("battery_lonking_monitor");

constexpr char* kPGV100UpInstallDriverName = const_cast<char*>("pgv100_up");
constexpr char* kPGV100DownInstallDriverName = const_cast<char*>("pgv100_down");
constexpr char* kIOOUTInstallDriverName = const_cast<char*>("io_single_out");
constexpr char* kUltarsonicInstallDriverName = const_cast<char*>("ultarsonic");
constexpr char* kKS103InstallDriverName = const_cast<char*>("ks103");
constexpr char* kHikvsUpInstallDriverName = const_cast<char*>("hikvs_up");
constexpr char* kHinsonMlsForwardInstallDriverName =
    const_cast<char*>("hinson_mls_forward");
constexpr char* kHinsonMlsBackwardInstallDriverName =
    const_cast<char*>("hinson_mls_backward");
constexpr char* kRfidDownInstallDriverName =
    const_cast<char*>("hinson_rfid_down");
constexpr char* kWeigh1InstallDriverName = const_cast<char*>("weighing_1");
constexpr char* kWeigh2InstallDriverName = const_cast<char*>("weighing_2");
constexpr char* KJoystickDriverName = const_cast<char*>("joy_stick");
constexpr char* KIoblueoneInstallDriverName = const_cast<char*>("io_blueone");
constexpr char* kWit61MotionDriverName = const_cast<char*>("wit61_motion");
constexpr char* kHikvsQrDownDriverName =
    const_cast<char*>("hikvs_qr_feedback_down");
constexpr char* kHikvsQrUpDriverName =
    const_cast<char*>("hikvs_qr_feedback_up");
}  // namespace install_device_name_list

#endif  // amr_COMMON_INCLUDE_amr_COMMON_amr_TOPIC_NAME_H_

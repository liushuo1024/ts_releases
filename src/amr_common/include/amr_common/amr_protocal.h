/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_amr_PROTOCAL_H_
#define amr_COMMON_INCLUDE_amr_COMMON_amr_PROTOCAL_H_
#include <string>

#include "amr_common/util/json11.h"

namespace amr_protocal {
// 100K 缓存
constexpr uint32_t kMaxUdpBuffer = 102400;

// 协议总包
struct amrUdpProtocal {
  std::string bind_id;
  std::string msg_type;  // 数据包类型
  std::string msg_data;  // 数据包应用数据段
  std::string msg_hash;
};

enum class Qos : uint8_t {
  QOS0 = 0,  // 消息发送一次，不管多方有木有收到
  QOS1 = 1,  // 消息至少发送一次，确保对方收到
  QOS2 = 2   // 消息只发送一次，确保对方收到,底层开销较大
};

// 协议数据包类型
namespace msg_type {
/**
 * a.定时触发型
 */
constexpr char kUpdateEvent[] = "UpdateEvent";

constexpr char kDongleEvent[] = "DongleInfoEvent";

// 车载向调度发送里程信息包
constexpr char kOdomInfoEvent[] = "OdomInfoEvent";
// 车载向调度发送库位信息包
constexpr char kStorageInfoEvent[] = "StorageInfoEvent";
// 车载向调度发送货物信息包
constexpr char kLoadInfoEvent[] = "LoadInfoEvent";
// 车载向调度发送静止放货信息包
constexpr char kForbidStorageInfoEvent[] = "ForbidStorageInfoEvent";
// 车载 <-->调度交互地图信息包
constexpr char kMapInfoEvent[] = "MapInfoEvent";
// 车载 <-->调度交互库位地图信息包
constexpr char kStorageMapInfoEvent[] = "StorageMapInfoEvent";
// 车载向调度息请求
constexpr char kAgvQueryInfoRequest[] = "AgvQueryInfoRequest";

/**
 * b.请求应答型
 */
// 任务接收回复
constexpr char kTaskResponse[] = "TaskResponse";

// agv <--> ds  请求从 agv 发出
// 任务完成请求
constexpr char kTaskFinishRequest[] = "TaskFinishRequest";
// 重定位完成请求
constexpr char kFinishRelocationRequest[] = "FinishRelocationRequest";

// ds <--> agv  请求从 ds 发出
// 任务请求
constexpr char kTaskRequest[] = "TaskRequest";

// 任务控制请求
constexpr char kTaskControlRequest[] = "TaskControlRequest";

// 标定功能请求
constexpr char kTaskCalibrationRequest[] = "TaskCalibrationRequest";

// 车载重定位请求
constexpr char kTaskReLocationRequest[] = "TaskRelocationRequest";

// 车载地图上传请求
constexpr char kTaskGetMapRequest[] = "TaskGetMapRequest";

// 切换车载音频请求
constexpr char kTaskControlAudioRequest[] = "TaskControlAudioRequest";

// 调度查询库位信息请求
constexpr char kTaskQueryStorageRequest[] = "StorageInfoRequest";

// 参数配置包上传请求
constexpr char kTaskPushParamRequest[] = "TaskPushParamRequest";

// 基础相关参数配包
constexpr char kTaskUpdateBasicParam[] = "TaskUpdateBasicParam";

// 设备相关参数配包
constexpr char kTaskUpdateDeviceParam[] = "TaskUpdateDeviceParam";

// 导航相关参数配包
constexpr char kTaskUpdateNavigationParam[] = "TaskUpdateNavigationParam";

// 动作相关参数配置包
constexpr char kTaskUpdateActionParam[] = "TaskUpdateActionParam";

// 避障相关参数配置包
constexpr char kTaskUpdateAvoidParam[] = "TaskUpdateAvoidParam";

// 避障区域相关参数配置包
constexpr char kTaskUpdateAvoidAeraParam[] = "TaskUpdateAvoidAeraParam";

// 定位相关参数配置
constexpr char kTaskUpdateLocalizerParam[] = "TaskUpdateLocalizerParam";

// 业务相关参数配置包
constexpr char kTaskUpdateLogicParam[] = "TaskUpdateLogicParam";

constexpr char kTaskUpdateParamOk[] = "TaskUpdateParamOk";

// 调度向小车查询电池信息包
constexpr char kBatteryInfoRequest[] = "BatteryInfoRequest";

// 调度查询里程计信息请求
constexpr char kTaskQueryOdomRequest[] = "OdomInfoRequest";

// 车载向调度回复电池信息包
constexpr char kBatteryInfoResponse[] = "BatteryInfoResponse";

// 单机任务下发数据包
constexpr char kSingleTask[] = "SingleTask";

// 嵌入式驱动配置包
constexpr char kTaskUpdateEmbeddedParam[] = "TaskUpdateEmbeddedParam";
}  // namespace msg_type
}  // namespace amr_protocal

#endif  // amr_COMMON_INCLUDE_amr_COMMON_amr_PROTOCAL_H_

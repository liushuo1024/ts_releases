/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_amr_ENUM_TYPE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_amr_ENUM_TYPE_H_
#include <iostream>

#include "amr_common/log_porting.h"

#define BIT_0 0
#define BIT_1 1
#define NO_LANDMARK 0

enum class AgvType : uint32_t {
  UNKONW = 0,
  FORKLIFT = 1,
  JACK_UP = 2,
  TRANSPLANT = 3,
  HEAP_FORKLIFT = 4,
  LINDE_FROKLIFT = 5,
  NUOLI_FROKLIFT = 6,
  TRACTOR_CAR = 7,
  REACH_FORKLIFT = 8,
  MAGNETC_JACK_UP = 9,
  ZHONGLI_2T_JACK_UP = 10,
  XP1_FROKLIFT = 13,
  XINMEI_FORKLIFT = 14,
  QINGDAO_JACK_UP = 15,
  QR2T_JACK_UP = 16,
  XP2_FROKLIFT = 17,
  RPL_FROKLIFT = 18,
  FSY_JACK_UP = 22,
  ROLLER = 23,
  SCAN_JACK_UP = 25,
};

// 交互中的数据标准单位：m, m/s, A,V; 设备_feedback.msg除外 --》
// 通过业务转化为标准单位

// 调度任务相关业务枚举
namespace dispacth {
// 导航类型 1:激光导航 2:二维码导航 3:磁条导航
enum class NaviType : int32_t { NONE = 0, FREE = 1, QR_CODE = 2, MAGNETIC = 3 };

// 移动类型 1:严格跟踪路径, 2:移动至目标姿态，不关心中间过程, 3:自由移动 ,
// 4:移动至列(主要应用在移动至库区)
enum class MoveType : int32_t {
  NONE = 0,
  TRACK_PATH = 1,
  TRACK_TARGET = 2,
  TRACK_FREE = 3,
  TRACK_COLUMN = 4,
  TRACK_FORBID_UN_LOAD = 5
};

// Step类型，用于区分移动还是操作，并统一这两者数据结构。1:移动至目标点, 2:操作,
// 3:边移动边操作, 4:终止点, 5:起始点
enum class StepType : int32_t {
  NONE = 0,
  MOVE = 1,
  OPERATION = 2,
  MOVE_OPERATION = 3,
  END_POINT = 4,
  START_POINT = 5
};

// 路径类型 1:点, 2:直线, 3:曲线, 4:圆弧, 5:旋转
enum class PathType : int32_t {
  NONE = 0,
  POINT = 1,
  STRAIGHT_LINE = 2,
  B_SPLINE_CURVES = 3,
  ARC = 4,
  ROTATE = 5,
  LAT = 6,
  DETOUR = 7,
};

// 避障策略码
enum class AvoidMapType : int {
  INIT = 0,
  AVOID_DIY_MAP_23 = 23,
  AVOID_DIY_MAP_24 = 24,
  AVOID_DIY_MAP_25 = 25,  // 避障减弱
  NONE = 26,              // 无避障
  BACK_LEFT = 27,         // 后退右转
  BACK_RIGHT = 28,        // 后退左转
  FORWARD_LEFT = 29,      // 前进左转
  FORWARD_RIGHT = 30,     // 前进右转
  FORWARD = 31            // 直行避障
};

// agv任务动作类型
enum class AgvTaskOperationType : uint32_t {
  NONE = 0,
  REST = 1,
  UP = 2,    // 抬货  (value=0:直接抬, value=1:需要抬货检测)
  DOWN = 3,  // 放货  (value=0:直接放, value=1:需要放货检测)
  HEIGHT_FORK_MOVE = 4,  // 叉腿高度移动  (operation_value 填 控制高度)
  CHARGE = 5,            // 自动充电 (operation_value: 0=关, 1=开)
  SELECT_DIR = 6,  // 选择磁条方向 (operation_value: 0=正前 1=左 2=右)
  PALLET_ROTATE = 7,  // 托盘旋转　（operation_value：-180~180 旋转角度）
  LOW_POWER_MODE = 9,  // 进入低功耗模式 (operation_value 填 0(关), 1(开))
  LATERAL_FORK_MOVE = 10,  // 前移叉腿移动(operation_value 填 前移量)
  PALLET_DETECT_BEFORE_UP =
      11,  // 抬货前托盘检测 (operation_value 填 检测地图类型)
  PALLET_DETECT_BEFORE_DOWN =
      12,  // 抬货前托盘检测 (operation_value 填 检测地图类型)
  QUERY_STORAGE_INFO = 13,  // 库位信息查询 (operation_value 填 库区id)
  AUDIO_LEVEL_CONTROL = 22,  // 音频音量调节(operation_value 填 音量值0~30)
  AUDIO_TYPE_CONTROL = 23,  // 音频类型调节(operation_value 填 值0~12)
  PALLET_CENTER_BEFORE_UP = 25,  // 托盘矫正取货 (operation_value 填 0)
  FORK_TILT = 26,                // 倾斜
  FORK_SIDESHIFT = 27,           // 侧移
  FORK_FORWARD = 28,
  FORK_BACKWARD = 29,

  /* -------decision maker 复合动作解析后的子任务
  ex: 顶升的抬货包含UP与PALLET_NOMOVE等多个动作 ---------*/
  DELAY = 50,            // 延时动作(operation_value 填延迟时间/s)
  CHECK_UP_WEIGHT = 51,  // 抬货重量检测(operation_value 填 值货物重量)
  CHECK_DOWN_WEIGHT = 52,  // 卸货重量检测(operation_value 填 值货物重量)

  CLAMP_COMPRESS = 53,  // 夹抱装置夹紧(operation_value 填)
  CLAMP_RELEASE = 54,   //  夹抱装置释放(operation_value 填)

  OPEN_LOOP_FORK_UP = 55,    // 开环抬升叉腿
  OPEN_LOOP_FORK_DOWN = 56,  // 开环下降叉腿

  PALLET_FRONT_LIMIT = 57,  // 检测托盘前方是否超板
  PALLET_BACK_LIMIT = 58,   // 检测托盘后方是否超板

  MEASURE_SIZE = 60,  // 取卸货前尺寸校验

  INIT = 99,

  /* -------顶升车子任务(100~199) ---------*/
  PALLET_NOMOVE =
      100,  // 托盘动态调整姿态保持对地静止(value:0=无，1=直行,2=旋转)
  PALLET_ZERO = 101,  // 托盘归零
  /* 电机失能(value: 1=左，2=右，４=顶升，8=旋转　多电机累加)*/
  MOTOR_DISABLE = 102,
  /* 电机使能(value: 1=左，2=右，４=顶升，8=旋转　多电机累加) */
  MOTOR_ENABLE = 103,
  MOTOR_CLEARALARM = 104,  // 消除电机报警
  WAIT = 105,  // 动作等待(value:1=进入等待，0=退出恢复原来动作,其他:等待时间）
  PALLET_CORRECT = 106,  // 托盘整定后进行纠正

  /* -------非标子任务(501~999) ---------*/
  OPEN_DOOR = 501,  // 开门动作
};
}  // namespace dispacth

// 导航相关业务枚举
namespace navigation {
// 导航移动类型
enum class MotionType : uint32_t {
  NONE = 0,                           // 无移动动作
  START_POINT = 1,                    // 起始点移动
  LINE = 2,                           // 直线移动
  BCURVE = 3,                         // B样条曲线移动（平滑曲线路径）
  ARC = 4,                            // 圆弧移动
  FORKVAN = 5,                        // 叉车式移动（适用于叉车类设备）
  TRACK_COLUMN = 6,                   // 跟踪列移动（主要用于库区定位）
  BCURVE_STABLIZE = 7,                // B样条曲线整定移动（带稳定控制的曲线移动）
  STRAIGHT_STABLIZE = 8,              // 直线整定移动（带稳定控制的直线移动）
  TRACK_FORBID_UN_LOAD = 9,           // 禁止卸载跟踪移动（特殊场景下的移动控制）
  ROTATE = 10,                        // 旋转移动（原地转向）
  CHARGE = 11,                        // 充电移动（自动导航到充电桩）
  UPSTABILIZE = 12,                   // 上升整定移动（抬升过程中的稳定控制）
  DOWNSTABILIZE = 13,                 // 下降整定移动（下降过程中的稳定控制）
  FIND_2_RFID = 14,                   // 寻找两个RFID点移动（用于精确定位）
  CAR_BACK = 15,                      // 倒车移动
  DOCKING = 16,                       // 对接移动（与设备对接的精确移动）
  STABILIZE_DOWN_NOROTATE = 17,       // 下降整定不旋转移动（特殊下降控制）
  FORK_ROTATE = 18,                   // 货叉旋转移动（货叉本身的旋转动作）
  SWITCH_MAP = 20,                    // 地图切换移动（在不同地图区域间切换）
  POWERON_SHAKING = 21,               // 上电抖动移动（设备启动时的自检动作）
  JACKUP_STRAIGHT_STABLIZE = 22,      // 顶升车直线整定移动（顶升车的特殊移动）
  TRANSPLANT_ROTATE = 23,             // 移植车旋转移动（移植设备的特殊旋转）
  FORK_PALLET_STABLIZER = 30,         // 货叉托盘稳定器移动（托盘稳定控制）
  TRANSPLANT_ASTERN = 31,              // 移植车倒车移动（移植设备的倒车动作）
  LAT_LINE = 32,
  TEB_LINE = 33
};
}  // namespace navigation

// 动作相关业务枚举
namespace action {
// 原子动作 amr_action_node -> amr_embedded_node
enum class AtomicActionType : uint32_t {
  /* -------通用原子动作 (0-99) ---------*/
  REST = 0,
  NONE = 1,
  UP = 2,      // 抬升(高度)  (operation_value 填 速度值)  滚筒:进货
  DOWN = 3,    // 下降(高度)  (operation_value 填 速度值)  滚筒:出货
  CHARGE = 4,  // 充电 (operation_value 填0(关), 1(开))
  BEEP = 5,
  OPEN_DOOR = 6,  // 开门信号
  WAIT = 10,  // 状态暂存，动作暂停等待（operation_value 填0(关), 1(开)）

  LOW_POWER_MODE_24V = 98,  // 24V低功耗 (operation_value 填0(关), 1(开))
  LOW_POWER_MODE = 99,      // 48V低功耗 (operation_value 填0(关), 1(开))
  /* -------叉车类动作 (100 -199) ---------*/
  LATERAL_FORK_FORWARD = 101,  // 水平叉腿前移(operation_value 填 速度值)
  LATERAL_FORK_BACKWARD = 102,  // 水平叉腿后移(operation_value 填 速度值)
  LATERAL_FORK_LEFT = 103,  // 水平叉腿左移(operation_value 填 速度值)
  LATERAL_FORK_RIGHT = 104,  // 水平叉腿右移(operation_value 填 速度值)
  FORK_LEAN_FORWARD = 106,   // 叉腿前倾(operation_value 填 角度值)
  FORK_LEAN_BACKWARD = 107,  // 叉腿后倾(operation_value 填 角度值)

  PALLET_DETECT = 105,  // 库位托盘检测 (operation_value 填 1)

  STEERING_RESET = 111,  // 舵轮电机继电器打开  (operation_value 填0(关), 1(开))
  STEERING_BRAKE = 112,  // 舵轮电机电机报闸  (operation_value 填0(关), 1(开))
  // 测试类动作
  TEST_STEERING_ANGLE = 151,  //
  TEST_CONTROL = 152,

  /* -------顶升类动作 (200 -299) ---------*/
  PALLET_ROTATION = 201,
  PALLET_ZERO = 202,
  /* bit占位 value= 1/2/4/8：left/right/lift/rotate 电机，多个电机时叠加*/
  MOTOR_DISABLE = 203,
  MOTOR_ENABLE = 204,
  MOTOR_CLEARALARM = 205,
  PALLET_NOMOVE = 206,
  MAGNETIC_CHARGE = 207,

  LIFT_MOTOR_BREAK_CONTROL = 208,
  ROTATE_MOTOR_BREAK_CONTROL = 209,
  MOVE_MOTOR_BREAK_CONTROL = 210,

  MAGNETIC_SWITCH_DIR = 211,  // 磁条顶升切换方向

  // 夹抱机构电机（clamp_right_motor_value为左电机速度，clamp_right_motor_value为右电机速度）
  CLAMP_MOTOR_CONTROL = 220,

  // 堆高和前移归零
  HEIGHT_FORK_ZERO = 230,
  LATERAL_FORK_ZERO = 231,

  // 预前移
  PRE_LATERAL_FORK_FORWARD = 301  // 预先水平叉腿前移(operation_value 填 速度值)

};

}  // namespace action

namespace location {

enum class LocalizerType : int32_t {
  NONE = 0,
  REFLECTOR = 1,
  QR_CODE = 2,
  RFID = 3,
  CARTO = 4,
  LINEAR = 5,
  CARTO_MIX_REF = 6
};

enum class QrType : uint8_t { NONE = 0, HIKVS = 1, PGV = 2 };

}  // namespace location

// 状态机状态
enum class AgvStateType : int32_t {
  NONE = 0,
  WAITING = 1,
  DOING = 2,
  LOWPOWERMODE = 4,
  PAUSE = 5,
  MANUAL = 6,
  FINISHING = 7,
  CHARGING = 8,
  FAULT = 9,
  EMERGENCY_PAUSE = 10,
  ERROR = 11,
  EMERGENCY_STOP = 12,
  POWERON_SHAKING = 21
};

enum class BatteryType : uint8_t {
  NONE = 0,
  CURTIS = 1,
  BAO_E = 2,
  ZHONG_LI = 3,
  LIN_DE = 4,
  ZHI_LI = 5,
  REBOT = 6,
  LONKING = 7,
  YUFENG = 8
};

// 移动模型类型
enum class MoevModelType : uint32_t { UNKONW = 0, UNICYCLE = 1, BICYCLE = 2 };
enum class ForkStateType : uint8_t { DOWN = 0, MIDDLE = 1, UP = 2 };

// 叉腿防撞检测
enum class ForkLegCheckType : uint8_t {
  NONE = 0,         // 左右都无检测
  LEFT_CHECK = 1,   // 左边检测到
  RIGHT_CHECK = 2,  // 右边检测到
  BOTH = 3          // 左右都检测到
};

// 货架检测
enum class ForkPalletState : uint8_t {
  NONE = 0,         // 左右都无检测
  LEFT_CHECK = 1,   // 左边检测到
  RIGHT_CHECK = 2,  // 右边检测到
  BOTH = 3          // 左右都检测到
};

// 载货状态
enum class LoadStateType : int32_t { UNKONW = 0, ON_LOAD = 1, NO_LOAD = 2 };

enum class AvoidMapType : int {
  INIT = 0,
  TIME_MIN_AVOID_AREA = 9,        // 随时间最小避障区域
  TIME_DECREASE_AVOID_AREA = 10,  // 随时间减弱避障区域
  AVOID_DIY_MAP_17 = 17,          // 自定义8
  AVOID_DIY_MAP_18 = 18,          // 自定义7
  AVOID_DIY_MAP_19 = 19,          // 自定义6
  AVOID_DIY_MAP_20 = 20,          // 自定义5
  AVOID_DIY_MAP_21 = 21,          // 自定义4
  AVOID_DIY_MAP_22 = 22,          // 自定义3
  AVOID_DIY_MAP_23 = 23,          // 自定义2
  AVOID_DIY_MAP_24 = 24,          // 自定义1
  AVOID_DIY_MAP_25 = 25,          // 避障减弱
  NONE = 26,                      // 无避障
  BACK_LEFT = 27,                 // 后退右转
  BACK_RIGHT = 28,                // 后退左转
  FORWARD_LEFT = 29,              // 前进左转
  FORWARD_RIGHT = 30,             // 前进右转
  FORWARD = 31                    // 直行避障
};

enum class AvoidLevel : uint8_t {
  NONE = 0,                        // 无避障
  LEVEL_I = 1,                     // 避障区域 避障等级1 - slowlevel1
  LEVEL_II = 2,                    // 避障区域 避障等级2 - slowlevel2
  LEVEL_III = 3,                   // 避障区域 避障等级3 - stop
  BUMP = 4,                        // 防撞条
  FORK_LEFT_LEG = 5,               // 叉车左叉腿防撞
  FORK_RIGHT_LEG = 6,              // 叉车右叉腿防撞
  FORK_LEG_BOTH = 7,               // 叉车左右叉腿都检测到
  UP_FORWARD_LASER_OBSTACLE = 9,  // 顶上前向避障雷达检测

  AVOID_LASER_0 = 10,              // 避障激光0(前左侧)
  AVOID_LASER_1 = 11,              // 避障激光1(前侧)
  AVOID_LASER_2 = 12,              // 避障激光2(前右侧)
  AVOID_LASER_3 = 13,              // 避障激光3(后中侧或托盘车右叉腿)
  AVOID_LASER_4 = 14,              // 避障激光4(后右侧)
  AVOID_LASER_5 = 15,              // 避障激光5(后左侧)

  AVOID_CAMERA_0 = 20,             // 前置左侧相机
  AVOID_CAMERA_1 = 21,             // 前置中间相机
  AVOID_CAMERA_2 = 22,             // 前置右侧相机
  AVOID_CAMERA_3 = 23,             // 后置右侧叉腿相机
  AVOID_CAMERA_4 = 24,             // 后置左侧叉腿相机
  AVOID_CAMERA_5 = 25,             // 备用相机1

  NAVI_LASER_0 = 30,               // 导航激光0号
  NAVI_LASER_1 = 31,               // 导航激光1号
  NAVI_LASER_2 = 32,               // 导航激光2号

  AVOID_ULTRASONIC = 41,     // 超声波避障：KS104
  FORWARD_ULTRASONIC = 42,   // 前向超声波检测
  BACKWARD_ULTRASONIC = 43,  // 后向超声波检测
  LEFT_ULTRASONIC = 44,      // 左向超声波检测
  RIGHT_ULTRASONIC = 45,     // 右向超声波检测

  PEDERSTRIAN_DETECT = 60,      // 行人识别
  
  INVAILD_SENSOR_DATA = 90,  // 无可用避障传感器数据

  AVOID_LASER_0_TIMEOUT = 100,   // 避障激光0 数据超时
  AVOID_LASER_1_TIMEOUT = 101,   // 避障激光1 数据超时
  AVOID_LASER_2_TIMEOUT = 102,   // 避障激光2 数据超时
  AVOID_LASER_3_TIMEOUT = 103,   // 避障激光3 数据超时
  AVOID_LASER_4_TIMEOUT = 104,   // 避障激光4 数据超时
  AVOID_LASER_5_TIMEOUT = 105,   // 避障激光5 数据超时

  AVOID_CAMERA_0_TIMEOUT = 110,  // 相机0 数据超时
  AVOID_CAMERA_1_TIMEOUT = 111,  // 相机1 数据超时
  AVOID_CAMERA_2_TIMEOUT = 112,  // 相机2 数据超时
  AVOID_CAMERA_3_TIMEOUT = 113,  // 相机3 数据超时
  AVOID_CAMERA_4_TIMEOUT = 114,  // 相机4 数据超时
  AVOID_CAMERA_5_TIMEOUT = 115,  // 相机5 数据超时

  LASER_0_TIMEOUT = 120,         // 导航激光0 数据超时
  LASER_1_TIMEOUT = 121,         // 导航激光1 数据超时
  LASER_2_TIMEOUT = 122,         // 导航激光1 数据超时

  SAFETY_IO_TIMEOUT = 131,       // 避障类io 数据超时
  ULTRASONIC_TIMEOUT = 141,      // 超声波 数据超时

  AVOID_OVERTIME = 200,  // 避障超时
};

enum class AvoidSpeedLevel : uint32_t {
  FREE = 0,
  SLOWLEVEL1 = 1,
  SLOWLEVEL2 = 2,
  STOP = 3
};

enum class StateType : int {
  MANUAL = 1,     // 手动
  AUTO = 2,       // 自动
  BUMP_ERROR = 3  // 防撞触发
};

// 功耗模式
// SLEEPING: 低功耗模式
// ACTIVE: 正常功耗模式
enum class PowerMode { UNKNOWN, SLEEPING, ACTIVE };

// 滚筒车滚筒状态
enum class RollerInOutState : uint8_t {
  ALL_NONE = 0,
  OUT = 1,
  MIDDLE = 2,
  OUT_AND_MIDDLE = 3,
  IN = 4,
  OUT_AND_IN = 5,
  IN_AND_MIDDLE = 6,
  ALL_BOTH = 7
};

// 顶升车托盘状态
enum class JackUpDownState {
  UP_NONE = 0,
  UP_1 = 1,
  UP_2 = 2,
  UP_BOTH = 3,
  DOWN_NONE = 4,
  DOWN_1 = 5,
  DOWN_2 = 6,
  DOWN_BOTH = 7,
};
// 顶升车托盘找零状态
enum class JackUpZeroType : uint8_t {
  ROATE_NONE = 0,
  ROATE_1 = 1,
  ROATE_2 = 2,
  ROATE_BOTH = 3,
};

// 无线IO遥控器状态
enum class RemoteIoStatus : uint8_t {
  REMOTE_IO_0 = 0,
  REMOTE_IO_1 = 1,  // 上升
  REMOTE_IO_2 = 2,  // 下降
  REMOTE_IO_3 = 3,  // 前进
  REMOTE_IO_4 = 4,  // 后退
  REMOTE_IO_5 = 5,  // 左转
  REMOTE_IO_6 = 6,  // 右转
  REMOTE_IO_7 = 7,  // 进入遥控模式
  REMOTE_IO_8 = 8,  // 退出遥控模式
  REMOTE_IO_9 = 9,
  REMOTE_IO_10 = 10,
  REMOTE_IO_11 = 11,
  REMOTE_IO_12 = 12,
  REMOTE_IO_13 = 13,
  REMOTE_IO_14 = 14,
  REMOTE_IO_15 = 15,
};

// 对接设备状态
enum class DockState : uint8_t {
  FAILED = 0,
  BACKING = 1,
  FRONTING = 2,
  ORIGINAL = 3,
  TIMEOUT = 4,
  SUCCESSED = 5,
};

enum class LoadState : uint32_t {
  UNKOWN = 0,
  HAVE_LOAD = 1,
  NO_LOAD = 2,
};

// 转向灯控制类型
enum class CorningLedType : uint8_t {
  LIGHT_OFF = 0,  //
  LIGHT_ALL_CONSTAN_ON = 1,
  LIGHT_ALL_BLINK = 2,
  LEFT_BLINK = 3,
  RIGHT_BLINK = 4,
  // v1.1.5新增状态灯
  GREEN_ALL_CONSTAN_ON = 5,
  RED_ALL_CONSTAN_ON = 6,
  YELLOW_ALL_BLINK = 7,
  YELLOW_LEFT_BLINK = 8,
  YELLOW_RIGHT_BLINK = 9,
  GREEN_ALL_BLINK = 10,
  BLUE_ALL_BLINK = 11
};

// 三色灯控制类型
enum class ThreeColorLedType : uint8_t {
  RED_ON = 1,     // 00000001
  YELLOW_ON = 2,  // 00000010
  GREEN_ON = 4,   // 00000100

  YELLOW_GREEN_ON = 6,  // 00000110
  RED_YELLOW_ON = 3,    // 00000011
  RED_GREEN_ON = 5,     // 00000101

  LIGHT_ALL_ON = 7,  // 00000111
  LIGHT_OFF = 0      // 00000000
};

// 先使用以下的
enum class AudioType : uint8_t {
  NONE = 0,
  BUMP_FAULT = 1,            // (防撞条触发)报警音
  FORWARD = 2,               // 前进运行
  BACKWARD = 3,              // 倒车（倒车请注意)
  OBSTACLE_WARN = 4,         // 减速避障(嘟嘟嘟提示音)
  OBSTACLE_STOP = 5,         // 停车避障(请移除障碍物)
  LOW_BATTERY = 6,           // 低电量(低电量请注意)
  FORK_LEG_SAFETY = 7,       // 叉腿避障(请移除障碍物)
  NETWORK_TIMEOUT = 8,       // 离线(机器人已离线)
  TASK_PAUSE = 9,            // 任务暂停(等待调度指令)
  LOCALIZER_ERROR = 10,      // 定位失败(定位失败)
  OFF_THE_TRACK = 11,        // 出轨(出轨请注意)
  PALLET_DETECT_ERROR = 12,  // 库位检测失败(库位检测失败)
  WARING_PERSON = 13,        // 识别到行人(行人请注意)
  INIT_FAILED = 14,          // 自检失败(自检失败,请注意)
  ROBOT_ERROR = 15,          // 故障(机器人故障)
  CHARGE_ERROR = 16,         // 充电失败(充电失败)   #调度下发
  TASK_ERROR = 17,           // 任务出错(任务出错)
  LOSE_LOAD = 18,            // 掉货(货物掉落请注意)
  INIT_SUCCEED = 19,         // 自检成功或重定位成功(准备就绪)
  NO_IN_WORK_AREA = 20,  // 不在工作区域告警(非工作区域请注意)  #调度下发
  TURN_LEFT_WARNING = 21,   // 左转(左转请注意)
  TURN_RIGHT_WARNING = 22,  // 右转(左转请注意)
  PICK_UP_CALL = 23,        // 请取货(请取货)
  UP_SHELF_LOSS = 24,       // 向上整定失败(托盘丢失)
  ROUTE_PLAN_ERROR = 25,  // 路径规划失败(路径规划失败)  #调度下发
  CHECK_WINDOW = 26,  // XP2手动切换到自动跳弹(弹窗请确认)  #调度下发
  MANUAL_MODE = 27,          // 手动操作
  DOWN_TAG_LOSS_ERROR = 28,  // 丢码(向下二维码丢失)
  UP_TAG_LOSS_ERROR = 29,    // 丢码(向上二维码丢失)
  LOCALIZER_LOSE = 30,       // 定位丢失(定位丢失)
  WARING_OCCUPATION =
      31,  // 手动时驶入授权占点(驶入授权占点区域 请驶离)  #调度下发
  DOCKING_TIME_OUT = 32,  // 对接超时
  WARING_STORAGE = 33,  // 库位分配异常(库位分配异常)  #调度下发
  ACTION_TIME_OUT = 34,     // ACTION超时
  EMERGENCY_STOP = 35,      // 急停处于按下状态
  STABILIZE_TIME_OUT = 36,  // 整定超时
  PALLET_LIMIT_ERROR = 37,  // 货物超板(货物超板请注意)
   
  FRONT_LASER0_OBSTACLE = 39,   // 前置0号激光避障
  FRONT_LASER1_OBSTACLE = 40,   // 前置1号激光避障
  FRONT_LASER2_OBSTACLE = 41,   // 前置2号激光避障
  FRONT_LASER3_OBSTACLE = 42,   // 前置3号激光避障
  BACK_LASER1_OBSTACLE = 43,    // 后置1号激光避障
  BACK_LASER2_OBSTACLE = 44,    // 后置2号激光避障
  FRONT_CAMERA1_OBSTACLE = 45,  // 前置1号相机避障
  FRONT_CAMERA2_OBSTACLE = 46,  // 前置2号相机避障
  FRONT_CAMERA0_OBSTACLE = 47,  // 前置0号相机避障
  BACK_CAMERA1_OBSTACLE = 48,   // 后置1号相机避障
  BACK_CAMERA2_OBSTACLE = 49,   // 后置2号相机避障
  NAVI_LASER1_OBSTACLE = 50,    // 导航1号激光避障
  NAVI_LASER2_OBSTACLE = 51,    // 导航2号激光避障
  NAVI_LASER3_OBSTACLE = 52,    // 导航3号激光避障
  BACK_LIGHT1_AVOID = 53,       // 后置1号光电避障
  BACK_LIGHT2_AVOID = 54,       // 后置2号光电避障
  ULTRASONIC_AVOID = 55,        // 超声波避障

  BACK_LASER3_OBSTACLE = 60,  // 后置3号激光避障
};

enum class LaserType : int { PPF = 0, SICK = 1, UAM = 2,SIM = 3 };

enum class SwitchMapType : int {
  SWITCH_WITH_RELOCATION = 1,
  SWITCH_WITH_TARGET_POINT = 2
};

enum class ClampLimitType : uint8_t {
  NONE = 0,
  IN_LIMIT = 1,
  OUT_LIMIT_I = 2,
  OUT_LIMIT_II = 3
};

namespace common {
enum class AgvErrorLevel : uint16_t {
  NONE = 0,
  WARN = 1,
  ERROR = 2,
  SERIOUS_ERROR = 3,
  FATAL = 4
};
enum class AgvErrorType : uint32_t { NONE = 0, DRIVER_NOT_READY = 1, BUMP = 2 };

}  // namespace common

#endif  // amr_COMMON_INCLUDE_amr_COMMON_amr_ENUM_TYPE_H_

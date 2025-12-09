#ifndef DECISION_MAKER_INCLUDE_DECISION_MAKER_ENUM_H_
#define DECISION_MAKER_INCLUDE_DECISION_MAKER_ENUM_H_
#include <ros/ros.h>

#include <mutex>
#include <string>

#include "amr_common/amr_enum_type.h"
#include "amr_common/amr_protocal.h"
#include "amr_common/geometry/amr_geometry.h"
#include "amr_msgs/audio_control.h"
#include "amr_msgs/battery.h"
#include "amr_msgs/charge_do_state.h"
#include "amr_msgs/dongle_feedback.h"
#include "amr_msgs/error_state.h"
#include "amr_msgs/fault_report.h"
#include "amr_msgs/led_control.h"
#include "amr_msgs/load_state.h"
#include "amr_msgs/manual.h"
#include "amr_msgs/move_feedback.h"
#include "amr_msgs/node_diagnostic.h"
#include "amr_msgs/pgv100_feedback.h"
#include "amr_msgs/safety_io_state.h"
#include "amr_msgs/safety_state.h"
#include "amr_msgs/task_control_request.h"
#include "amr_msgs/task_finish_request.h"
#include "amr_msgs/task_request.h"
#include "amr_msgs/task_response.h"
#include "amr_msgs/update_event.h"
#include "amr_msgs/weighing_feedback.h"
#include "amr_msgs/weighting_loaded.h"

namespace decision_maker {

using AgvErrorType = common::AgvErrorType;
using AgvErrorLevel = common::AgvErrorLevel;
using Pose = amr_geometry::Pose;

enum class StepFinishState : uint8_t {
  DONE_AND_SUCCEED = 1,
  DONE_BUT_FAILED = 2,
  NOT_DONE = 3,
  END_STEP_FINISH = 4,
  DONE_STEP_TO_OPERATION = 5,
  DONE_STEP_TO_END = 6,
  GOAL_ERROR = 7

};

enum class GoalFinishState : uint8_t {
  FAILED = 0,
  SUCCEEDED = 1,
  SUCCEEDED_TO_OPERATION = 2,
  SUCCEEDED_TO_END = 3
};

enum class ErrorType : uint16_t {
  NORMAL = 0,
  TASK_SEQUENCE_NUMBER_ERROR = 1,
  TASK_ORDER_ID_ERROR = 2,
  SEQUENCE_ID_ERROR = 3,
  TASK_TYPE_ERROR = 4,
  DONGLE_DIED = 5,
  LOST_GOODS = 6
  // 警告(自行增加)
};

enum class GoalType : uint32_t {
  NONE = 0,
  TRACK_PATH_GOAL = 1,
  ACTION_GOAL = 2,
  TRACK_PATH_ACTION_GOAL = 3,
  END = 4
};

enum class TaskControlType : int32_t {
  NONE = 0,
  STOP_RECOVER = 1,
  PAUSE_STOP = 2,
  EMERGENCY_STOP = 3,
  SINGLE_TASK_START = 4,
  SINGLE_TASK_STOP = 5,
  TASK_CANCEL = 11
};

enum class LoadStateType : int32_t { UNKONW = 0, ON_LOAD = 1, NO_LOAD = 2 };

// enum class AgvErrorLevel : uint16_t {
//   NONE = 0,
//   WAITING = 1,
//   ERROR = 2,
//   FATAL = 3
// };

enum class ResponseType : int32_t {
  NONE = 0,
  TASK_RECEIVED = 1,
  TASK_ABNORMAL = 2,
  CONTROL_RECEIVED = 3
  // FINISH_RESPONSE = 4,
};

enum class StateErrorType {
  NONE = 0,
  CHARGEERROR = 1,
  FORKERROR = 2,
  FAULT = 3,
  LOST_PALLET = 4
};

enum class DirectionType : uint8_t {
  NONE = 0,
  FORWARD = 1,
  TURN_LEFT = 2,
  TURN_RIGHT = 3,
  BACKWARD = 4,
};

struct TaskEndPoint {
  TaskEndPoint() : sequence_id(1) /* 由1开始*/, sequence_num(0), point_id(0) {}
  int sequence_id;
  int sequence_num;
  int point_id;
  Pose end_pose;
};

struct OdomInfo {
  OdomInfo() : odom(0.), total_duration(0.), avoid_duration(0.) {}
  double odom;
  double total_duration;
  double avoid_duration;
};

struct SafetyState {
  AvoidLevel type = AvoidLevel::NONE;
  AvoidSpeedLevel level = AvoidSpeedLevel::FREE;
};

struct LedType {
  CorningLedType corning_led_type;
  ThreeColorLedType three_color_led_type;
};

struct FinishResponseType {
  std::string type;
  bool state;
};

struct TaskControlAudio {
  AudioType audio_type;
  bool enable;
  TaskControlAudio() {
    std::memset(&audio_type, 0, sizeof(AudioType));
    enable = false;
  }
};

enum class PowerOnState : uint8_t { NONE = 0, DOING = 1, DONE = 2, FAILED = 3 };
// 满足触发条件时，超时会将数据清零
class TimedManagerState {
 public:
  TimedManagerState() : is_trigger_(false), state_(0) {}
  TimedManagerState(const TimedManagerState&) = delete;
  TimedManagerState& operator=(const TimedManagerState&) = delete;

  void set(const bool& trigger, const int& state, const ros::Time& time) {
    std::unique_lock<std::mutex> lock(mutex_);
    is_trigger_ = trigger;
    state_ = state;
    start_time_ = time;
  }

  void set_trigger(const bool& trigger) {
    std::unique_lock<std::mutex> lock(mutex_);
    is_trigger_ = trigger;
  }

  void set_state(const int& state) {
    std::unique_lock<std::mutex> lock(mutex_);
    state_ = state;
  }

  void set_start_time(const ros::Time& time) {
    std::unique_lock<std::mutex> lock(mutex_);
    start_time_ = time;
  }

  inline const bool is_trigger() { return is_trigger_; }

  inline const uint8_t state() { return state_; }

  inline const ros::Time start_time() { return start_time_; }

 private:
  std::mutex mutex_;
  ros::Time start_time_;
  bool is_trigger_;
  uint8_t state_;
};

struct PalletCenter {
  ros::Time time;
  Pose pallet_center_pose;
  double height;
  int id;
  PalletCenter() : id(0), height(0.) {}
};

struct Event {
  // 控制信号
  TaskControlType task_control;
  // 调度控制音频信号
  TaskControlAudio task_audio;
  // 手自动
  bool manual;
  // 急停信号
  bool emergency_stop;
  // 是否充电
  bool charging;
  // 音频时间管理信号 语音&&清除故障使用
  std::shared_ptr<TimedManagerState> re_location_state_ptr;

  std::shared_ptr<TimedManagerState> power_on_state_ptr;
  // 结束任务响应
  FinishResponseType task_finish_response;

  void ResetTaskFinish() {
    if (task_finish_response.type ==
            amr_protocal::msg_type::kTaskFinishRequest &&
        task_finish_response.state) {
      task_finish_response = {"", false};
    }
  }

  // 开启掉货检测 抬货之后触发
  bool enable_lose_pallet_check;
  // 手自动切换后关闭掉货检测
  bool close_check_weight;
    // 检测叉腿上升或下降状态
  bool check_fork_state;
  // 货物重量
  int load_weight;
  // 货物重量基准
  int load_weight_base;
  // 叉腿高度
  int forklift_height;
  // 托盘中心位姿
  PalletCenter pallet_center;

  ForkStateType pallet_fork_state;
  // 载货状态
  LoadStateType heap_load_state;
  LoadStateType load_state;
  // 二维码id
  uint32_t qr_num_;
  // 错误等级
  AgvErrorLevel agv_error_level;
  //错误来源类型
  uint32_t agv_error_type;
};

struct IndicatorType {
  bool finish_request;
  bool finish_location_request;
  bool bummp_error;
  bool pederstrain_waring;
  bool avoid_sensor_timeout_error;
  bool localizer_match_error;
  bool localizer_init_error;
  bool shelf_loss_error;
  bool down_tag_loss_error;
  bool up_tag_loss_error;
  bool down_qr_camera_error;
  bool up_qr_camera_error;
  bool get_velocity_error;
  bool docking_time_out_error;
  bool stabilize_time_out_error;
  bool action_time_out_error;
  bool navi_error;
  bool embedded_driver_error;
  bool pallet_detect_error;
  bool switch_audio_type;
  bool dongle_alive_state_;
  bool up_weight_check_error;
  bool pallet_limit_error;
  bool down_weight_check_error;
  DirectionType direction_type;
  StateErrorType error_code;

  int current_map_id;      // 当前所在地图
  int current_section_id;  // 当前所在区域
};

struct AudioStateType {
  AudioType type;
  uint8_t level;
  uint32_t repeat;
};

struct Cmd {
  float velocity;
  float omega;
};

struct ReportType {
  std::string order_id;
  // task_manager current_step pointId
  std::string node_id;
  int point_id;  // 表示agv当前所处的位置点
  // 状态机状态
  int agv_status;  // agv 状态, 双方约定枚举类型
  // task_manager current_step sequenceId
  int task_sequence;  // 当前正在处理的任务指令的序列号
  // safety setting
  int avoid_strategy_code;  // 目前先用调度发下来的避障区域

  int ultrasonic_obstacle;  // 是否打开超声波避障

  int current_section_id;  // 当前所在区域

  TaskEndPoint task_end_point;  // 当前任务终点属性

  double remain_distance;  // 当前任务与终点距离
};

}  // namespace decision_maker

#endif  // DECISION_MAKER_INCLUDE_DECISION_MAKER_ENUM_H_

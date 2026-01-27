/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_NODE_DIAGNOSTIC_MANAGER_H_
#define amr_COMMON_INCLUDE_amr_COMMON_NODE_DIAGNOSTIC_MANAGER_H_
#include <mutex>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include "amr_common/node_diagnostic_info.h"
#include "amr_msgs/error_info.h"

namespace amr_diagnostic {
template <typename T>
class NodeStatusManager {
 public:
  NodeStatusManager() : time_(ros::Time::now()) {
    error_status_.first = T::NORMAL;
  }

  // 故障错误按优先级排序  1>2
  void SetNodeStatus(const T& status, const std::vector<double>& error_info) {
    std::unique_lock<std::mutex> lock(mutex_);
    bool is_changed = false;
    if (error_status_.first == T::NORMAL) {
      error_status_.first = status;
      is_changed = true;
    } else {
      error_status_.first = static_cast<uint16_t>(error_status_.first) >=
                                    static_cast<uint16_t>(status)
                                ? error_status_.first
                                : status;
      is_changed = error_status_.first == status ? true : false;
    }
    if (is_changed) error_status_.second = error_info;
  }

  void SetNodeStatus(const T& status) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (error_status_.first == T::NORMAL) {
      error_status_.first = status;
    } else {
      error_status_.first = static_cast<uint16_t>(error_status_.first) >=
                                    static_cast<uint16_t>(status)
                                ? error_status_.first
                                : status;
    }
    error_status_.second.clear();
  }

  void ClearNodeStatus() {
    std::unique_lock<std::mutex> lock(mutex_);
    error_status_.first = T::NORMAL;
    error_status_.second.clear();
  }

  void PeriodClearNodeStatus(const double& time) {
    // 周期性清除状态
    if (ros::Time::now() - time_ < ros::Duration(time)) return;
    std::unique_lock<std::mutex> lock(mutex_);
    error_status_.first = T::NORMAL;
    error_status_.second.clear();
    time_ = ros::Time::now();
  }

  inline const std::pair<T, std::vector<double>> GetNodeStatus() {
    std::unique_lock<std::mutex> lock(mutex_);
    return error_status_;
  }

 private:
  std::mutex mutex_;
  ros::Time time_;
  std::pair<T, std::vector<double>> error_status_;
};

}  // namespace amr_diagnostic

#endif  // amr_COMMON_INCLUDE_amr_COMMON_NODE_DIAGNOSTIC_MANAGER_H_

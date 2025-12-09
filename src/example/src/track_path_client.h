/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef DECISION_MAKER_INCLUDE_DECISION_MAKER_TRACK_PATH_CLIENT_H_
#define DECISION_MAKER_INCLUDE_DECISION_MAKER_TRACK_PATH_CLIENT_H_
#include <actionlib/client/simple_action_client.h>

#include <string>

#include "amr_common/log_porting.h"
#include "amr_msgs/track_pathAction.h"
#include "decision_maker/decision_maker_enum.h"
#include "ros/ros.h"

namespace decision_maker {

class TrackPathClient {
 public:
  TrackPathClient() = delete;
  explicit TrackPathClient(const std::string action_name, bool flag = true)
      : client_(action_name, flag) {
    finish_state_ = GoalFinishState::FAILED;
    is_client_available_ = false;
    task_done_ = false;
    could_send_task_ = true;
    error_ = false;
    motion_type_ = 0;
  }

  void SendTrackPathGoal(amr_msgs::track_pathGoal goal) {
    LOG_DEBUG("Waiting for [track_path] server to start.");
    client_.waitForServer();
    LOG_DEBUG("[track_path] server started.");
    client_.sendGoal(goal, boost::bind(&TrackPathClient::doneCb, this, _1, _2),
                     boost::bind(&TrackPathClient::activeCb, this),
                     boost::bind(&TrackPathClient::feedbackCb, this, _1));
    task_done_ = false;
    could_send_task_ = false;
    finish_state_ = GoalFinishState::FAILED;
  }

  void CancelTrackPathGoal() {
    if (is_client_available_) {
      client_.cancelGoal();
      is_client_available_ = false;
      return;
    }
    LOG_ERROR("[track_path] state error, cancel goal failed!");
  }

  GoalFinishState FinishState() { return finish_state_; }

  bool ActionAccepted() { return is_client_available_; }

  bool TaskDone() { return task_done_; }

  void ResetTaskPermission() { could_send_task_ = true; }

  void ResetFinishState() { finish_state_ = GoalFinishState::FAILED; }

  void ResetTaskDone() { task_done_ = false; }

  // 外部判断任务是否可发的接口
  bool SendTaskPermission() {
    return (!is_client_available_) && could_send_task_ && (!task_done_);
  }

  bool ActionError() { return error_; }

  uint32_t ActionMotionType() { return motion_type_; }

 private:
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const amr_msgs::track_pathResultConstPtr& result) {
    finish_state_ = static_cast<GoalFinishState>(result->is_finish);
    LOG_INFO_STREAM("[track_path] finish!!! state:" << +result->is_finish);
    // TODO(@ssh) 完成时任务该目标已经结束
    task_done_ = true;
    is_client_available_ = false;
  }

  void activeCb() {
    is_client_available_ = true;
    LOG_DEBUG("Goal is active! Begin [track_path].");
  }

  void feedbackCb(const amr_msgs::track_pathFeedbackConstPtr& feedback) {
    LOG_DEBUG("[track_path] state: %d", feedback->error);
    LOG_DEBUG("[track_path] motion_type: %d", feedback->motion_type);
    error_ = feedback->error;
    motion_type_ = feedback->motion_type;
  }

  actionlib::SimpleActionClient<amr_msgs::track_pathAction> client_;

  GoalFinishState finish_state_;
  bool is_client_available_;
  bool could_send_task_;
  bool task_done_;
  bool error_;
  uint32_t motion_type_;
};

}  // namespace decision_maker

#endif  // DECISION_MAKER_INCLUDE_DECISION_MAKER_TRACK_PATH_CLIENT_H_

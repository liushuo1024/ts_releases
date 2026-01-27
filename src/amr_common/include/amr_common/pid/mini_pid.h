/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_PID_MINI_PID_H_
#define amr_COMMON_INCLUDE_amr_COMMON_PID_MINI_PID_H_
#include <sstream>
#include <string>

#include "amr_common/log_porting.h"

class MiniPID {
 public:
  MiniPID(double, double, double);
  MiniPID(double, double, double, double);
  void setP(double);
  void setI(double);
  void setD(double);
  void setF(double);
  void setPID(double, double, double);
  void setPID(double, double, double, double);
  void setMaxIOutput(double);
  void setOutputLimits(double);
  void setOutputLimits(double, double);
  void setDirection(bool);
  void setSetpoint(double);
  void reset();
  void setOutputRampRate(double);
  void setSetpointRange(double);
  void setOutputFilter(double);
  double getOutput();
  double getOutput(double);
  double getOutput(double, double);
  double getOutputFromDiff(double);

  std::string toString() {
    std::stringstream ss;
    ss << "P: " << P_ << ", I: " << I_ << ", D: " << D_;
    return ss.str();
  }

 private:
  bool bounded(double, double, double);
  double clamp(double, double, double);
  void checkSigns();
  void init();
  double P_;
  double I_;
  double D_;
  double F_;

  double maxIOutput_;
  double maxError_;
  double errorSum_;

  double maxOutput_;
  double minOutput_;

  double setpoint_;

  double lastActual_;

  bool firstRun_;
  bool reversed_;

  double outputRampRate_;
  double lastOutput_;

  double outputFilter_;

  double setpointRange_;
};

struct PID_Handle {
  float kp;
  float ki;
  float kd;
  float integrator;
  float previous_error;
  float previous_error1;
  float previous_error2;
  float debug_p_out;
  float debug_i_out;
  float debug_d_out;

  void clear() {
    previous_error = 0.f;
    previous_error1 = 0.f;
    previous_error2 = 0.f;
    debug_p_out = 0.f;
    debug_i_out = 0.f;
    debug_d_out = 0.f;
  }
};
// classic pid
template <typename T>
T PID_Process(PID_Handle *handle, T error) {
  T output = 0;
  // static T err1 = 0.0;
  // static T err2 = 0.0;
  handle->integrator =
      error + handle->previous_error1 + handle->previous_error2;
  handle->previous_error = handle->previous_error1;
  handle->debug_p_out = handle->kp * error;
  handle->debug_i_out = handle->ki * handle->integrator;
  handle->debug_d_out = handle->kd * (error - handle->previous_error);

  output = handle->debug_p_out + handle->debug_i_out + handle->debug_d_out;
  handle->previous_error2 = handle->previous_error1;
  handle->previous_error1 = error;
  return output;
}

struct IncreasingPIDHandle {
  float kp;
  float ki;
  float kd;
  float diff_last;
  float diff_prev;
  float incpid;
};
// Incremental pid
template <typename T>
float IncrementalPIDProcess(IncreasingPIDHandle *handle, T error) {
  T output = 0;

  handle->incpid =
      handle->kp * (error - handle->diff_last) + handle->ki * error +
      handle->kd * (error - 2 * handle->diff_last + handle->diff_prev);
  handle->diff_prev = handle->diff_last;
  handle->diff_last = error;

  output += handle->incpid;

  return output;
}

#endif  // amr_COMMON_INCLUDE_amr_COMMON_PID_MINI_PID_H_

/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_COMMON_H_
#define amr_COMMON_INCLUDE_amr_COMMON_COMMON_H_
#include <angles/angles.h>

#include <chrono>
#include <cmath>

#include "log_porting.h"
#include "math.h"

// TODO(@someone) 做成通用运动函数库
namespace common {

#define PI_MATH 3.1415926

// 1 (m/s) = 60/(d * pi) (rpm)
// 轮子速度---> rpm
template <typename T>
T WheelVelocityToRPM(T velocity, T diameter, T radio) {
  return velocity / M_PI / diameter * 60.0 * radio;
}

template <typename T>
T RPMToWheelVelocity(T rpm, T diameter, T radio) {
  return rpm * M_PI * diameter / 60.0 / radio;
}

// 1 (rad/s) = 60 / (2 * pi) (rpm)
// 旋转电机角速度 --> rpm
template <typename T>
T RadPSToRPM(T radps, T radio) {
  T ret = 60.0 * radio * radps / (2 * M_PI);
  if (std::fabs(ret) > 3000) {
    LOG_ERROR("[RadPSToRPM] over 3000 rpm");
    ret = math::Sign(radps) * 3000;
  }
  return ret;
}

template <typename T>
T RPMToRadPS(T radps, T radio) {
  return 60.0 * radio * radps / (2 * M_PI);
}

template <typename T>
T LiftVPSToRPM(T lift_v, T radio, T screw) {
  T ret = 60.0 * lift_v * radio / screw;
  if (std::fabs(ret) > 3000) {
    LOG_ERROR("[RadPSToRPM] over 3000 rpm");
    ret = math::Sign(lift_v) * 3000;
  }
  LOG_DEBUG("[RadPSToRPM] ret:%f", ret);
  return ret;
}

template <typename T>
T RPMToLiftVPS(T radps, T radio, T screw) {
  return radps * screw / 60.f / radio;
}

// 模型速度 --> 左右轮子速度 according to the formula:
// Vl = V - W * L / 2
// Vr = V + W * L / 2
template <typename T>
T CalcLeftVelocity(T v, T w, T wheel_track) {
  return v - w * wheel_track / 2.;
}

template <typename T>
T CalcRightVelocity(T v, T w, T wheel_track) {
  return v + w * wheel_track / 2.;
}

// 左右轮子速度 --> 模型线速度速度和角速度
template <typename T>
T WheelVelToVelocity(T left_vel, T right_vel) {
  return (left_vel + right_vel) / 2.;
}

template <typename T>
T WheelVelToOmega(T left_vel, T right_vel, T wheel_track) {
  return (right_vel - left_vel) / wheel_track;
}

// 大小端转化
template <typename T>
T GetHtons(const uint8_t* byArr) {
  return ((static_cast<T>(byArr[0]) << 8) & 0xFF00) | byArr[1];
}

// 修改json string中指定字段参数
template <typename T>
bool JsonModifyField(std::string& json_str, std::string field_str, T value) {
  size_t index = json_str.find(field_str);
  bool is_find_symol1 = false;
  bool is_find_symol2 = false;
  bool is_find_symol3 = false;
  size_t base = 0;
  size_t offset = 0;
  ros::Time calculate_time = ros::Time::now();
  while (1) {
    // 超时退出循环
    if (ros::Time::now() - calculate_time > ros::Duration(1.0)) {
      LOG_ERROR("[JsonModifyField] find field over time!!!!");
      return false;
    }
    if (!is_find_symol1) {
      index++;
      is_find_symol1 = json_str[index] == ':' ? true : false;
      continue;
    }
    if (!is_find_symol2) {
      index++;
      is_find_symol2 = json_str[index] != ' ' ? true : false;
      base = is_find_symol2 == true ? index : base;
      continue;
    }
    if (!is_find_symol3) {
      index++;
      is_find_symol3 = json_str[index] == ',' ? true : false;
      offset = is_find_symol3 == true ? (index - base) : offset;
      continue;
    }
    break;
  }
  std::stringstream ss_temp;
  ss_temp << value;
  std::string value_str = ss_temp.str();
  json_str.replace(base, offset, value_str);
  return true;
}

template <typename T>
std::string GetCurrentTime() {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>
      tp = std::chrono::time_point_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(
      tp.time_since_epoch());
  return std::to_string(tmp.count());
}

enum class Direction {
  None = 0,
  XPositive = 0x1100,  // on X Axis positive direction
  XNegative = 0x1300,  // on X Axis negative direction
  YPositive = 0x1400,  // on Y Axis positive direction
  YNegative = 0x1200,  // on Y Axis negative direction
  XPosYPos = 0x1500,
  XPosYNeg = 0x1600,
  XNegYPos = 0x1700,
  XNegYNeg = 0x1800,
};

// wheel omega to motor rotate speed
template <typename T>
T rads2rmin(T rad_persec, int reduceRatio) {
  return 60 * reduceRatio * rad_persec / (2 * PI_MATH);
}

template <typename T1, typename T2>
T1 Omega2PalletRotater(T1 omega, T2 reduce_ratio) {
  return rads2rmin(omega, reduce_ratio);
}

// motor rotate speed to wheel omega
template <typename T>
T rmin2rads(T r_permin, T reduceRatio) {
  return r_permin * 2.0f * PI_MATH / 60.0f / reduceRatio;
}

// degree conver to rad
template <typename T>
T Degree2rad(T degree) {
  return degree * 2.0f * PI_MATH / 360.0f;
}

// rad conver to degree
template <typename T>
T rad2degree(T rad) {
  return rad * 360.0f / (2.0f * PI_MATH);
}

//
template <typename T>
T normalize_angle_positive(T angle) {
  return static_cast<T>(
      fmod(fmod(angle, 2.0f * PI_MATH) + 2.0f * PI_MATH, 2.0f * PI_MATH));
}

//
// template <typename T>
// T normalize_angle(T angle) {
//   T a = normalize_angle_positive(angle);
//   if (a > PI_MATH) a -= 2.0f * PI_MATH;
//   return a;
// }
//
template <typename T>
T AngleSubtract(T param1, T param2) {
  return angles::normalize_angle(param1 - param2);
}
//
template <typename T>
T AngleAddition(T param1, T param2) {
  T angle = param1 + param2;
  if (angle > PI_MATH)
    return fmod(fmod(angle, 2 * PI_MATH) - 2 * PI_MATH, 2 * PI_MATH);
  else if (angle < -PI_MATH)
    return fmod(fmod(angle, 2 * PI_MATH) + 2 * PI_MATH, 2 * PI_MATH);
  else
    return angle;
}

template <typename T>
bool fNearby(T left, T right, T diff) {
  return fabs(left - right) <= diff ? true : false;
}

template <typename T>
Direction Angle2Direction(T current_angle) {
  if (fNearby(current_angle, static_cast<T>(0), static_cast<T>(PI_MATH / 4)))
    return Direction::XPositive;
  if (fNearby(current_angle, static_cast<T>(PI_MATH / 2),
              static_cast<T>(PI_MATH / 4)))
    return Direction::YPositive;
  if (fNearby(current_angle, static_cast<T>(-PI_MATH / 2),
              static_cast<T>(PI_MATH / 4)))
    return Direction::YNegative;
  return Direction::XNegative;
}

//
template <typename T>
double Direction2Angle(T direction) {
  switch (direction) {
    case Direction::XPositive:
      return 0.00f;
    case Direction::XNegative:
      return PI_MATH;
    case Direction::YPositive:
      return PI_MATH / 2;
    case Direction::YNegative:
      return -PI_MATH / 2;
    case Direction::XPosYPos:
      return PI_MATH / 4;
    case Direction::XPosYNeg:
      return -PI_MATH / 4;
    case Direction::XNegYPos:
      return PI_MATH * 3 / 4;
    case Direction::XNegYNeg:
      return -PI_MATH * 3 / 4;
    default:
      return -1;
  }
}
template <typename T>
float fbound(T raw, T rmin, T rmax) {
  return fmin(fmax(raw, rmin), rmax);
}

template <typename T>
T AccDecOptimalTime(T cur_speed, T tar_speed, T dist_to_target, T max_acc,
                    T max_dec, T max_speed_on_route, T t_control_sec) {
  const float Torlerance =
      0.01f;  // To judge whether an input value is in the neighborhood of 0.
  T next_speed = 0;
  if (dist_to_target > 0 && max_acc > 0 && max_dec > 0 &&
      t_control_sec > 0) {  // check whether the inputs are the correct ranges
    if (tar_speed >=
        0) {  // It is expected that AGV moves forward at destination
      if (cur_speed >= -Torlerance &&
          dist_to_target >= 0) {  // AGV currently moves forward
        T max_speed_for_optimal_control = sqrt(
            2 * max_acc * max_dec * (dist_to_target) / (max_acc + max_dec) +
            (max_dec * pow(cur_speed, 2) + max_acc * pow(tar_speed, 2)) /
                (max_acc + max_dec));
        if (max_speed_for_optimal_control <= cur_speed) {  // Slow down
          next_speed = sqrt(pow(tar_speed, 2) + 2 * max_dec * (dist_to_target));
        } else {  // speed up
          T next_speed_tmp = cur_speed + max_acc * t_control_sec;
          next_speed =
              fmin(fmin(fmax(fabs(max_speed_on_route), fabs(tar_speed)),
                        max_speed_for_optimal_control),
                   next_speed_tmp);
        }
      } else {
        next_speed = 0;
      }
    } else {  // AGV moves backward at destination
      if (cur_speed <= Torlerance &&
          dist_to_target >= 0) {  // AGV currently moves backward
        float max_speed_for_optimal_control = -sqrt(
            2 * max_acc * max_dec * (dist_to_target) / (max_acc + max_dec) +
            (max_dec * pow(cur_speed, 2) + max_acc * pow(tar_speed, 2)) /
                (max_acc + max_dec));
        if (max_speed_for_optimal_control >= cur_speed) {  // slow down
          next_speed =
              -sqrt(pow(tar_speed, 2) + 2 * max_dec * (dist_to_target));
        } else {  // speed up
          T next_speed_tmp = cur_speed - max_acc * t_control_sec;
          next_speed =
              fmax(fmax(-fmax(fabs(max_speed_on_route), fabs(tar_speed)),
                        max_speed_for_optimal_control),
                   next_speed_tmp);
        }
      } else {
        next_speed = 0;
      }
    }
  } else {
    next_speed = 0;
  }

  return next_speed;
}

//
template <typename T>
T AccDecStop(T cur_speed, T dist_to_target, T max_acc, T max_dec,
             T max_speed_on_route, T tor, T t_control_sec) {
  const T tar_speed = 0.0f;
  float next_speed = 0;

  if (dist_to_target > tor) {
    next_speed =
        AccDecOptimalTime(cur_speed, tar_speed, dist_to_target, max_acc,
                          max_dec, max_speed_on_route, t_control_sec);
  } else if (dist_to_target < -tor) {
    next_speed =
        -AccDecOptimalTime(-cur_speed, tar_speed, -dist_to_target, max_acc,
                           max_dec, max_speed_on_route, t_control_sec);
  } else {
    next_speed = 0.0f;  // 2 * dist_to_target;
  }
  return next_speed;
}

/**
 *  t:从开始已经走过的路程
    b:beginning position，起始位置
    c:change，要移动的距离，就是终点位置减去起始位置。
    d: duration ，缓和效果持续的时间。-持续缓动总距离
*/
// linear dec..
template <typename T>
T LinearEase(T t, T b, T c, T d) {
  return c * (1 - t / d) + b;
}
template <typename T>
T SineEaseInOut(T t, T b, T c, T d) {
  return -c / 2 * (cos(PI_MATH * (1 - t / d)) - 1) + b;
}
template <typename T>
T QuadEaseInOut(T t, T b, T c, T d) {
  if ((2 * t / d) < 1) {
    t /= d / 2;
    return c - c / 2 * t * t + b;
  }
  t = t - 1;
  return -c / 2 * (t * (2 - t) - 1) + b;
}
template <typename T>
T CubicEaseInOut(T t, T b, T c, T d) {
  if ((2 * t / d) < 1) {
    t /= d / 2;
    return c - c / 2 * t * t * t + b;
  }
  t = t - 2;
  return c - c / 2 * (t * t * t + 2) + b;
}
template <typename T>
T QuartEaseInOut(T t, T b, T c, T d) {
  if ((2 * t / d) < 1) {
    t /= d / 2;
    return c - c / 2 * t * t * t * t + b;
  }
  t = t - 2;
  return c + c / 2 * (t * t * t * t - 2) + b;
}
template <typename T>
T QuintEaseInOut(T t, T b, T c, T d) {
  if ((2 * t / d) < 1) {
    t /= d / 2;
    return c - c / 2 * t * t * t * t * t + b;
  }
  t = t - 2;
  return c - c / 2 * (t * t * t * t * t + 2) + b;
}
template <typename T>
T ExpoEaseInOut(T t, T b, T c, T d) {
  if (t == 0) return b;
  if (t == d) return b + c;
  if ((2 * t / d) < 1) {
    t /= d / 2;
    return c - c / 2 * pow(2, 10 * (t - 1)) + b;
  }
  t = t - 1;
  return c - c / 2 * (-pow(2, -10 * t) + 2) + b;
}
template <typename T>
T CircEaseInOut(T t, T b, T c, T d) {
  if ((2 * t / d) < 1) {
    t /= d / 2;
    return c + c / 2 * (sqrt(1 - t * t) - 1) + b;
  }
  t = t - 2;
  return c - c / 2 * (sqrt(1 - t * t) + 1) + b;
}

enum SpeedFormulaType { FROM, DISTANCE, OPTIMAL, MAXSPEED };

/**
 * \class SpeedFormula
 */
template <SpeedFormulaType DISTANCE>
class SpeedFormula {
 public:
  SpeedFormula(double from, double to, double acc)
      : from_(from), to_(to), acc_(acc) {}

  double get() {
    return std::fabs((std::pow(to_, 2) - std::pow(from_, 2)) / acc_ / 2.0);
  }

 private:
  double from_;
  double to_;
  double acc_;
};

template <>
class SpeedFormula<FROM> {
 public:
  SpeedFormula(double to, double acc, double distance)
      : to_(to), acc_(acc), distance_(distance) {}

  double get() {
    float to2 = std::pow(to_, 2) + 2 * acc_ * distance_;
    return to2 >= 0.0 ? std::sqrt(to2) : 0.0;
  }

 private:
  double to_;
  double acc_;
  double distance_;
};

template <>
class SpeedFormula<OPTIMAL> {
 public:
  SpeedFormula(double from, double to, double acc, double dec, double distance)
      : from_(from), to_(to), acc_(acc), dec_(dec), distance_(distance) {}

  double get() {
    float to2 =
        2 * acc_ * dec_ * distance_ / (acc_ + dec_) +
        (dec_ * std::pow(from_, 2) + acc_ * std::pow(to_, 2)) / (acc_ + dec_);
    // float to2 = std::pow(to_, 2) + 2 * dec_ * distance_;
    return to2 >= 0.0 ? std::sqrt(to2) : 0.0;
  }

 private:
  double from_;
  double to_;
  double acc_;
  double dec_;
  double distance_;
};

//计算系统所能达到的最大速度
template <>
class SpeedFormula<MAXSPEED> {
 public:
  SpeedFormula(double from, double to, double accdec, double distance)
      : from_(from), to_(to), accdec_(accdec), distance_(distance) {}

  double get() {
    float max =
        ((std::pow(from_, 2) + std::pow(to_, 2)) - 2 * accdec_ * distance_) / 2;
    return max >= 0.0 ? std::sqrt(max) : 0.0;
  }

 private:
  double from_;
  double to_;
  double accdec_;
  double distance_;
};

static double GetLinearSpeed(double cur, double tar, double dist,
                             const double& max_v, const double& min_v,
                             const double& acc, const double& dec,
                             const double& margin,
                             const float& controller_frequency) {
  double time_cycle = 1. / controller_frequency;

  if (cur * dist > 0 && cur * math::Sign(dist) * tar < 0) {
    LOG_ERROR(
        "No speed strategy can handle this situation. (from: %f, to: %f, "
        "distance: %f)",
        cur, tar, dist);
    return 0.0;
  }

  // 接近停止点 低速行驶
  if (std::fabs(dist) < margin && tar == 0.) {
    auto final_v =
        math::Sign(dist) * math::Clamp(std::fabs(dist) * 0.3, min_v, 0.1);
    return final_v;
  }
  // 平滑驶入低速点
  dist = tar == 0. ? dist - math::Sign(dist) * margin : dist;
  tar = tar == 0. ? std::fabs(margin) * 0.3 : tar;

  if (SpeedFormula<OPTIMAL>(std::fabs(cur), std::fabs(tar), acc, dec,
                            std::fabs(dist))
          .get() <= (std::fabs(cur))) {
    // T Curve decelerate
    double ret =
        math::Sign(dist) *
        math::Clamp(SpeedFormula<FROM>(tar, dec, std::fabs(dist)).get(),
                    std::fabs(min_v), std::fabs(max_v));
    LOG_WARN("decelerate: dist:%f, cur:%f, tar:%f, set:%f, dec:%f, cycle:%f",
             dist, cur, tar, ret, dec, time_cycle);
    return ret;
  } else {
    // T Curve acceleration
    auto v = math::Clamp(std::fabs(cur) + acc * time_cycle, std::fabs(min_v),
                         std::fabs(max_v));
    auto optimal_v =
        math::Clamp(SpeedFormula<OPTIMAL>(std::fabs(cur), std::fabs(tar), acc,
                                          dec, std::fabs(dist))
                        .get(),
                    std::fabs(min_v), std::fabs(max_v));
    auto final_v = math::Sign(dist) * std::min(v, optimal_v);
    LOG_WARN("acceleration: dist:%f, cur:%f, tar:%f, set:%f, acc:%f, cycle:%f",
             dist, cur, tar, final_v, acc, time_cycle);
    return final_v;
  }
}

}  // namespace common

#endif  // amr_COMMON_INCLUDE_amr_COMMON_COMMON_H_

/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_MATH_H_
#define amr_COMMON_INCLUDE_amr_COMMON_MATH_H_
#include <angles/angles.h>

#include <cmath>

#include "amr_common/geometry/line.h"
#include "eigen3/Eigen/Dense"

namespace math {
using amr_geometry::Line;
using amr_geometry::Point;
using amr_geometry::Pose;

#define PI 3.141593f

template <typename T>
T GetNumPoint(const T& num) {
  return num - static_cast<int>(num);
}
template <typename T>
int Rount2Int(const T& num) {
  return std::lround(num);
}

template <typename T>
T Deg2Rad(const T& deg) {
  return deg / 180.0 * M_PI;
}

template <typename T>
T RPLocalization(const T& abs_pos1, const T& abs_pos2, const T& rl_pos1,
                 const T& rl_pos2) {
  T pos_to_localize = (rl_pos2 * abs_pos1) / (rl_pos2 - rl_pos1) +
                      (rl_pos1 * abs_pos2) / (rl_pos1 - rl_pos2);
  return pos_to_localize;
}

template <typename T>
T Rad2Deg(const T& rad) {
  return rad / M_PI * 180.0;
}

template <typename T>
T Meter2Millimeter(const T& meter) {
  return meter * 1000.0;
}

template <typename T>
T Meter2CMillimeter(const T& meter) {
  return meter * 10000.0;
}
// 拉线传感器读数乘以0.0001转换成米
template <typename T>
T ZeroPointMM2Meter(const T& mmillimeter) {
  return mmillimeter * 0.0001;
}

template <typename T>
T Millimeter2Meter(const T& millimeter) {
  return millimeter / 1000.0;
}

template <typename T>
T Meter2Centimeter(const T& meter) {
  return meter * 100.0;
}

template <typename T>
T Centimeter2meter(const T& centimeter) {
  return centimeter / 100.0;
}

template <typename T>
T Meter2Decimetre(const T& meter) {
  return meter * 10.0;
}

template <typename T>
T Decimetre2meter(const T& decimetre) {
  return decimetre / 10.0;
}

template <typename T>
bool NearZero(const T& val, double bound = 0.0001) {
  return std::fabs(static_cast<double>(val)) <= bound;
}

template <typename T>
double CosineTheorem(T r1, T r2, T angle) {
  return std::sqrt(std::pow(r1, 2) + std::pow(r2, 2) -
                   2 * r1 * r2 * std::cos(angle));
}

template <typename T>
int Sign(T val) {
  return 0 == static_cast<double>(val) ? 0 : (val > 0 ? 1 : -1);
}

template <typename T>
double GetVectorRad(T x, T y) {
  double l = std::sqrt(x * x + y * y);
  double ret = std::acos(x / l);
  if (y < 0) ret = -ret;
  return ret;
}

template <typename T>
int InsertGap(T val, int gap = 2000) {
  return static_cast<int>(val) + gap;
}

template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

/**
 * \brief get the distance between point1 and point2
 * \param pt1 the current point
 * \param pt2 the target point
 * \return the distance
 */
double GetDistance(const Point& pt1, const Point& pt2);
/**
 * \brief get the distance between  pose1 and point2
 * \param pose1 the current pose
 * \param pt2 the target point
 * \return the distance
 */
double GetDistance(const Pose& pose1, const Point& pt2);
/**
 * \brief get the distance between  point1 and pose2
 * \param pt1 the current point
 * \param pose2 the target pose
 * \return the distance
 */
double GetDistance(const Point& pt1, const Pose& pose2);
/**
 * \brief get the distance between  pose1 and pose2
 * \param pose1 the current pose
 * \param pose2 the target pose
 * \return the distance
 */
double GetDistance(const Pose& pose1, const Pose& pose2);

/**
 * \brief get the spin angle
 * \param pose1 the current pose
 * \return the spin angle
 */
double GetAngelSpin(const Pose& fromPose, const Pose& toPose);

/**
 * \brief get Vertical dist between pose1 and pose2
 * \param pose1 current pose
 * \param pose2 target pose
 * \return dist
 */
double GetVerticalDist(const Pose& pose1, const Pose& pose2);

/**
 * \brief get signed dist between pose1 and pose2
 * \param pose1 current pose
 * \param pose2 target pose
 * \return dist
 */
double GetSignedDistance2Pose(const Pose& pose1, const Pose& pose2);

/**
 * \brief get Projecting dist between pose1 and pose2
 * \param pose1 current pose
 * \param pose2 target pose
 * \return dist
 */
double ProjectingDistance(const Pose& pose1, const Pose& pose2);

/**
 * \brief get Intersect point between pose1 and pose2
 * \param pose1 current pose
 * \param pose2 target pose
 * \return Intersect point
 */
Point Intersect(const Pose& pose1, const Pose& pose2);

/**
 * \brief get Centerpoint between pose1 and pose2
 * \param pose1 current pose
 * \param pose2 target pose
 * \return Centerpoint
 */
Point GetCenterPoint(const Pose& pose1, const Pose& pose2);

/**
 * \brief get current navigation dist-bias according to the route
 * \param pose current pose of the robot
 * \param line route of the robot
 * \return dist-bias of current pose to route
 */
double GetDistFromLineAndPose(const Line& line, const Pose& pose);

/**
 * \brief get whethe the two value nearby diff range
 * \param left-high right-low diff-range
 * \return status
 */
bool Nearby(double left, double right, double diff);

/**
 * \brief get angle of the value cover direction
 * \param angle value
 * \return direction angle
 */
double Direction2Angle(double angle);

/**
 * \brief get fix angle with angle value continuity
 * \param current_angle current robot angle
 * \param raw_angle     measurement angle
 * \return fix angle
 */
double GetFixEKFAngle(double current_angle, double raw_angle);

/**
 * \brief accord laser pose to compute robot pose
 * \param laser_pose     laser pose
 * \param install_x      laser install x
 * \param install_y      laser install y
 * \param install_theta  laser install theta
 * \return robot pose
 */
Pose LaserPoseToRobotPose(const Pose& laser_pose, const double& install_x,
                          const double& install_y, const double& install_theta);

/**
 * \brief divide valsue with sub part
 * \param origin     origin num
 * \param sub      divide sub value
 * \return robot pose
 */
int GetNumDivideIndex(const double& origin, const double& sub);

double FilterLowPass(const double& k, const double& taret_value,
                     const double& cmd_value);

// 方向取反 + 归一化到 (-π, π]
double ReverseRad(double angle_rad);
}  // namespace math

#endif  // amr_COMMON_INCLUDE_amr_COMMON_MATH_H_

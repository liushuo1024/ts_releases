/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_LINE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_LINE_H_
#include <cmath>

#include "amr_common/geometry/pose.h"

namespace amr_geometry {

/**
 * \class Line
 * \brief a line is described as y = kx + b in world-coordinate system
 */
class Line {
 public:
  /**
   * \brief contruct a line: y = kx + b
   */
  Line(double a, double b, double c) : a_(a), b_(b), c_(c) {}

  /**
   * \brief construct a line with one point and its angle of world-coordinate
   * \param pt point through line
   * \param angle line angle(rad) in world-coordinate-system
   */
  Line(const Point& pt1, double angle) {
    if (std::fabs(std::cos(static_cast<float>(angle)) < 1e-6)) {
      a_ = 1.0;
      b_ = 0.0;
      c_ = -pt1.x();
    } else if (std::fabs(std::sin(static_cast<float>(angle)) < 1e-6)) {
      a_ = 0.0;
      b_ = 1.0;
      c_ = -pt1.y();
    } else {
      a_ = std::tan(static_cast<float>(angle));
      b_ = -1.0;
      c_ = pt1.y() - pt1.x() * a_;
    }
  }

  /**
   * \brief contruct a line with two points
   * \param begin one of the two points
   * \param end one of the two points
   */
  Line(const Point& pt1, const Point& pt2) {
    a_ = pt2.y() - pt1.y();
    b_ = pt1.x() - pt2.x();
    c_ = pt2.x() * pt1.y() - pt1.x() * pt2.y();
  }

  /**
   * \brief a setter
   * \param a line-a
   */
  inline void SetA(double a) { a_ = a; }

  /**
   * \brief b setter
   * \param b line-b
   */
  inline void SetB(double b) { b_ = b; }

  /**
   * \brief c setter
   * \param c line-c
   */
  inline void SetC(double c) { c_ = c; }

  /**
   * \brief a getter
   */
  inline const double a() const { return a_; }

  /**
   * \brief b getter
   */
  inline const double b() const { return b_; }

  /**
   * \brief c getter
   */
  inline const double c() const { return c_; }

  /**
   * \brief get current navigation dist-bias according to the route
   * \param pose current pose of the robot
   * \return dist-bias of current pose to route
   */
  double GetDistFromLineAndPose(const Pose& pose) {
    return std::abs((a_ * pose.point().x() + b_ * pose.point().y() + c_) /
                    std::hypot(a_, b_));
  }

 private:
  // line equation: ax + by + c = 0;
  double a_;
  double b_;
  double c_;
};

}  // namespace amr_geometry

#endif  // amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_LINE_H_

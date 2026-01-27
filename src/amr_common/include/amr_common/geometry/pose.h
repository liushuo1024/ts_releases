/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POSE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POSE_H_
#include <cmath>

#include "amr_common/geometry/point.h"

namespace amr_geometry {

/**
 * \class Pose
 */
class Pose {
 public:
  /**
   * \brief default constructor, create an instance with point(0,0) and yaw(0)
   */
  Pose() : point_(0.0, 0.0), yaw_(0.0) {}

  /**
   * \brief construct a pose with (x, y, yaw)
   * \param x coordinate x
   * \param y coordinate y
   * \param yaw coordinate yaw
   */
  Pose(double x, double y, double yaw) : point_(x, y), yaw_(yaw) {}

  /**
   * \brief construct a pose with point p and yaw
   * \param p a point
   * \param yaw pose yaw
   */
  Pose(Point p, double yaw) : point_(p), yaw_(yaw) {}

  /**
   * \brief default copy constructor
   * \param another pose
   */
  Pose(const Pose &pose) : point_(pose.point_), yaw_(pose.yaw_) {}

  /**
   * \brief assign self with another pose
   * \param another pose
   * \return self
   */
  Pose &operator=(const Pose &pose) {
    if (this == &pose) {
      return *this;
    }

    this->point_ = pose.point_;
    this->yaw_ = pose.yaw_;
    return *this;
  }

  /**
   * \brief set pose by (x, y, yaw)
   * \param x point-x
   * \param y point-y
   * \param yaw point-yaw
   */
  inline void set(double x, double y, double yaw) {
    point_.set_x(x);
    point_.set_y(y);
    yaw_ = yaw;
  }

  /**
   * \brief point setter
   * \param p point to set
   */
  inline void set_point(Point p) { point_ = p; }

  /**
   * \brief yaw setter
   * \param yaw yaw to set
   */
  inline void set_yaw(double yaw) { yaw_ = yaw; }

  /**
   * \brief point getter
   * \return self point
   */
  inline const Point point() const { return point_; }

  /**
   * \brief point.x getter
   */
  inline const double x() const { return point_.x(); }

  /**
   * \brief point.y getter
   */
  inline const double y() const { return point_.y(); }

  /**
   * \brief yaw getter
   * \return self yaw
   */
  inline const double yaw() const { return yaw_; }

  /**
   * \brief theta getter
   * \return self yaw
   */
  inline const double theta() const { return yaw_; }

 private:
  Point point_;
  double yaw_;
};

}  // namespace amr_geometry

#endif  // amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POSE_H_

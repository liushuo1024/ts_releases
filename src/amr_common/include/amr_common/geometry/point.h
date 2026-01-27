/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POINT_H_
#define amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POINT_H_
#include <cmath>
#include <eigen3/Eigen/Geometry>
namespace amr_geometry {

/**
 * \class Point
 */
class Point {
 public:
  /**
   * \brief default constructor, init a point with coordinate(0,0)
   */
  Point() : x_(0.0), y_(0.0) {}

  /**
   * \brief construct a point with coordinate(x,y)
   * \param x x-coordinate
   * \param y y-coordinate
   */
  Point(double x, double y) : x_(x), y_(y) {}

  /**
   * \brief copy constructor
   * \param p another Point type instance
   */
  Point(const Point &p) : x_(p.x_), y_(p.y_) {}

  /**
   * \brief assign current point values with a new point
   * \param p another Point type instance
   */
  Point &operator=(const Point &p) {
    if (this == &p) {
      return *this;
    }

    this->x_ = p.x_;
    this->y_ = p.y_;
    return *this;
  }

  bool operator==(const Point &p) {
    if ((this->x_ == p.x()) && (this->y_ == p.y())) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * \brief x setter
   * \param x x-coordinate
   */
  inline void set_x(double x) { x_ = x; }

  /**
   * \brief y setter
   * \param y y-coordinate
   */
  inline void set_y(double y) { y_ = y; }

  /**
   * \brief x,y setter
   * \param x x-coordinate
   * \param y y-coordinate
   */
  inline void set(double x, double y) {
    x_ = x;
    y_ = y;
  }

  /**
   * \brief x getter
   * \return x-coordinate
   */
  inline const double x() const { return x_; }

  /**
   * \brief y getter
   * \return y-coordinate
   */
  inline const double y() const { return y_; }

    /**
   * \brief get the distance from current point to another point
   * \param point the target point to calculate distanc
   * \return the distance
   */
  inline const double getDistance(Point point) const {
    return std::hypot(point.x() - x_, point.y() - y_);
  }

  double GetThetatoPoint(Point pt) {
    return std::atan2(pt.y() - this->y(), pt.x() - this->x());
  }

  
  Point getTransitionalPoint(const Point &pt, float theta,
                             float radius) const {
    auto x =
        x_ - (x_ - pt.x()) * radius * std::tan(theta / 2) / getDistance(pt);
    auto y =
        y_ - (y_ - pt.y()) * radius * std::tan(theta / 2) / getDistance(pt);
    return Point(x, y);
  }

 private:
  double x_;
  double y_;
};

}  // namespace amr_geometry

#endif  // amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POINT_H_

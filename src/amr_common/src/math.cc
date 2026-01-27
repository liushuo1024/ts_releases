/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */

#include "amr_common/math.h"

namespace math {

double GetDistance(const Point& pt1, const Point& pt2) {
  return std::hypot(pt2.x() - pt1.x(), pt2.y() - pt1.y());
}

double GetDistance(const Pose& pose1, const Point& pt2) {
  return std::hypot(pt2.x() - pose1.x(), pt2.y() - pose1.y());
}

double GetDistance(const Point& pt1, const Pose& pose2) {
  return std::hypot(pt1.x() - pose2.x(), pt1.y() - pose2.y());
}

double GetDistance(const Pose& pose1, const Pose& pose2) {
  return std::hypot(pose1.x() - pose2.x(), pose1.y() - pose2.y());
}

double GetAngelSpin(const Pose& fromPose, const Pose& toPose) {
  double t_1 =
      atan2f(-(fromPose.y() - toPose.y()), -(fromPose.x() - toPose.x()));
  return angles::shortest_angular_distance(fromPose.yaw(), t_1);
  // double angle = t_1 - fromPose.yaw();
  // angle = fmod(fmod(angle, 2.0f * PI) + 2.0f * PI, 2.0f * PI);
  // if (angle > PI) angle -= 2.0f * PI;
  // return angle;
}

double GetVerticalDist(const Pose& pose1, const Pose& pose2) {
  Point startPoint = pose2.point();
  Point endPoint(startPoint.x() + std::cos(static_cast<float>(pose2.yaw())),
                 startPoint.y() + std::sin(static_cast<float>(pose2.yaw())));

  double triangleArea = ((startPoint.x() - pose1.point().x()) *
                             (endPoint.y() - pose1.point().y()) -
                         (startPoint.y() - pose1.point().y()) *
                             (endPoint.x() - pose1.point().x())) /
                        2.0;
  double a = endPoint.y() - startPoint.y();
  double b = startPoint.x() - endPoint.x();
  double c = endPoint.x() * startPoint.y() - startPoint.x() * endPoint.y();
  if (triangleArea <= 0) {
    return -std::abs((a * pose1.point().x() + b * pose1.point().y() + c) /
                     std::hypot(a, b));
  } else {
    return std::abs((a * pose1.point().x() + b * pose1.point().y() + c) /
                    std::hypot(a, b));
  }
}

double GetSignedDistance2Pose(const Pose& pose1, const Pose& pose2) {
  Eigen::Vector2d self(std::cos(static_cast<float>(pose1.yaw())),
                       std::sin(static_cast<float>(pose1.yaw())));
  Eigen::Vector2d connection(pose2.x() - pose1.x(), pose2.y() - pose1.y());
  auto sign = self.dot(connection) >= 0. ? 1. : -1.;

  return sign * std::hypot(pose2.y() - pose1.y(), pose2.x() - pose1.x());
}

double ProjectingDistance(const Pose& pose1, const Pose& pose2) {
  Eigen::Vector2d next_yaw(std::cos(pose2.yaw()), std::sin(pose2.yaw()));
  Eigen::Vector2d next2pose(pose1.x() - pose2.x(), pose1.y() - pose2.y());
  return -next2pose.dot(next_yaw);
}

Point Intersect(const Pose& pose1, const Pose& pose2) {
  double a1 = std::sin(static_cast<float>(pose1.yaw()));
  double b1 = -std::cos(static_cast<float>(pose1.yaw()));
  double c1 = -a1 * pose1.x() - b1 * pose1.y();

  double a2 = std::sin(static_cast<float>(pose2.yaw()));
  double b2 = -std::cos(static_cast<float>(pose2.yaw()));
  double c2 = -a2 * pose2.x() - b2 * pose2.y();

  return Point((b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1),
               (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2));
}

Point GetCenterPoint(const Pose& pose1, const Pose& pose2) {
  double a1 = std::cos(static_cast<float>(pose1.yaw()));
  double b1 = std::sin(static_cast<float>(pose1.yaw()));
  double c1 = -a1 * pose1.x() - b1 * pose1.y();

  double a2 = std::cos(static_cast<float>(pose2.yaw()));
  double b2 = std::sin(static_cast<float>(pose2.yaw()));
  double c2 = -a2 * pose2.x() - b2 * pose2.y();

  auto x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
  auto y = (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2);

  return Point(x, y);
}

double GetDistFromLineAndPose(const Line& line, const Pose& pose) {
  return std::abs(
      (line.a() * pose.point().x() + line.b() * pose.point().y() + line.c()) /
      std::hypot(line.a(), line.b()));
}

bool Nearby(double left, double right, double diff) {
  return fabs(left - right) <= diff ? true : false;
}

double Direction2Angle(double angle) {
  if (Nearby(angle, 0, PI / 4)) {
    return 0.00f;
  }
  if (Nearby(angle, PI / 2, PI / 4)) {
    return PI / 2;
  }
  if (Nearby(angle, -PI / 2, PI / 4)) {
    return -PI / 2;
  }
  return PI;
}

double GetFixEKFAngle(double current_angle, double raw_angle) {
  auto normalize_current = angles::normalize_angle(current_angle);
  auto normalize_raw = angles::normalize_angle(raw_angle);
  if (std::abs(normalize_current) > PI / 2 &&
      std::abs(normalize_raw) > PI / 2 &&
      normalize_current * normalize_raw < 0) {
    return Sign(current_angle) * (2 * PI - std::abs(normalize_raw));
  }
  return normalize_raw;
}

Pose LaserPoseToRobotPose(const Pose& laser_pose, const double& install_x,
                          const double& install_y,
                          const double& install_theta) {
  Eigen::Vector2d laser_coordinate(laser_pose.x(), laser_pose.y());
  Eigen::Rotation2D<double> laser_rotation(laser_pose.theta() - install_theta);
  Eigen::Vector2d laser_installation(install_x, install_y);
  Eigen::Vector2d robot_pose =
      laser_coordinate - laser_rotation.toRotationMatrix() * laser_installation;
  return Pose(robot_pose.x(), robot_pose.y(),
              laser_pose.theta() - install_theta);
}

int GetNumDivideIndex(const double& origin, const double& sub) {
  return std::abs(math::GetNumPoint<double>(origin / sub)) <= 0.5
             ? static_cast<int>(origin / sub - 1)
             : static_cast<int>(origin / sub - 1) + 1;
}

double FilterLowPass(const double& k, const double& taret_value,
                     const double& cmd_value) {
  double tempD = 0.0, temp_sum1 = 0.0, temp_sum2 = 0.0;
  tempD = 1.0 / (k + 1.0);
  temp_sum1 = (1.0 - tempD) * cmd_value;
  temp_sum2 = tempD * taret_value;
  return temp_sum1 + temp_sum2;
}
double ReverseRad(double angle_rad) {
    // 将角度归一化到[0, 2π)
    angle_rad = fmod(angle_rad, 2 * M_PI);
    if (angle_rad < 0) {
        angle_rad += 2 * M_PI;
    }
    
    // 反转180度
    double reversed = angle_rad + M_PI;
    
    // 归一化到[-π, π]
    reversed = fmod(reversed, 2 * M_PI);
    if (reversed > M_PI) {
        reversed -= 2 * M_PI;
    } else if (reversed <= -M_PI) {
        reversed += 2 * M_PI;
    }
    
    return reversed;
}
}  // namespace math

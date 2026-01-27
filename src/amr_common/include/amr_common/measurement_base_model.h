/**
 * Copyright (c) 2019 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_MEASUREMENT_BASE_MODEL_H_
#define amr_COMMON_INCLUDE_amr_COMMON_MEASUREMENT_BASE_MODEL_H_

#include <ros/ros.h>

#include <cmath>
#include <deque>
#include <memory>

#include "amr_common/kalman/LinearizedMeasurementModel.hpp"


#include "amr_common/system_model.h"

namespace amr_common {

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template <typename T>
class PoseMeasurement : public Kalman::Vector<T, 3> {
 public:
  KALMAN_VECTOR(PoseMeasurement, T, 3)

  static constexpr size_t kX = 0;
  static constexpr size_t kY = 1;
  static constexpr size_t kTheta = 2;

  T x() const { return (*this)[kX]; }
  T y() const { return (*this)[kY]; }
  T theta() const { return (*this)[kTheta]; }

  T &x() { return (*this)[kX]; }
  T &y() { return (*this)[kY]; }
  T &theta() { return (*this)[kTheta]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are
 * known. The robot can measure the direct distance to both the landmarks, for
 * instance through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance
 * representation (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template <typename T,
          template <class> class CovarianceBase = Kalman::StandardBase>
class MeasurementBaseModel
    : public Kalman::LinearizedMeasurementModel<State<T>, PoseMeasurement<T>,
                                                CovarianceBase> {
 public:
  //! State type shortcut definition
  typedef State<T> S;

  //! Measurement type shortcut definition
  typedef PoseMeasurement<T> M;

  MeasurementBaseModel() {
    // Setup noise jacobian. As this one is static, we can define it once
    // and do not need to update it dynamically
    this->V.setIdentity();
  }

  void setNoise(T m_dev_x, T m_dev_y, T m_dev_theta) {
    this->V(0, 0) = m_dev_x;
    this->V(1, 1) = m_dev_y;
    this->V(2, 2) = m_dev_theta;
  }

  inline void SetTransform(const State<T> &transform) {
    transform_ = transform;
  }

  /**
   * @brief Definition of (possibly non-linear) measurement function
   *
   * This function maps the system state to the measurement that is expected
   * to be received from the sensor assuming the system is currently in the
   * estimated state.
   *
   * @param [in] x The system state in current time-step
   * @returns The (predicted) sensor measurement for the system state
   */

  M h(const S &x) const {
    M measurement;
    measurement << x.x(), x.y(), angles::normalize_angle(x.theta());
    return measurement;
  }

  inline const Kalman::Vector<T, 3> transform() const { return transform_; }

 protected:
  /**
   * @brief Update jacobian matrices for the system state transition function
   * using current state
   *
   * This will re-compute the (state-dependent) elements of the jacobian
   * matrices to linearize the non-linear measurement function \f$h(x)\f$ around
   * the current state \f$x\f$.
   *
   * @note This is only needed when implementing a LinearizedSystemModel,
   *       for usage with an ExtendedKalmanFilter or
   * SquareRootExtendedKalmanFilter. When using a fully non-linear filter such
   * as the UnscentedKalmanFilter or its square-root form then this is not
   * needed.
   *
   * @param x The current system state around which to linearize
   * @param u The current system control input
   */

  void updateJacobians(const S &x) {
    // H = dh/dx (Jacobian of measurement function w.r.t. the state)
    this->H.setIdentity();
  }

  Kalman::Vector<T, 3> transform_;
};

}  // namespace amr_common
#endif  // amr_COMMON_INCLUDE_amr_COMMON_MEASUREMENT_BASE_MODEL_H_

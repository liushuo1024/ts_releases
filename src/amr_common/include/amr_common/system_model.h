/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_SYSTEM_MODEL_H_
#define amr_COMMON_INCLUDE_amr_COMMON_SYSTEM_MODEL_H_
#include <angles/angles.h>

#include "amr_common/kalman/LinearizedSystemModel.hpp"
namespace amr_common {
/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template <typename T>
class State : public Kalman::Vector<T, 3> {
 public:
  KALMAN_VECTOR(State, T, 3)

  //! X-position
  static constexpr size_t X = 0;
  //! Y-Position
  static constexpr size_t Y = 1;
  //! Orientation
  static constexpr size_t THETA = 2;

  inline T x() const { return (*this)[X]; }
  inline T y() const { return (*this)[Y]; }
  inline T theta() const { return (*this)[THETA]; }

  inline T& x() { return (*this)[X]; }
  inline T& y() { return (*this)[Y]; }
  inline T& theta() { return (*this)[THETA]; }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template <typename T>
class Control : public Kalman::Vector<T, 3> {
 public:
  KALMAN_VECTOR(Control, T, 3)
  //! Velocity
  static constexpr size_t V = 0;
  //! Angular Rate (Orientation-change)
  static constexpr size_t OMEGA = 1;
  //! Cycle time
  static constexpr size_t DT = 2;

  inline T v() const { return (*this)[V]; }
  inline T omega() const { return (*this)[OMEGA]; }
  inline T dt() const { return (*this)[DT]; }

  inline T& v() { return (*this)[V]; }
  inline T& omega() { return (*this)[OMEGA]; }
  inline T& dt() { return (*this)[DT]; }
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance
 * representation (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template <typename T,
          template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>,
                                                         CovarianceBase> {
 public:
  //! State type shortcut definition
  typedef State<T> S;

  //! Control type shortcut definition
  typedef Control<T> C;

  /**
   * @brief Definition of (non-linear) state transition function
   *
   * This function defines how the system state is propagated through time,
   * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
   * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$
   * and the system control input \f$u\f$.
   *
   * @param [in] x The system state in current time-step
   * @param [in] u The control vector input
   * @returns The (predicted) system state in the next time-step
   */
  S f(const S& x, const C& u) const {
    //! Predicted state vector after transition
    S x_;
    // New orientation given by old orientation plus orientation change
    x_.theta() = angles::normalize_angle(x.theta() + u.omega() * u.dt());
    // New x-position given by old x-position plus change in x-direction
    // Change in x-direction is given by the cosine of the (new) orientation
    // times the velocity
    x_.x() = x.x() + cos(x_.theta()) * u.v() * u.dt();
    x_.y() = x.y() + sin(x_.theta()) * u.v() * u.dt();
    // Return transitioned state vector
    return x_;
  }

  void setSystemNoise(T dev_x, T dev_y, T dev_theta) {
    this->W.setIdentity();
    this->W(0, 0) = dev_x;
    this->W(1, 1) = dev_y;
    this->W(2, 2) = dev_theta;
  }

 protected:
  /**
   * @brief Update jacobian matrices for the system state transition function
   * using current state
   *
   * This will re-compute the (state-dependent) elements of the jacobian
   * matrices to linearize the non-linear state transition function \f$f(x,u)\f$
   * around the current state \f$x\f$.
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
  void updateJacobians(const S& x, const C& u) {
    // F = df/dx (Jacobian of state transition w.r.t. the state)
    this->F.setZero();
    // partial derivative of x.x() w.r.t. x.x()
    this->F(S::X, S::X) = 1;
    // partial derivative of x.x() w.r.t. x.theta()
    this->F(S::X, S::THETA) =
        -std::sin(x.theta() + u.omega() * u.dt()) * u.v() * u.dt();
    // partial derivative of x.y() w.r.t. x.y()
    this->F(S::Y, S::Y) = 1;
    // partial derivative of x.y() w.r.t. x.theta()
    this->F(S::Y, S::THETA) =
        std::cos(x.theta() + u.omega() * u.dt()) * u.v() * u.dt();
    // partial derivative of x.theta() w.r.t. x.theta()
    this->F(S::THETA, S::THETA) = 1;
  }
};
}  // namespace amr_common
#endif  // amr_COMMON_INCLUDE_amr_COMMON_SYSTEM_MODEL_H_

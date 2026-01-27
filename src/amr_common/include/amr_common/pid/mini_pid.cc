/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#include "amr_common/pid/mini_pid.h"

#include <ros/ros.h>

MiniPID::MiniPID(double p, double i, double d) {
  init();
  P_ = p;
  I_ = i;
  D_ = d;
}

MiniPID::MiniPID(double p, double i, double d, double f) {
  init();
  P_ = p;
  I_ = i;
  D_ = d;
  F_ = f;
}

void MiniPID::init() {
  P_ = 0;
  I_ = 0;
  D_ = 0;
  F_ = 0;

  maxIOutput_ = 0;
  maxError_ = 0;
  errorSum_ = 0;
  maxOutput_ = 0;
  minOutput_ = 0;
  setpoint_ = 0;
  lastActual_ = 0;
  firstRun_ = true;
  reversed_ = false;
  outputRampRate_ = 0;
  lastOutput_ = 0;
  outputFilter_ = 0;
  setpointRange_ = 0;
}

//**********************************
// Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter. <br>
 * this->responds quicly to changes in setpoint_, and provides most of the
 * initial driving force to make corrections. <br> Some systems can be used with
 * only a P_ gain, and many can be operated with only PI.<br> For position based
 * controllers, this->is the first parameter to tune, with I_ second. <br> For
 * rate controlled systems, this->is often the second after F_.
 *
 * @param p Proportional gain. Affects output according to
 * <b>output+=P_*(setpoint_-current_value)</b>
 */
void MiniPID::setP(double p) {
  P_ = p;
  checkSigns();
}

/**
 * Changes the I_ parameter <br>
 * this->is used for overcoming disturbances, and ensuring that the controller
 * always gets to the control mode. Typically tuned second for "Position" based
 * modes, and third for "Rate" or continuous based modes. <br> Affects output
 * through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
 *
 * @see {@link #setMaxIOutput(double) setMaxIOutput} for how to restrict
 *
 * @param i New gain value for the Integral term
 */
void MiniPID::setI(double i) {
  if (I_ != 0) {
    errorSum_ = errorSum_ * I_ / i;
  }
  if (maxIOutput_ != 0) {
    maxError_ = maxIOutput_ / i;
  }
  I_ = i;
  checkSigns();
  /* Implementation note:
   * this->Scales the accumulated error to avoid output errors.
   * As an example doubling the I_ term cuts the accumulated error in half,
   * which results in the output change due to the I_ term constant during the
   * transition.
   *
   */
}

void MiniPID::setD(double d) {
  D_ = d;
  checkSigns();
}

/**Configure the FeedForward parameter. <br>
 * this->is excellent for Velocity, rate, and other	continuous control modes
 * where you can expect a rough output value based solely on the setpoint_.<br>
 * Should not be used in "position" based control modes.
 *
 * @param f Feed forward gain. Affects output according to
 * <b>output+=F_*Setpoint</b>;
 */
void MiniPID::setF(double f) {
  F_ = f;
  checkSigns();
}

/** Create a new PID object.
 * @param p Proportional gain. Large if large difference between setpoint_ and
 * target.
 * @param i Integral gain.	Becomes large if setpoint_ cannot reach target
 * quickly.
 * @param d Derivative gain. Responds quickly to large changes in error. Small
 * values prevents P_ and I_ terms from causing overshoot.
 */
void MiniPID::setPID(double p, double i, double d) {
  P_ = p;
  I_ = i;
  D_ = d;
  checkSigns();
}

void MiniPID::setPID(double p, double i, double d, double f) {
  P_ = p;
  I_ = i;
  D_ = d;
  F_ = f;
  checkSigns();
}

/**Set the maximum output value contributed by the I_ component of the system
 * this->can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void MiniPID::setMaxIOutput(double maximum) {
  /* Internally maxError_ and Izone are similar, but scaled for different
   * purposes. The maxError_ is generated for simplifying math, since
   * calculations against the max error are far more common than changing the I_
   * term or Izone.
   */
  maxIOutput_ = maximum;
  if (I_ != 0) {
    maxError_ = std::fabs(maxIOutput_ / I_);
  }
}

/**Specify a maximum output. If a single parameter is specified, the minimum is
 * set to (-maximum).
 * @param output
 */
void MiniPID::setOutputLimits(double output) {
  setOutputLimits(-output, output);
}

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void MiniPID::setOutputLimits(double minimum, double maximum) {
  if (maximum < minimum) return;
  maxOutput_ = maximum;
  minOutput_ = minimum;

  // Ensure the bounds of the I_ term are within the bounds of the allowable
  // output swing
  if (maxIOutput_ == 0 || maxIOutput_ > (maximum - minimum)) {
    setMaxIOutput(maximum - minimum);
  }
}

/** Set the operating direction of the PID controller
 * @param reversed_ Set true to reverse PID output
 */
void MiniPID::setDirection(bool reversed_) { this->reversed_ = reversed_; }

//**********************************
// Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint_
 */
void MiniPID::setSetpoint(double setpoint_) { this->setpoint_ = setpoint_; }

/** Calculate the PID value needed to hit the target setpoint_.
 * Automatically re-calculates the output at each call.
 * @param actual The monitored value
 * @param target The target value
 * @return calculated output value for driving the actual to the target
 */
double MiniPID::getOutput(double actual, double setpoint_) {
  double output;
  double Poutput;
  double Ioutput;
  double Doutput;
  double Foutput;

  this->setpoint_ = setpoint_;

  // Ramp the setpoint_ used for calculations if user has opted to do so
  if (setpointRange_ != 0) {
    setpoint_ =
        clamp(setpoint_, actual - setpointRange_, actual + setpointRange_);
  }

  // Do the simple parts of the calculations
  double error = setpoint_ - actual;
  
  // Calculate F_ output. Notice, this->depends only on the setpoint_, and not
  // the error.
  Foutput = F_ * setpoint_;

  // Calculate P_ term
  Poutput = P_ * error;

  // If this->is our first time running this-> we don't actually _have_ a
  // previous input or output. For sensor, sanely assume it was exactly where it
  // is now. For last output, we can assume it's the current time-independent
  // outputs.
  if (firstRun_) {
    lastActual_ = actual;
    lastOutput_ = Poutput + Foutput;
    firstRun_ = false;
  }

  // Calculate D_ Term
  // Note, this->is negative. this->actually "slows" the system if it's doing
  // the correct thing, and small values helps prevent output spikes and
  // overshoot

  Doutput = -D_ * (actual - lastActual_);
  lastActual_ = actual;

  // The Iterm is more complex. There's several things to factor in to make it
  // easier to deal with.
  // 1. maxIoutput restricts the amount of output contributed by the Iterm.
  // 2. prevent windup by not increasing errorSum_ if we're already running
  // against our max Ioutput
  // 3. prevent windup by not increasing errorSum_ if output is
  // output=maxOutput_
  Ioutput = I_ * errorSum_;
  if (maxIOutput_ != 0) {
    Ioutput = clamp(Ioutput, -maxIOutput_, maxIOutput_);
  }

  // And, finally, we can just add the terms up
  output = Foutput + Poutput + Ioutput + Doutput;
  LOG_DEBUG("Po: %lf, Io: %lf, Do: %lf, ErrorSum: %lf, error: %lf", Poutput,
            Ioutput, Doutput, errorSum_, error);

  // Figure out what we're doing with the error.
  if (minOutput_ != maxOutput_ && !bounded(output, minOutput_, maxOutput_)) {
    errorSum_ = error;
    LOG_DEBUG("errorSum_(%lf) = error!", errorSum_);
    // reset the error sum to a sane level
    // Setting to current error ensures a smooth transition when the P_ term
    // decreases enough for the I_ term to start acting upon the controller
    // From that point the I_ term will build up as would be expected
  } else if (outputRampRate_ != 0 &&
             !bounded(output, lastOutput_ - outputRampRate_,
                      lastOutput_ + outputRampRate_)) {
    errorSum_ = error;
    LOG_DEBUG("errorSum_(%lf) = error!", errorSum_);
  } else if (maxIOutput_ != 0) {
    errorSum_ = clamp(errorSum_ + error, -maxError_, maxError_);
    LOG_DEBUG("clamped errorSum_(%lf)! [%lf, %lf]", errorSum_, -maxError_,
              maxError_);
    // In addition to output limiting directly, we also want to prevent I_ term
    // buildup, so restrict the error directly
  } else {
    errorSum_ += error;
  }

  // Restrict output to our specified output and ramp limits
  if (outputRampRate_ != 0) {
    output = clamp(output, lastOutput_ - outputRampRate_,
                   lastOutput_ + outputRampRate_);
  }
  if (minOutput_ != maxOutput_) {
    output = clamp(output, minOutput_, maxOutput_);
  }
  if (outputFilter_ != 0) {
    output = lastOutput_ * outputFilter_ + output * (1 - outputFilter_);
  }

  lastOutput_ = output;
  return output;
}

/**
 * Calculates the PID value using the last provided setpoint_ and actual valuess
 * @return calculated output value for driving the actual to the target
 */
double MiniPID::getOutput() { return getOutput(lastActual_, setpoint_); }

/**
 *
 * @param actual
 * @return calculated output value for driving the actual to the target
 */
double MiniPID::getOutput(double actual) {
  return getOutput(actual, setpoint_);
}

/**
 *
 * @param actual
 * @return calculated output value for diff
 */
double MiniPID::getOutputFromDiff(double diff) { return getOutput(0.f, diff); }

/**
 * Resets the controller. this->erases the I_ term buildup, and removes D_ gain
 * on the next loop.
 */
void MiniPID::reset() {
  firstRun_ = true;
  errorSum_ = 0;
}

/**Set the maximum rate the output can increase per cycle.
 * @param rate
 */
void MiniPID::setOutputRampRate(double rate) { outputRampRate_ = rate; }

/** Set a limit on how far the setpoint_ can be from the current position
 * <br>Can simplify tuning by helping tuning over a small range applies to a
 * much larger range. <br>this->limits the reactivity of P_ term, and restricts
 * impact of large D_ term during large setpoint_ adjustments. Increases lag and
 * I_ term if range is too small.
 * @param range
 */
void MiniPID::setSetpointRange(double range) { setpointRange_ = range; }

/**Set a filter on the output to reduce sharp oscillations. <br>
 * 0.1 is likely a sane starting value. Larger values P_ and D_ oscillations,
 * but force larger I_ values. Uses an exponential rolling sum filter, according
 * to a simple <br> <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
 * @param output valid between [0..1), meaning [current output only.. historical
 * output only)
 */
void MiniPID::setOutputFilter(double strength) {
  if (strength == 0 || bounded(strength, 0, 1)) {
    outputFilter_ = strength;
  }
}

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise
 */
double MiniPID::clamp(double value, double min, double max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool MiniPID::bounded(double value, double min, double max) {
  return (min < value) && (value < max);
}

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed_ value
 */
void MiniPID::checkSigns() {
  if (reversed_) {  // all values should be below zero
    if (P_ > 0) P_ *= -1;
    if (I_ > 0) I_ *= -1;
    if (D_ > 0) D_ *= -1;
    if (F_ > 0) F_ *= -1;
  } else {  // all values should be above zero
    if (P_ < 0) P_ *= -1;
    if (I_ < 0) I_ *= -1;
    if (D_ < 0) D_ *= -1;
    if (F_ < 0) F_ *= -1;
  }
}

#pragma once

#include <Arduino.h>
#include <math.h>

#include "Config.h"

// The TB6612FNG channel driver lives in the shared MiniRobotBase library
// (MiniRobotMotors.h). Only the controller-specific PID is defined here.

// Fractional-order PID: PI^lambda D^mu, the same shape as the four-wheel
// controller so gains transfer between the two robots. The integral and
// derivative keep their classic form -- that is what gives a true pole at the
// origin, so a constant wheel-speed error is still integrated away to zero.
// The fractional order only reshapes what feeds them: the error passes through
// a Grunwald-Letnikov (GL) short-memory filter of the residual order
// (lambda-1) before the integral and (mu-1) before the derivative. At
// lambda == mu == 1.0 the GL filter is exactly the identity (order-0 binomial
// coefficients are 1,0,0,...), so the default configuration reduces to classic
// PID and takes the cheap path below.
struct WheelPID {
  static const uint8_t GL_MEMORY_LENGTH = 20;

  float kp;
  float ki;
  float kd;
  float lambda;
  float mu;
  float integral;
  float previousShapedError;
  float previousTarget;
  float errorHistory[GL_MEMORY_LENGTH];  // errorHistory[0] is the newest sample
  uint8_t historyLen;

  void configure(float p, float i, float d, float lam, float m) {
    kp = p;
    ki = i;
    kd = d;
    lambda = lam;
    mu = m;
    reset();
  }

  void reset() {
    integral = 0.0f;
    previousShapedError = 0.0f;
    previousTarget = 0.0f;
    for (uint8_t j = 0; j < GL_MEMORY_LENGTH; ++j) errorHistory[j] = 0.0f;
    historyLen = 0;
  }

  // GL binomial coefficient recursion: c0 = 1, cj = c(j-1) * (1 - (alpha+1)/j).
  // dtScale converts the unitless GL sum to physical units (dt^alpha).
  float glShape(float alpha, float dtScale) const {
    float coeff = 1.0f;
    float sum = errorHistory[0];
    for (uint8_t j = 1; j < historyLen; ++j) {
      coeff *= (1.0f - (alpha + 1.0f) / (float)j);
      sum += coeff * errorHistory[j];
    }
    return sum * dtScale;
  }

  // Order 1.0 means identity shaping, so skip both the 20-tap GL sum and the
  // pow() call. That is the default configuration and pow() is expensive on an
  // 8-bit AVR inside a 100 Hz loop.
  float shapedError(float order) const {
    if (fabs(order - 1.0f) < 1e-4f) return errorHistory[0];
    return glShape(order - 1.0f, pow(0.01f, 1.0f - order));
  }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Clear accumulated state when stopped or when wheel direction changes.
    if (fabs(target) < MIN_WHEEL_SPEED_MM_S || (target * previousTarget < 0.0f)) {
      reset();
    }
    previousTarget = target;

    const float error = target - measured;
    for (uint8_t j = GL_MEMORY_LENGTH - 1; j > 0; --j) errorHistory[j] = errorHistory[j - 1];
    errorHistory[0] = error;
    if (historyLen < GL_MEMORY_LENGTH) ++historyLen;

    integral += shapedError(lambda) * dt;
    // Anti-windup: cap the integral by its output contribution (ki*integral,
    // in PWM), not a fixed error-seconds bound. See MAX_INTEGRAL_PWM in
    // Config.h for why.
    if (ki > 0.0001f) {
      const float limit = MAX_INTEGRAL_PWM / ki;
      integral = constrain(integral, -limit, limit);
    } else {
      integral = constrain(integral, -INTEGRAL_FALLBACK_LIMIT, INTEGRAL_FALLBACK_LIMIT);
    }

    const float shapedForDerivative = shapedError(mu);
    const float derivative = (shapedForDerivative - previousShapedError) / dt;
    previousShapedError = shapedForDerivative;

    return kp * error + ki * integral + kd * derivative;
  }
};

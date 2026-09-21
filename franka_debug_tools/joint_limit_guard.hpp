#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// Torque resistance is an operating aid, not a safety-rated position constraint.
struct JointLimitGuard {
  std::array<double, 7> lower, upper, stiffness, damping, torque_cap;
  double activation, reserve, prediction;
  void validate() const {
    if (!std::isfinite(activation) || !std::isfinite(reserve) ||
        !std::isfinite(prediction) || reserve <= 0 || activation <= reserve || prediction < 0)
      throw std::invalid_argument("Invalid joint-limit margins");
    for (size_t i=0; i<7; ++i)
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
          upper[i]-lower[i] <= 2*activation || !std::isfinite(stiffness[i]) ||
          !std::isfinite(damping[i]) || !std::isfinite(torque_cap[i]) ||
          stiffness[i]<=0 || damping[i]<0 || torque_cap[i]<=0)
        throw std::invalid_argument("Invalid joint-limit configuration");
  }
  bool inside(size_t i, double q) const {
    // The inner reserve sets full resistance, not a stop boundary.
    return std::isfinite(q) && q > lower[i] && q < upper[i];
  }
  double torque(size_t i, double q, double dq) const {
    double result=0;
    for (int side : {-1, 1}) {
      const double distance = side > 0 ? upper[i]-q : q-lower[i];
      const double toward = std::max(0.0, side*dq);
      // Velocity preview starts braking earlier; it is not a stopping-distance guarantee.
      const double penetration = std::max(0.0, activation-distance+prediction*toward);
      const double x=std::clamp(penetration/(activation-reserve), 0.0, 1.0);
      const double blend=x*x*(3-2*x);
      result -= side*blend*(stiffness[i]*penetration+damping[i]*toward);
    }
    return std::clamp(result, -torque_cap[i], torque_cap[i]);
  }
};

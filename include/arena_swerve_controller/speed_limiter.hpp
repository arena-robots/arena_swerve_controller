// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#ifndef ARENA_SWERVE_CONTROLLER__SPEED_LIMITER_HPP_
#define ARENA_SWERVE_CONTROLLER__SPEED_LIMITER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

namespace arena_swerve_controller
{

class SpeedLimiter
{
public:
  SpeedLimiter() = default;

  SpeedLimiter(
    bool has_velocity_limits, double min_velocity, double max_velocity,
    bool has_acceleration_limits, double min_acceleration, double max_acceleration)
  : has_velocity_limits_(has_velocity_limits),
    has_acceleration_limits_(has_acceleration_limits)
  {
    std::tie(min_velocity_, max_velocity_) = bounds(min_velocity, max_velocity);
    std::tie(min_acceleration_, max_acceleration_) = bounds(min_acceleration, max_acceleration);
  }

  double limit(double velocity, double previous_velocity, double period) const
  {
    if (has_velocity_limits_) {
      velocity = std::clamp(velocity, min_velocity_, max_velocity_);
    }
    if (has_acceleration_limits_ && period > 0.0) {
      const double dv = std::clamp(
        velocity - previous_velocity, min_acceleration_ * period, max_acceleration_ * period);
      velocity = previous_velocity + dv;
    }
    return velocity;
  }

private:
  static std::pair<double, double> bounds(double min, double max)
  {
    const double inf = std::numeric_limits<double>::infinity();
    const double hi = std::isnan(max) ? inf : max;
    const double lo = std::isnan(min) ? -hi : min;
    return {lo, hi};
  }

  bool has_velocity_limits_{false};
  bool has_acceleration_limits_{false};
  double min_velocity_{0.0};
  double max_velocity_{0.0};
  double min_acceleration_{0.0};
  double max_acceleration_{0.0};
};

}  // namespace arena_swerve_controller

#endif  // ARENA_SWERVE_CONTROLLER__SPEED_LIMITER_HPP_

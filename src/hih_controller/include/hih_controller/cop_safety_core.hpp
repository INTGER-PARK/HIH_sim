#pragma once

// ROS-independent CoP boundary and Euler-offset rate integration helpers.
// e=cop-clamp(cop,-safe,+safe) [m]; rate=direction*gain*e [deg/s].
// The bounded rate integrates into an Euler reference offset [deg], not Body p/q/r.
// TRACK integrates usable samples; HOLD preserves the offset; RETURN_TO_ZERO
// returns disabled channels at their rate limit. There is no hidden integrator.
// Boundary defaults live in cop_boundary_defaults.hpp; role tuning lives in each
// CoP attitude node. Translation nodes also reuse the boundary and filter math.

#include <algorithm>
#include <cmath>
#include <string>

namespace hih_controller::cop_safety
{
inline double deadzoneError(double cop_m, double safe_m)
{
  // Signed excess beyond the boundary [m], not distance from the center.
  // |cop|<=safe: 0, cop>safe: cop-safe, cop<-safe: cop+safe.
  return cop_m - std::clamp(cop_m, -safe_m, safe_m);
}

inline bool hysteresisActive(
  bool was_active, double cop_m, double safe_m, double hysteresis_m)
{
  // OFF->ON: |cop|>safe. ON->OFF: |cop|<safe-hysteresis.
  // An active latch inside the boundary still produces zero integration rate.
  const double magnitude = std::abs(cop_m);
  const double release_m = std::max(0.0, safe_m - std::max(0.0, hysteresis_m));
  return was_active ? magnitude >= release_m : magnitude > safe_m;
}

inline double moveToward(double current, double target, double max_step)
{
  // Move at most max_step toward the target without overshooting.
  const double step = std::max(0.0, max_step);
  if (current < target) return std::min(current + step, target);
  return std::max(current - step, target);
}

enum class MotionMode {TRACK, HOLD, RETURN_TO_ZERO};

inline double angularRateFromErrorDegS(
  double error_m, double velocity_gain_deg_per_m_s,
  double direction, double rate_limit_deg_s)
{
  // Units: [deg/(m*s)]*[m]=[deg/s]; direction is normally +1 or -1.
  // The caller supplies the y-to-yaw / z-to-pitch correction sign.
  // Zero error must give exactly zero rate even with large finite gains.
  if (error_m == 0.0 || velocity_gain_deg_per_m_s == 0.0) return 0.0;
  return std::clamp(
    direction * velocity_gain_deg_per_m_s * error_m,
    -rate_limit_deg_s, rate_limit_deg_s);
}

inline double advanceOffsetDeg(
  double offset_deg, double rate_reference_deg_s, MotionMode mode,
  double dt, double offset_limit_deg, double rate_limit_deg_s)
{
  // Integrate once using the actual timer dt; skip invalid time intervals.
  if (!std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(offset_deg) || !std::isfinite(rate_reference_deg_s) ||
    !std::isfinite(offset_limit_deg) || offset_limit_deg <= 0.0 ||
    !std::isfinite(rate_limit_deg_s) || rate_limit_deg_s <= 0.0)
  {
    return offset_deg;
  }
  if (mode == MotionMode::HOLD) return offset_deg;
  if (mode == MotionMode::RETURN_TO_ZERO) {
    return moveToward(offset_deg, 0.0, rate_limit_deg_s * dt);
  }
  const double rate = std::clamp(
    rate_reference_deg_s, -rate_limit_deg_s, rate_limit_deg_s);
  // Clamp the stored state directly; do not keep an unbounded hidden integrator.
  return std::clamp(
    offset_deg + rate * dt, -offset_limit_deg, offset_limit_deg);
}

inline std::string removedParameterReason(const std::string & name)
{
  // Reject old angle-PD gains rather than silently changing their units.
  if (name == "yaw_kp_deg_per_m") return "use yaw_velocity_gain_deg_per_m_s [deg/(m*s)]";
  if (name == "pitch_kp_deg_per_m") return "use pitch_velocity_gain_deg_per_m_s [deg/(m*s)]";
  if (name == "yaw_kd_deg_per_mps" || name == "pitch_kd_deg_per_mps" ||
    name == "cop_y_dot_limit_mps" || name == "cop_z_dot_limit_mps")
  {
    return "angular-rate accumulation uses no D term or derivative limit";
  }
  return {};
}

inline double lpfAlpha(double cutoff_hz, double dt)
{
  // LPF: filtered += alpha*(raw-filtered), alpha=1-exp(-2*pi*fc*dt).
  // Bypass when fc<=0; the node handles input resets and freshness.
  if (!(dt > 0.0) || !std::isfinite(dt)) return 1.0;
  if (!(cutoff_hz > 0.0) || !std::isfinite(cutoff_hz)) return 1.0;
  constexpr double kPi = 3.14159265358979323846;
  return std::clamp(1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt), 0.0, 1.0);
}
}  // namespace hih_controller::cop_safety

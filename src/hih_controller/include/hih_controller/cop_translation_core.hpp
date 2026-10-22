#pragma once

// ROS-independent CoP translation math, separately instantiated by each role.
// e=cop-clamp(cop,-safe,+safe) [m]; v=clamp(direction*gain*e,+/-rate_limit) [m/s].
// Only the new tangent increment R_WB*[0,v_y,v_z]*dt is rotated into World.
// Previously accumulated World offsets do not rotate when attitude changes.
// Each y/z contribution has its own norm limit; the combined norm is bounded
// by hypot(y_limit,z_limit). Saturation stores only the attainable increment.
// TRACK integrates; HOLD preserves offsets; OFF returns contributions to zero.
// Axis settings receive shared boundaries from cop_boundary_defaults.hpp.
// The gain has units [1/s]; there is no position PD or derivative term.

#include "hih_controller/cop_safety_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace hih_controller::cop_translation
{
using Vec3 = std::array<double, 3>;

struct AxisSettings
{
  AxisSettings(double safe_boundary_m, double boundary_hysteresis_m)
  : safe_m(safe_boundary_m), hysteresis_m(boundary_hysteresis_m) {}

  double safe_m;                      // Plate-axis safety half-width [m], supplied from shared defaults.
  double hysteresis_m;                // Boundary-release hysteresis width [m], supplied from shared defaults.
  double velocity_gain_per_s{1.00};    // CoP excess-to-velocity gain [1/s].
  double direction{1.0};              // Correction sign/scale, normally +/-1; the final velocity is limited.
  double offset_limit_m{100.00};         // Norm limit of this channel's accumulated World contribution [m].
  double rate_limit_mps{0.05};         // Maximum TRACK and OFF-return speed [m/s].
};

inline std::string validationError(const AxisSettings & s)
{
  const std::array<double, 6> values{
    s.safe_m, s.hysteresis_m, s.velocity_gain_per_s, s.direction,
    s.offset_limit_m, s.rate_limit_mps};
  for (double value : values) {
    if (!std::isfinite(value)) return "all axis values must be finite";
  }
  if (s.safe_m < 0.0 || s.hysteresis_m < 0.0 || s.velocity_gain_per_s < 0.0) {
    return "safe boundary, hysteresis and velocity gain must be nonnegative";
  }
  if (s.hysteresis_m > s.safe_m) return "hysteresis must not exceed safe boundary";
  if (std::abs(s.direction) < 1e-9) return "direction must be nonzero";
  if (s.offset_limit_m <= 0.0 || s.rate_limit_mps <= 0.0) {
    return "offset and rate limits must be positive";
  }
  return {};
}

// Reject old position-PD parameters instead of silently changing their units.
// Nodes apply this validation to startup and runtime parameter requests.
inline std::string removedParameterReason(const std::string & name)
{
  if (name == "y_kp_m_per_m") return "use y_velocity_gain_per_s [1/s]";
  if (name == "z_kp_m_per_m") return "use z_velocity_gain_per_s [1/s]";
  if (name == "y_kd_m_per_mps" || name == "z_kd_m_per_mps" ||
    name == "cop_y_dot_limit_mps" || name == "cop_z_dot_limit_mps")
  {
    return "velocity accumulation uses no D term or derivative limit";
  }
  return {};
}

class Axis
{
public:
  // Receive a filtered plate y/z CoP sample [m].
  // No derivative term is used, so sample-time differentiation is unnecessary.
  void receive(double cop_m, const AxisSettings & s)
  {
    if (!std::isfinite(cop_m)) {
      resetInput();
      return;
    }
    boundary_active_ = cop_safety::hysteresisActive(
      boundary_active_, cop_m, s.safe_m, s.hysteresis_m);
    error_m_ = boundary_active_ ? cop_safety::deadzoneError(cop_m, s.safe_m) : 0.0;
    have_sample_ = true;
  }

  double velocityCommand(bool usable, const AxisSettings & s) const
  {
    if (!usable || !have_sample_) return 0.0;
    return std::clamp(
      s.direction * s.velocity_gain_per_s * error_m_,
      -s.rate_limit_mps, s.rate_limit_mps);
  }

  // Reset input on source changes/contact renewal; keep accumulated World offsets.
  void resetInput()
  {
    have_sample_ = boundary_active_ = false;
    error_m_ = 0.0;
  }
  double error() const {return error_m_;}  // Current signed boundary excess [m].

private:
  bool have_sample_{false}, boundary_active_{false};
  double error_m_{0.0};
};

enum class MotionMode {TRACK, HOLD, RETURN_TO_ZERO};

inline double norm(const Vec3 & value)
{
  return std::hypot(value[0], value[1], value[2]);
}

inline Vec3 add(const Vec3 & a, const Vec3 & b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline Vec3 scale(const Vec3 & value, double factor)
{
  return {value[0] * factor, value[1] * factor, value[2] * factor};
}

// Rotate the tangent vector [0,y,z] into World with Rz(yaw)*Ry(pitch)*Rx(roll).
// Rotating the complete stored offset would move the command during HOLD.
// Use this only for new increments; RPY is in radians.
inline Vec3 worldOffset(const Vec3 & rpy, double plate_y, double plate_z)
{
  const double sr = std::sin(rpy[0]), cr = std::cos(rpy[0]);
  const double sp = std::sin(rpy[1]), cp = std::cos(rpy[1]);
  const double sy = std::sin(rpy[2]), cy = std::cos(rpy[2]);
  return {
    (cy * sp * sr - sy * cr) * plate_y + (cy * sp * cr + sy * sr) * plate_z,
    (sy * sp * sr + cy * cr) * plate_y + (sy * sp * cr - cy * sr) * plate_z,
    cp * sr * plate_y + cp * cr * plate_z};
}

// Express the accumulated World offset in current Body axes for telemetry.
// Its current x projection may be nonzero after increments at earlier attitudes.
inline Vec3 plateOffset(const Vec3 & rpy, const Vec3 & world)
{
  const Vec3 x{
    std::cos(rpy[2]) * std::cos(rpy[1]),
    std::sin(rpy[2]) * std::cos(rpy[1]), -std::sin(rpy[1])};
  const Vec3 y = worldOffset(rpy, 1.0, 0.0);
  const Vec3 z = worldOffset(rpy, 0.0, 1.0);
  const auto dot = [&world](const Vec3 & axis) {
      return axis[0] * world[0] + axis[1] * world[1] + axis[2] * world[2];
    };
  return {dot(x), dot(y), dot(z)};
}

// Largest fraction in [0,1] keeping offset+fraction*increment inside the norm limit.
// Store only the attainable increment so reverse commands work immediately.
// The caller guarantees the current offset already satisfies its limit.
inline double boundedFraction(const Vec3 & offset, const Vec3 & increment, double limit)
{
  if (norm(add(offset, increment)) <= limit) return 1.0;
  const auto dot = [](const Vec3 & a, const Vec3 & b) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
  const double a = dot(increment, increment);
  if (!(a > 0.0)) return 0.0;
  const double b = 2.0 * dot(offset, increment);
  const double c = dot(offset, offset) - limit * limit;
  const double discriminant = std::max(0.0, b * b - 4.0 * a * c);
  return std::clamp((-b + std::sqrt(discriminant)) / (2.0 * a), 0.0, 1.0);
}

class AccumulatedOffset
{
public:
  Vec3 world() const {return add(y_world_, z_world_);}
  double yDistance() const {return norm(y_world_);}
  double zDistance() const {return norm(z_world_);}

  // Only accept a smaller limit if the current contribution already fits.
  // Otherwise return with OFF first, avoiding an abrupt command truncation.
  bool fitsLimits(const AxisSettings & y, const AxisSettings & z) const
  {
    constexpr double tolerance = 1e-12;
    return yDistance() <= y.offset_limit_m + tolerance &&
      zDistance() <= z.offset_limit_m + tolerance &&
      norm(world()) <= std::hypot(y.offset_limit_m, z.offset_limit_m) + tolerance;
  }

  void advance(
    const Vec3 & rpy, double velocity_y, double velocity_z,
    MotionMode mode_y, MotionMode mode_z, double dt,
    const AxisSettings & y, const AxisSettings & z)
  {
    if (!std::isfinite(dt) || dt <= 0.0) return;
    if (!std::isfinite(velocity_y) || !std::isfinite(velocity_z)) return;
    if (mode_y == MotionMode::RETURN_TO_ZERO && mode_z == MotionMode::RETURN_TO_ZERO) {
      // Master OFF: shrink both contributions by the same factor. Independent
      // return rates could undo a cancellation and create new net World motion.
      double fraction = 1.0;
      if (yDistance() > 0.0) fraction = std::min(fraction, y.rate_limit_mps * dt / yDistance());
      if (zDistance() > 0.0) fraction = std::min(fraction, z.rate_limit_mps * dt / zDistance());
      y_world_ = scale(y_world_, 1.0 - fraction);
      z_world_ = scale(z_world_, 1.0 - fraction);
      return;
    }
    if (mode_y == MotionMode::TRACK || mode_z == MotionMode::TRACK) {
      for (double angle : rpy) {
        if (!std::isfinite(angle)) return;
      }
    }
    const double vy = std::clamp(velocity_y, -y.rate_limit_mps, y.rate_limit_mps);
    const double vz = std::clamp(velocity_z, -z.rate_limit_mps, z.rate_limit_mps);
    Vec3 dy = increment(y_world_, worldOffset(rpy, vy, 0.0), mode_y, dt, y);
    Vec3 dz = increment(z_world_, worldOffset(rpy, 0.0, vz), mode_z, dt, z);
    dy = scale(dy, boundedFraction(y_world_, dy, y.offset_limit_m));
    dz = scale(dz, boundedFraction(z_world_, dz, z.offset_limit_m));
    const double fraction = boundedFraction(
      world(), add(dy, dz), std::hypot(y.offset_limit_m, z.offset_limit_m));
    y_world_ = add(y_world_, scale(dy, fraction));
    z_world_ = add(z_world_, scale(dz, fraction));
  }

  void reset() {y_world_ = z_world_ = {0.0, 0.0, 0.0};}

private:
  static Vec3 increment(
    const Vec3 & current, const Vec3 & velocity_world,
    MotionMode mode, double dt, const AxisSettings & settings)
  {
    if (mode == MotionMode::TRACK) return scale(velocity_world, dt);
    if (mode == MotionMode::HOLD) return {0.0, 0.0, 0.0};
    const double distance = norm(current);
    if (distance <= 0.0) return {0.0, 0.0, 0.0};
    const double remaining = cop_safety::moveToward(
      distance, 0.0, settings.rate_limit_mps * dt);
    return scale(current, remaining / distance - 1.0);
  }

  Vec3 y_world_{0.0, 0.0, 0.0}, z_world_{0.0, 0.0, 0.0};
};
}  // namespace hih_controller::cop_translation

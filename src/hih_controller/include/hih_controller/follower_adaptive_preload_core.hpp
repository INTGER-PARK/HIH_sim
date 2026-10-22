#pragma once

// ROS-independent tangential-force adaptive normal preload and reference ramp.
// Node defaults are defined only in follower_admittance_controller.cpp.
// Ft<=start requests no boost; Ft>=full requests max_boost; interpolate between.
// Both raw and filtered normal forces must indicate contact before adding boost.
// Missing, stale or nonfinite measurements hold the reference. Rise/fall rates
// and the existing force-reference rate limit bound every usable update.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace hih_controller::follower_adaptive_preload
{
struct Settings
{
  bool enabled;
  double tangent_start_n;
  double tangent_full_n;
  double max_boost_n;
  double rise_rate_nps;
  double fall_rate_nps;
  double min_normal_n;
};

inline std::string validationError(const Settings & s)
{
  const std::array<double, 6> values{
    s.tangent_start_n, s.tangent_full_n, s.max_boost_n,
    s.rise_rate_nps, s.fall_rate_nps, s.min_normal_n};
  for (double value : values) {
    if (!std::isfinite(value)) return "adaptive preload parameters must be finite";
  }
  if (s.tangent_start_n < 0.0 || s.tangent_full_n <= s.tangent_start_n) {
    return "adaptive_preload_tangent_full_n must exceed nonnegative tangent_start_n";
  }
  if (s.max_boost_n < 0.0 || s.rise_rate_nps <= 0.0 || s.fall_rate_nps <= 0.0 ||
    s.min_normal_n <= 0.0)
  {
    return "adaptive preload boost must be nonnegative and rates/contact threshold positive";
  }
  return {};
}

class Ramp
{
public:
  void reset()
  {
    boost_request_n_ = 0.0;
    tangent_n_ = std::numeric_limits<double>::quiet_NaN();
    sample_usable_ = contact_usable_ = adapting_ = return_pending_ = false;
  }

  // OFF removes only the requested boost. The next usable update returns the
  // applied reference at fall_rate instead of stepping the normal force target.
  void disable()
  {
    boost_request_n_ = 0.0;
    adapting_ = false;
  }

  double target(double base_n, double min_n, double max_n) const
  {
    return std::clamp(base_n + boost_request_n_, min_n, max_n);
  }

  double advance(
    double reference_n, double base_n, double min_n, double max_n,
    double legacy_rate_nps, double dt,
    double tangent_n, double raw_normal_n, double filtered_normal_n,
    bool sample_fresh, bool normal_axis_available, const Settings & s)
  {
    sample_usable_ = sample_fresh && std::isfinite(tangent_n) && tangent_n >= 0.0 &&
      std::isfinite(raw_normal_n) && std::isfinite(filtered_normal_n);
    contact_usable_ = sample_usable_ && raw_normal_n >= s.min_normal_n &&
      filtered_normal_n >= s.min_normal_n;
    tangent_n_ = sample_usable_ ? tangent_n : std::numeric_limits<double>::quiet_NaN();
    const bool permitted = s.enabled && normal_axis_available;
    if (!permitted) {
      disable();
    } else if (sample_usable_) {
      const double fraction = contact_usable_ ? std::clamp(
        (tangent_n - s.tangent_start_n) / (s.tangent_full_n - s.tangent_start_n),
        0.0, 1.0) : 0.0;
      boost_request_n_ = s.max_boost_n * fraction;
      if (boost_request_n_ > 0.0) return_pending_ = true;
    }
    adapting_ = permitted && contact_usable_ && boost_request_n_ > 0.0;

    // Do not change the reference without a usable force sample and elapsed time.
    if (!sample_usable_ || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(reference_n) || !std::isfinite(base_n) ||
      !std::isfinite(min_n) || !std::isfinite(max_n) || min_n < 0.0 || max_n < min_n ||
      !std::isfinite(legacy_rate_nps) || legacy_rate_nps < 0.0)
    {
      return reference_n;
    }

    const double desired_n = target(base_n, min_n, max_n);
    double rate_nps = legacy_rate_nps;
    if (permitted || (return_pending_ && reference_n > base_n)) {
      // Retain f_ref_rate_max as the overall reference-rate ceiling.
      rate_nps = std::min(rate_nps,
        desired_n >= reference_n ? s.rise_rate_nps : s.fall_rate_nps);
    }
    const double step_n = rate_nps * dt;
    const double next_n = std::clamp(
      reference_n + std::clamp(desired_n - reference_n, -step_n, step_n),
      min_n, max_n);
    if (!permitted && next_n <= base_n) return_pending_ = false;
    return next_n;
  }

  double boostRequest() const {return boost_request_n_;}
  double tangentForce() const {return tangent_n_;}
  bool sampleUsable() const {return sample_usable_;}
  bool contactUsable() const {return contact_usable_;}
  bool adapting() const {return adapting_;}

private:
  double boost_request_n_{0.0};
  double tangent_n_{std::numeric_limits<double>::quiet_NaN()};
  bool sample_usable_{false}, contact_usable_{false}, adapting_{false};
  bool return_pending_{false};
};
}  // namespace hih_controller::follower_adaptive_preload

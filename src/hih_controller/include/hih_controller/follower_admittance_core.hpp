#ifndef HIH_CONTROLLER__FOLLOWER_ADMITTANCE_CORE_HPP_
#define HIH_CONTROLLER__FOLLOWER_ADMITTANCE_CORE_HPP_

// Follower admittance axis gating and final output-reference constraints.
// AxisMask order is [x,y,z,roll,pitch,yaw]. The controller owns the defaults.
// Virtual contact-frame increments are rotated to World, then the same mask
// also gates final World position / RPY offset components. A disabled output
// is exactly zero, including approach and contact-center rotation compensation.
// This can restrict motion along a tilted plate normal. These masks do not
// constrain the task reference or separate CoP controller offsets.
// Pipeline: virtual integration -> contactIncrementWorld -> accumulate World
// offset -> constrainOutputReference -> subtract entry reference -> publish.

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>

namespace hih_controller
{
namespace follower_admittance
{
using AxisMask = std::array<bool, 6>;  // [x,y,z,roll,pitch,yaw]; false removes the component.

// The same enable flags gate Body/contact virtual axes and final World/RPY
// command components. Thus a disabled output never receives a projected trim.
inline Eigen::Vector3d maskTranslation(Eigen::Vector3d value, const AxisMask & enabled)
{
  // Mask the first three components without changing coordinate frames.
  // The caller chooses Body/World and assigns the returned copy as needed.
  for (std::size_t i = 0; i < 3; ++i) {
    if (!enabled[i]) {
      value[i] = 0.0;
    }
  }
  return value;
}

inline Eigen::Vector3d maskRotation(Eigen::Vector3d value, const AxisMask & enabled)
{
  // Mask the roll/pitch/yaw offset components using the last three flags.
  // The controller uses radians internally and converts to degrees for the message.
  for (std::size_t i = 0; i < 3; ++i) {
    if (!enabled[i + 3]) {
      value[i] = 0.0;
    }
  }
  return value;
}

inline Eigen::Vector3d contactIncrementWorld(
  const Eigen::Matrix3d & r_wb, const Eigen::Vector3d & before,
  const Eigen::Vector3d & after, const AxisMask & enabled)
{
  // r_wb rotates Body/contact coordinates into World; before/after are virtual offsets [m].
  // Remove disabled contact-axis components from after-before.
  // Clearing a disabled virtual state is not a new commanded displacement;
  // otherwise an attitude change could reproject an old World offset.
  // Rotate the permitted increment to World and apply the World-output mask.
  // Return this update's increment [m], not the complete accumulated offset.
  // Clearing a disabled virtual state is not a commanded displacement.
  const Eigen::Vector3d delta_contact = maskTranslation(after - before, enabled);
  return maskTranslation(r_wb * delta_contact, enabled);
}

struct OutputReference
{
  Eigen::Vector3d position_world;  // Constrained absolute World position reference [m].
  Eigen::Vector3d attitude_rad;    // Constrained absolute RPY reference [rad].
};

inline OutputReference constrainOutputReference(
  const Eigen::Vector3d & position_world, const Eigen::Vector3d & attitude_rad,
  const Eigen::Vector3d & entry_position_world, const Eigen::Vector3d & entry_attitude_rad,
  const Eigen::Vector3d & last_position_world, const AxisMask & enabled,
  double maximum_position_step)
{
  // Inputs include approach and contact-center rotation compensation.
  // entry_* is the reference captured on enable, where the output offset is zero.
  // last_position_world is the previous published, constrained reference.
  // maximum_position_step is the allowed displacement per update [m].
  // Positive infinity bypasses the position rate limit; attitude is not rate-limited here.
  //
  // Zero the disabled offset relative to entry, not the absolute reference.
  // Subtract entry from this result to form the published admittance offsets.
  Eigen::Vector3d offset = maskTranslation(position_world - entry_position_world, enabled);
  const Eigen::Vector3d previous =
    maskTranslation(last_position_world - entry_position_world, enabled);

  // A removed axis must neither consume the allowed-axis rate budget nor
  // survive as a slowly decaying command after a runtime disable.
  const Eigen::Vector3d delta = offset - previous;
  const double distance = delta.norm();
  if (std::isfinite(maximum_position_step) &&
    distance > std::max(0.0, maximum_position_step) && distance > 1e-12)
  {
    offset = previous + delta * (std::max(0.0, maximum_position_step) / distance);
  }

  return {
    // Apply the mask after limiting so disabled components remain exactly zero.
    // The node must save this result as its next reference history.
    entry_position_world + maskTranslation(offset, enabled),
    entry_attitude_rad + maskRotation(attitude_rad - entry_attitude_rad, enabled)};
}
}  // namespace follower_admittance
}  // namespace hih_controller

#endif  // HIH_CONTROLLER__FOLLOWER_ADMITTANCE_CORE_HPP_

#pragma once

// ROS-independent three-axis torque disturbance observer and compensation.
// The hih_torque_dob node reads Body angular velocity and the final wrench
// command. It publishes minitrone/dob_status for the role's wrench controller.
// Vectors use Body x/y/z (roll/pitch/yaw); nominal inertia is diagonal.
// h=Jn*omega; d=dh/dt+omega x h-tau_sent; a Q-filter produces d_hat.
// d_hat includes inertia error and actuation lag as well as external contact
// torque, so it is not pure contact ground truth. This is separate from 6D MoB.
// Axis flags gate compensation only. Enabling compensation resets the observer,
// which starts from zero. Configure defaults in torque_dob.cpp or ROS parameters.
// Run the node in /leader or /follower; this header itself is not an executable.

#include <algorithm>
#include <array>
#include <cmath>

namespace hih_dob {
using Vec3 = std::array<double, 3>;  // Body x/y/z order; units are specified at each operation.
using Mask3 = std::array<bool, 3>;   // Compensation enable flags for roll/pitch/yaw.
constexpr double kTwoPi = 6.28318530717958647692;  // Two pi converts Hz into angular frequency [rad/s].

inline bool finite(const Vec3 & v) {
  // Check all components for finite values; no freshness or magnitude test here.
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline Vec3 cross(const Vec3 & a, const Vec3 & b) {
  // Cross product; the observer uses omega x (Jn*omega) for gyroscopic torque.
  return {a[1]*b[2]-a[2]*b[1],
          a[2]*b[0]-a[0]*b[2],
          a[0]*b[1]-a[1]*b[0]};
}

// Filtered inverse rigid-body model:
//   h = Jn*omega
//   d = dh/dt + omega x h - tau_sent
// tau_sent is the latest FINAL torque command published by wrench_controller.
// The Q-filter is already part of the observer; no extra ramp/slew controller
// is used for compensation startup.
class Observer {
public:
  void reset() {
    // Clear the estimate and treat the next sample as the initial reference.
    // The initialization branch replaces previous momentum, gyro term and timestamp.
    initialized_=false;
    estimate_={0,0,0};
  }

  const Vec3 & estimate() const { return estimate_; }  // Latest filtered disturbance estimate [N*m].
  bool initialized() const { return initialized_; }   // Initial sample captured; separate from compensation enable state.

  bool update(double t, const Vec3 & omega, const Vec3 & tau_sent,
              const Vec3 & inertia, const Vec3 & cutoff, double max_dt) {
    // t [s], Body omega [rad/s], final torque command [N*m].
    // Nominal diagonal inertia [kg*m^2], axis cutoffs [Hz], maximum dt [s].
    // The node validates positive finite configuration values before calling update.
    // Return true only when this sample produced a new disturbance estimate.
    // The caller decides whether the observer output is currently usable.
    if (!std::isfinite(t) || !finite(omega) || !finite(tau_sent)) {
      reset();
      return false;
    }

    Vec3 h{};
    // h=Jn*omega [kg*m^2/s]; c=omega x h [N*m].
    for (int i=0; i<3; ++i) h[i]=inertia[i]*omega[i];
    const Vec3 c=cross(omega,h);

    if (!initialized_) {
      // The first sample seeds differentiation history and returns a zero estimate.
      previous_h_=h;
      previous_c_=c;
      time_=t;
      estimate_={0,0,0};
      initialized_=true;
      return false;
    }

    const double dt=t-time_;
    // Skip repeated timestamps because they contain no new dynamics.
    // Reset on reversed time or long gaps instead of differentiating across them.
    if (dt==0.0) return false;
    if (dt<0.0 || dt>max_dt) {
      reset();
      return update(t,omega,tau_sent,inertia,cutoff,max_dt);
    }

    for (int i=0; i<3; ++i) {
      // beta=1-exp(-2*pi*fc*dt); expm1 avoids cancellation for small fc*dt.
      // raw disturbance = (h_new-h_old)/dt + (c_new+c_old)/2 - tau_sent.
      // The Q-filter smooths numerical differentiation: estimate=(1-beta)*old+beta*raw.
      const double beta=-std::expm1(-kTwoPi*cutoff[i]*dt);
      estimate_[i]=(1.0-beta)*estimate_[i]+beta*(
        (h[i]-previous_h_[i])/dt +
        0.5*(c[i]+previous_c_[i]) -
        tau_sent[i]);
    }

    previous_h_=h;
    previous_c_=c;
    time_=t;

    if (!finite(estimate_)) {
      reset();
      return false;
    }
    return true;
  }

private:
  // Per-instance three-axis history, never shared with another role.
  bool initialized_{false};
  double time_{0};
  Vec3 previous_h_{}, previous_c_{}, estimate_{};
};

// Static compensation map. Smooth startup is NOT produced here:
// pressing [d] resets Observer, so d_hat itself starts from zero and the
// existing Q-filter naturally reconstructs the disturbance.
inline Vec3 compensation(const Vec3 & estimate, bool enabled,
                         const Mask3 & axes, const Vec3 & gain,
                         const Vec3 & limit) {
  // Estimate and output [N*m], dimensionless gain, torque limits [N*m].
  // tau_comp=clamp(-gain*d_hat,-limit,+limit) opposes the estimated disturbance.
  // Disabled master/axis outputs are exactly zero.
  // The node validates settings such as nonnegative limits before calling this map.
  // This function has no filter/reset state; the node resets the observer on enable.
  Vec3 result{};
  for (int i=0; i<3; ++i) {
    if (!enabled || !axes[i]) continue;
    result[i]=std::clamp(-gain[i]*estimate[i],-limit[i],limit[i]);
  }
  return result;
}

inline bool fresh(double age, double timeout) {
  // Check reception age against the interval [0,timeout] [s].
  // The caller supplies steady-clock age; this function reads no clocks.
  return std::isfinite(age) && age>=0.0 && age<=timeout;
}
}  // namespace hih_dob

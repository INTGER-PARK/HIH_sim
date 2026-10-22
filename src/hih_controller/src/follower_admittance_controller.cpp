// Follower-specific admittance controller.
// This file is intentionally independent from the Leader version so
// code-level tuning defaults and future control logic can diverge per vehicle.
// Default ROS namespace is hard-coded to /follower; standalone ros2 run needs no __ns remap.
//
// This node does not command actuators. It compares the momentum-observer
// contact wrench with the desired contact wrench, integrates the six virtual
// mass-damper-spring axes, and publishes bounded position/attitude offsets.
// The Follower wrench controller superposes those offsets on the task command.
// PASSTHROUGH, ADMITTANCE, and HOLD transitions preserve a continuous handoff;
// keep that state-machine behavior when tuning the equations below.

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <hih_interfaces/msg/attitude_cmd.hpp>
#include <hih_interfaces/msg/cmd.hpp>
#include <hih_interfaces/msg/minitrone_state.hpp>
#include <hih_interfaces/msg/wrench.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "hih_controller/follower_admittance_core.hpp"
#include "hih_controller/follower_adaptive_preload_core.hpp"

namespace admittance = hih_controller::follower_admittance;
namespace adaptive_preload = hih_controller::follower_adaptive_preload;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr std::size_t kDof = 6;

const std::array<std::string, kDof> kAxisNames = {
  "x", "y", "z", "roll", "pitch", "yaw"};

// ============================================================================
// Admittance tuning defaults (rebuild after editing).
// Axis order for every array: x, y, z, roll, pitch, yaw.
// Virtual wrench/velocity axes follow the Body-attached contact plate frame C.
// The plate center is offset from the CoM, but its axes are parallel to Body.
// Each enable flag ALSO gates the corresponding final World position / RPY
// offset. Disabled cmd_admittance / att_cmd_admittance fields are exactly zero,
// including automatic approach and contact-center rotation compensation.
// This output constraint can restrict motion along a tilted plate's normal.
// ROS parameters can override these defaults.
// M*q_ddot + D*q_dot + K*q = external wrench error.
// Translation: M [kg], D [N*s/m], K [N/m].
// Rotation: M [kg*m^2], D [N*m*s/rad], K [N*m/rad].
// ============================================================================
constexpr std::array<bool, kDof> axis_enabled_default = {
  true,   // x
  true,  // y
  true,  // z
  false,  // roll
  false,  // pitch
  true   // yaw
};

constexpr std::array<double, kDof> m_default = {1.5, 1.5, 1.00, 1.00, 1.00, 1.00};
constexpr std::array<double, kDof> d_default = {20.0, 20.0, 5.0, 2.0, 2.0, 2.0};
constexpr std::array<double, kDof> k_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Input deadband and saturation.
constexpr std::array<double, kDof> deadband_default = {
  0.05, 0.05, 0.05, 0.01, 0.01, 0.01};
constexpr std::array<double, kDof> input_max_default = {
  2.0, 2.0, 2.0, 0.50, 0.50, 0.50};

// Virtual acceleration, velocity, and displacement limits.
constexpr std::array<double, kDof> ddq_max_default = {
  1.00, 1.00, 1.00,
  30.0 * kDegToRad, 30.0 * kDegToRad, 30.0 * kDegToRad};
constexpr std::array<double, kDof> dq_max_default = {
  2.00, 2.00, 2.00,
  10.0 * kDegToRad, 10.0 * kDegToRad, 10.0 * kDegToRad};
constexpr std::array<double, kDof> q_max_default = {
  1.00, 1.00, 1.00,
  10.0 * kDegToRad, 10.0 * kDegToRad, 15.0 * kDegToRad};

// Wrench low-pass cutoff frequencies [Hz], in the same axis order.
constexpr std::array<double, kDof> cutoff_hz_default = {
  10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
// End of axis tuning defaults.

// ============================================================================
// Tangential-force adaptive preload defaults are defined only here.
// B/b toggles preload; adaptive_preload_enabled is the ROS parameter equivalent.
// The U/J base target remains f_normal_des; adaptation adds only a boost.
//   Ft = ||F_contact - n * (n.dot(F_contact))|| [N]
//   boost = max_boost * clamp((Ft-start)/(full-start), 0, 1) [N]
//   target = clamp(f_normal_des + boost, f_normal_min, f_normal_max) [N]
// Observe filtered MoB contact force even with y/z admittance disabled.
// With n=(1,0,0), Ft=hypot(Fy,Fz), including static payload support force.
// This block uses force magnitude, without Leader trajectory or payload/slip GT.
// ============================================================================
namespace adaptive_preload_tuning
{
constexpr bool enabled = false;            // Initial adaptive preload enable flag.
constexpr double tangent_start_n = 0.5;    // No additional preload below this tangential force [N].
constexpr double tangent_full_n = 3.0;     // Maximum boost at or above this tangential force [N].
constexpr double max_boost_n = 4.0;        // Maximum additional normal-force target [N].
constexpr double rise_rate_nps = 1.0;      // Reference rise rate [N/s].
constexpr double fall_rate_nps = 0.5;      // Reference fall rate after reduced load or B OFF [N/s].
constexpr double min_normal_n = 0.8;       // Minimum usable normal compression [N].
constexpr double status_period_sec = 0.1;  // Status publication period [s], 10 Hz.
// The existing f_ref_rate_max also bounds these rates; the inner loop sets response.
}  // namespace adaptive_preload_tuning

Eigen::Matrix3d rotationWorldFromBody(const Eigen::Vector3d & rpy)
{
  const double r = rpy.x();
  const double p = rpy.y();
  const double y = rpy.z();

  const double sr = std::sin(r);
  const double cr = std::cos(r);
  const double sp = std::sin(p);
  const double cp = std::cos(p);
  const double sy = std::sin(y);
  const double cy = std::cos(y);

  Eigen::Matrix3d r_wb;
  r_wb <<
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr;
  return r_wb;
}

Eigen::Matrix3d rotationExp(const Eigen::Vector3d & phi)
{
  const double angle = phi.norm();
  if (angle < 1e-10) {
    Eigen::Matrix3d hat;
    hat <<
      0.0, -phi.z(), phi.y(),
      phi.z(), 0.0, -phi.x(),
      -phi.y(), phi.x(), 0.0;
    return Eigen::Matrix3d::Identity() + hat;
  }
  return Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
}

Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & r)
{
  // ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  const double cp = std::cos(pitch);

  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(cp) > 1e-7) {
    roll = std::atan2(r(2, 1), r(2, 2));
    yaw = std::atan2(r(1, 0), r(0, 0));
  } else {
    // Near pitch = +-90 deg, choose roll = 0 and preserve a valid yaw.
    roll = 0.0;
    yaw = std::atan2(-r(0, 1), r(1, 1));
  }
  return Eigen::Vector3d(roll, pitch, yaw);
}

double unwrapNear(double angle, double reference)
{
  return reference + std::atan2(std::sin(angle - reference), std::cos(angle - reference));
}

double moveToward(double current, double target, double max_step)
{
  const double step = std::max(0.0, max_step);
  if (current < target) {
    return std::min(current + step, target);
  }
  return std::max(current - step, target);
}
}  // namespace

class FollowerAdmittanceController6Dof : public rclcpp::Node
{
public:
  enum class Mode
  {
    PASSTHROUGH,
    ADMITTANCE,
    HOLD
  };

  FollowerAdmittanceController6Dof()
  : rclcpp::Node("follower_admittance_controller", "/follower")
  {
    // ------------------------------------------------------------------------
    // Global and per-axis enable settings
    // ------------------------------------------------------------------------
    const bool initially_enabled = declare_parameter<bool>("enabled", false);
    for (std::size_t i = 0; i < kDof; ++i) {
      axis_enabled_[i] = declare_parameter<bool>(
        "enable_" + kAxisNames[i], axis_enabled_default[i]);
    }

    // Contact surface direction in BODY frame: robot -> contact surface.
    n_face_body_ <<
      declare_parameter<double>("n_face_b_x", 1.0),
      declare_parameter<double>("n_face_b_y", 0.0),
      declare_parameter<double>("n_face_b_z", 0.0);
    if (n_face_body_.norm() < 1e-6) {
      n_face_body_ = Eigen::Vector3d::UnitX();
    }
    n_face_body_.normalize();

    // CoM B -> center of the +X plate face C, in Body axes. The default
    // matches Follower.xml: 0.212 m plate center + 0.003 m half-thickness.
    contact_center_body_ <<
      declare_parameter<double>("contact_center_b_x", 0.215),
      declare_parameter<double>("contact_center_b_y", 0.0),
      declare_parameter<double>("contact_center_b_z", 0.0);
    if (!contact_center_body_.array().isFinite().all()) {
      throw std::invalid_argument("contact_center_b_* must be finite");
    }

    // ------------------------------------------------------------------------
    // Normal-force tracking
    // ------------------------------------------------------------------------
    f_normal_des_ = declare_parameter<double>("f_normal_des", 10.0);
    f_normal_step_ = std::abs(declare_parameter<double>("f_normal_step", 0.05));
    f_normal_min_ = declare_parameter<double>("f_normal_min", 0.0);
    f_normal_max_ = declare_parameter<double>("f_normal_max", 20.0);
    f_ref_rate_max_ = std::abs(declare_parameter<double>("f_ref_rate_max", 1.0));
    auto_approach_ = declare_parameter<bool>("auto_approach", false);
    approach_speed_mps_ = std::max(0.0, declare_parameter<double>("approach_speed_mps", 0.03));
    approach_contact_force_n_ = std::max(0.0, declare_parameter<double>("approach_contact_force_n", 0.8));
    approach_max_distance_m_ = std::max(0.0, declare_parameter<double>("approach_max_distance_m", 1.00));
    normal_force_integral_gain_ =
      declare_parameter<double>("normal_force_integral_gain", 0.0);
    normal_force_integral_limit_ = std::abs(
      declare_parameter<double>("normal_force_integral_limit", 1.0));
    handoff_blend_time_ = std::max(
      0.0, declare_parameter<double>("handoff_blend_time", 2.0));
    reference_rate_limit_ = std::max(
      0.0, declare_parameter<double>("reference_rate_limit", 0.20));
    if (f_normal_min_ > f_normal_max_) {
      std::swap(f_normal_min_, f_normal_max_);
    }
    if (!std::isfinite(f_normal_des_) || !std::isfinite(f_normal_step_) ||
      !std::isfinite(f_normal_min_) || !std::isfinite(f_normal_max_) ||
      !std::isfinite(f_ref_rate_max_) || f_normal_min_ < 0.0)
    {
      throw std::invalid_argument("normal-force settings must be finite with nonnegative bounds");
    }
    f_normal_des_ = std::clamp(f_normal_des_, f_normal_min_, f_normal_max_);

    adaptive_preload_settings_ = {
      declare_parameter<bool>("adaptive_preload_enabled", adaptive_preload_tuning::enabled),
      declare_parameter<double>("adaptive_preload_tangent_start_n", adaptive_preload_tuning::tangent_start_n),
      declare_parameter<double>("adaptive_preload_tangent_full_n", adaptive_preload_tuning::tangent_full_n),
      declare_parameter<double>("adaptive_preload_max_boost_n", adaptive_preload_tuning::max_boost_n),
      declare_parameter<double>("adaptive_preload_rise_rate_nps", adaptive_preload_tuning::rise_rate_nps),
      declare_parameter<double>("adaptive_preload_fall_rate_nps", adaptive_preload_tuning::fall_rate_nps),
      declare_parameter<double>("adaptive_preload_min_normal_n", adaptive_preload_tuning::min_normal_n)};
    const auto preload_error = adaptive_preload::validationError(adaptive_preload_settings_);
    if (!preload_error.empty()) throw std::invalid_argument(preload_error);

    // ------------------------------------------------------------------------
    // Six-axis virtual M, D, K.
    // First 3 axes: [m, m/s] related translational motion.
    // Last 3 axes: [rad, rad/s] related rotational motion.
    // ------------------------------------------------------------------------
    for (std::size_t i = 0; i < kDof; ++i) {
      adm_m_[i] = std::max(
        std::abs(declare_parameter<double>("adm_m_" + kAxisNames[i], m_default[i])),
        1e-6);
      adm_d_[i] = std::max(
        0.0, declare_parameter<double>("adm_d_" + kAxisNames[i], d_default[i]));
      adm_k_[i] = std::max(
        0.0, declare_parameter<double>("adm_k_" + kAxisNames[i], k_default[i]));
    }

    // ------------------------------------------------------------------------
    // Independent LPF cutoff frequency for Fx,Fy,Fz,Mx,My,Mz.
    // ------------------------------------------------------------------------
    for (std::size_t i = 0; i < kDof; ++i) {
      cutoff_hz_[i] = std::max(
        0.0, declare_parameter<double>(
          "cutoff_" + kAxisNames[i] + "_hz", cutoff_hz_default[i]));
    }

    for (std::size_t i = 0; i < kDof; ++i) {
      input_deadband_[i] = std::abs(declare_parameter<double>(
        "input_deadband_" + kAxisNames[i], deadband_default[i]));
      input_max_[i] = std::abs(declare_parameter<double>(
        "input_max_" + kAxisNames[i], input_max_default[i]));
      ddq_max_[i] = std::abs(declare_parameter<double>(
        "ddq_max_" + kAxisNames[i], ddq_max_default[i]));
      dq_max_[i] = std::abs(declare_parameter<double>(
        "dq_max_" + kAxisNames[i], dq_max_default[i]));
      q_max_[i] = std::abs(declare_parameter<double>(
        "q_max_" + kAxisNames[i], q_max_default[i]));
    }

    wrench_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("wrench_timeout_sec", 0.10));

    // ------------------------------------------------------------------------
    // ROS I/O
    // ------------------------------------------------------------------------
    sub_cmd_ = create_subscription<hih_interfaces::msg::Cmd>(
      "minitrone/cmd", 10,
      std::bind(&FollowerAdmittanceController6Dof::onCmd, this, std::placeholders::_1));

    sub_att_cmd_ = create_subscription<hih_interfaces::msg::AttitudeCmd>(
      "minitrone/att_cmd", 10,
      std::bind(&FollowerAdmittanceController6Dof::onAttCmd, this, std::placeholders::_1));

    sub_state_ = create_subscription<hih_interfaces::msg::MinitroneState>(
      "minitrone/state", 10,
      std::bind(&FollowerAdmittanceController6Dof::onState, this, std::placeholders::_1));

    sub_external_wrench_ = create_subscription<hih_interfaces::msg::Wrench>(
      "minitrone/external_wrench_hat_second_order", 10,
      std::bind(
        &FollowerAdmittanceController6Dof::onExternalWrench, this, std::placeholders::_1));

    // true: enter 6-DOF admittance, false: hold current measured pose.
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      "minitrone/admittance_enable", 10,
      std::bind(&FollowerAdmittanceController6Dof::onEnable, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<hih_interfaces::msg::Cmd>(
      "minitrone/cmd_admittance", 10);
    pub_att_cmd_ = create_publisher<hih_interfaces::msg::AttitudeCmd>(
      "minitrone/att_cmd_admittance", 10);
    pub_active_ = create_publisher<std_msgs::msg::Bool>(
      "minitrone/admittance_active", 10);
    pub_des_force_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/admittance_des_force", 10);
    pub_contact_wrench_ = create_publisher<hih_interfaces::msg::Wrench>(
      "minitrone/admittance/contact_wrench", 10);
    pub_blend_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/admittance/blend", 10);
    pub_preload_target_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/admittance/normal_force_target", 10);
    pub_normal_reference_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/admittance/normal_force_reference", 10);
    pub_tangent_force_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/admittance/tangential_force", 10);
    pub_preload_enabled_ = create_publisher<std_msgs::msg::Bool>(
      "minitrone/admittance/adaptive_preload_enabled", 10);
    pub_preload_active_ = create_publisher<std_msgs::msg::Bool>(
      "minitrone/admittance/adaptive_preload_active", 10);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &FollowerAdmittanceController6Dof::onSetParameters,
        this,
        std::placeholders::_1));

    setupKeyboard();
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&FollowerAdmittanceController6Dof::pollKeyboard, this));

    last_control_time_ = now();
    pending_enable_ = initially_enabled;
    publishStatus();

    RCLCPP_INFO(
      get_logger(),
      "6-DOF admittance ready. O=toggle admittance/HOLD, "
      "B=toggle adaptive preload, P=PASSTHROUGH, U/J=base normal force +/- %.2f N",
      f_normal_step_);
  }

  ~FollowerAdmittanceController6Dof() override
  {
    restoreKeyboard();
  }

private:
  void onCmd(const hih_interfaces::msg::Cmd::SharedPtr msg)
  {
    upstream_pos_cmd_ <<
      static_cast<double>(msg->pos_cmd[0]),
      static_cast<double>(msg->pos_cmd[1]),
      static_cast<double>(msg->pos_cmd[2]);
    have_cmd_ = true;
  }

  void onAttCmd(const hih_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    upstream_att_cmd_rad_ <<
      static_cast<double>(msg->roll_ref) * kDegToRad,
      static_cast<double>(msg->pitch_ref) * kDegToRad,
      static_cast<double>(msg->yaw_ref) * kDegToRad;
    have_att_cmd_ = true;
  }

  void onExternalWrench(const hih_interfaces::msg::Wrench::SharedPtr msg)
  {
    external_force_body_ <<
      static_cast<double>(msg->force[0]),
      static_cast<double>(msg->force[1]),
      static_cast<double>(msg->force[2]);
    external_moment_body_ <<
      static_cast<double>(msg->moment[0]),
      static_cast<double>(msg->moment[1]),
      static_cast<double>(msg->moment[2]);
    if (!external_force_body_.array().isFinite().all() ||
      !external_moment_body_.array().isFinite().all())
    {
      have_external_wrench_ = false;
      wrench_filter_initialized_ = false;
      RCLCPP_WARN(get_logger(), "non-finite external wrench rejected; force reference held");
      return;
    }
    have_external_wrench_ = true;
    last_wrench_time_ = now();
  }

  void onEnable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    requestEnabled(msg->data);
  }

  void requestEnabled(bool enabled)
  {
    const auto result = set_parameter(rclcpp::Parameter("enabled", enabled));
    if (!result.successful) {
      RCLCPP_ERROR(
        get_logger(), "failed to set enabled=%s: %s",
        enabled ? "true" : "false", result.reason.c_str());
    }
  }

  void requestAdaptivePreload(bool enabled)
  {
    const auto result = set_parameter(rclcpp::Parameter("adaptive_preload_enabled", enabled));
    if (!result.successful) {
      RCLCPP_ERROR(get_logger(), "adaptive preload toggle rejected: %s", result.reason.c_str());
    }
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool axes_changed = false;

    // Validate the complete candidate configuration before changing node state.
    // Reject invalid mixed updates atomically, before ordinary parameter processing.
    auto next_preload = adaptive_preload_settings_;
    bool preload_parameters_changed = false;
    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "adaptive_preload_enabled") next_preload.enabled = parameter.as_bool();
        else if (name == "adaptive_preload_tangent_start_n") next_preload.tangent_start_n = parameter.as_double();
        else if (name == "adaptive_preload_tangent_full_n") next_preload.tangent_full_n = parameter.as_double();
        else if (name == "adaptive_preload_max_boost_n") next_preload.max_boost_n = parameter.as_double();
        else if (name == "adaptive_preload_rise_rate_nps") next_preload.rise_rate_nps = parameter.as_double();
        else if (name == "adaptive_preload_fall_rate_nps") next_preload.fall_rate_nps = parameter.as_double();
        else if (name == "adaptive_preload_min_normal_n") next_preload.min_normal_n = parameter.as_double();
        else if ((name == "f_normal_des" || name == "f_ref_rate_max") &&
          !std::isfinite(parameter.as_double()))
        {
          throw std::invalid_argument("normal-force target/rate must be finite");
        }
        if (name.rfind("adaptive_preload_", 0) == 0) preload_parameters_changed = true;
      }
      const auto error = adaptive_preload::validationError(next_preload);
      if (!error.empty()) throw std::invalid_argument(error);
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
      return result;
    }
    const bool preload_toggle = next_preload.enabled != adaptive_preload_settings_.enabled;
    adaptive_preload_settings_ = next_preload;
    if (!adaptive_preload_settings_.enabled) adaptive_preload_ramp_.disable();
    if (preload_toggle) {
      RCLCPP_INFO(get_logger(), "adaptive preload %s (base %.2f N; boost limit %.2f N)",
        adaptive_preload_settings_.enabled ? "ON" : "OFF -> ramp to base",
        f_normal_des_, adaptive_preload_settings_.max_boost_n);
    }

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (name == "enabled") {
        setEnabledInternal(parameter.as_bool());
        continue;
      }
      if (name == "f_normal_des") {
        f_normal_des_ = std::clamp(
          parameter.as_double(), f_normal_min_, f_normal_max_);
        continue;
      }
      if (name == "f_ref_rate_max") {
        f_ref_rate_max_ = std::abs(parameter.as_double());
        continue;
      }
      if (name == "normal_force_integral_gain") {
        normal_force_integral_gain_ = parameter.as_double();
        continue;
      }

      for (std::size_t i = 0; i < kDof; ++i) {
        if (name == "enable_" + kAxisNames[i]) {
          axis_enabled_[i] = parameter.as_bool();
          axes_changed = true;
          if (!axis_enabled_[i]) {
            if (i < 3) {
              // Clear the final World component directly. Removing a Body
              // projection would also modify other, still-enabled outputs.
              contact_offset_world_[i] = 0.0;
            }
            q_[i] = 0.0;
            dq_[i] = 0.0;
          }
        } else if (name == "adm_m_" + kAxisNames[i]) {
          adm_m_[i] = std::max(std::abs(parameter.as_double()), 1e-6);
        } else if (name == "adm_d_" + kAxisNames[i]) {
          adm_d_[i] = std::max(0.0, parameter.as_double());
        } else if (name == "adm_k_" + kAxisNames[i]) {
          adm_k_[i] = std::max(0.0, parameter.as_double());
        } else if (name == "cutoff_" + kAxisNames[i] + "_hz") {
          cutoff_hz_[i] = std::max(0.0, parameter.as_double());
        } else if (name == "input_deadband_" + kAxisNames[i]) {
          input_deadband_[i] = std::abs(parameter.as_double());
        } else if (name == "input_max_" + kAxisNames[i]) {
          input_max_[i] = std::abs(parameter.as_double());
        } else if (name == "ddq_max_" + kAxisNames[i]) {
          ddq_max_[i] = std::abs(parameter.as_double());
        } else if (name == "dq_max_" + kAxisNames[i]) {
          dq_max_[i] = std::abs(parameter.as_double());
        } else if (name == "q_max_" + kAxisNames[i]) {
          q_max_[i] = std::abs(parameter.as_double());
        }
      }
    }
    if (axes_changed && mode_ == Mode::ADMITTANCE && have_state_) {
      contact_offset_world_ = admittance::maskTranslation(contact_offset_world_, axis_enabled_);
      base_pos_world_ = entry_pos_ref_world_ + admittance::maskTranslation(
        base_pos_world_ - entry_pos_ref_world_, axis_enabled_);
      last_pos_ref_world_ = entry_pos_ref_world_ + admittance::maskTranslation(
        last_pos_ref_world_ - entry_pos_ref_world_, axis_enabled_);
      last_att_ref_rad_ = entry_att_ref_rad_ + admittance::maskRotation(
        last_att_ref_rad_ - entry_att_ref_rad_, axis_enabled_);
      // Publish the disabled fields as zero immediately, even without a new
      // state message. Clearing history also prevents offsets reappearing on ON.
      updateAndPublish();
    }
    if (preload_parameters_changed) {
      preload_status_initialized_ = false;
      publishStatus();
    }
    return result;
  }

  void onState(const hih_interfaces::msg::MinitroneState::SharedPtr msg)
  {
    pos_ <<
      static_cast<double>(msg->pos[0]),
      static_cast<double>(msg->pos[1]),
      static_cast<double>(msg->pos[2]);
    vel_ <<
      static_cast<double>(msg->vel[0]),
      static_cast<double>(msg->vel[1]),
      static_cast<double>(msg->vel[2]);
    rpy_ <<
      static_cast<double>(msg->rpy[0]),
      static_cast<double>(msg->rpy[1]),
      static_cast<double>(msg->rpy[2]);
    w_body_ <<
      static_cast<double>(msg->w_rpy[0]),
      static_cast<double>(msg->w_rpy[1]),
      static_cast<double>(msg->w_rpy[2]);
    have_state_ = true;

    if (pending_enable_) {
      pending_enable_ = false;
      enterAdmittanceMode();
    }
    if (mode_ == Mode::HOLD && !hold_initialized_) {
      captureCurrentHoldPose();
    }

    updateAndPublish();
  }

  void setEnabledInternal(bool enabled)
  {
    if (enabled && mode_ == Mode::ADMITTANCE && !pending_enable_) {
      return;
    }
    if (!enabled && mode_ == Mode::HOLD && !pending_enable_) {
      return;
    }

    if (enabled) {
      if (!have_state_) {
        pending_enable_ = true;
        RCLCPP_WARN(get_logger(), "admittance requested; waiting for state");
        return;
      }
      enterAdmittanceMode();
    } else {
      pending_enable_ = false;
      enterHoldMode();
    }
    if (enabled && have_state_) {
      // Publish the new measured-pose reference before announcing ACTIVE.
      updateAndPublish();
    } else {
      publishStatus();
    }
  }

  void enterAdmittanceMode()
  {
    if (!have_state_) {
      pending_enable_ = true;
      return;
    }

    mode_ = Mode::ADMITTANCE;
    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);

    base_pos_world_ = pos_;
    const Eigen::Vector3d approach_direction_world = admittance::maskTranslation(
      r_wb * admittance::maskTranslation(n_face_body_, axis_enabled_), axis_enabled_);
    approach_active_ = auto_approach_ && approach_direction_world.norm() > 1e-6;
    approach_travel_m_ = 0.0;
    base_rotation_world_ = r_wb;
    q_.setZero();
    dq_.setZero();
    contact_offset_world_.setZero();
    admittance_blend_ = 0.0;
    if (approach_active_) {
      admittance_blend_ = 1.0;
    }
    wrench_filter_initialized_ = false;
    normal_force_integral_ = 0.0;
    adaptive_preload_ramp_.reset();

    const Eigen::Matrix<double, 6, 1> wrench_c = rawContactWrench();
    const double current_normal_force = std::max(
      0.0, -n_face_body_.dot(wrench_c.head<3>()));
    f_normal_ref_active_ = std::clamp(
      current_normal_force,
      f_normal_min_,
      f_normal_des_);

    // Offset is zero at activation. The wrench controller adds subsequent
    // displacements to its existing position/attitude references, retaining
    // the same PID cascade and the hover-compensating integrator states.
    entry_pos_ref_world_ = base_pos_world_;
    entry_att_ref_rad_ = rpy_;
    // This node's HOLD output is not selected by the wrench controller during
    // Mode 3. Do not let its stale startup reference seed the rate limiter.
    last_pos_ref_world_ = entry_pos_ref_world_;
    last_att_ref_rad_ = entry_att_ref_rad_;
    output_initialized_ = true;

    RCLCPP_INFO(
      get_logger(),
      "ADMITTANCE ON: normal force %.3f N -> desired %.3f N",
      current_normal_force,
      f_normal_des_);
  }

  void captureCurrentHoldPose()
  {
    hold_pos_world_ = pos_;
    hold_att_rad_ = rpy_;
    if (output_initialized_) {
      hold_att_rad_.z() = unwrapNear(hold_att_rad_.z(), last_att_ref_rad_.z());
    }
    hold_initialized_ = true;
  }

  void enterHoldMode()
  {
    mode_ = Mode::HOLD;
    approach_active_ = false;
    adaptive_preload_ramp_.reset();
    hold_initialized_ = false;
    if (!have_state_) {
      return;
    }

    // Requirement: when admittance is disabled, the lower position/attitude
    // controller receives the pose measured at that exact moment, not an old
    // upstream reference.
    captureCurrentHoldPose();

    q_.setZero();
    dq_.setZero();
    contact_offset_world_.setZero();
    normal_force_integral_ = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "ADMITTANCE OFF -> HOLD at p=[%.3f %.3f %.3f], rpy=[%.2f %.2f %.2f] deg",
      hold_pos_world_.x(), hold_pos_world_.y(), hold_pos_world_.z(),
      hold_att_rad_.x() * kRadToDeg,
      hold_att_rad_.y() * kRadToDeg,
      hold_att_rad_.z() * kRadToDeg);
  }

  void enterPassthroughMode()
  {
    mode_ = Mode::PASSTHROUGH;
    adaptive_preload_ramp_.reset();
    pending_enable_ = false;
    q_.setZero();
    dq_.setZero();
    contact_offset_world_.setZero();
    normal_force_integral_ = 0.0;
    RCLCPP_INFO(get_logger(), "PASSTHROUGH mode");
    publishStatus();
  }

  bool wrenchIsFresh(const rclcpp::Time & t_now) const
  {
    if (!have_external_wrench_) {
      return false;
    }
    const double age = (t_now - last_wrench_time_).seconds();
    return std::isfinite(age) && age >= 0.0 &&
      (wrench_timeout_sec_ <= 0.0 || age <= wrench_timeout_sec_);
  }

  bool normalAxisAvailable() const
  {
    if (!have_state_) return false;
    // Check that normal motion survives both the virtual-axis and World-output masks.
    const auto direction = admittance::maskTranslation(
      rotationWorldFromBody(rpy_) * admittance::maskTranslation(n_face_body_, axis_enabled_),
      axis_enabled_);
    return direction.norm() > 1e-6;
  }

  Eigen::Matrix<double, 6, 1> rawContactWrench() const
  {
    // The MoB publishes the resultant external wrench about the CoM B.
    // C shares Body axes, so only the moment's reference point changes.
    Eigen::Matrix<double, 6, 1> wrench_c;
    wrench_c.head<3>() = external_force_body_;
    wrench_c.tail<3>() =
      external_moment_body_ - contact_center_body_.cross(external_force_body_);
    return wrench_c;
  }

  void updateFilteredWrench(double dt, const Eigen::Matrix<double, 6, 1> & raw)
  {
    if (!wrench_filter_initialized_) {
      wrench_filtered_c_ = raw;
      wrench_filter_initialized_ = true;
      return;
    }

    for (std::size_t i = 0; i < kDof; ++i) {
      if (cutoff_hz_[i] <= 0.0) {
        wrench_filtered_c_[i] = raw[i];
        continue;
      }
      const double alpha = 1.0 - std::exp(-2.0 * kPi * cutoff_hz_[i] * dt);
      wrench_filtered_c_[i] +=
        std::clamp(alpha, 0.0, 1.0) * (raw[i] - wrench_filtered_c_[i]);
    }
  }

  Eigen::Matrix<double, 6, 1> computeAdmittanceInput(double dt)
  {
    // External wrench convention:
    //   measured wrench = environment acting on robot.
    // Desired wrench at the plate center has -Fdes*n along the plate normal,
    // zero tangential force and zero contact moment.
    Eigen::Matrix<double, 6, 1> desired_external_wrench_c =
      Eigen::Matrix<double, 6, 1>::Zero();
    desired_external_wrench_c.head<3>() =
      -f_normal_ref_active_ * n_face_body_;

    Eigen::Matrix<double, 6, 1> input =
      wrench_filtered_c_ - desired_external_wrench_c;

    const double f_normal_hat = std::max(
      0.0, -n_face_body_.dot(wrench_filtered_c_.head<3>()));
    const double normal_error = f_normal_ref_active_ - f_normal_hat;

    normal_force_integral_ += normal_error * dt;
    normal_force_integral_ = std::clamp(
      normal_force_integral_,
      -normal_force_integral_limit_,
      normal_force_integral_limit_);
    input.head<3>() +=
      normal_force_integral_gain_ * normal_force_integral_ * n_face_body_;

    for (std::size_t i = 0; i < kDof; ++i) {
      if (!axis_enabled_[i]) {
        input[i] = 0.0;
        q_[i] = 0.0;
        dq_[i] = 0.0;
        continue;
      }
      if (std::abs(input[i]) <= input_deadband_[i]) {
        input[i] = 0.0;
      }
      input[i] = std::clamp(input[i], -input_max_[i], input_max_[i]);
    }
    return input;
  }

  void integrateAdmittance(
    const Eigen::Matrix<double, 6, 1> & input, double dt,
    const Eigen::Matrix3d & r_wb)
  {
    // Resolve the accumulated contact-point offset in CURRENT plate axes.
    // Rotate only incremental motion into World, so changing attitude alone
    // never rotates an already integrated position command.
    const Eigen::Vector3d q_contact_before =
      r_wb.transpose() * contact_offset_world_;
    q_.head<3>() = q_contact_before;
    for (std::size_t i = 0; i < kDof; ++i) {
      if (!axis_enabled_[i]) {
        q_[i] = 0.0;
        dq_[i] = 0.0;
        continue;
      }

      double ddq =
        (input[i] - adm_d_[i] * dq_[i] - adm_k_[i] * q_[i]) / adm_m_[i];
      ddq = std::clamp(ddq, -ddq_max_[i], ddq_max_[i]);

      dq_[i] += ddq * dt;
      dq_[i] = std::clamp(dq_[i], -dq_max_[i], dq_max_[i]);

      q_[i] += dq_[i] * dt;
      if (q_[i] >= q_max_[i]) {
        q_[i] = q_max_[i];
        if (dq_[i] > 0.0) {
          dq_[i] = 0.0;
        }
      } else if (q_[i] <= -q_max_[i]) {
        q_[i] = -q_max_[i];
        if (dq_[i] < 0.0) {
          dq_[i] = 0.0;
        }
      }
    }
    contact_offset_world_ += admittance::contactIncrementWorld(
      r_wb, q_contact_before, q_.head<3>(), axis_enabled_);
    // Do not accumulate hidden World offsets while their outputs are disabled.
    contact_offset_world_ = admittance::maskTranslation(contact_offset_world_, axis_enabled_);
  }

  void computeOutputReference(
    Eigen::Vector3d & pos_ref_world,
    Eigen::Vector3d & att_ref_rad) const
  {
    if (mode_ == Mode::PASSTHROUGH) {
      pos_ref_world = have_cmd_ ? upstream_pos_cmd_ : pos_;
      att_ref_rad = have_att_cmd_ ? upstream_att_cmd_rad_ : rpy_;
      return;
    }

    if (mode_ == Mode::HOLD) {
      pos_ref_world = hold_initialized_ ? hold_pos_world_ : pos_;
      att_ref_rad = hold_initialized_ ? hold_att_rad_ : rpy_;
      return;
    }

    // The angular virtual state follows the current plate/Body axes.
    const Eigen::Vector3d delta_rotation_world =
      rotationWorldFromBody(rpy_) * q_.tail<3>();
    const Eigen::Matrix3d r_ref =
      rotationExp(delta_rotation_world) * base_rotation_world_;
    const Eigen::Vector3d admittance_att_ref = rpyFromRotation(r_ref);

    // A rotation about the CoM otherwise sweeps the plate center. Compensate
    // that motion so the translation state describes the contact center C.
    const Eigen::Vector3d admittance_pos_ref =
      base_pos_world_ + contact_offset_world_ +
      (base_rotation_world_ - r_ref) * contact_center_body_;

    pos_ref_world =
      (1.0 - admittance_blend_) * entry_pos_ref_world_ +
      admittance_blend_ * admittance_pos_ref;
    att_ref_rad =
      (1.0 - admittance_blend_) * entry_att_ref_rad_ +
      admittance_blend_ * admittance_att_ref;

    if (output_initialized_) {
      att_ref_rad.x() = unwrapNear(att_ref_rad.x(), last_att_ref_rad_.x());
      att_ref_rad.y() = unwrapNear(att_ref_rad.y(), last_att_ref_rad_.y());
      att_ref_rad.z() = unwrapNear(att_ref_rad.z(), last_att_ref_rad_.z());
    }
  }

  void updateAndPublish()
  {
    const rclcpp::Time t_now = now();
    double dt = (t_now - last_control_time_).seconds();
    last_control_time_ = t_now;
    const bool valid_dt = std::isfinite(dt) && dt > 0.0 && dt <= 0.2;
    if (!valid_dt) {
      dt = 1.0 / 400.0;
    }

    const bool wrench_fresh = wrenchIsFresh(t_now);
    Eigen::Matrix<double, 6, 1> raw_contact_wrench =
      Eigen::Matrix<double, 6, 1>::Zero();
    if (mode_ == Mode::ADMITTANCE && wrench_fresh) {
      raw_contact_wrench = rawContactWrench();
      hih_interfaces::msg::Wrench contact_msg;
      for (std::size_t i = 0; i < 3; ++i) {
        contact_msg.force[i] = static_cast<float>(raw_contact_wrench[i]);
        contact_msg.moment[i] = static_cast<float>(raw_contact_wrench[i + 3]);
      }
      pub_contact_wrench_->publish(contact_msg);
    }
    if (mode_ == Mode::ADMITTANCE) {
      if (approach_active_) {
        if (wrench_fresh) {
          const double normal = std::max(
            0.0, -n_face_body_.dot(raw_contact_wrench.head<3>()));
          if (normal >= approach_contact_force_n_) {
            approach_active_ = false;
            base_pos_world_ = last_pos_ref_world_;
            // Keep the original activation pose as the offset origin. Resetting
            // it at contact would suddenly remove the approach displacement.
            f_normal_ref_active_ = std::clamp(normal, f_normal_min_, f_normal_des_);
            wrench_filter_initialized_ = false;
            RCLCPP_INFO(get_logger(), "local contact %.3f N; force admittance begins", normal);
          } else if (approach_travel_m_ < approach_max_distance_m_) {
            const Eigen::Vector3d direction_world = admittance::maskTranslation(
              rotationWorldFromBody(rpy_) *
              admittance::maskTranslation(n_face_body_, axis_enabled_), axis_enabled_);
            const double direction_norm = direction_world.norm();
            if (direction_norm > 1e-6) {
              const double step = std::min(approach_speed_mps_ * dt,
                (approach_max_distance_m_ - approach_travel_m_) / direction_norm);
              const Eigen::Vector3d displacement = direction_world * step;
              base_pos_world_ += displacement;
              approach_travel_m_ += displacement.norm();
            } else {
              approach_active_ = false;
              RCLCPP_INFO(get_logger(), "automatic approach stopped: no enabled approach direction");
            }
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "approach travel exhausted without local contact");
          }
        }
      }
      if (!approach_active_) {
        admittance_blend_ = handoff_blend_time_ <= 1e-6 ? 1.0 :
          moveToward(admittance_blend_, 1.0, dt / handoff_blend_time_);
        if (wrench_fresh) {
          updateFilteredWrench(dt, raw_contact_wrench);
        }
        // Observe contact force before masking/saturation; Ft remains valid with y/z OFF.
        const Eigen::Vector3d force_c = wrench_filtered_c_.head<3>();
        const double filtered_normal = -n_face_body_.dot(force_c);
        const double raw_normal = -n_face_body_.dot(raw_contact_wrench.head<3>());
        const double tangent = (force_c - n_face_body_ * n_face_body_.dot(force_c)).norm();
        f_normal_ref_active_ = adaptive_preload_ramp_.advance(
          f_normal_ref_active_, f_normal_des_, f_normal_min_, f_normal_max_,
          f_ref_rate_max_, valid_dt ? dt : 0.0,
          tangent, raw_normal, filtered_normal,
          wrench_fresh && wrench_filter_initialized_, normalAxisAvailable(),
          adaptive_preload_settings_);

        if (wrench_fresh) {
          const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);
          const Eigen::Matrix<double, 6, 1> input = computeAdmittanceInput(dt);
          integrateAdmittance(input, dt, r_wb);
          if (!q_.array().isFinite().all() || !dq_.array().isFinite().all() ||
            !contact_offset_world_.array().isFinite().all())
          {
            q_.setZero();
            dq_.setZero();
            contact_offset_world_.setZero();
            normal_force_integral_ = 0.0;
            RCLCPP_ERROR(get_logger(), "non-finite admittance state rejected and reset");
          }
        } else {
          // Never interpret a stale/missing estimator value as zero contact force.
          wrench_filter_initialized_ = false;
          dq_.setZero();
          q_.head<3>() = admittance::maskTranslation(
            rotationWorldFromBody(rpy_).transpose() * contact_offset_world_, axis_enabled_);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "external wrench missing/stale: admittance state frozen");
        }
      }
    }

    Eigen::Vector3d pos_ref_world;
    Eigen::Vector3d att_ref_rad;
    computeOutputReference(pos_ref_world, att_ref_rad);
    if (mode_ == Mode::ADMITTANCE) {
      const double maximum_step = output_initialized_ && reference_rate_limit_ > 0.0 ?
        reference_rate_limit_ * dt : std::numeric_limits<double>::infinity();
      const auto reference = admittance::constrainOutputReference(
        pos_ref_world, att_ref_rad, entry_pos_ref_world_, entry_att_ref_rad_,
        last_pos_ref_world_, axis_enabled_, maximum_step);
      pos_ref_world = reference.position_world;
      att_ref_rad = reference.attitude_rad;
    } else if (output_initialized_ && reference_rate_limit_ > 0.0) {
      Eigen::Vector3d delta = pos_ref_world - last_pos_ref_world_;
      const double max_delta = reference_rate_limit_ * dt;
      if (delta.norm() > max_delta && delta.norm() > 1e-12) {
        pos_ref_world =
          last_pos_ref_world_ + delta * (max_delta / delta.norm());
      }
    }

    // The lower controller expects a World-frame CoM offset. The virtual
    // translation is integrated at C; the approach and rotation compensation
    // are included relative to the reference captured at activation, subject
    // to the final World/RPY output mask above.
    const Eigen::Vector3d position_offset_world =
      pos_ref_world - entry_pos_ref_world_;
    hih_interfaces::msg::Cmd cmd_msg;
    cmd_msg.pos_cmd[0] = static_cast<float>(position_offset_world.x());
    cmd_msg.pos_cmd[1] = static_cast<float>(position_offset_world.y());
    cmd_msg.pos_cmd[2] = static_cast<float>(position_offset_world.z());

    hih_interfaces::msg::AttitudeCmd att_msg;
    // Relative RPY angles are added to the existing attitude target. Angle
    // unwrapping in computeOutputReference keeps yaw offset continuous.
    const Eigen::Vector3d attitude_offset_rad = att_ref_rad - entry_att_ref_rad_;
    att_msg.roll_ref = static_cast<float>(attitude_offset_rad.x() * kRadToDeg);
    att_msg.pitch_ref = static_cast<float>(attitude_offset_rad.y() * kRadToDeg);
    att_msg.yaw_ref = static_cast<float>(attitude_offset_rad.z() * kRadToDeg);

    if (mode_ == Mode::ADMITTANCE) {
      pub_cmd_->publish(cmd_msg);
      pub_att_cmd_->publish(att_msg);
    }

    last_pos_ref_world_ = pos_ref_world;
    last_att_ref_rad_ = att_ref_rad;
    output_initialized_ = true;
    publishStatus();
  }

  void publishStatus()
  {
    std_msgs::msg::Bool active_msg;
    active_msg.data = mode_ == Mode::ADMITTANCE;
    pub_active_->publish(active_msg);

    std_msgs::msg::Float64 force_msg;
    force_msg.data = f_normal_des_;
    pub_des_force_->publish(force_msg);
    force_msg.data = admittance_blend_;
    pub_blend_->publish(force_msg);

    // Publish status at 10 Hz to avoid excessive CSV rows from 400 Hz updates.
    const auto t_now = now();
    if (preload_status_initialized_) {
      const double age = (t_now - last_preload_status_time_).seconds();
      if (age >= 0.0 && age < adaptive_preload_tuning::status_period_sec) return;
    }
    preload_status_initialized_ = true;
    last_preload_status_time_ = t_now;
    force_msg.data = adaptive_preload_ramp_.target(f_normal_des_, f_normal_min_, f_normal_max_);
    pub_preload_target_->publish(force_msg);
    const bool normal_active = mode_ == Mode::ADMITTANCE && !approach_active_ && normalAxisAvailable();
    force_msg.data = normal_active ? f_normal_ref_active_ : 0.0;
    pub_normal_reference_->publish(force_msg);
    force_msg.data = adaptive_preload_ramp_.tangentForce();
    pub_tangent_force_->publish(force_msg);
    active_msg.data = adaptive_preload_settings_.enabled;
    pub_preload_enabled_->publish(active_msg);
    active_msg.data = normal_active && adaptive_preload_ramp_.adapting();
    pub_preload_active_->publish(active_msg);
  }

  void adjustDesiredForce(double delta)
  {
    const double old_force = f_normal_des_;
    f_normal_des_ = std::clamp(
      f_normal_des_ + delta,
      f_normal_min_,
      f_normal_max_);
    RCLCPP_INFO(
      get_logger(),
      "desired normal force %.2f -> %.2f N",
      old_force,
      f_normal_des_);
    publishStatus();
  }

  void setupKeyboard()
  {
    keyboard_enabled_ = isatty(STDIN_FILENO);
    if (!keyboard_enabled_) {
      RCLCPP_WARN(get_logger(), "stdin is not a TTY; keyboard disabled");
      return;
    }

    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
      keyboard_enabled_ = false;
      RCLCPP_WARN(get_logger(), "failed to read terminal settings");
      return;
    }

    termios raw = old_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      keyboard_enabled_ = false;
      RCLCPP_WARN(get_logger(), "failed to set terminal raw mode");
      return;
    }
    termios_configured_ = true;
  }

  void restoreKeyboard()
  {
    if (termios_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
      termios_configured_ = false;
    }
  }

  void pollKeyboard()
  {
    if (!keyboard_enabled_) {
      return;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);
    timeval timeout{0, 0};

    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      return;
    }

    char key = '\0';
    if (read(STDIN_FILENO, &key, 1) != 1) {
      return;
    }

    if (key == 'O' || key == 'o') {
      requestEnabled(mode_ != Mode::ADMITTANCE);
    } else if (key == 'P' || key == 'p') {
      requestEnabled(false);
      enterPassthroughMode();
    } else if (key == 'U' || key == 'u') {
      adjustDesiredForce(+f_normal_step_);
    } else if (key == 'J' || key == 'j') {
      adjustDesiredForce(-f_normal_step_);
    } else if (key == 'B' || key == 'b') {
      requestAdaptivePreload(!adaptive_preload_settings_.enabled);
    }
  }

  // ROS interfaces
  rclcpp::Subscription<hih_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<hih_interfaces::msg::AttitudeCmd>::SharedPtr sub_att_cmd_;
  rclcpp::Subscription<hih_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<hih_interfaces::msg::Wrench>::SharedPtr sub_external_wrench_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;

  rclcpp::Publisher<hih_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::Publisher<hih_interfaces::msg::AttitudeCmd>::SharedPtr pub_att_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_active_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_des_force_;
  rclcpp::Publisher<hih_interfaces::msg::Wrench>::SharedPtr pub_contact_wrench_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_blend_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_preload_target_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_normal_reference_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_tangent_force_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_preload_enabled_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_preload_active_;

  rclcpp::TimerBase::SharedPtr keyboard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Input commands and state
  Eigen::Vector3d upstream_pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upstream_att_cmd_rad_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_moment_body_{Eigen::Vector3d::Zero()};

  // Contact C follows the Body axes and is displaced from CoM B by r_BC.
  Eigen::Matrix3d base_rotation_world_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d n_face_body_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d contact_center_body_{Eigen::Vector3d::UnitX() * 0.215};
  Eigen::Vector3d base_pos_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_offset_world_{Eigen::Vector3d::Zero()};

  // HOLD pose and last published reference
  Eigen::Vector3d hold_pos_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hold_att_rad_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_pos_ref_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_att_ref_rad_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d entry_pos_ref_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d entry_att_ref_rad_{Eigen::Vector3d::Zero()};

  // Virtual C-frame state; contact-point translation accumulates in World.
  Eigen::Matrix<double, 6, 1> q_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> dq_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> wrench_filtered_c_{
    Eigen::Matrix<double, 6, 1>::Zero()};

  std::array<bool, kDof> axis_enabled_{};
  std::array<double, kDof> adm_m_{};
  std::array<double, kDof> adm_d_{};
  std::array<double, kDof> adm_k_{};
  std::array<double, kDof> cutoff_hz_{};
  std::array<double, kDof> input_deadband_{};
  std::array<double, kDof> input_max_{};
  std::array<double, kDof> ddq_max_{};
  std::array<double, kDof> dq_max_{};
  std::array<double, kDof> q_max_{};

  double f_normal_des_{6.0};
  double f_normal_step_{0.1};
  double f_normal_min_{0.0};
  double f_normal_max_{20.0};
  double f_ref_rate_max_{1.0};
  double f_normal_ref_active_{0.0};
  // The constructor populates all settings from the tuning block and ROS parameters.
  adaptive_preload::Settings adaptive_preload_settings_;
  adaptive_preload::Ramp adaptive_preload_ramp_;
  double normal_force_integral_gain_{0.0};
  double normal_force_integral_limit_{1.0};
  double normal_force_integral_{0.0};
  double wrench_timeout_sec_{0.10};
  double handoff_blend_time_{2.0};
  double reference_rate_limit_{0.30};
  double admittance_blend_{0.0};
  double approach_speed_mps_{0.05};
  double approach_contact_force_n_{0.8};
  double approach_max_distance_m_{0.34};
  double approach_travel_m_{0.0};
  bool auto_approach_{false};
  bool approach_active_{false};

  rclcpp::Time last_control_time_;
  rclcpp::Time last_wrench_time_;
  rclcpp::Time last_preload_status_time_;
  bool preload_status_initialized_{false};

  Mode mode_{Mode::PASSTHROUGH};
  bool pending_enable_{false};
  bool have_cmd_{false};
  bool have_att_cmd_{false};
  bool have_state_{false};
  bool have_external_wrench_{false};
  bool wrench_filter_initialized_{false};
  bool output_initialized_{false};
  bool hold_initialized_{false};
  bool keyboard_enabled_{false};
  bool termios_configured_{false};
  termios old_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerAdmittanceController6Dof>());
  rclcpp::shutdown();
  return 0;
}

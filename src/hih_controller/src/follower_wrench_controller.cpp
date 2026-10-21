// Follower-specific wrench controller.
// This file is intentionally independent from the Leader version so
// code-level tuning defaults and future control logic can diverge per vehicle.
// Default ROS namespace is hard-coded to /follower; standalone ros2 run needs no __ns remap.

#include <rclcpp/rclcpp.hpp>
#include <hih_interfaces/msg/cmd.hpp>
#include <hih_interfaces/msg/minitrone_state.hpp>
#include <hih_interfaces/msg/wrench.hpp>
#include <hih_interfaces/msg/torque_dob.hpp>
#include <hih_interfaces/msg/attitude_cmd.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <limits>
#include <stdexcept>
#include <array>
#include <chrono>
#include <tuple>
#include <vector>

namespace {
// Twelve axis rows are the single source of all 36 independently declared ROS
// PID gains. PX4 names use MPC_* and MC_*; these explicit layer/axis names avoid
// the PX4 firmware's shared XY parameters and expose the requested full PID.
struct GainRow { const char * layer; const char * axis; double kp, ki, kd; };
constexpr std::array<GainRow, 12> kGainDefaults{{
  {"POS", "X", 24.0/6.0, 1.5/6.0, 0.01},
  {"POS", "Y", 24.0/6.0, 1.5/6.0, 0.01},
  {"POS", "Z", 24.0/10.0, 1.2/10.0, 0.01},
  {"VEL", "X", 6.0/2.7, 0.01, 0.01},
  {"VEL", "Y", 6.0/2.7, 0.01, 0.01},
  {"VEL", "Z", 10.0/2.7, 0.01, 0.01},
  {"ATT", "ROLL", 6.0, 0.00, 0.8},
  {"ATT", "PITCH", 6.0, 0.00, 0.8},
  {"ATT", "YAW", 6.0, 0.00, 0.8},
  {"RATE", "ROLL", 1.0, 0.000, 0.001},
  {"RATE", "PITCH", 1.0, 0.000, 0.001},
  {"RATE", "YAW", 1.0, 0.000, 0.001},
}};
static_assert(kGainDefaults.size() * 3 == 36, "four 3-axis PID layers require 36 gains");

struct PidLayer {
  Eigen::Vector3d kp{Eigen::Vector3d::Zero()}, ki{Eigen::Vector3d::Zero()}, kd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d integral{Eigen::Vector3d::Zero()};
  Eigen::Vector3d error{Eigen::Vector3d::Zero()}, p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d i{Eigen::Vector3d::Zero()}, d{Eigen::Vector3d::Zero()};
  Eigen::Vector3d output{Eigen::Vector3d::Zero()};

  void reset() { integral.setZero(); error.setZero(); p.setZero(); i.setZero(); d.setZero(); output.setZero(); }

  Eigen::Vector3d update(const Eigen::Vector3d & e, const Eigen::Vector3d & measured_derivative,
                         double dt, const Eigen::Vector3d & out_limit,
                         const Eigen::Vector3d & integral_limit, bool allow_integration)
  {
    // PX4-inspired derivative on measurement avoids setpoint kicks. The caller
    // low-pass filters measured derivatives before passing them to this layer.
    error = e;
    p = kp.cwiseProduct(e);
    d = -kd.cwiseProduct(measured_derivative);
    for (int a = 0; a < 3; ++a) {
      const double candidate = std::clamp(integral[a] + e[a] * dt,
                                          -integral_limit[a], integral_limit[a]);
      const double unsaturated = p[a] + ki[a] * candidate + d[a];
      // Conditional integration: stop accumulating into an output limit.
      if (allow_integration && !((unsaturated > out_limit[a] && e[a] > 0.0) ||
            (unsaturated < -out_limit[a] && e[a] < 0.0))) {
        integral[a] = candidate;
      }
      i[a] = ki[a] * integral[a];
      output[a] = std::clamp(p[a] + i[a] + d[a], -out_limit[a], out_limit[a]);
    }
    return output;
  }
};

Eigen::Matrix3d rotationWorldFromBody(const Eigen::Vector3d & rpy)
{
  // MuJoCo framequat is wxyz and maps body to world. State.rpy is XYZ Euler
  // extracted from it, so R_WB = Rz(yaw) Ry(pitch) Rx(roll).
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Vector3d attitudeErrorBody(const Eigen::Matrix3d & desired,
                                  const Eigen::Matrix3d & actual)
{
  // PX4 uses 2*imag(q_current^{-1} q_desired), canonicalizing the quaternion
  // to select the shortest rotation. The error is expressed in body axes.
  Eigen::Quaterniond q(actual.transpose() * desired);
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  return 2.0 * q.vec();
}
}  // namespace

class FollowerWrenchController : public rclcpp::Node
{
public:
  static constexpr double deg_to_rad = M_PI / 180.0;

  FollowerWrenchController() : rclcpp::Node("follower_wrench_controller", "/follower")
  {
    // ===================== (MINITRONE) params =====================
    // 
    this->declare_parameter<double>("mass", 2.6);
    this->declare_parameter<double>("gravity", 9.81);
    mass_ = this->get_parameter("mass").as_double();
    grav_ = this->get_parameter("gravity").as_double();
    hold_initial_position_ = declare_parameter<bool>("hold_initial_position", false);
    admittance_off_debounce_sec_ = std::max(
      0.0, declare_parameter<double>("admittance_off_debounce_sec", 0.05));
    // Optional Torque DoB messages are accepted only while fresh. If the
    // separately-run DoB node is absent or dies, the correction becomes zero
    // and the original PID -> allocator path remains intact.
    dob_timeout_sec_ = std::max(
      0.02, declare_parameter<double>("dob_timeout_sec", 0.15));

    cop_safety_timeout_sec_ = std::max(
      0.02, declare_parameter<double>("cop_safety_timeout_sec", 0.15));
    cop_translation_timeout_sec_ = std::max(
      0.02, declare_parameter<double>("cop_translation_timeout_sec", 0.20));
    suppress_pitch_dob_when_cop_safety_active_ = declare_parameter<bool>(
      "suppress_pitch_dob_when_cop_safety_active", true);
    suppress_yaw_dob_when_cop_safety_active_ = declare_parameter<bool>(
      "suppress_yaw_dob_when_cop_safety_active", true);

    if (!std::isfinite(mass_) || mass_ <= 0.0 || !std::isfinite(grav_)) {
      throw std::invalid_argument("mass must be positive and gravity must be finite");
    }

    // All gains are explicitly declared here, one scalar per layer/axis/term.
    // Initial values retain the old MuJoCo translation gain product and
    // attitude torque scale; the newly added I/D terms are conservative.
    for (std::size_t row = 0; row < kGainDefaults.size(); ++row) {
      const auto & spec = kGainDefaults[row];
      PidLayer & layer = layers_[row / 3];
      const int axis = static_cast<int>(row % 3);
      const std::string suffix = std::string("_") + spec.layer + "_" + spec.axis;
      layer.kp[axis] = declareGain("KP" + suffix, spec.kp);
      layer.ki[axis] = declareGain("KI" + suffix, spec.ki);
      layer.kd[axis] = declareGain("KD" + suffix, spec.kd);
    }
    velocity_limit_ << declarePositive("velocity_limit_x", 2.0),
      declarePositive("velocity_limit_y", 2.0), declarePositive("velocity_limit_z", 2.0);
    acceleration_limit_ << declarePositive("acceleration_limit_x", 20.0),
      declarePositive("acceleration_limit_y", 20.0), declarePositive("acceleration_limit_z", 20.0);
    rate_limit_ << declarePositive("rate_limit_roll", 4.0),
      declarePositive("rate_limit_pitch", 4.0), declarePositive("rate_limit_yaw", 3.0);
    torque_limit_ << declarePositive("torque_limit_roll", 5.0),
      declarePositive("torque_limit_pitch", 5.0), declarePositive("torque_limit_yaw", 5.0);
    pos_integral_limit_ = declarePositive("position_integral_limit", 5.0);
    vel_integral_limit_ = declarePositive("velocity_integral_limit", 5.0);
    att_integral_limit_ = declarePositive("attitude_integral_limit", 0.5);
    rate_integral_limit_ = declarePositive("rate_integral_limit", 0.5);
    derivative_cutoff_hz_ = declarePositive("derivative_cutoff_hz", 30.0);
    gain_callback_ = add_on_set_parameters_callback(
      std::bind(&FollowerWrenchController::onGainParameters, this, std::placeholders::_1));

    const std::string cmd_topic =
      declare_parameter<std::string>("cmd_topic", "minitrone/cmd");
    const std::string att_cmd_topic =
      declare_parameter<std::string>("att_cmd_topic", "minitrone/att_cmd");
    const std::string admittance_cmd_topic =
      declare_parameter<std::string>("admittance_cmd_topic", "minitrone/cmd_admittance");
    const std::string admittance_att_cmd_topic =
      declare_parameter<std::string>("admittance_att_cmd_topic", "minitrone/att_cmd_admittance");
    const std::string admittance_active_topic =
      declare_parameter<std::string>("admittance_active_topic", "minitrone/admittance_active");
    const std::string cop_safety_offset_topic =
      declare_parameter<std::string>(
        "cop_safety_offset_topic", "minitrone/cop_safety_attitude_offset");
    const std::string cop_safety_active_topic =
      declare_parameter<std::string>(
        "cop_safety_active_topic", "minitrone/cop_safety_active");
    const std::string cop_translation_offset_topic =
      declare_parameter<std::string>(
        "cop_translation_offset_topic", "minitrone/cop_translation_position_offset");

    // ===================== ROS I/O =====================
    sub_cmd_ = this->create_subscription<hih_interfaces::msg::Cmd>(
      cmd_topic, 10, std::bind(&FollowerWrenchController::onCmd, this, std::placeholders::_1));

    sub_admittance_cmd_ = this->create_subscription<hih_interfaces::msg::Cmd>(
      admittance_cmd_topic, 10,
      std::bind(&FollowerWrenchController::onAdmittanceCmd, this, std::placeholders::_1));

    sub_state_ = this->create_subscription<hih_interfaces::msg::MinitroneState>(
      "minitrone/state", 10, std::bind(&FollowerWrenchController::onState, this, std::placeholders::_1));

    sub_att_cmd_ = this->create_subscription<hih_interfaces::msg::AttitudeCmd>(
      att_cmd_topic, 10, std::bind(&FollowerWrenchController::onAttCmd, this, std::placeholders::_1));

    sub_admittance_att_cmd_ =
      this->create_subscription<hih_interfaces::msg::AttitudeCmd>(
      admittance_att_cmd_topic, 10,
      std::bind(&FollowerWrenchController::onAdmittanceAttCmd, this, std::placeholders::_1));

    sub_admittance_active_ = this->create_subscription<std_msgs::msg::Bool>(
      admittance_active_topic, 10,
      std::bind(&FollowerWrenchController::onAdmittanceActive, this, std::placeholders::_1));

    sub_cop_safety_offset_ =
      this->create_subscription<hih_interfaces::msg::AttitudeCmd>(
      cop_safety_offset_topic, 10,
      std::bind(&FollowerWrenchController::onCopSafetyOffset, this, std::placeholders::_1));
    sub_cop_safety_active_ = this->create_subscription<std_msgs::msg::Bool>(
      cop_safety_active_topic, 10,
      std::bind(&FollowerWrenchController::onCopSafetyActive, this, std::placeholders::_1));

    sub_cop_translation_offset_ = this->create_subscription<hih_interfaces::msg::Cmd>(
      cop_translation_offset_topic, 10,
      [this](const hih_interfaces::msg::Cmd::SharedPtr msg) {
        const Eigen::Vector3d incoming(
          static_cast<double>(msg->pos_cmd[0]),
          static_cast<double>(msg->pos_cmd[1]),
          static_cast<double>(msg->pos_cmd[2]));
        if (!incoming.allFinite()) return;
        cop_translation_position_offset_ = incoming;
        cop_translation_offset_last_rx_ = std::chrono::steady_clock::now();
        have_cop_translation_offset_ = true;
      });

    sub_reset_ = this->create_subscription<std_msgs::msg::Bool>(
      "minitrone/controller_reset", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { if (msg->data) resetControllers(); });
    sub_allocator_saturated_ = this->create_subscription<std_msgs::msg::Bool>(
      "minitrone/allocator_saturated", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { allocator_saturated_ = msg->data; });

    // Thin optional interface for the separately-run Torque DoB.
    // Relative topic names make one executable work independently under
    // /leader and /follower namespaces. This controller never starts the DoB.
    sub_dob_status_ = this->create_subscription<hih_interfaces::msg::TorqueDob>(
      "minitrone/dob_status", 10,
      std::bind(&FollowerWrenchController::onDobStatus, this, std::placeholders::_1));

    // The passive-aligning filter consumes the conventional controller output.
    pub_wrench_ = this->create_publisher<hih_interfaces::msg::Wrench>(
      "minitrone/wrench_cmd", 10);
    pub_att_ref_ = this->create_publisher<hih_interfaces::msg::AttitudeCmd>(
      "minitrone/att_ref", 10);

    pos_cmd_.setZero();
    att_cmd_.setZero();
    last_time_ = this->now();
  }

private:
  rcl_interfaces::msg::SetParametersResult onGainParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::vector<std::tuple<std::size_t, int, int, double>> changes;
    for (const auto & parameter : parameters) {
      for (std::size_t row = 0; row < kGainDefaults.size(); ++row) {
        const auto & spec = kGainDefaults[row];
        const std::string suffix = std::string("_") + spec.layer + "_" + spec.axis;
        const std::array<std::string, 3> names{{"KP" + suffix, "KI" + suffix, "KD" + suffix}};
        for (int term = 0; term < 3; ++term) {
          if (parameter.get_name() != names[term]) continue;
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
              !std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0) {
            result.successful = false;
            result.reason = parameter.get_name() + " must be finite and nonnegative double";
            return result;
          }
          changes.emplace_back(row / 3, static_cast<int>(row % 3), term, parameter.as_double());
        }
      }
    }
    for (const auto & [layer, axis, term, value] : changes) {
      if (term == 0) layers_[layer].kp[axis] = value;
      if (term == 1) layers_[layer].ki[axis] = value;
      if (term == 2) layers_[layer].kd[axis] = value;
    }
    if (!changes.empty()) resetControllers();
    return result;
  }

  double declareGain(const std::string & name, double value)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || result < 0.0) {
      throw std::invalid_argument(name + " must be finite and nonnegative");
    }
    return result;
  }

  double declarePositive(const std::string & name, double value)
  {
    const double result = declareGain(name, value);
    if (result <= 0.0) throw std::invalid_argument(name + " must be positive");
    return result;
  }

  void resetControllers()
  {
    // Explicit reset and clock discontinuity reset all PID states. Switching
    // admittance ON/OFF preserves hover compensation already in the integrals.
    for (auto & layer : layers_) layer.reset();
    derivative_initialized_ = false;
    filtered_vel_.setZero(); filtered_accel_.setZero();
    filtered_rate_.setZero(); filtered_alpha_.setZero();
  }

  void onDobStatus(const hih_interfaces::msg::TorqueDob::SharedPtr msg)
  {
    Eigen::Vector3d incoming(
      static_cast<double>(msg->dob_compensation_torque[0]),
      static_cast<double>(msg->dob_compensation_torque[1]),
      static_cast<double>(msg->dob_compensation_torque[2]));

    dob_last_rx_ = std::chrono::steady_clock::now();

    // Observation may continue while compensation is OFF. Only a valid,
    // keyboard-enabled DoB correction is allowed into the control path.
    if (!msg->dob_observer_valid || !msg->dob_compensation_enabled ||
        !incoming.allFinite()) {
      dob_compensation_.setZero();
      dob_active_ = false;
      return;
    }

    for (int axis = 0; axis < 3; ++axis) {
      if (!msg->dob_axis_enabled[axis]) incoming[axis] = 0.0;
    }
    dob_compensation_ = incoming;
    dob_active_ = true;
  }

  void onCmd(const hih_interfaces::msg::Cmd::SharedPtr msg)
  {
    const Eigen::Vector3d incoming_cmd(
      static_cast<double>(msg->pos_cmd[0]),
      static_cast<double>(msg->pos_cmd[1]),
      static_cast<double>(msg->pos_cmd[2]));
    if (!incoming_cmd.allFinite()) return;
    if (waiting_for_position_sync_) {
      if ((incoming_cmd - pos_cmd_).norm() > position_sync_tolerance_) {
        return;
      }
      waiting_for_position_sync_ = false;
      RCLCPP_INFO(get_logger(), "position teleop synchronized after admittance OFF");
    }
    pos_cmd_ = incoming_cmd;
    have_cmd_ = true;
  }

  void onAdmittanceCmd(const hih_interfaces::msg::Cmd::SharedPtr msg)
  {
    if (!admittance_active_) return;
    const Eigen::Vector3d incoming(static_cast<double>(msg->pos_cmd[0]),
                                   static_cast<double>(msg->pos_cmd[1]),
                                   static_cast<double>(msg->pos_cmd[2]));
    if (!incoming.allFinite()) return;
    // The admittance node publishes a WORLD-frame displacement, not a new
    // absolute position target. The existing position setpoint remains the base.
    admittance_pos_cmd_ = incoming;
    have_admittance_cmd_ = true;
  }

  void onAttCmd(const hih_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    // hih_cmd publishes attitude commands in degrees.
    const Eigen::Vector3d incoming(static_cast<double>(msg->roll_ref) * deg_to_rad,
                                   static_cast<double>(msg->pitch_ref) * deg_to_rad,
                                   static_cast<double>(msg->yaw_ref) * deg_to_rad);
    if (!incoming.allFinite()) return;
    att_cmd_ = incoming;
    have_att_cmd_ = true;
  }

  void onAdmittanceAttCmd(const hih_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    if (!admittance_active_) return;
    const Eigen::Vector3d incoming(static_cast<double>(msg->roll_ref) * deg_to_rad,
                                   static_cast<double>(msg->pitch_ref) * deg_to_rad,
                                   static_cast<double>(msg->yaw_ref) * deg_to_rad);
    if (!incoming.allFinite()) return;
    admittance_att_cmd_ = incoming;
    have_admittance_att_cmd_ = true;
  }

  void onCopSafetyOffset(
    const hih_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    const Eigen::Vector3d incoming(
      static_cast<double>(msg->roll_ref) * deg_to_rad,
      static_cast<double>(msg->pitch_ref) * deg_to_rad,
      static_cast<double>(msg->yaw_ref) * deg_to_rad);
    if (!incoming.allFinite()) return;
    cop_safety_attitude_offset_ = incoming;
    cop_safety_offset_last_rx_ = std::chrono::steady_clock::now();
    have_cop_safety_offset_ = true;
  }

  void onCopSafetyActive(const std_msgs::msg::Bool::SharedPtr msg)
  {
    cop_safety_requested_active_ = msg->data;
    cop_safety_active_last_rx_ = std::chrono::steady_clock::now();
    have_cop_safety_active_ = true;
  }

  bool copSafetyFreshAndActive() const
  {
    if (!have_cop_safety_offset_ || !have_cop_safety_active_ ||
        !cop_safety_requested_active_) {
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const double offset_age = std::chrono::duration<double>(
      now - cop_safety_offset_last_rx_).count();
    const double active_age = std::chrono::duration<double>(
      now - cop_safety_active_last_rx_).count();

    return offset_age >= 0.0 && active_age >= 0.0 &&
      offset_age <= cop_safety_timeout_sec_ &&
      active_age <= cop_safety_timeout_sec_;
  }

  bool copTranslationOffsetFresh() const
  {
    if (!have_cop_translation_offset_) return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      cop_translation_offset_last_rx_).count();
    return age >= 0.0 && age <= cop_translation_timeout_sec_;
  }

  void onAdmittanceActive(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // More than one admittance publisher can otherwise alternate true/false at
    // the state rate.  A true sample is authoritative immediately; require a
    // continuous false interval before leaving admittance mode.
    const auto steady_now = std::chrono::steady_clock::now();
    if (msg->data) {
      admittance_off_pending_ = false;
    } else if (admittance_active_ && admittance_off_debounce_sec_ > 0.0) {
      if (!admittance_off_pending_) {
        admittance_off_pending_ = true;
        admittance_off_since_ = steady_now;
        return;
      }
      const double false_duration =
        std::chrono::duration<double>(steady_now - admittance_off_since_).count();
      if (false_duration < admittance_off_debounce_sec_) {
        return;
      }
      admittance_off_pending_ = false;
    }

    if (admittance_active_ != msg->data) {
      const bool was_active = admittance_active_;
      admittance_active_ = msg->data;
      if (admittance_active_) {
        // Await a new zero-initialized offset from this activation. While
        // waiting, keep the old position/yaw references and PID states.
        have_admittance_cmd_ = false;
        have_admittance_att_cmd_ = false;
      }
      if (was_active && !admittance_active_ && have_state_) {
        // Hand the measured pose to the conventional controller at the exact
        // OFF transition. Ignore the teleop's old 100 Hz command until the
        // teleop synchronizes itself to this captured position.
        pos_cmd_ = pos_;
        att_cmd_ = rpy_;
        have_cmd_ = true;
        have_att_cmd_ = true;
        waiting_for_position_sync_ = true;
      }
      RCLCPP_INFO(
        get_logger(),
        "admittance state changed: %s",
        admittance_active_ ? "ACTIVE" : "OFF (conventional HOLD)");
    }
  }

  void onState(const hih_interfaces::msg::MinitroneState::SharedPtr msg)
  {
    pos_    << static_cast<double>(msg->pos[0]),   static_cast<double>(msg->pos[1]),   static_cast<double>(msg->pos[2]);
    vel_    << static_cast<double>(msg->vel[0]),   static_cast<double>(msg->vel[1]),   static_cast<double>(msg->vel[2]);
    rpy_    << static_cast<double>(msg->rpy[0]),   static_cast<double>(msg->rpy[1]),   static_cast<double>(msg->rpy[2]);
    w_body_ << static_cast<double>(msg->w_rpy[0]), static_cast<double>(msg->w_rpy[1]), static_cast<double>(msg->w_rpy[2]);

    if (!pos_.allFinite() || !vel_.allFinite() || !rpy_.allFinite() || !w_body_.allFinite()) {
      resetControllers();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "non-finite state rejected");
      return;
    }

    // The cooperation scene can arm each controller at its own first measured
    // position. A later position command replaces this hold target normally.
    if (!have_cmd_ && hold_initial_position_) {
      pos_cmd_ = pos_;
      have_cmd_ = true;
      RCLCPP_INFO(get_logger(), "holding initial position until a command arrives");
    }
    // A position-only teleop may omit attitude commands. Capture this
    // vehicle's initial yaw so it will hold its heading.
    if (!have_state_ && !have_att_cmd_) {
      att_cmd_ = Eigen::Vector3d(0.0, 0.0, rpy_.z());
      have_att_cmd_ = true;
    }
    have_state_ = true;
    tryPublish();
  }

  void tryPublish()
  {
    // A received position reference or the configured initial-position hold
    // starts this drone's controller.
    if (!have_state_ || !have_cmd_) return;

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    const bool valid_dt = dt > 0.0 && dt <= 0.2;
    if (!valid_dt) dt = 1.0 / 400.0;

    // The 6D admittance integrates a virtual displacement. Superpose its
    // WORLD-frame position offset on the conventional position reference;
    // one unchanged four-layer cascade tracks the resulting reference.
    Eigen::Vector3d pos_ref = pos_cmd_;
    if (admittance_active_ && have_admittance_cmd_) {
      pos_ref += admittance_pos_cmd_;
    }
    if (copTranslationOffsetFresh()) {
      // Experimental CoP translation trim. This deliberately superposes on
      // the ordinary task/admittance reference so interaction is observable.
      pos_ref += cop_translation_position_offset_;
    }

    if (!valid_dt) {
      resetControllers();
    }

    // The simulator publishes world position/velocity, XYZ Euler attitude,
    // and body gyro. Its `acc` field changes convention with sensor mode, so
    // never use it for velocity D. Filter derivatives of measured world velocity
    // and body gyro instead, following PX4's derivative-on-measurement policy.
    if (!derivative_initialized_) {
      filtered_vel_ = vel_;
      filtered_rate_ = w_body_;
      previous_vel_ = vel_;
      previous_rate_ = w_body_;
      derivative_initialized_ = true;
    } else {
      const double alpha = 1.0 - std::exp(-2.0 * M_PI * derivative_cutoff_hz_ * dt);
      filtered_vel_ += alpha * (vel_ - filtered_vel_);
      filtered_rate_ += alpha * (w_body_ - filtered_rate_);
      filtered_accel_ += alpha * ((vel_ - previous_vel_) / dt - filtered_accel_);
      filtered_alpha_ += alpha * ((w_body_ - previous_rate_) / dt - filtered_alpha_);
      previous_vel_ = vel_;
      previous_rate_ = w_body_;
    }

    // World-frame translation cascade. Both layers have separate XYZ P/I/D
    // gains and integral states. Each stage clamps its output independently.
    const Eigen::Vector3d vel_sp = layers_[0].update(
      pos_ref - pos_, filtered_vel_, dt, velocity_limit_,
      Eigen::Vector3d::Constant(pos_integral_limit_), !allocator_saturated_);
    const Eigen::Vector3d acceleration_cmd = layers_[1].update(
      vel_sp - vel_, filtered_accel_, dt, acceleration_limit_,
      Eigen::Vector3d::Constant(vel_integral_limit_), !allocator_saturated_);

    // MuJoCo uses world +Z upward with gravity (0,0,-g). The required
    // actuator force is m*(a_cmd - gravity_vector), then rotated into body.
    const Eigen::Vector3d force_world = mass_ *
      (acceleration_cmd + Eigen::Vector3d(0.0, 0.0, grav_));
    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);
    const Eigen::Vector3d F_body = r_wb.transpose() * force_world;

    // The orientation offset is also relative to the conventional target.
    Eigen::Vector3d att_ref = att_cmd_;
    if (admittance_active_ && have_admittance_att_cmd_) {
      att_ref += admittance_att_cmd_;
    }

    const bool cop_safety_active = copSafetyFreshAndActive();
    if (cop_safety_active) {
      att_ref += cop_safety_attitude_offset_;
    }

    hih_interfaces::msg::AttitudeCmd att_ref_msg;
    att_ref_msg.roll_ref = static_cast<float>(att_ref.x() / deg_to_rad);
    att_ref_msg.pitch_ref = static_cast<float>(att_ref.y() / deg_to_rad);
    att_ref_msg.yaw_ref = static_cast<float>(att_ref.z() / deg_to_rad);
    pub_att_ref_->publish(att_ref_msg);
    // Body-frame rotation cascade. The quaternion error is SO(3)-aware and
    // never subtracts Euler angles. The fully actuated craft keeps force and
    // torque independent; attitude does not derive from horizontal force.
    const Eigen::Vector3d e_rotation = attitudeErrorBody(
      rotationWorldFromBody(att_ref), r_wb);
    const Eigen::Vector3d rate_sp = layers_[2].update(
      e_rotation, filtered_rate_, dt, rate_limit_,
      Eigen::Vector3d::Constant(att_integral_limit_), !allocator_saturated_);
    const Eigen::Vector3d M_body_pid = layers_[3].update(
      rate_sp - w_body_, filtered_alpha_, dt, torque_limit_,
      Eigen::Vector3d::Constant(rate_integral_limit_),
      !allocator_saturated_ && !dob_saturated_);

    // Optional Torque DoB correction. No DOB node/message => exact PID torque.
    Eigen::Vector3d dob_correction = Eigen::Vector3d::Zero();
    if (dob_active_) {
      const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - dob_last_rx_).count();
      if (age >= 0.0 && age <= dob_timeout_sec_) {
        dob_correction = dob_compensation_;
        if (cop_safety_active) {
          if (suppress_pitch_dob_when_cop_safety_active_ &&
              std::abs(cop_safety_attitude_offset_.y()) > 1e-9) {
            dob_correction.y() = 0.0;
          }
          if (suppress_yaw_dob_when_cop_safety_active_ &&
              std::abs(cop_safety_attitude_offset_.z()) > 1e-9) {
            dob_correction.z() = 0.0;
          }
        }
      } else {
        dob_active_ = false;
        dob_compensation_.setZero();
      }
    }

    const Eigen::Vector3d M_body_unclamped = M_body_pid + dob_correction;
    Eigen::Vector3d M_body = M_body_unclamped;
    for (int axis = 0; axis < 3; ++axis) {
      M_body[axis] = std::clamp(
        M_body[axis], -torque_limit_[axis], torque_limit_[axis]);
    }
    // Feed the DOB-induced final torque saturation into the next rate-loop
    // anti-windup decision. Allocator saturation handling remains unchanged.
    dob_saturated_ =
      (M_body - M_body_unclamped).cwiseAbs().maxCoeff() > 1e-9;

    hih_interfaces::msg::Wrench w;
    w.moment[0] = static_cast<float>(M_body(0));
    w.moment[1] = static_cast<float>(M_body(1));
    w.moment[2] = static_cast<float>(M_body(2));
    w.force[0]  = static_cast<float>(F_body(0));
    w.force[1]  = static_cast<float>(F_body(1));
    w.force[2]  = static_cast<float>(F_body(2));

    // Control calculation and publishing are driven only by fresh state data.
    pub_wrench_->publish(w);
  }

  // ROS
  rclcpp::Subscription<hih_interfaces::msg::Cmd>::SharedPtr            sub_cmd_;
  rclcpp::Subscription<hih_interfaces::msg::Cmd>::SharedPtr            sub_admittance_cmd_;
  rclcpp::Subscription<hih_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<hih_interfaces::msg::AttitudeCmd>::SharedPtr     sub_att_cmd_;
  rclcpp::Subscription<hih_interfaces::msg::AttitudeCmd>::SharedPtr     sub_admittance_att_cmd_;
  rclcpp::Subscription<hih_interfaces::msg::AttitudeCmd>::SharedPtr     sub_cop_safety_offset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                  sub_cop_safety_active_;
  rclcpp::Subscription<hih_interfaces::msg::Cmd>::SharedPtr              sub_cop_translation_offset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        sub_admittance_active_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        sub_reset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        sub_allocator_saturated_;
  rclcpp::Subscription<hih_interfaces::msg::TorqueDob>::SharedPtr       sub_dob_status_;
  rclcpp::Publisher<hih_interfaces::msg::Wrench>::SharedPtr            pub_wrench_;
  rclcpp::Publisher<hih_interfaces::msg::AttitudeCmd>::SharedPtr       pub_att_ref_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gain_callback_;

  rclcpp::Time last_time_;

  // state/command
  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d admittance_pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d att_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d admittance_att_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cop_safety_attitude_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cop_translation_position_offset_{Eigen::Vector3d::Zero()};

  std::array<PidLayer, 4> layers_;
  Eigen::Vector3d velocity_limit_{Eigen::Vector3d::Ones()};
  Eigen::Vector3d acceleration_limit_{Eigen::Vector3d::Ones()};
  Eigen::Vector3d rate_limit_{Eigen::Vector3d::Ones()};
  Eigen::Vector3d torque_limit_{Eigen::Vector3d::Ones()};
  double pos_integral_limit_{10.0}, vel_integral_limit_{10.0};
  double att_integral_limit_{1.0}, rate_integral_limit_{1.0};
  double derivative_cutoff_hz_{30.0};
  Eigen::Vector3d filtered_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_rate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_alpha_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_rate_{Eigen::Vector3d::Zero()};
  bool derivative_initialized_{false};
  bool allocator_saturated_{false};
  bool dob_saturated_{false};
  bool dob_active_{false};
  double dob_timeout_sec_{0.15};
  Eigen::Vector3d dob_compensation_{Eigen::Vector3d::Zero()};
  std::chrono::steady_clock::time_point dob_last_rx_{};

  double cop_safety_timeout_sec_{0.15};
  bool suppress_pitch_dob_when_cop_safety_active_{true};
  bool suppress_yaw_dob_when_cop_safety_active_{true};
  bool cop_safety_requested_active_{false};
  bool have_cop_safety_offset_{false};
  bool have_cop_safety_active_{false};
  std::chrono::steady_clock::time_point cop_safety_offset_last_rx_{};
  std::chrono::steady_clock::time_point cop_safety_active_last_rx_{};

  double cop_translation_timeout_sec_{0.20};
  bool have_cop_translation_offset_{false};
  std::chrono::steady_clock::time_point cop_translation_offset_last_rx_{};

  double mass_{2.7};
  double grav_{9.81};
  bool have_state_{false}, have_cmd_{false}, have_att_cmd_{false};
  bool hold_initial_position_{false};
  bool have_admittance_cmd_{false}, have_admittance_att_cmd_{false};
  bool admittance_active_{false};
  bool admittance_off_pending_{false};
  double admittance_off_debounce_sec_{0.05};
  std::chrono::steady_clock::time_point admittance_off_since_{};
  bool waiting_for_position_sync_{false};
  const double position_sync_tolerance_{0.05};

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerWrenchController>());
  rclcpp::shutdown();
  return 0;
}

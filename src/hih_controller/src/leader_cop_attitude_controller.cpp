// Leader CoP attitude-offset controller.
// CoP y/z -> yaw/pitch rate reference -> accumulated Euler offset; roll offset=0.
// LeaderWrenchController adds this trim to the task/admittance attitude.
// Feedback is restricted to MOB (0) and SOLVER (2), in /leader.
// See README.md for the four CoP controllers.

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <hih_interfaces/msg/attitude_cmd.hpp>
#include <hih_interfaces/msg/center_of_pressure.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include "hih_controller/cop_boundary_defaults.hpp"
#include "hih_controller/cop_feedback.hpp"
#include "hih_controller/cop_safety_core.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

using hih_controller::cop_feedback::CopSource;
using hih_controller::cop_feedback::kCopSourceCount;
using hih_controller::cop_feedback::nextSource;
using hih_controller::cop_feedback::parseSource;
using hih_controller::cop_feedback::sourceIndex;
using hih_controller::cop_feedback::sourceName;
using hih_controller::cop_feedback::sourceParameterValue;

struct SourceSample
{
  bool received{false};
  bool valid{false};
  double y{0.0};
  double z{0.0};
  double normal_force{0.0};
  Clock::time_point rx_time{};
};
}  // namespace

// Convert boundary excess into bounded Euler-offset rates, then integrate.
// y-CoP drives yaw; z-CoP drives pitch. Shared boundaries live in the header.
// Safe or stale inputs hold offsets; master/axis OFF returns them to zero.
// Source changes reset the input filter/latch, preserving accumulated offsets.
// Euler offset rates are not Body p/q/r. The existing attitude loop tracks
// the component-wise pitch/yaw reference plus these offsets.

namespace tuning
{
constexpr const char * default_cop_source = "mob";  // c: MOB <-> SOLVER.
constexpr bool enable_y_safety = true;             // y-CoP drives yaw; 't' is the master toggle.
constexpr bool enable_z_safety = true;             // z-CoP -> pitch.
// Read shared boundary and hysteresis defaults directly in the constructor.
constexpr double min_normal_force_n = 1.0;         // Minimum usable normal compression [N].
constexpr double cop_lpf_cutoff_hz = 20.0;           // Selected CoP low-pass cutoff [Hz]; zero bypasses the filter.

// The gain sets the correction angular rate, not a target angle: [deg/(m*s)] * [m] = [deg/s].
constexpr double yaw_velocity_gain_deg_per_m_s = 50.0;
constexpr double yaw_direction = 1.0;             // Keep the existing y->yaw sign: positive y_CoP should produce positive yaw.
constexpr double yaw_offset_limit_deg = 45.0;       // Absolute limit on the accumulated yaw offset [deg].
constexpr double yaw_rate_limit_deg_s = 2.0;      // Rate limit for TRACK and return-to-zero when OFF [deg/s].
constexpr double pitch_velocity_gain_deg_per_m_s = 50.0;
constexpr double pitch_direction = -1.0;           // Keep the existing z->pitch sign: positive z_CoP should produce negative pitch.
constexpr double pitch_offset_limit_deg = 45.0;     // Absolute limit on the accumulated pitch offset [deg].
constexpr double pitch_rate_limit_deg_s = 2.0;    // Rate limit for TRACK and return-to-zero when OFF [deg/s].
// Example: if e_y=+0.05 m, yaw_rate=-1.5 deg/s, then -0.015 deg is added every dt=0.01 s.
// Once the CoP enters the boundary, the current offset is held. The gain and accumulated angle limit are independent.
constexpr double cop_timeout_sec = 0.10;
constexpr double publish_rate_hz = 100.0;
}  // namespace tuning

class LeaderCopAttitudeController : public rclcpp::Node
{
public:
  LeaderCopAttitudeController()
  : rclcpp::Node("leader_cop_attitude_controller", "/leader")
  {
    // Reject obsolete angle-PD settings instead of interpreting them as rate gains.
    for (const auto & override : get_node_parameters_interface()->get_parameter_overrides()) {
      const auto reason = hih_controller::cop_safety::removedParameterReason(override.first);
      if (!reason.empty()) throw std::invalid_argument(override.first + " removed: " + reason);
    }
    enabled_ = declare_parameter<bool>("enabled", false);
    const bool enable_keyboard = declare_parameter<bool>("enable_keyboard", true);

    const std::string initial_source =
      declare_parameter<std::string>("cop_source", tuning::default_cop_source);
    if (!parseSource(initial_source, source_)) {
      throw std::invalid_argument(
        "cop_source must be one of: mob, solver");
    }

    enable_y_safety_ =
      declare_parameter<bool>("enable_y_safety", tuning::enable_y_safety);
    enable_z_safety_ =
      declare_parameter<bool>("enable_z_safety", tuning::enable_z_safety);

    safe_y_m_ = positive("safe_y_m", hih_controller::cop_boundary_defaults::safe_y_m);
    safe_z_m_ = positive("safe_z_m", hih_controller::cop_boundary_defaults::safe_z_m);
    safe_y_hysteresis_m_ =
      nonnegative("safe_y_hysteresis_m", hih_controller::cop_boundary_defaults::safe_y_hysteresis_m);
    safe_z_hysteresis_m_ =
      nonnegative("safe_z_hysteresis_m", hih_controller::cop_boundary_defaults::safe_z_hysteresis_m);
    if (safe_y_hysteresis_m_ >= safe_y_m_ ||
        safe_z_hysteresis_m_ >= safe_z_m_) {
      throw std::invalid_argument(
        "safe_[y/z]_hysteresis_m must be smaller than safe_[y/z]_m");
    }

    min_normal_force_n_ =
      nonnegative("min_normal_force_n", tuning::min_normal_force_n);
    cop_lpf_cutoff_hz_ =
      nonnegative("cop_lpf_cutoff_hz", tuning::cop_lpf_cutoff_hz);

    yaw_velocity_gain_deg_per_m_s_ =
      nonnegative("yaw_velocity_gain_deg_per_m_s", tuning::yaw_velocity_gain_deg_per_m_s);
    yaw_direction_ = direction("yaw_direction", tuning::yaw_direction);
    yaw_offset_limit_deg_ =
      positive("yaw_offset_limit_deg", tuning::yaw_offset_limit_deg);
    yaw_rate_limit_deg_s_ =
      positive("yaw_rate_limit_deg_s", tuning::yaw_rate_limit_deg_s);

    pitch_velocity_gain_deg_per_m_s_ =
      nonnegative("pitch_velocity_gain_deg_per_m_s", tuning::pitch_velocity_gain_deg_per_m_s);
    pitch_direction_ = direction("pitch_direction", tuning::pitch_direction);
    pitch_offset_limit_deg_ =
      positive("pitch_offset_limit_deg", tuning::pitch_offset_limit_deg);
    pitch_rate_limit_deg_s_ =
      positive("pitch_rate_limit_deg_s", tuning::pitch_rate_limit_deg_s);

    cop_timeout_sec_ = positive("cop_timeout_sec", tuning::cop_timeout_sec);
    publish_rate_hz_ = positive("publish_rate_hz", tuning::publish_rate_hz);

    // Subscribe to both supported sources. The selector decides which source
    // enters the common filter/controller path.
    cop_hat_sub_ = create_subscription<hih_interfaces::msg::CenterOfPressure>(
      "minitrone/cop_hat", 10,
      [this](const hih_interfaces::msg::CenterOfPressure::SharedPtr msg) {
        onCop(CopSource::MOB, *msg);
      });
    cop_solver_sub_ = create_subscription<hih_interfaces::msg::CenterOfPressure>(
      "minitrone/cop_solver_real", 10,
      [this](const hih_interfaces::msg::CenterOfPressure::SharedPtr msg) {
        onCop(CopSource::SOLVER, *msg);
      });

    offset_pub_ = create_publisher<hih_interfaces::msg::AttitudeCmd>(
      "minitrone/cop_safety_attitude_offset", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "minitrone/cop_safety_active", 10);
    pitch_offset_pub_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/cop_safety_pitch_offset_deg", 10);
    yaw_offset_pub_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/cop_safety_yaw_offset_deg", 10);
    boundary_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "minitrone/cop_attitude/boundary", 10);
    yaw_rate_pub_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/cop_attitude/yaw_rate_reference_deg_s", 10);
    pitch_rate_pub_ = create_publisher<std_msgs::msg::Float64>(
      "minitrone/cop_attitude/pitch_rate_reference_deg_s", 10);
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&LeaderCopAttitudeController::onParameters, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LeaderCopAttitudeController::tick, this));

    if (enable_keyboard) {
      setupTerminal();
      keyboard_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LeaderCopAttitudeController::processKeyboard, this));
    }

    last_tick_ = Clock::now();

    printHelp();
    RCLCPP_INFO(
      get_logger(),
      "CoP safety started in %s | master=%s | source=%s | "
      "y=%s (+/-%.3f m) | z=%s (+/-%.3f m)",
      get_namespace(),
      enabled_ ? "ON" : "OFF",
      sourceName(source_),
      enable_y_safety_ ? "ON" : "OFF", safe_y_m_,
      enable_z_safety_ ? "ON" : "OFF", safe_z_m_);
  }

  ~LeaderCopAttitudeController() override
  {
    enabled_ = false;
    output_yaw_deg_ = 0.0;
    output_pitch_deg_ = 0.0;
    publishZeroAndInactive();
    restoreTerminal();
  }

private:
  SourceSample & sample(CopSource source)
  {
    return samples_[sourceIndex(source)];
  }

  const SourceSample & sample(CopSource source) const
  {
    return samples_[sourceIndex(source)];
  }

  bool fresh(const SourceSample & source_sample, Clock::time_point now) const
  {
    if (!source_sample.received) return false;
    const double age =
      std::chrono::duration<double>(now - source_sample.rx_time).count();
    return age >= 0.0 && age <= cop_timeout_sec_;
  }

  void setupTerminal()
  {
    if (!isatty(STDIN_FILENO)) {
      input_fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK);
      if (input_fd_ == -1) {
        RCLCPP_WARN(
          get_logger(),
          "No interactive terminal: ROS parameters still work, "
          "but [t]/[c]/[p]/[q] keys are unavailable.");
        input_fd_ = STDIN_FILENO;
        return;
      }
      close_input_fd_ = true;
    } else {
      input_fd_ = STDIN_FILENO;
    }

    if (tcgetattr(input_fd_, &original_termios_) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(input_fd_, TCSANOW, &raw) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return;
    }

    original_flags_ = fcntl(input_fd_, F_GETFL, 0);
    if (original_flags_ == -1 ||
        fcntl(input_fd_, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      RCLCPP_WARN(get_logger(), "failed to enable nonblocking terminal input");
      (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
      return;
    }
    terminal_ready_ = true;
  }

  void restoreTerminal()
  {
    if (terminal_ready_) {
      (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
      (void)fcntl(input_fd_, F_SETFL, original_flags_);
      terminal_ready_ = false;
    }
    if (close_input_fd_) {
      close(input_fd_);
      close_input_fd_ = false;
      input_fd_ = STDIN_FILENO;
    }
  }

  void printHelp() const
  {
    std::printf(
      "\n[CoP Attitude] MOB/SOLVER feedback\n"
      "  t : toggle master CoP safety ON/OFF\n"
      "  c : cycle source MOB -> SOLVER -> MOB\n"
      "  p : print source / y-z CoP / yaw-pitch correction\n"
      "  q : publish zero offset and quit this node\n"
      "  source id: 0=MOB, 2=SOLVER(cop_solver_real)\n\n");
    std::fflush(stdout);
  }

  void toggleEnabled()
  {
    const bool target = !enabled_;
    const auto result = set_parameter(rclcpp::Parameter("enabled", target));
    if (!result.successful) {
      RCLCPP_WARN(
        get_logger(), "Failed to toggle CoP safety: %s", result.reason.c_str());
      return;
    }

    if (!enabled_) {
      resetSafetyAxisState();
    }

    RCLCPP_INFO(
      get_logger(),
      "CoP safety %s | source=%s | y=%s | z=%s",
      enabled_ ? "ON" : "OFF",
      sourceName(source_),
      enable_y_safety_ ? "ON" : "OFF",
      enable_z_safety_ ? "ON" : "OFF");
  }

  void cycleSource()
  {
    const CopSource target = nextSource(source_);
    const auto result = set_parameter(
      rclcpp::Parameter("cop_source", sourceParameterValue(target)));
    if (!result.successful) {
      RCLCPP_WARN(
        get_logger(), "Failed to change CoP source: %s", result.reason.c_str());
      return;
    }

    const auto now = Clock::now();
    const auto & selected = sample(source_);
    if (!selected.received) {
      RCLCPP_WARN(
        get_logger(),
        "CoP source -> %s, but no message has been received yet",
        sourceName(source_));
    } else if (!fresh(selected, now)) {
      RCLCPP_WARN(
        get_logger(),
        "CoP source -> %s, but its latest message is stale",
        sourceName(source_));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "CoP source -> %s (LPF/input reset; accumulated offset retained)",
        sourceName(source_));
    }
  }

  void printStatus() const
  {
    const auto & selected = sample(source_);
    RCLCPP_INFO(
      get_logger(),
      "CoP attitude %s | source=%s fresh=%d valid=%d Fn=%.3f N | "
      "CoP(y,z)=(%+.4f,%+.4f) e=(%+.4f,%+.4f) m | "
      "rate(yaw,pitch)=(%+.3f,%+.3f) deg/s -> offset=(%+.3f,%+.3f) deg",
      enabled_ ? "ON" : "OFF", sourceName(source_),
      fresh(selected, Clock::now()) ? 1 : 0, selected.valid ? 1 : 0,
      selected.normal_force, filtered_y_m_, filtered_z_m_,
      cop_y_error_m_, cop_z_error_m_, yaw_rate_reference_deg_s_, pitch_rate_reference_deg_s_,
      output_yaw_deg_, output_pitch_deg_);
  }

  void publishZeroAndInactive()
  {
    // SIGINT can shut down the ROS context before this node is destroyed.
    if (!rclcpp::ok()) return;
    hih_interfaces::msg::AttitudeCmd offset;
    offset.roll_ref = 0.0F;
    offset.pitch_ref = 0.0F;
    offset.yaw_ref = 0.0F;
    offset_pub_->publish(offset);

    std_msgs::msg::Bool active;
    active.data = false;
    active_pub_->publish(active);

    std_msgs::msg::Float64 scalar;
    scalar.data = 0.0;
    pitch_offset_pub_->publish(scalar);
    yaw_offset_pub_->publish(scalar);
    yaw_rate_pub_->publish(scalar);
    pitch_rate_pub_->publish(scalar);
    publishBoundary();
  }

  void processKeyboard()
  {
    if (!terminal_ready_) return;

    char key = 0;
    ssize_t n = 0;
    while ((n = read(input_fd_, &key, 1)) > 0) {
      if (key == 't' || key == 'T') {
        toggleEnabled();
      } else if (key == 'c' || key == 'C') {
        cycleSource();
      } else if (key == 'p' || key == 'P') {
        printStatus();
      } else if (key == 'q' || key == 'Q') {
        enabled_ = false;
        output_yaw_deg_ = 0.0;
        output_pitch_deg_ = 0.0;
        publishZeroAndInactive();
        RCLCPP_INFO(get_logger(), "Zero CoP-safety offset published; quitting");
        rclcpp::shutdown();
        return;
      }
    }

    if (n == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "keyboard read failed: %s", std::strerror(errno));
    }
  }

  double positive(const std::string & name, double value)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return result;
  }

  double nonnegative(const std::string & name, double value)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || result < 0.0) {
      throw std::invalid_argument(name + " must be finite and nonnegative");
    }
    return result;
  }

  double direction(const std::string & name, double value)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || std::abs(result) < 1e-9) {
      throw std::invalid_argument(name + " must be finite and nonzero");
    }
    return result;
  }

  void resetFilterForSource(CopSource source)
  {
    filter_initialized_ = false;
    have_filter_time_ = false;
    resetSafetyAxisState();

    const auto now = Clock::now();
    const auto & selected = sample(source);
    if (selected.received && selected.valid && fresh(selected, now) &&
      selected.normal_force >= min_normal_force_n_) {
      filtered_y_m_ = selected.y;
      filtered_z_m_ = selected.z;
      filter_initialized_ = true;
      last_filter_time_ = now;
      have_filter_time_ = true;
    }
  }

  void resetSafetyAxisState()
  {
    // Reset input error/latch only; clearing offsets would cause a command jump.
    cop_y_error_m_ = cop_z_error_m_ = 0.0;
    yaw_rate_reference_deg_s_ = pitch_rate_reference_deg_s_ = 0.0;
    y_boundary_active_ = z_boundary_active_ = false;
  }

  void setSource(CopSource source)
  {
    if (source_ == source) return;
    source_ = source;
    resetFilterForSource(source_);
  }

  void updateSelectedFilter(
    const SourceSample & selected, Clock::time_point now)
  {
    if (!selected.valid) return;

    double dt = 1.0 / publish_rate_hz_;
    if (have_filter_time_) {
      dt = std::chrono::duration<double>(now - last_filter_time_).count();
    }
    last_filter_time_ = now;
    have_filter_time_ = true;

    if (!filter_initialized_ || !(dt > 0.0) || dt > 0.2) {
      filtered_y_m_ = selected.y;
      filtered_z_m_ = selected.z;
      filter_initialized_ = true;
      return;
    }

    const double alpha =
      hih_controller::cop_safety::lpfAlpha(cop_lpf_cutoff_hz_, dt);

    filtered_y_m_ += alpha * (selected.y - filtered_y_m_); // cut off frequeuncy filtering
    filtered_z_m_ += alpha * (selected.z - filtered_z_m_);

  }

  void onCop(
    CopSource source,
    const hih_interfaces::msg::CenterOfPressure & msg)
  {
    const auto now = Clock::now();
    auto & source_sample = sample(source);

    source_sample.received = true;
    source_sample.rx_time = now;
    source_sample.y = static_cast<double>(msg.y);
    source_sample.z = static_cast<double>(msg.z);
    source_sample.normal_force = static_cast<double>(msg.normal_force);
    source_sample.valid =
      msg.valid &&
      std::isfinite(source_sample.y) &&
      std::isfinite(source_sample.z) &&
      std::isfinite(source_sample.normal_force);

    if (source == source_) {
      if (source_sample.valid && source_sample.normal_force >= min_normal_force_n_) {
        updateSelectedFilter(source_sample, now);
      } else {
        // On renewed contact, reset the filter/latch and retain the attitude offset.
        filter_initialized_ = false;
        have_filter_time_ = false;
        resetSafetyAxisState();
      }
    }
  }

  void publishBoundary()
  {
    // PyQt contract: master, y/z enables, y/z half-widths [m], MOB/SOLVER ID.
    // OFF is still published so the GUI can remove a previously drawn boundary.
    std_msgs::msg::Float64MultiArray state;
    state.data = {
      enabled_ ? 1.0 : 0.0, enable_y_safety_ ? 1.0 : 0.0, enable_z_safety_ ? 1.0 : 0.0,
      safe_y_m_, safe_z_m_, static_cast<double>(source_)};
    boundary_pub_->publish(state);
  }

  void tick()
  {
    publishBoundary();  // Publish the configured boundary even while OFF or out of contact.
    const auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    // Skip zero, reversed or >0.2 s timer intervals without inventing an integration dt.
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2) dt = 0.0;

    const auto & selected = sample(source_);
    const bool contact_usable = fresh(selected, now) && selected.valid &&
      selected.normal_force >= min_normal_force_n_;
    if (!contact_usable) {
      filter_initialized_ = false;
      have_filter_time_ = false;
      resetSafetyAxisState();
    }
    const bool usable = contact_usable && filter_initialized_;
    using hih_controller::cop_safety::MotionMode;
    const auto mode = [this, usable](bool axis_enabled) {
        if (!enabled_ || !axis_enabled) return MotionMode::RETURN_TO_ZERO;
        return usable ? MotionMode::TRACK : MotionMode::HOLD;
      };
    const auto mode_y = mode(enable_y_safety_), mode_z = mode(enable_z_safety_);

    // y-CoP drives yaw and z-CoP drives pitch using signed boundary excess [m].
    // Inside the boundary, zero rate holds the offset even if hysteresis stays active.
    const auto update_error = [](bool track, bool & latch, double cop,
        double safe, double hysteresis) {
        if (!track) {latch = false; return 0.0;}
        latch = hih_controller::cop_safety::hysteresisActive(latch, cop, safe, hysteresis);
        return latch ? hih_controller::cop_safety::deadzoneError(cop, safe) : 0.0;
      };
    cop_y_error_m_ = update_error(mode_y == MotionMode::TRACK, y_boundary_active_,
      filtered_y_m_, safe_y_m_, safe_y_hysteresis_m_);
    cop_z_error_m_ = update_error(mode_z == MotionMode::TRACK, z_boundary_active_,
      filtered_z_m_, safe_z_m_, safe_z_hysteresis_m_);
    yaw_rate_reference_deg_s_ = hih_controller::cop_safety::angularRateFromErrorDegS(
      cop_y_error_m_, yaw_velocity_gain_deg_per_m_s_, yaw_direction_, yaw_rate_limit_deg_s_);
    pitch_rate_reference_deg_s_ = hih_controller::cop_safety::angularRateFromErrorDegS(
      cop_z_error_m_, pitch_velocity_gain_deg_per_m_s_, pitch_direction_, pitch_rate_limit_deg_s_);

    // TRACK: offset <- clamp(offset + rate*dt, +/-angle_limit).
    // HOLD preserves offsets; OFF returns at rate_limit*dt without hidden windup.
    output_yaw_deg_ = hih_controller::cop_safety::advanceOffsetDeg(
      output_yaw_deg_, yaw_rate_reference_deg_s_, mode_y, dt,
      yaw_offset_limit_deg_, yaw_rate_limit_deg_s_);
    output_pitch_deg_ = hih_controller::cop_safety::advanceOffsetDeg(
      output_pitch_deg_, pitch_rate_reference_deg_s_, mode_z, dt,
      pitch_offset_limit_deg_, pitch_rate_limit_deg_s_);

    // Keep the output gate active while an accumulated offset must still be applied.
    // The gate becomes false only when the OFF return reaches exactly zero.
    const bool active = output_yaw_deg_ != 0.0 || output_pitch_deg_ != 0.0;
    hih_interfaces::msg::AttitudeCmd offset;
    offset.roll_ref = 0.0F;
    offset.pitch_ref = static_cast<float>(output_pitch_deg_);
    offset.yaw_ref = static_cast<float>(output_yaw_deg_);
    offset_pub_->publish(offset);
    std_msgs::msg::Bool active_msg;
    active_msg.data = active;
    active_pub_->publish(active_msg);
    std_msgs::msg::Float64 scalar;
    scalar.data = output_pitch_deg_; pitch_offset_pub_->publish(scalar);
    scalar.data = output_yaw_deg_; yaw_offset_pub_->publish(scalar);
    // TRACK rate telemetry can differ from actual rates during saturation or OFF return.
    scalar.data = yaw_rate_reference_deg_s_; yaw_rate_pub_->publish(scalar);
    scalar.data = pitch_rate_reference_deg_s_; pitch_rate_pub_->publish(scalar);
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      const auto & name = p.get_name();
      const auto removed = hih_controller::cop_safety::removedParameterReason(name);
      if (!removed.empty()) {
        result.successful = false;
        result.reason = name + " removed: " + removed;
        return result;
      }

      if (name == "enabled" ||
          name == "enable_y_safety" ||
          name == "enable_z_safety") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = name + " must be bool";
          return result;
        }
      } else if (name == "cop_source") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "cop_source must be string: mob or solver";
          return result;
        }
        CopSource parsed;
        if (!parseSource(p.as_string(), parsed)) {
          result.successful = false;
          result.reason = "cop_source must be one of: mob, solver";
          return result;
        }
      } else if (
        name == "safe_y_m" ||
        name == "safe_z_m" ||
        name == "yaw_offset_limit_deg" ||
        name == "pitch_offset_limit_deg" ||
        name == "yaw_rate_limit_deg_s" ||
        name == "pitch_rate_limit_deg_s" ||
        name == "cop_timeout_sec")
      {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(p.as_double()) || p.as_double() <= 0.0) {
          result.successful = false;
          result.reason = name + " must be finite and positive";
          return result;
        }
      } else if (
        name == "min_normal_force_n" ||
        name == "cop_lpf_cutoff_hz" ||
        name == "safe_y_hysteresis_m" ||
        name == "safe_z_hysteresis_m" ||
        name == "yaw_velocity_gain_deg_per_m_s" ||
        name == "pitch_velocity_gain_deg_per_m_s")
      {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(p.as_double()) || p.as_double() < 0.0) {
          result.successful = false;
          result.reason = name + " must be finite and nonnegative";
          return result;
        }
      } else if (name == "yaw_direction" || name == "pitch_direction") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(p.as_double()) ||
            std::abs(p.as_double()) < 1e-9) {
          result.successful = false;
          result.reason = name + " must be finite and nonzero";
          return result;
        }
      } else if (name == "publish_rate_hz" || name == "enable_keyboard") {
        result.successful = false;
        result.reason = name + " requires node restart";
        return result;
      }
    }

    double next_yaw_limit = yaw_offset_limit_deg_, next_pitch_limit = pitch_offset_limit_deg_;
    for (const auto & p : params) {
      if (p.get_name() == "yaw_offset_limit_deg") next_yaw_limit = p.as_double();
      else if (p.get_name() == "pitch_offset_limit_deg") next_pitch_limit = p.as_double();
    }
    // Reject limit changes that would abruptly truncate the current attitude command.
    if (std::abs(output_yaw_deg_) > next_yaw_limit + 1e-12 ||
      std::abs(output_pitch_deg_) > next_pitch_limit + 1e-12)
    {
      result.successful = false;
      result.reason = "angle limits exclude held offset; return to zero before reducing limits";
      return result;
    }
    double next_safe_y = safe_y_m_;
    double next_safe_z = safe_z_m_;
    double next_hyst_y = safe_y_hysteresis_m_;
    double next_hyst_z = safe_z_hysteresis_m_;
    for (const auto & p : params) {
      if (p.get_name() == "safe_y_m") next_safe_y = p.as_double();
      else if (p.get_name() == "safe_z_m") next_safe_z = p.as_double();
      else if (p.get_name() == "safe_y_hysteresis_m") next_hyst_y = p.as_double();
      else if (p.get_name() == "safe_z_hysteresis_m") next_hyst_z = p.as_double();
    }
    if (next_hyst_y >= next_safe_y || next_hyst_z >= next_safe_z) {
      result.successful = false;
      result.reason =
        "safe_[y/z]_hysteresis_m must be smaller than safe_[y/z]_m";
      return result;
    }

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "enabled") enabled_ = p.as_bool();
      else if (name == "cop_source") {
        CopSource parsed;
        (void)parseSource(p.as_string(), parsed);
        setSource(parsed);
      }
      else if (name == "enable_y_safety") enable_y_safety_ = p.as_bool();
      else if (name == "enable_z_safety") enable_z_safety_ = p.as_bool();
      else if (name == "safe_y_m") safe_y_m_ = p.as_double();
      else if (name == "safe_z_m") safe_z_m_ = p.as_double();
      else if (name == "safe_y_hysteresis_m") safe_y_hysteresis_m_ = p.as_double();
      else if (name == "safe_z_hysteresis_m") safe_z_hysteresis_m_ = p.as_double();
      else if (name == "min_normal_force_n") min_normal_force_n_ = p.as_double();
      else if (name == "cop_lpf_cutoff_hz") cop_lpf_cutoff_hz_ = p.as_double();
      else if (name == "yaw_velocity_gain_deg_per_m_s") yaw_velocity_gain_deg_per_m_s_ = p.as_double();
      else if (name == "yaw_direction") yaw_direction_ = p.as_double();
      else if (name == "yaw_offset_limit_deg") yaw_offset_limit_deg_ = p.as_double();
      else if (name == "yaw_rate_limit_deg_s") yaw_rate_limit_deg_s_ = p.as_double();
      else if (name == "pitch_velocity_gain_deg_per_m_s") pitch_velocity_gain_deg_per_m_s_ = p.as_double();
      else if (name == "pitch_direction") pitch_direction_ = p.as_double();
      else if (name == "pitch_offset_limit_deg") pitch_offset_limit_deg_ = p.as_double();
      else if (name == "pitch_rate_limit_deg_s") pitch_rate_limit_deg_s_ = p.as_double();
      else if (name == "cop_timeout_sec") cop_timeout_sec_ = p.as_double();
    }

    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "enabled" ||
          name == "enable_y_safety" ||
          name == "enable_z_safety" ||
          name == "safe_y_m" ||
          name == "safe_z_m" ||
          name == "safe_y_hysteresis_m" ||
          name == "safe_z_hysteresis_m") {
        resetSafetyAxisState();
        break;
      }
    }

    return result;
  }

  CopSource source_{CopSource::MOB};
  std::array<SourceSample, kCopSourceCount> samples_{};

  bool enabled_{false};
  bool enable_y_safety_{tuning::enable_y_safety};
  bool enable_z_safety_{tuning::enable_z_safety};

  // Current parameter values, populated from shared defaults and runtime overrides.
  double safe_y_m_;
  double safe_z_m_;
  double safe_y_hysteresis_m_;
  double safe_z_hysteresis_m_;
  double min_normal_force_n_{tuning::min_normal_force_n};
  double cop_lpf_cutoff_hz_{tuning::cop_lpf_cutoff_hz};

  double yaw_velocity_gain_deg_per_m_s_{tuning::yaw_velocity_gain_deg_per_m_s};
  double yaw_direction_{tuning::yaw_direction};
  double yaw_offset_limit_deg_{tuning::yaw_offset_limit_deg};
  double yaw_rate_limit_deg_s_{tuning::yaw_rate_limit_deg_s};

  double pitch_velocity_gain_deg_per_m_s_{tuning::pitch_velocity_gain_deg_per_m_s};
  double pitch_direction_{tuning::pitch_direction};
  double pitch_offset_limit_deg_{tuning::pitch_offset_limit_deg};
  double pitch_rate_limit_deg_s_{tuning::pitch_rate_limit_deg_s};

  double cop_timeout_sec_{tuning::cop_timeout_sec};
  double publish_rate_hz_{tuning::publish_rate_hz};

  double filtered_y_m_{0.0};
  double filtered_z_m_{0.0};
  double cop_y_error_m_{0.0};
  double cop_z_error_m_{0.0};
  double yaw_rate_reference_deg_s_{0.0};
  double pitch_rate_reference_deg_s_{0.0};
  double output_yaw_deg_{0.0};
  double output_pitch_deg_{0.0};

  bool filter_initialized_{false};
  bool have_filter_time_{false};
  bool y_boundary_active_{false};
  bool z_boundary_active_{false};

  Clock::time_point last_filter_time_{};
  Clock::time_point last_tick_{};

  rclcpp::Subscription<hih_interfaces::msg::CenterOfPressure>::SharedPtr
    cop_hat_sub_, cop_solver_sub_;
  rclcpp::Publisher<hih_interfaces::msg::AttitudeCmd>::SharedPtr
    offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr boundary_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    pitch_offset_pub_, yaw_offset_pub_, pitch_rate_pub_, yaw_rate_pub_;
  rclcpp::TimerBase::SharedPtr timer_, keyboard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;

  bool terminal_ready_{false};
  bool close_input_fd_{false};
  int input_fd_{STDIN_FILENO};
  int original_flags_{0};
  termios original_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LeaderCopAttitudeController>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("leader_cop_attitude_controller"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}


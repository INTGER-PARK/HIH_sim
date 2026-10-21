// Independent Follower CoP translation-offset node.
// The input/output interface is role-local; no Leader-Follower messages are used.
// Plate y/z correction velocities are rotated into World and integrated.
// Existing World offset is held when CoP is safe or feedback becomes stale.
// FollowerWrenchController adds this offset to its task AND admittance reference.
// See README.md for available nodes and run commands.

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <hih_interfaces/msg/center_of_pressure.hpp>
#include <hih_interfaces/msg/cmd.hpp>
#include <hih_interfaces/msg/minitrone_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hih_controller/cop_boundary_defaults.hpp"
#include "hih_controller/cop_feedback.hpp"
#include "hih_controller/cop_translation_core.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;
using Axis = hih_controller::cop_translation::Axis;
using AxisSettings = hih_controller::cop_translation::AxisSettings;
using AccumulatedOffset = hih_controller::cop_translation::AccumulatedOffset;
using MotionMode = hih_controller::cop_translation::MotionMode;
using hih_controller::cop_translation::removedParameterReason;

using hih_controller::cop_feedback::CopSource;
using hih_controller::cop_feedback::kCopSourceCount;
using hih_controller::cop_feedback::nextSource;
using hih_controller::cop_feedback::parseSource;
using hih_controller::cop_feedback::sourceIndex;
using hih_controller::cop_feedback::sourceName;
using hih_controller::cop_feedback::sourceParameterValue;
using hih_controller::cop_feedback::kCopSources;
using hih_controller::cop_feedback::kCopTopics;

struct SourceSample
{
  bool received{false}, valid{false};
  double y{0.0}, z{0.0}, normal_force{0.0};
  Clock::time_point rx_time{};
};

// Boundary/hysteresis defaults are defined only in cop_boundary_defaults.hpp.
// AxisSettings stores the current values and requires both at construction.
// Boundary/gains/limits below can also be changed at runtime through ROS params.
struct Settings
{
  AxisSettings y{
    hih_controller::cop_boundary_defaults::safe_y_m,
    hih_controller::cop_boundary_defaults::safe_y_hysteresis_m};
  AxisSettings z{
    hih_controller::cop_boundary_defaults::safe_z_m,
    hih_controller::cop_boundary_defaults::safe_z_hysteresis_m};
  double min_normal_force_n{1.0};
  double cop_lpf_cutoff_hz{5.0};
  double cop_timeout_sec{0.10};
  double state_timeout_sec{0.10};
  double publish_rate_hz{100.0};
};

std::map<std::string, double *> numericParameters(Settings & s)
{
  return {
    {"safe_y_m", &s.y.safe_m}, {"safe_z_m", &s.z.safe_m},
    {"safe_y_hysteresis_m", &s.y.hysteresis_m},
    {"safe_z_hysteresis_m", &s.z.hysteresis_m},
    {"y_velocity_gain_per_s", &s.y.velocity_gain_per_s},
    {"z_velocity_gain_per_s", &s.z.velocity_gain_per_s},
    {"y_direction", &s.y.direction}, {"z_direction", &s.z.direction},
    {"y_offset_limit_m", &s.y.offset_limit_m}, {"z_offset_limit_m", &s.z.offset_limit_m},
    {"y_rate_limit_mps", &s.y.rate_limit_mps}, {"z_rate_limit_mps", &s.z.rate_limit_mps},
    {"min_normal_force_n", &s.min_normal_force_n},
    {"cop_lpf_cutoff_hz", &s.cop_lpf_cutoff_hz},
    {"cop_timeout_sec", &s.cop_timeout_sec},
    {"state_timeout_sec", &s.state_timeout_sec}, {"publish_rate_hz", &s.publish_rate_hz}};
}

std::string validationError(const Settings & s)
{
  const auto y_error = hih_controller::cop_translation::validationError(s.y);
  if (!y_error.empty()) return "y axis: " + y_error;
  const auto z_error = hih_controller::cop_translation::validationError(s.z);
  if (!z_error.empty()) return "z axis: " + z_error;
  if (!std::isfinite(s.min_normal_force_n) || s.min_normal_force_n < 0.0 ||
      !std::isfinite(s.cop_lpf_cutoff_hz) || s.cop_lpf_cutoff_hz < 0.0) {
    return "min_normal_force_n and cop_lpf_cutoff_hz must be finite and nonnegative";
  }
  if (!std::isfinite(s.cop_timeout_sec) || s.cop_timeout_sec <= 0.0 ||
      !std::isfinite(s.state_timeout_sec) || s.state_timeout_sec <= 0.0 ||
      !std::isfinite(s.publish_rate_hz) || s.publish_rate_hz <= 0.0 ||
      s.publish_rate_hz > 1000000.0) {
    return "timeouts must be positive; publish_rate_hz must be in (0, 1000000]";
  }
  return {};
}
}  // namespace

class FollowerCopTranslationController : public rclcpp::Node
{
public:
  FollowerCopTranslationController()
  : rclcpp::Node("follower_cop_translation_controller", "/follower")
  {
    // Check resolved CLI/YAML/programmatic overrides, not just declared params.
    for (const auto & override : get_node_parameters_interface()->get_parameter_overrides()) {
      const auto reason = removedParameterReason(override.first);
      if (!reason.empty()) throw std::invalid_argument(override.first + " removed: " + reason);
    }
    enabled_ = declare_parameter<bool>("enabled", false);
    enable_y_ = declare_parameter<bool>("enable_y_translation", true);
    enable_z_ = declare_parameter<bool>("enable_z_translation", true);
    enable_keyboard_ = declare_parameter<bool>("enable_keyboard", true);
    if (!parseSource(declare_parameter<std::string>("cop_source", "mob"), source_)) {
      throw std::invalid_argument("cop_source must be mob or solver");
    }
    for (const auto & item : numericParameters(settings_)) {
      *item.second = declare_parameter<double>(item.first, *item.second);
    }
    const auto error = validationError(settings_);
    if (!error.empty()) throw std::invalid_argument(error);

    // Public source IDs are 0 and 2; subscription/sample indices are 0 and 1.
    for (std::size_t i = 0; i < kCopTopics.size(); ++i) {
      const CopSource source = kCopSources[i];
      cop_subs_[i] = create_subscription<hih_interfaces::msg::CenterOfPressure>(
        kCopTopics[i], 10,
        [this, source](const hih_interfaces::msg::CenterOfPressure::SharedPtr msg) {
          onCop(source, *msg);
        });
    }
    state_sub_ = create_subscription<hih_interfaces::msg::MinitroneState>(
      "minitrone/state", 10,
      [this](const hih_interfaces::msg::MinitroneState::SharedPtr msg) {
        const std::array<double, 3> incoming{msg->rpy[0], msg->rpy[1], msg->rpy[2]};
        for (double value : incoming) {
          if (!std::isfinite(value)) {
            have_state_ = false;
            return;
          }
        }
        rpy_ = incoming;
        state_stamp_ = Clock::now();
        have_state_ = true;
      });
    offset_pub_ = create_publisher<hih_interfaces::msg::Cmd>(
      "minitrone/cop_translation_position_offset", 10);
    plate_offset_pub_ = create_publisher<hih_interfaces::msg::Cmd>(
      "minitrone/cop_translation_plate_offset", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "minitrone/cop_translation_active", 10);
    source_pub_ = create_publisher<std_msgs::msg::Int32>(
      "minitrone/cop_translation_source_id", 10);
    boundary_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "minitrone/cop_translation/boundary", 10);
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&FollowerCopTranslationController::onParameters, this, std::placeholders::_1));

    last_tick_ = Clock::now();
    const auto period = std::chrono::duration<double>(1.0 / settings_.publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FollowerCopTranslationController::tick, this));
    if (enable_keyboard_) {
      setupTerminal();
      keyboard_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&FollowerCopTranslationController::processKeyboard, this));
    }
    RCLCPP_INFO(
      get_logger(), "Follower CoP velocity accumulation | enabled=%d source=%s | safe y=%.3f z=%.3f m",
      enabled_ ? 1 : 0, sourceName(source_), settings_.y.safe_m, settings_.z.safe_m);
    if (terminal_ready_) {
      std::printf("[CoP translation] t=ON/OFF c=source p=status q=zero and quit\n");
      std::fflush(stdout);
    }
  }

  ~FollowerCopTranslationController() override
  {
    // SIGINT may already have shut down ROS. Never throw from the destructor.
    restoreTerminal();
    if (rclcpp::ok()) {
      enabled_ = false;
      publishBoundary();
      offsets_.reset();
      publish({0.0, 0.0, 0.0});
    }
  }

private:
  static double ageSeconds(Clock::time_point stamp, Clock::time_point now)
  {
    if (stamp == Clock::time_point{}) return INFINITY;
    return std::chrono::duration<double>(now - stamp).count();
  }

  bool fresh(const SourceSample & s, Clock::time_point now) const
  {
    const double age = ageSeconds(s.rx_time, now);
    return s.received && age >= 0.0 && age <= settings_.cop_timeout_sec;
  }

  bool stateFresh(Clock::time_point now) const
  {
    const double age = ageSeconds(state_stamp_, now);
    return have_state_ && age >= 0.0 && age <= settings_.state_timeout_sec;
  }

  const SourceSample & selectedSample() const
  {
    return samples_[sourceIndex(source_)];
  }

  void resetFilter()
  {
    filter_initialized_ = false;
    y_axis_.resetInput();
    z_axis_.resetInput();
  }

  void updateFilter(const SourceSample & s)
  {
    const double dt = std::chrono::duration<double>(s.rx_time - last_filter_time_).count();
    if (!filter_initialized_ || !(dt > 0.0) || dt > settings_.cop_timeout_sec) {
      // A new contact/source seeds the LPF; the accumulated World offset is retained.
      resetFilter();
      filtered_y_m_ = s.y;
      filtered_z_m_ = s.z;
    } else {
      const double alpha = hih_controller::cop_safety::lpfAlpha(settings_.cop_lpf_cutoff_hz, dt);
      filtered_y_m_ += alpha * (s.y - filtered_y_m_);
      filtered_z_m_ += alpha * (s.z - filtered_z_m_);
    }
    filter_initialized_ = true;
    last_filter_time_ = s.rx_time;
    y_axis_.receive(filtered_y_m_, settings_.y);
    z_axis_.receive(filtered_z_m_, settings_.z);
  }

  void seedSelectedFilter()
  {
    resetFilter();
    const auto & s = selectedSample();
    if (fresh(s, Clock::now()) && s.valid && s.normal_force >= settings_.min_normal_force_n) {
      updateFilter(s);
    }
  }

  void onCop(CopSource source, const hih_interfaces::msg::CenterOfPressure & msg)
  {
    auto & s = samples_[sourceIndex(source)];
    s.received = true;
    s.rx_time = Clock::now();
    s.y = msg.y;
    s.z = msg.z;
    s.normal_force = msg.normal_force;
    s.valid = msg.valid && std::isfinite(s.y) && std::isfinite(s.z) &&
      std::isfinite(s.normal_force);
    if (source == source_) {
      if (s.valid && s.normal_force >= settings_.min_normal_force_n) updateFilter(s);
      else resetFilter();
    }
  }

  void publish(const std::array<double, 3> & world)
  {
    hih_interfaces::msg::Cmd offset;
    for (std::size_t i = 0; i < world.size(); ++i) {
      offset.pos_cmd[i] = static_cast<float>(world[i]);
    }
    offset_pub_->publish(offset);
    // Diagnostic projection only: never rotate this back into the output command.
    const auto plate = hih_controller::cop_translation::plateOffset(rpy_, world);
    for (std::size_t i = 0; i < plate.size(); ++i) {
      offset.pos_cmd[i] = static_cast<float>(plate[i]);
    }
    plate_offset_pub_->publish(offset);
    std_msgs::msg::Bool active;
    active.data = offsets_.yDistance() > 1e-7 || offsets_.zDistance() > 1e-7;
    active_pub_->publish(active);
    std_msgs::msg::Int32 source;
    source.data = static_cast<int>(source_);
    source_pub_->publish(source);
  }

  void publishBoundary()
  {
    // PyQt contract: master, y/z enables, y/z half-widths [m], MOB/SOLVER ID.
    // OFF is still published so the GUI can remove a previously drawn boundary.
    std_msgs::msg::Float64MultiArray state;
    state.data = {
      enabled_ ? 1.0 : 0.0, enable_y_ ? 1.0 : 0.0, enable_z_ ? 1.0 : 0.0,
      settings_.y.safe_m, settings_.z.safe_m, static_cast<double>(source_)};
    boundary_pub_->publish(state);
  }

  void tick()
  {
    // Publish settings independently of contact validity and correction activity.
    publishBoundary();
    const auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    if (!(dt > 0.0) || dt > 0.2) dt = 0.0;  // Do not invent motion across an invalid timer gap.
    const auto & s = selectedSample();
    const bool contact_usable = fresh(s, now) && s.valid &&
      s.normal_force >= settings_.min_normal_force_n;
    const bool state_usable = stateFresh(now);
    if (!contact_usable || !state_usable) resetFilter();
    const bool usable = contact_usable && filter_initialized_ && state_usable;
    const auto mode = [this, usable](bool axis_enabled) {
        if (!enabled_ || !axis_enabled) return MotionMode::RETURN_TO_ZERO;
        return usable ? MotionMode::TRACK : MotionMode::HOLD;
      };
    const auto mode_y = mode(enable_y_), mode_z = mode(enable_z_);
    const double velocity_y = y_axis_.velocityCommand(mode_y == MotionMode::TRACK, settings_.y);
    const double velocity_z = z_axis_.velocityCommand(mode_z == MotionMode::TRACK, settings_.z);
    offsets_.advance(rpy_, velocity_y, velocity_z, mode_y, mode_z, dt, settings_.y, settings_.z);
    const auto world = offsets_.world();
    publish(world);  // HOLD remains publishable without a fresh attitude transform.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "source=%s enabled=%d CoP(y,z)=(%+.3f,%+.3f) -> v=(%+.3f,%+.3f) m/s | "
      "World offset=(%+.3f,%+.3f,%+.3f) m",
      sourceName(source_), enabled_ ? 1 : 0, filtered_y_m_, filtered_z_m_,
      velocity_y, velocity_z, world[0], world[1], world[2]);
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    // Validate a candidate first. Rejecting an invalid batch must leave every
    // live setting/source/enable flag unchanged.
    Settings next = settings_;
    bool next_enabled = enabled_, next_y = enable_y_, next_z = enable_z_;
    CopSource next_source = source_;
    auto numeric = numericParameters(next);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    bool changed = false;
    for (const auto & p : params) {
      const auto & name = p.get_name();
      const auto removed = removedParameterReason(name);
      if (!removed.empty()) {
        result.reason = name + " removed: " + removed;
        return result;
      }
      if (name == "publish_rate_hz" || name == "enable_keyboard") {
        result.reason = name + " requires node restart";
        return result;
      }
      if (name == "enabled" || name == "enable_y_translation" || name == "enable_z_translation") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.reason = name + " must be bool";
          return result;
        }
        if (name == "enabled") next_enabled = p.as_bool();
        else if (name == "enable_y_translation") next_y = p.as_bool();
        else next_z = p.as_bool();
        changed = true;
      } else if (name == "cop_source") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            !parseSource(p.as_string(), next_source)) {
          result.reason = "cop_source must be mob or solver";
          return result;
        }
        changed = true;
      } else if (numeric.count(name)) {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.reason = name + " must be double";
          return result;
        }
        *numeric.at(name) = p.as_double();
        changed = true;
      }
    }
    result.reason = validationError(next);
    if (!result.reason.empty()) return result;
    if (!offsets_.fitsLimits(next.y, next.z)) {
      result.reason = "offset limits exclude held World offset; return to zero before reducing limits";
      return result;
    }
    settings_ = next;
    enabled_ = next_enabled;
    enable_y_ = next_y;
    enable_z_ = next_z;
    source_ = next_source;
    // Re-seed the selected LPF on source/gain/boundary changes.
    // Existing World offset is retained; OFF returns the requested channels to zero.
    if (changed) seedSelectedFilter();
    result.successful = true;
    return result;
  }

  void setupTerminal()
  {
    // Headless launch/test processes use ROS params and never take /dev/tty
    // away from another interactive node.
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_INFO(get_logger(), "No interactive stdin; use ROS parameters for ON/OFF and source");
      return;
    }
    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) return;
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ == -1) return;
    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      return;
    }
    terminal_ready_ = true;
  }

  void restoreTerminal()
  {
    if (!terminal_ready_) return;
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    (void)fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    terminal_ready_ = false;
  }

  void processKeyboard()
  {
    if (!terminal_ready_) return;
    char key = 0;
    while (read(STDIN_FILENO, &key, 1) > 0) {
      if (key == 't' || key == 'T') {
        (void)set_parameter(rclcpp::Parameter("enabled", !enabled_));
      } else if (key == 'c' || key == 'C') {
        const auto next = nextSource(source_);
        (void)set_parameter(rclcpp::Parameter("cop_source", std::string(sourceParameterValue(next))));
      } else if (key == 'p' || key == 'P') {
        const auto world = offsets_.world();
        RCLCPP_INFO(
          get_logger(), "enabled=%d source=%s | CoP(y,z)=(%+.4f,%+.4f) m | "
          "error=(%+.4f,%+.4f) m | World offset=(%+.4f,%+.4f,%+.4f) m | "
          "channel distances=(%.4f,%.4f) m",
          enabled_ ? 1 : 0, sourceName(source_), filtered_y_m_, filtered_z_m_,
          y_axis_.error(), z_axis_.error(), world[0], world[1], world[2],
          offsets_.yDistance(), offsets_.zDistance());
      } else if (key == 'q' || key == 'Q') {
        enabled_ = false;
        publishBoundary();
        offsets_.reset();
        publish({0.0, 0.0, 0.0});
        rclcpp::shutdown();
        return;
      }
    }
  }

  Settings settings_{};
  bool enabled_{false}, enable_y_{true}, enable_z_{true}, enable_keyboard_{true};
  CopSource source_{CopSource::MOB};
  std::array<SourceSample, kCopSourceCount> samples_{};
  Axis y_axis_, z_axis_;
  AccumulatedOffset offsets_;
  bool have_state_{false}, filter_initialized_{false};
  std::array<double, 3> rpy_{};
  double filtered_y_m_{0.0}, filtered_z_m_{0.0};
  Clock::time_point state_stamp_{}, last_filter_time_{}, last_tick_{};
  std::array<rclcpp::Subscription<hih_interfaces::msg::CenterOfPressure>::SharedPtr, kCopSourceCount> cop_subs_;
  rclcpp::Subscription<hih_interfaces::msg::MinitroneState>::SharedPtr state_sub_;
  rclcpp::Publisher<hih_interfaces::msg::Cmd>::SharedPtr offset_pub_, plate_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr boundary_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr source_pub_;
  rclcpp::TimerBase::SharedPtr timer_, keyboard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  bool terminal_ready_{false};
  int original_flags_{0};
  termios original_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FollowerCopTranslationController>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("follower_cop_translation_controller"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}




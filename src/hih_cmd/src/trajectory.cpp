#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace hih_cmd::trajectory
{
// Trajectory defaults live here, in the same file as the ROS node and path equations.
struct PathSettings
{
  double x_amplitude{0.3};      // x_amplitude_m: World X excursion [m].
  double y_amplitude{0.3};      // y_amplitude_m: World Y excursion [m].
  double z_amplitude{0.5};      // z_amplitude_m: YZ rectangle height [m].
  double path_speed{0.08};      // path_speed_mps: X/rectangle peak speed [m/s].
  double dwell{1.0};            // corner_dwell_s: X/rectangle endpoint hold [s].
  double omega{0.2};            // lissajous_omega_rad_s: X angular frequency [rad/s].
  double ratio{2.0};            // lissajous_y_ratio: Y/X frequency ratio.
  double ramp{3.0};             // lissajous_ramp_s: amplitude ramp duration [s].
  double return_speed{0.03};    // return_speed_mps: return peak speed [m/s].
  double return_min_time{2.5};  // return_min_time_s: minimum return duration [s].
};

using Point = std::array<double, 3>;

inline bool finite(const Point & p)
{
  return std::all_of(p.begin(), p.end(), [](double x) {return std::isfinite(x);});
}

inline double smooth(double phase)
{
  const double u = std::clamp(phase, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

inline Point mix(const Point & a, const Point & b, double u)
{
  return {a[0] + (b[0] - a[0]) * u,
          a[1] + (b[1] - a[1]) * u,
          a[2] + (b[2] - a[2]) * u};
}

inline double distance(const Point & a, const Point & b)
{
  return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]);
}

enum class CopSource {MOB, SOLVER};

inline bool parseSource(std::string text, CopSource & source)
{
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (text == "mob") {source = CopSource::MOB; return true;}
  if (text == "solver" || text == "cop_solver_real") {
    source = CopSource::SOLVER;
    return true;
  }
  return false;
}

inline std::size_t sourceIndex(CopSource source)
{
  return source == CopSource::MOB ? 0U : 1U;
}

struct CopSample
{
  bool received{false}, valid{false};
  double received_time{0.0};
};

struct ForceSample
{
  bool received{false}, valid{false};
  double normal_force{0.0}, received_time{0.0};
};

inline bool fresh(double stamp, double now, double timeout = 0.25)
{
  const double age = now - stamp;
  return std::isfinite(age) && age >= 0.0 && age < timeout;
}

inline bool contactReady(
  const CopSample & cop, const ForceSample & force, double now, double threshold)
{
  // Contact validity comes exclusively from the selected MOB/SOLVER CoP.
  // Preserve the existing Leader MoB normal-force threshold for a/lift.
  return cop.received && cop.valid && fresh(cop.received_time, now) &&
         force.received && force.valid && fresh(force.received_time, now) &&
         std::isfinite(force.normal_force) && force.normal_force >= threshold;
}

enum class PathState {IDLE, RUNNING, RETURNING, COMPLETE};
enum class KeyAction {IGNORED, STARTED, RETURN_STARTED, WAITING_FOR_STATE};

class PathGenerator
{
public:
  explicit PathGenerator(const PathSettings & settings = {}) : settings_(settings)
  {
    const std::array<double, 10> values{
      settings.x_amplitude, settings.y_amplitude, settings.z_amplitude,
      settings.path_speed, settings.dwell, settings.omega, settings.ratio,
      settings.ramp, settings.return_speed, settings.return_min_time};
    for (double value : values) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Trajectory settings must be finite and positive");
      }
    }
  }

  static bool isPathKey(char key) {return key == 'l' || key == 't' || key == 'r';}

  KeyAction press(char key, const Point & measured, double now, bool state_fresh)
  {
    if (!isPathKey(key) || !std::isfinite(now)) return KeyAction::IGNORED;
    if (key == key_ && state_ == PathState::RETURNING) return KeyAction::IGNORED;
    if (key == key_ && state_ == PathState::RUNNING) {
      // Start from the current reference to avoid a position-command jump.
      // Destination is the measured pose captured on the FIRST key press.
      return_from_ = sample(now);
      return_duration_ = std::max(settings_.return_min_time,
        1.875 * distance(return_from_, origin_) / settings_.return_speed);
      start_time_ = now;
      state_ = PathState::RETURNING;
      return KeyAction::RETURN_STARTED;
    }
    if (!state_fresh || !finite(measured)) return KeyAction::WAITING_FOR_STATE;
    // This deliberately has no approach/contact/lift/READY prerequisite.
    origin_ = measured;
    key_ = key;
    start_time_ = now;
    state_ = PathState::RUNNING;
    vertices_.clear();
    if (key == 't') {
      vertices_ = {Point{0, 0, 0}, Point{settings_.x_amplitude, 0, 0},
        Point{-settings_.x_amplitude, 0, 0}, Point{0, 0, 0}};
    } else if (key == 'r') {
      vertices_ = {Point{0, 0, 0}, Point{0, -settings_.y_amplitude, 0},
        Point{0, -settings_.y_amplitude, settings_.z_amplitude},
        Point{0, settings_.y_amplitude, settings_.z_amplitude},
        Point{0, settings_.y_amplitude, 0}, Point{0, 0, 0}};
    }
    cycle_duration_ = 0.0;
    for (std::size_t i = 1; i < vertices_.size(); ++i) {
      cycle_duration_ += segmentDuration(vertices_[i - 1], vertices_[i]) + settings_.dwell;
    }
    return KeyAction::STARTED;
  }

  Point sample(double now)
  {
    const double elapsed = std::max(0.0, now - start_time_);
    if (state_ == PathState::RETURNING) {
      if (elapsed >= return_duration_) {
        state_ = PathState::COMPLETE;
        return origin_;
      }
      return mix(return_from_, origin_, smooth(elapsed / return_duration_));
    }
    if (state_ != PathState::RUNNING) return origin_;
    if (key_ == 'l') {
      const double envelope = smooth(elapsed / settings_.ramp);
      return {origin_[0] + envelope * settings_.x_amplitude * std::sin(settings_.omega * elapsed),
              origin_[1] + envelope * settings_.y_amplitude * std::sin(settings_.ratio * settings_.omega * elapsed),
              origin_[2]};
    }
    // Closed cycles: t is X only; r is YZ only and includes the start point.
    // Evaluate by elapsed time so slow ROS ticks cannot skip/reorder corners.
    double remaining = std::fmod(elapsed, cycle_duration_);
    for (std::size_t i = 1; i < vertices_.size(); ++i) {
      const double travel = segmentDuration(vertices_[i - 1], vertices_[i]);
      if (remaining <= travel + settings_.dwell) {
        const Point offset = mix(vertices_[i - 1], vertices_[i], smooth(remaining / travel));
        return {origin_[0] + offset[0], origin_[1] + offset[1], origin_[2] + offset[2]};
      }
      remaining -= travel + settings_.dwell;
    }
    return origin_;
  }

  void reset() {state_ = PathState::IDLE; key_ = 0;}
  const Point & origin() const {return origin_;}
  PathState state() const {return state_;}
  char key() const {return key_;}
  double cycleDuration() const {return cycle_duration_;}
  double returnDuration() const {return return_duration_;}

private:
  double segmentDuration(const Point & a, const Point & b) const
  {
    // Quintic interpolation peak speed: 1.875 * length / duration.
    return std::max(1.0, 1.875 * distance(a, b) / settings_.path_speed);
  }
  PathSettings settings_;
  Point origin_{}, return_from_{};
  std::vector<Point> vertices_;
  PathState state_{PathState::IDLE};
  char key_{0};
  double start_time_{0.0}, cycle_duration_{0.0}, return_duration_{0.0};
};
}  // namespace hih_cmd::trajectory

// Standalone regression tests include this file and compile only the path logic.
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "hih_interfaces/msg/attitude_cmd.hpp"
#include "hih_interfaces/msg/center_of_pressure.hpp"
#include "hih_interfaces/msg/cmd.hpp"
#include "hih_interfaces/msg/minitrone_state.hpp"
#include "hih_interfaces/msg/wrench.hpp"

using Clock = std::chrono::steady_clock;
namespace trajectory = hih_cmd::trajectory;
using trajectory::Point;

namespace
{
double seconds(Clock::time_point from, Clock::time_point to)
{
  return std::chrono::duration<double>(to - from).count();
}

double nowSeconds(Clock::time_point now)
{
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}
}  // namespace

class HIHTrajectory : public rclcpp::Node
{
public:
  HIHTrajectory() : Node("hih_trajectory")
  {
    // Preserve the current repository's automatic approach/lift defaults.
    // Manual l/t/r paths do not require this sequence to have run.
    approach_speed_ = positive("approach_speed_mps", 0.03);
    approach_limit_ = positive("approach_max_distance_m", 10.00);
    contact_threshold_ = positive("contact_threshold_n", 0.8);
    desired_force_ = positive("desired_normal_force_n", 10.0);
    lift_height_ = positive("lift_height_m", 0.5);
    lift_time_ = positive("lift_time_s", 60.0);

    trajectory::PathSettings settings;
    settings.x_amplitude = positive("x_amplitude_m", settings.x_amplitude);
    settings.y_amplitude = positive("y_amplitude_m", settings.y_amplitude);
    settings.z_amplitude = positive("z_amplitude_m", settings.z_amplitude);
    settings.path_speed = positive("path_speed_mps", settings.path_speed);
    settings.dwell = positive("corner_dwell_s", settings.dwell);
    settings.omega = positive("lissajous_omega_rad_s", settings.omega);
    settings.ratio = positive("lissajous_y_ratio", settings.ratio);
    settings.ramp = positive("lissajous_ramp_s", settings.ramp);
    settings.return_speed = positive("return_speed_mps", settings.return_speed);
    settings.return_min_time = positive("return_min_time_s", settings.return_min_time);
    path_ = trajectory::PathGenerator(settings);
    const bool enable_keyboard = declare_parameter<bool>("enable_keyboard", true);
    if (!trajectory::parseSource(declare_parameter<std::string>("cop_source", "mob"), cop_source_)) {
      throw std::invalid_argument("cop_source must be mob or solver (cop_solver_real)");
    }

    leader_cmd_ = create_publisher<hih_interfaces::msg::Cmd>("/leader/minitrone/cmd", 10);
    leader_att_ = create_publisher<hih_interfaces::msg::AttitudeCmd>(
      "/leader/minitrone/att_cmd", 10);
    enable_ = create_publisher<std_msgs::msg::Bool>("/follower/minitrone/admittance_enable", 10);
    authority_ = create_publisher<std_msgs::msg::Bool>("/hih/trajectory_active", 10);
    state_sub_ = create_subscription<hih_interfaces::msg::MinitroneState>(
      "/leader/minitrone/state", 10,
      [this](hih_interfaces::msg::MinitroneState::SharedPtr m) {
        const Point incoming{m->pos[0], m->pos[1], m->pos[2]};
        const double incoming_yaw = static_cast<double>(m->rpy[2]) * 180.0 / kPi;
        if (!trajectory::finite(incoming) || !std::isfinite(incoming_yaw)) {
          have_state_ = false;
          return;
        }
        measured_ = incoming;
        yaw_deg_ = incoming_yaw;
        have_state_ = true;
        state_stamp_ = Clock::now();
      });
    wrench_sub_ = create_subscription<hih_interfaces::msg::Wrench>(
      "/leader/minitrone/external_wrench_hat_second_order", 10,
      [this](hih_interfaces::msg::Wrench::SharedPtr m) {
        // Environment-on-robot convention: compression is -body X.
        force_.received = true;
        force_.valid = std::isfinite(m->force[0]);
        force_.normal_force = force_.valid ? std::max(0.0, -static_cast<double>(m->force[0])) : 0.0;
        force_.received_time = nowSeconds(Clock::now());
      });
    // Only the two supported CoP sources. Never subscribe to 4-touch cop_real.
    cop_subs_[0] = create_subscription<hih_interfaces::msg::CenterOfPressure>(
      "/leader/minitrone/cop_hat", 10,
      [this](hih_interfaces::msg::CenterOfPressure::SharedPtr m) {
        receiveCop(trajectory::CopSource::MOB, *m);
      });
    cop_subs_[1] = create_subscription<hih_interfaces::msg::CenterOfPressure>(
      "/leader/minitrone/cop_solver_real", 10,
      [this](hih_interfaces::msg::CenterOfPressure::SharedPtr m) {
        receiveCop(trajectory::CopSource::SOLVER, *m);
      });
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return onParameters(parameters);
      });

    if (enable_keyboard) setupTerminal();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
    RCLCPP_INFO(get_logger(),
      "a: approach/lift; l: XY Lissajous; t: X-only cycle; r: YZ rectangle; "
      "l/t/r start at the current pose immediately; repeat the same key to return slowly");
  }

  ~HIHTrajectory() override
  {
    if (terminal_ready_) (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    if (rclcpp::ok()) {
      // Release trajectory ownership while preserving the user's admittance setting.
      if (owned_) publishAuthority(false);
    }
  }

private:
  enum class Phase {IDLE, APPROACH, PRELOAD, LIFT, READY, PATH, RETURN, RELEASE};
  static constexpr double kPi = 3.14159265358979323846;

  double positive(const std::string & name, double value)
  {
    const double n = declare_parameter<double>(name, value);
    if (!(n > 0.0) || !std::isfinite(n)) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return n;
  }

  void setupTerminal()
  {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_term_) != 0) {
      throw std::runtime_error("Run hih_trajectory in a focused interactive terminal");
    }
    termios raw = old_term_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("Unable to set keyboard mode");
    }
    terminal_ready_ = true;
  }

  bool stateFresh(Clock::time_point now) const
  {
    return have_state_ && state_stamp_ != Clock::time_point{} &&
           trajectory::fresh(nowSeconds(state_stamp_), nowSeconds(now));
  }

  void receiveCop(trajectory::CopSource source, const hih_interfaces::msg::CenterOfPressure & m)
  {
    auto & sample = cops_[trajectory::sourceIndex(source)];
    sample.received = true;
    sample.valid = m.valid && std::isfinite(m.y) && std::isfinite(m.z) &&
                   std::isfinite(m.normal_force) && m.normal_force > 0.0;
    sample.received_time = nowSeconds(Clock::now());
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    auto next_source = cop_source_;
    for (const auto & p : parameters) {
      if (p.get_name() == "cop_source") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            !trajectory::parseSource(p.as_string(), next_source)) {
          result.reason = "cop_source must be mob or solver; touch/real are unsupported";
          return result;
        }
      } else if (p.get_name() != "use_sim_time") {
        // Other tuning was already startup-only; reject misleading live edits.
        result.reason = p.get_name() + " requires node restart";
        return result;
      }
    }
    cop_source_ = next_source;
    result.successful = true;
    return result;
  }

  bool contactReady(Clock::time_point now, double threshold) const
  {
    return trajectory::contactReady(
      cops_[trajectory::sourceIndex(cop_source_)], force_, nowSeconds(now), threshold);
  }

  void setPhase(Phase p, Clock::time_point now) {phase_ = p; phase_start_ = now;}

  void publishAuthority(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    authority_->publish(msg);
  }

  void publishLeader()
  {
    hih_interfaces::msg::Cmd cmd;
    cmd.pos_cmd = {static_cast<float>(target_[0]), static_cast<float>(target_[1]),
                   static_cast<float>(target_[2])};
    leader_cmd_->publish(cmd);
    hih_interfaces::msg::AttitudeCmd att;
    att.yaw_ref = static_cast<float>(origin_yaw_);
    att.roll_ref = att.pitch_ref = 0.0F;
    leader_att_->publish(att);
  }

  void start(Clock::time_point now)
  {
    if (!stateFresh(now)) {
      RCLCPP_WARN(get_logger(), "Waiting for a fresh Leader state");
      return;
    }
    path_.reset();
    origin_ = target_ = measured_;
    origin_yaw_ = yaw_deg_;
    approach_limit_reported_ = false;
    owned_ = true;
    publishAuthority(true);
    publishLeader();
    // Retain automatic ON from 'a'; returning or exiting does not send OFF.
    std_msgs::msg::Bool on;
    on.data = true;
    enable_->publish(on);
    setPhase(Phase::APPROACH, now);
    RCLCPP_INFO(get_logger(), "Independent approach started at %.3f m/s", approach_speed_);
  }

  void key(char c, Clock::time_point now)
  {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (c == 'a' && phase_ == Phase::IDLE) {
      start(now);
      return;
    }
    if (!trajectory::PathGenerator::isPathKey(c)) return;
    const auto action = path_.press(c, measured_, nowSeconds(now), stateFresh(now));
    if (action == trajectory::KeyAction::STARTED) {
      // Also accepted during APPROACH/PRELOAD/LIFT/READY: the manual path
      // replaces that sequence, anchored to measured pose, never an old target.
      target_ = path_.origin();
      origin_yaw_ = yaw_deg_;
      owned_ = true;
      publishAuthority(true);
      publishLeader();
      setPhase(Phase::PATH, now);
      RCLCPP_INFO(get_logger(), "Trajectory %c started at measured [%.3f %.3f %.3f]; repeat %c to return",
        c, target_[0], target_[1], target_[2], c);
    } else if (action == trajectory::KeyAction::RETURN_STARTED) {
      setPhase(Phase::RETURN, now);
      RCLCPP_INFO(get_logger(), "Returning to first-key position over %.2f s", path_.returnDuration());
    } else if (action == trajectory::KeyAction::WAITING_FOR_STATE) {
      RCLCPP_WARN(get_logger(), "Trajectory %c needs a fresh Leader state (<0.25 s)", c);
    }
  }

  void readKeyboard(Clock::time_point now)
  {
    if (!terminal_ready_) return;
    for (int n = 0; n < 16; ++n) {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(STDIN_FILENO, &set);
      timeval tv{0, 0};
      if (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) <= 0) break;
      char c;
      if (read(STDIN_FILENO, &c, 1) == 1) key(c, now);
    }
  }

  void tick()
  {
    const auto now = Clock::now();
    readKeyboard(now);
    publishAuthority(owned_);
    if (!owned_) return;
    const double t = seconds(phase_start_, now);
    if (phase_ == Phase::APPROACH) {
      if (contactReady(now, contact_threshold_)) {
        setPhase(Phase::PRELOAD, now);
        RCLCPP_INFO(get_logger(), "Leader local contact %.3f N; waiting for preload", force_.normal_force);
      } else {
        const double step = std::min(approach_speed_ * t, approach_limit_);
        target_ = {origin_[0] + std::cos(origin_yaw_ * kPi / 180.0) * step,
                   origin_[1] + std::sin(origin_yaw_ * kPi / 180.0) * step, origin_[2]};
        if (step >= approach_limit_ && !approach_limit_reported_) {
          RCLCPP_WARN(get_logger(), "Leader approach limit reached; holding until local contact");
          approach_limit_reported_ = true;
        }
      }
    } else if (phase_ == Phase::PRELOAD) {
      if (contactReady(now, desired_force_)) {
        lift_from_ = target_;
        setPhase(Phase::LIFT, now);
        RCLCPP_INFO(get_logger(), "Leader normal force %.3f N >= %.3f N; lifting %.2f m",
          force_.normal_force, desired_force_, lift_height_);
      }
    } else if (phase_ == Phase::LIFT) {
      target_ = lift_from_;
      target_[2] += lift_height_ * trajectory::smooth(t / lift_time_);
      if (t >= lift_time_) {
        setPhase(Phase::READY, now);
        RCLCPP_INFO(get_logger(), "Lift complete; l/t/r can select a path (READY is optional)");
      }
    } else if (phase_ == Phase::PATH || phase_ == Phase::RETURN) {
      target_ = path_.sample(nowSeconds(now));
      if (phase_ == Phase::RETURN && path_.state() == trajectory::PathState::COMPLETE) {
        // Leader trajectory completion does not end Follower admittance.
        // Preserve admittance enabled by launch, 'a', or manual control.
        setPhase(Phase::RELEASE, now);
      }
    } else if (phase_ == Phase::RELEASE && t >= 0.3) {
      owned_ = false;
      path_.reset();
      publishAuthority(false);
      setPhase(Phase::IDLE, now);
      RCLCPP_INFO(get_logger(), "Control returned to teleop; l/t/r or a starts another run");
      return;
    }
    publishLeader();
  }

  double approach_speed_, approach_limit_, contact_threshold_, desired_force_;
  double lift_height_, lift_time_;
  double yaw_deg_{0.0}, origin_yaw_{0.0};
  Point measured_{}, origin_{}, target_{}, lift_from_{};
  trajectory::PathGenerator path_;
  std::array<trajectory::CopSample, 2> cops_{};
  trajectory::ForceSample force_;
  trajectory::CopSource cop_source_{trajectory::CopSource::MOB};
  Phase phase_{Phase::IDLE};
  Clock::time_point state_stamp_{}, phase_start_{};
  bool have_state_{false}, owned_{false};
  bool approach_limit_reported_{false}, terminal_ready_{false};
  termios old_term_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hih_interfaces::msg::Cmd>::SharedPtr leader_cmd_;
  rclcpp::Publisher<hih_interfaces::msg::AttitudeCmd>::SharedPtr leader_att_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_, authority_;
  rclcpp::Subscription<hih_interfaces::msg::MinitroneState>::SharedPtr state_sub_;
  rclcpp::Subscription<hih_interfaces::msg::Wrench>::SharedPtr wrench_sub_;
  std::array<rclcpp::Subscription<hih_interfaces::msg::CenterOfPressure>::SharedPtr, 2> cop_subs_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HIHTrajectory>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "hih_trajectory: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

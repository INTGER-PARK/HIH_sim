#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <hih_interfaces/msg/wrench.hpp>

namespace
{
// Two pi, for converting sinusoidal frequency to angular frequency.
constexpr double kTwoPi = 6.28318530717958647692;
}

class ExternalWrenchCmd : public rclcpp::Node
{
public:
  ExternalWrenchCmd()
  : rclcpp::Node("hih_external_wrench_cmd")
  {
    // Declare ROS parameters and defaults.
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<double>("force_step", 0.5);
    this->declare_parameter<double>("moment_step", 0.1);
    this->declare_parameter<double>("force_limit", 50.0);
    this->declare_parameter<double>("moment_limit", 10.0);
    // Sine disturbance amplitude.
    this->declare_parameter<double>("moment_disturbance_amplitude", 0.3);
    // Sine disturbance frequency [Hz].
    this->declare_parameter<double>("moment_disturbance_frequency_hz", 0.5);

    // Read parameters into node state.
    double publish_hz = this->get_parameter("publish_hz").as_double();
    force_step_ = this->get_parameter("force_step").as_double();
    moment_step_ = this->get_parameter("moment_step").as_double();
    force_limit_ = this->get_parameter("force_limit").as_double();
    moment_limit_ = this->get_parameter("moment_limit").as_double();
    moment_disturbance_amplitude_ = this->get_parameter("moment_disturbance_amplitude").as_double();
    moment_disturbance_frequency_hz_ = this->get_parameter("moment_disturbance_frequency_hz").as_double();

    // Normalize negative disturbance amplitude and frequency.
    if (publish_hz <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_hz <= 0.0, using 50 Hz");
      publish_hz = 50.0;
    }
    if (moment_disturbance_amplitude_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "moment_disturbance_amplitude < 0.0, using absolute value");
      moment_disturbance_amplitude_ = std::abs(moment_disturbance_amplitude_);
    }
    if (moment_disturbance_frequency_hz_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "moment_disturbance_frequency_hz < 0.0, using absolute value");
      moment_disturbance_frequency_hz_ = std::abs(moment_disturbance_frequency_hz_);
    }

    // Configure nonblocking terminal input.
    setupTerminal();

    // Publish an external wrench command in the node namespace.
    pub_wrench_ = this->create_publisher<hih_interfaces::msg::Wrench>("minitrone/external_wrench_cmd", 10);

    // Run onTimer at the configured publication rate.
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ExternalWrenchCmd::onTimer, this));

    printHelp(); // Print keyboard controls.
    RCLCPP_INFO(
      this->get_logger(),
      "ExternalWrenchCmd started. force_step=%.3f N, moment_step=%.3f Nm, force_limit=%.3f N, moment_limit=%.3f Nm, disturbance=%.3f Nm @ %.2f Hz",
      force_step_, moment_step_, force_limit_, moment_limit_,
      moment_disturbance_amplitude_, moment_disturbance_frequency_hz_);
  }

  ~ExternalWrenchCmd() override
  {
    // Publish a zero wrench and restore terminal settings on exit.
    publishZeroCommand();
    restoreTerminal();
  }

private:
  void setupTerminal()
  {
    // Use nonblocking terminal input without waiting for Enter.
    if (!isatty(STDIN_FILENO)) {
      input_fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK);
      if (input_fd_ == -1) {
        RCLCPP_WARN(this->get_logger(), "stdin is not a terminal and /dev/tty is unavailable; keyboard input is unavailable");
        input_fd_ = STDIN_FILENO;
        return;
      }
      close_input_fd_ = true;
    } else {
      input_fd_ = STDIN_FILENO;
    }

    if (tcgetattr(input_fd_, &original_termios_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO)); // Disable canonical input and terminal echo.
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(input_fd_, TCSANOW, &raw) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return;
    }

    original_flags_ = fcntl(input_fd_, F_GETFL, 0);
    if (original_flags_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "fcntl(F_GETFL) failed: %s", std::strerror(errno));
      return;
    }

    if (fcntl(input_fd_, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      RCLCPP_ERROR(this->get_logger(), "fcntl(F_SETFL) failed: %s", std::strerror(errno));
      return;
    }

    terminal_ready_ = true;
  }

  void restoreTerminal()
  {
    // Restore the original terminal state.
    if (!terminal_ready_) {
      if (close_input_fd_) {
        close(input_fd_);
        close_input_fd_ = false;
        input_fd_ = STDIN_FILENO;
      }
      return;
    }

    (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
    (void)fcntl(input_fd_, F_SETFL, original_flags_);
    terminal_ready_ = false;
    if (close_input_fd_) {
      close(input_fd_);
      close_input_fd_ = false;
      input_fd_ = STDIN_FILENO;
    }
  }

  void printHelp() const
  {
    // Print keyboard controls.
    std::printf(
      "\n"
      "[external_wrench_cmd key map]\n"
      "  q/a: Mx +/-    w/s: My +/-    e/d: Mz +/-\n"
      "  i/k: Fx +/-    j/l: Fy +/-    u: Fz +\n"
      "  o: toggle roll(Mx) sine disturbance\n"
      "  p: toggle pitch(My) sine disturbance\n"
      "  y: toggle yaw(Mz) sine disturbance\n"
      "  z: reset wrench to zero       x: quit node\n"
      "  wrench is body-frame and persists until changed or reset\n"
      "  disturbance: %.3f Nm @ %.2f Hz\n\n",
      moment_disturbance_amplitude_, moment_disturbance_frequency_hz_);
    std::fflush(stdout);
  }

  void onTimer()
  {
    // Read keys and publish the current wrench at each timer tick.
    processKeyboardInput();
    publishCommand();
  }

  void processKeyboardInput()
  {
    if (!terminal_ready_) {
      return;
    }

    char ch = 0;
    ssize_t n = 0;
    // Read pending keyboard input without blocking.
    while ((n = read(input_fd_, &ch, 1)) > 0) {
      switch (ch) {
        case 'q': increment(moment_[0], +moment_step_, moment_limit_, "Mx"); break; // Increase Mx.
        case 'a': increment(moment_[0], -moment_step_, moment_limit_, "Mx"); break; // Decrease Mx.
        case 'w': increment(moment_[1], +moment_step_, moment_limit_, "My"); break;
        case 's': increment(moment_[1], -moment_step_, moment_limit_, "My"); break;
        case 'e': increment(moment_[2], +moment_step_, moment_limit_, "Mz"); break;
        case 'd': increment(moment_[2], -moment_step_, moment_limit_, "Mz"); break;
        case 'i': increment(force_[0], +force_step_, force_limit_, "Fx"); break;
        case 'k': increment(force_[0], -force_step_, force_limit_, "Fx"); break;
        case 'j': increment(force_[1], +force_step_, force_limit_, "Fy"); break;
        case 'l': increment(force_[1], -force_step_, force_limit_, "Fy"); break;
        case 'u': increment(force_[2], +force_step_, force_limit_, "Fz"); break;
        case 'o':
          // Toggle the roll (Mx) sinusoidal disturbance.
          toggleDisturbance(roll_disturbance_enabled_, roll_disturbance_start_time_, "roll(Mx)");
          break;
        case 'p':
          // Toggle the pitch (My) sinusoidal disturbance.
          toggleDisturbance(pitch_disturbance_enabled_, pitch_disturbance_start_time_, "pitch(My)");
          break;
        case 'y':
          // Toggle the yaw (Mz) sinusoidal disturbance.
          toggleDisturbance(yaw_disturbance_enabled_, yaw_disturbance_start_time_, "yaw(Mz)");
          break;
        case 'z':
          // Reset all wrench components and disturbances.
          force_.fill(0.0);
          moment_.fill(0.0);
          roll_disturbance_enabled_ = false;
          pitch_disturbance_enabled_ = false;
          yaw_disturbance_enabled_ = false;
          RCLCPP_INFO(this->get_logger(), "external wrench reset to zero");
          break;
        case 'x':
          // Exit the node.
          RCLCPP_INFO(this->get_logger(), "quit requested");
          force_.fill(0.0);
          moment_.fill(0.0);
          roll_disturbance_enabled_ = false;
          pitch_disturbance_enabled_ = false;
          yaw_disturbance_enabled_ = false;
          publishZeroCommand();
          rclcpp::shutdown();
          return;
        default:
          break;
      }
    }

    if (n == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "keyboard read failed: %s", std::strerror(errno));
    }
  }

  void increment(double & value, double delta, double limit, const char * name)
  {
    // Clamp each increment to its configured magnitude limit.
    const double before = value;
    value = std::clamp(value + delta, -limit, limit);
    RCLCPP_INFO(this->get_logger(), "%s: %.3f -> %.3f", name, before, value);
  }

  void toggleDisturbance(bool & enabled, rclcpp::Time & start_time, const char * name)
  {
    // Toggle the sine disturbance and reset its time origin when enabled.
    enabled = !enabled;
    start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "%s sine disturbance %s", name, enabled ? "on" : "off");
  }

  double disturbanceMoment(bool enabled, const rclcpp::Time & start_time) const
  {
    // Evaluate the sinusoidal moment disturbance.
    if (!enabled || moment_disturbance_amplitude_ == 0.0 || moment_disturbance_frequency_hz_ == 0.0) {
      return 0.0;
    }

    // Elapsed time since this disturbance was enabled [s].
    const double elapsed = (this->now() - start_time).seconds();
    
    // Moment disturbance: A*sin(2*pi*f*t).
    return moment_disturbance_amplitude_ * std::sin(kTwoPi * moment_disturbance_frequency_hz_ * elapsed);
  }

  void publishCommand()
  {
    hih_interfaces::msg::Wrench msg;
    for (size_t i = 0; i < 3; ++i) {
      msg.force[i] = static_cast<float>(force_[i]);
      msg.moment[i] = static_cast<float>(moment_[i]);
    }
    
    // Add the roll sine disturbance and apply the moment limit.
    msg.moment[0] = static_cast<float>(std::clamp(
      moment_[0] + disturbanceMoment(roll_disturbance_enabled_, roll_disturbance_start_time_),
      -moment_limit_,
      moment_limit_));
      
    // Add the pitch sine disturbance and apply the moment limit.
    msg.moment[1] = static_cast<float>(std::clamp(
      moment_[1] + disturbanceMoment(pitch_disturbance_enabled_, pitch_disturbance_start_time_),
      -moment_limit_,
      moment_limit_));

    // Add the yaw sine disturbance and apply the moment limit.
    msg.moment[2] = static_cast<float>(std::clamp(
      moment_[2] + disturbanceMoment(yaw_disturbance_enabled_, yaw_disturbance_start_time_),
      -moment_limit_,
      moment_limit_));
      
    pub_wrench_->publish(msg);
  }

  void publishZeroCommand()
  {
    // Publish the zero wrench five times when resetting or exiting.
    if (!pub_wrench_) {
      return;
    }

    hih_interfaces::msg::Wrench msg;
    for (size_t i = 0; i < 3; ++i) {
      msg.force[i] = 0.0f;
      msg.moment[i] = 0.0f;
    }

    for (int i = 0; i < 5; ++i) {
      pub_wrench_->publish(msg);
    }
  }

  // ROS publisher and timer.
  rclcpp::Publisher<hih_interfaces::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Base force and moment applied to the vehicle.
  std::array<double, 3> force_{0.0, 0.0, 0.0};
  std::array<double, 3> moment_{0.0, 0.0, 0.0};

  // Configuration parameters.
  double force_step_{0.5};
  double moment_step_{0.02};
  double force_limit_{3.0};
  double moment_limit_{0.2};
  double moment_disturbance_amplitude_{0.3}; // Amplitude [N*m].
  double moment_disturbance_frequency_hz_{0.5}; // Frequency [Hz].

  // Per-axis disturbance state.
  bool roll_disturbance_enabled_{false};
  bool pitch_disturbance_enabled_{false};
  bool yaw_disturbance_enabled_{false};
  rclcpp::Time roll_disturbance_start_time_{};
  rclcpp::Time pitch_disturbance_start_time_{};
  rclcpp::Time yaw_disturbance_start_time_{};

  // Terminal input state.
  bool terminal_ready_{false};
  bool close_input_fd_{false};
  int input_fd_{STDIN_FILENO};
  int original_flags_{0};
  termios original_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalWrenchCmd>());
  rclcpp::shutdown();
  return 0;
}

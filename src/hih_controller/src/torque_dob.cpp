#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <hih_interfaces/msg/mob_observer_input.hpp>
#include <hih_interfaces/msg/torque_dob.hpp>
#include <hih_interfaces/msg/wrench.hpp>
#include <std_msgs/msg/bool.hpp>
#include "hih_controller/torque_dob_core.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

// ==================== DOB TUNING DEFAULTS ====================
// Array order: [roll (Body x), pitch (Body y), yaw (Body z)].
// These are provisional defaults, not identified optimal hardware values.
// ROS parameters can override these defaults.
namespace tuning {
constexpr hih_dob::Mask3 axis={true,true,true}; // Axis compensation flags; all three axes are always observed.
constexpr hih_dob::Vec3 inertia={0.0730017,0.08790154,0.10388154}; // inertia_xx/yy/zz [kg*m^2]: nominal J.
constexpr hih_dob::Vec3 cutoff={3.0, 3.0, 3.0}; // Q-filter cutoff [Hz]; higher bandwidth increases noise sensitivity.
constexpr hih_dob::Vec3 gain={0.85,0.85,0.85}; // Compensation fraction in tau_comp=-gain*d_hat, range [0,1].
constexpr hih_dob::Vec3 limit={2.0,2.0,2.0}; // Per-axis compensation torque limit [N*m].
constexpr double max_dt=0.05; // Reset the observer after larger simulation-time gaps [s].
constexpr double timeout=0.15; // Disable compensation when state/wrench input becomes stale [s].
}  // namespace tuning
// ========================================================================

namespace {
using hih_dob::Mask3;
using hih_dob::Vec3;
using Clock=std::chrono::steady_clock;

const std::array<std::string,3> axis_name={"roll","pitch","yaw"};
const std::array<std::string,3> inertia_name={"inertia_xx","inertia_yy","inertia_zz"};

struct Config {
  Mask3 axes{};
  Vec3 inertia{},cutoff{},gain{},limit{};
  double max_dt{},timeout{};
};

void validate(const Config & c) {
  auto bounded=[](double value,double low,double high,const char * name) {
    if (!std::isfinite(value) || value<low || value>high)
      throw std::invalid_argument(name);
  };
  for(int i=0;i<3;++i) {
    bounded(c.inertia[i],1e-6,100.0,"inertia must be finite and positive");
    bounded(c.cutoff[i],0.01,30.0,"cutoff_hz must be in [0.01,30]");
    bounded(c.gain[i],0.0,1.0,"gain must be in [0,1]");
    bounded(c.limit[i],0.0,5.0,"compensation_limit must be in [0,5] Nm");
  }
  bounded(c.max_dt,0.001,0.2,"max_dt_sec must be in [0.001,0.2]");
  bounded(c.timeout,0.02,0.5,"input_timeout_sec must be in [0.02,0.5]");
}
}  // namespace

class TorqueDob : public rclcpp::Node {
public:
  TorqueDob():Node("hih_torque_dob") {
    for(int i=0;i<3;++i) {
      declare_parameter(axis_name[i]+"_enabled",tuning::axis[i]);
      declare_parameter(inertia_name[i],tuning::inertia[i]);
      declare_parameter("cutoff_hz_"+axis_name[i],tuning::cutoff[i]);
      declare_parameter("gain_"+axis_name[i],tuning::gain[i]);
      declare_parameter("compensation_limit_"+axis_name[i],tuning::limit[i]);
    }
    declare_parameter("max_dt_sec",tuning::max_dt);
    declare_parameter("input_timeout_sec",tuning::timeout);
    config_=readConfig();
    validate(config_);

    // Reuse only step/sim_time/body gyro from the timestamped plant packet.
    // This is NOT a subscription to the MoB estimate; actuation fields are ignored.
    const auto state_topic=topic("state_topic","minitrone/mob_observer_input");
    // Latest FINAL torque command sent by wrench_controller to the allocator.
    const auto command_topic=topic("command_topic","minitrone/wrench_cmd");
    const auto status_topic=topic("status_topic","minitrone/dob_status");

    pub_=create_publisher<hih_interfaces::msg::TorqueDob>(status_topic,10);
    sub_command_=create_subscription<hih_interfaces::msg::Wrench>(
      command_topic,10,[this](hih_interfaces::msg::Wrench::SharedPtr msg) {
        const Vec3 incoming{
          static_cast<double>(msg->moment[0]),
          static_cast<double>(msg->moment[1]),
          static_cast<double>(msg->moment[2])};
        if (!hih_dob::finite(incoming)) {
          have_command_=false;
          disableAndReset("non-finite wrench command");
          return;
        }
        command_=incoming;
        have_command_=true;
        command_rx_=Clock::now();
      });
    sub_state_=create_subscription<hih_interfaces::msg::MobObserverInput>(
      state_topic,10,[this](hih_interfaces::msg::MobObserverInput::SharedPtr msg) {
        onState(*msg);
      });
    sub_reset_=create_subscription<std_msgs::msg::Bool>(
      "minitrone/controller_reset",10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) disableAndReset("controller reset");
      });

    parameters_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful=true;
        try { validate(readConfig(parameters)); }
        catch(const std::exception & e) {
          result.successful=false;
          result.reason=e.what();
        }
        return result;
      });

    setupTerminal();
    keyboard_timer_=create_wall_timer(
      std::chrono::milliseconds(20),[this]() { processKeyboard(); });
    watchdog_=create_wall_timer(
      std::chrono::milliseconds(20),[this]() {
        if (have_sample_ &&
            (!fresh(state_rx_) || !have_command_ || !fresh(command_rx_))) {
          disableAndReset("input timeout");
        }
      });

    printHelp();
    RCLCPP_INFO(
      get_logger(),
      "Torque DoB observation started. Compensation is OFF; press [d] to enable.");
  }

  ~TorqueDob() override {
    // Normal object destruction: publish an explicit zero request when possible.
    compensation_enabled_=false;
    correction_={0,0,0};
    publishStatus(observer_valid_);
    restoreTerminal();
  }

private:
  std::string topic(const std::string & name,const std::string & value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    return declare_parameter<std::string>(name,value,descriptor);
  }

  Config readConfig(const std::vector<rclcpp::Parameter> & proposed={}) const {
    auto p=[this,&proposed](const std::string & name) {
      for(const auto & item:proposed)
        if(item.get_name()==name) return item;
      return get_parameter(name);
    };

    Config c;
    for(int i=0;i<3;++i) {
      c.axes[i]=p(axis_name[i]+"_enabled").as_bool();
      c.inertia[i]=p(inertia_name[i]).as_double();
      c.cutoff[i]=p("cutoff_hz_"+axis_name[i]).as_double();
      c.gain[i]=p("gain_"+axis_name[i]).as_double();
      c.limit[i]=p("compensation_limit_"+axis_name[i]).as_double();
    }
    c.max_dt=p("max_dt_sec").as_double();
    c.timeout=p("input_timeout_sec").as_double();
    return c;
  }

  bool fresh(Clock::time_point time) const {
    return hih_dob::fresh(
      std::chrono::duration<double>(Clock::now()-time).count(),
      config_.timeout);
  }

  void setupTerminal() {
    if (!isatty(STDIN_FILENO)) {
      input_fd_=open("/dev/tty",O_RDONLY|O_NONBLOCK);
      if(input_fd_==-1) {
        RCLCPP_WARN(
          get_logger(),
          "No interactive terminal: observation works, but [d]/[r]/[p]/[q] keys are unavailable.");
        input_fd_=STDIN_FILENO;
        return;
      }
      close_input_fd_=true;
    } else {
      input_fd_=STDIN_FILENO;
    }

    if(tcgetattr(input_fd_,&original_termios_)!=0) {
      RCLCPP_WARN(get_logger(),"tcgetattr failed: %s",std::strerror(errno));
      return;
    }

    termios raw=original_termios_;
    raw.c_lflag&=static_cast<unsigned>(~(ICANON|ECHO));
    raw.c_cc[VMIN]=0;
    raw.c_cc[VTIME]=0;
    if(tcsetattr(input_fd_,TCSANOW,&raw)!=0) {
      RCLCPP_WARN(get_logger(),"tcsetattr failed: %s",std::strerror(errno));
      return;
    }

    original_flags_=fcntl(input_fd_,F_GETFL,0);
    if(original_flags_==-1 ||
       fcntl(input_fd_,F_SETFL,original_flags_|O_NONBLOCK)==-1) {
      RCLCPP_WARN(get_logger(),"failed to enable nonblocking terminal input");
      (void)tcsetattr(input_fd_,TCSANOW,&original_termios_);
      return;
    }
    terminal_ready_=true;
  }

  void restoreTerminal() {
    if(terminal_ready_) {
      (void)tcsetattr(input_fd_,TCSANOW,&original_termios_);
      (void)fcntl(input_fd_,F_SETFL,original_flags_);
      terminal_ready_=false;
    }
    if(close_input_fd_) {
      close(input_fd_);
      close_input_fd_=false;
      input_fd_=STDIN_FILENO;
    }
  }

  void printHelp() const {
    std::printf(
      "\n[Torque DoB] 3-axis observation starts with this node\n"
      "  d : toggle disturbance compensation ON/OFF\n"
      "      ON transition resets the observer/Q-filter to d_hat=0.\n"
      "  r : reset observer and force compensation OFF\n"
      "  p : print current estimates/correction\n"
      "  q : publish zero correction and quit this DoB node\n\n");
    std::fflush(stdout);
  }

  void processKeyboard() {
    if(!terminal_ready_) return;
    char key=0;
    ssize_t n=0;
    while((n=read(input_fd_,&key,1))>0) {
      if(key=='d') {
        toggleCompensation();
      } else if(key=='r') {
        disableAndReset("manual reset");
        RCLCPP_INFO(get_logger(),"Observer reset; compensation OFF");
      } else if(key=='p') {
        printStatus();
      } else if(key=='q') {
        compensation_enabled_=false;
        correction_={0,0,0};
        publishStatus(observer_valid_);
        RCLCPP_INFO(get_logger(),"Zero correction published; quitting Torque DoB");
        rclcpp::shutdown();
        return;
      }
    }
    if(n==-1 && errno!=EAGAIN && errno!=EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),*get_clock(),2000,
        "keyboard read failed: %s",std::strerror(errno));
    }
  }

  void toggleCompensation() {
    if(compensation_enabled_) {
      // OFF is immediate by design. The structural bumpless treatment is for
      // engagement: enabling never connects a pre-existing nonzero estimate.
      compensation_enabled_=false;
      correction_={0,0,0};
      publishStatus(observer_valid_);
      RCLCPP_INFO(get_logger(),"DoB compensation OFF; observation continues");
      return;
    }

    if(!have_sample_ || !have_command_ || !fresh(state_rx_) || !fresh(command_rx_)) {
      RCLCPP_WARN(get_logger(),"Cannot enable compensation: observer inputs are not ready");
      return;
    }

    // Structural bumpless engagement:
    // discard the monitor-only estimate and reinitialize the EXISTING Q-filter.
    // The next sample starts at d_hat=0; no ramp/blend/slew controller is added.
    observer_.reset();
    observer_valid_=false;
    correction_={0,0,0};
    compensation_enabled_=true;
    publishStatus(false);
    RCLCPP_INFO(
      get_logger(),
      "DoB compensation ON: observer reinitialized, correction will rebuild from zero via Q-filter");
  }

  void printStatus() const {
    const auto & estimate=observer_.estimate();
    RCLCPP_INFO(
      get_logger(),
      "DOB %s | valid=%s | d_hat=[%.4f %.4f %.4f] Nm | comp=[%.4f %.4f %.4f] Nm",
      compensation_enabled_?"COMP ON":"OBSERVE ONLY",
      observer_valid_?"true":"false",
      estimate[0],estimate[1],estimate[2],
      correction_[0],correction_[1],correction_[2]);
  }

  void publishStatus(bool valid) {
    hih_interfaces::msg::TorqueDob msg;
    msg.dob_step=step_;
    msg.dob_sim_time=sim_time_;
    msg.dob_observer_valid=valid;
    msg.dob_compensation_enabled=compensation_enabled_;
    msg.dob_axis_enabled=config_.axes;
    msg.dob_estimated_torque=observer_.estimate();
    msg.dob_compensation_torque=correction_;
    msg.dob_compensation_limit=config_.limit;
    pub_->publish(msg);
  }

  void resetObserver(bool keep_compensation) {
    observer_.reset();
    observer_valid_=false;
    correction_={0,0,0};
    if(!keep_compensation) compensation_enabled_=false;
  }

  void disableAndReset(const char * reason) {
    const bool was_active=compensation_enabled_;
    resetObserver(false);
    publishStatus(false);
    if(was_active) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),*get_clock(),2000,
        "DoB compensation disabled: %s",reason);
    }
  }

  void onState(const hih_interfaces::msg::MobObserverInput & msg) {
    const Config next=readConfig();
    const bool model_changed=
      next.inertia!=config_.inertia || next.cutoff!=config_.cutoff;
    const bool compensation_map_changed=
      next.axes!=config_.axes || next.gain!=config_.gain || next.limit!=config_.limit;
    config_=next;

    if(model_changed || (compensation_enabled_ && compensation_map_changed)) {
      // Avoid a parameter-change torque step while compensation is active.
      // Reuse the same structural solution as [d] engagement: reset d_hat to 0.
      resetObserver(compensation_enabled_);
    }

    const Vec3 omega=msg.w_rpy;
    if(!std::isfinite(msg.sim_time) || !hih_dob::finite(omega)) {
      disableAndReset("non-finite state");
      return;
    }

    if(have_sample_ && msg.sim_time==sim_time_ && msg.step==step_) return;
    if(have_sample_ &&
       (msg.sim_time<=sim_time_ || msg.sim_time-sim_time_>config_.max_dt ||
        msg.step<=step_)) {
      resetObserver(compensation_enabled_);
    }

    if(!have_command_ || !fresh(command_rx_)) {
      disableAndReset("missing/stale wrench command");
      return;
    }

    sim_time_=msg.sim_time;
    step_=msg.step;
    state_rx_=Clock::now();
    have_sample_=true;

    const bool updated=observer_.update(
      sim_time_,omega,command_,config_.inertia,config_.cutoff,config_.max_dt);
    if(updated) observer_valid_=true;

    correction_=hih_dob::compensation(
      observer_.estimate(),
      compensation_enabled_ && observer_valid_,
      config_.axes,config_.gain,config_.limit);
    publishStatus(observer_valid_);
  }

  Config config_;
  hih_dob::Observer observer_;
  Vec3 command_{},correction_{};

  Clock::time_point command_rx_{},state_rx_{};
  bool have_command_{false};
  bool have_sample_{false};
  bool observer_valid_{false};
  bool compensation_enabled_{false};
  double sim_time_{0.0};
  uint64_t step_{0};

  rclcpp::Publisher<hih_interfaces::msg::TorqueDob>::SharedPtr pub_;
  rclcpp::Subscription<hih_interfaces::msg::MobObserverInput>::SharedPtr sub_state_;
  rclcpp::Subscription<hih_interfaces::msg::Wrench>::SharedPtr sub_command_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_,watchdog_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;

  bool terminal_ready_{false};
  bool close_input_fd_{false};
  int input_fd_{STDIN_FILENO};
  int original_flags_{0};
  termios original_termios_{};
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    rclcpp::spin(std::make_shared<TorqueDob>());
  } catch(const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hih_torque_dob"),"%s",e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

// Leader normal-axis indirect force controller.
//
// Purpose
// -------
// Keep the Leader plate normal preload near a desired value without replacing
// the existing trajectory/position controller.  This node converts normal-force
// error into a BODY +X velocity correction, integrates it into a bounded
// position offset, rotates that scalar offset into World, and publishes only
// the offset.  leader_wrench_controller superposes it on the ordinary Leader
// position reference.
//
// Control law
// -----------
//   F_n     = max(0, -F_ext,B,x)
//   e_F     = F_des - F_n
//   v_n     = sat(Kp * deadband(e_F), +/- v_max)
//   delta_x = integral(v_n dt), bounded by +/- x_max
//   delta_p_W = R_WB * [delta_x, 0, 0]^T
//
// This is intentionally a P-only indirect force controller, not a second position
// controller and not a full admittance model.  It does not publish attitude,
// actuator wrench, or a replacement trajectory.
//
// Standalone:
//   ros2 run hih_controller leader_normal_force
//
// Useful overrides:
//   ros2 run hih_controller leader_normal_force --ros-args
//     -p desired_normal_force_n:=10.0 -p kp_mps_per_n:=0.01
//     -p velocity_limit_mps:=0.03 -p offset_limit_m:=0.05
//
// The node starts enabled but does not arm until local normal force exceeds
// contact_threshold_n.  Before contact its published offset is exactly zero.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "hih_interfaces/msg/cmd.hpp"
#include "hih_interfaces/msg/normal_force.hpp"
#include "hih_interfaces/msg/minitrone_state.hpp"
#include "hih_interfaces/msg/wrench.hpp"

class LeaderNormalForce : public rclcpp::Node
{
public:
  LeaderNormalForce()
  : rclcpp::Node("leader_normal_force", "/leader")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    desired_force_n_ = positive("desired_normal_force_n", 10.0);
    kp_mps_per_n_ = nonnegative("kp_mps_per_n", 0.01);
    force_deadband_n_ = nonnegative("force_deadband_n", 0.20);
    velocity_limit_mps_ = positive("velocity_limit_mps", 0.03);
    offset_limit_m_ = positive("offset_limit_m", 0.05);
    contact_threshold_n_ = nonnegative("contact_threshold_n", 0.8);
    force_timeout_sec_ = positive("force_timeout_sec", 0.15);
    state_timeout_sec_ = positive("state_timeout_sec", 0.15);
    control_rate_hz_ = positive("control_rate_hz", 100.0);

    const auto period_ns = static_cast<int64_t>(
      std::llround(1.0e9 / control_rate_hz_));
    if (period_ns <= 0) {
      throw std::invalid_argument("control_rate_hz produces an invalid timer period");
    }

    force_sub_ = create_subscription<hih_interfaces::msg::Wrench>(
      "minitrone/external_wrench_hat_second_order", 10,
      [this](const hih_interfaces::msg::Wrench::SharedPtr msg) {
        const double fx = static_cast<double>(msg->force[0]);
        if (!std::isfinite(fx)) {
          return;
        }
        // The payload reaction on the Leader plate is -Body-X.
        normal_force_n_ = std::max(0.0, -fx);
        force_stamp_ = SteadyClock::now();
        have_force_ = true;

        hih_interfaces::msg::NormalForce out;
        out.normal_force = normal_force_n_;
        normal_force_pub_->publish(out);
      });

    state_sub_ = create_subscription<hih_interfaces::msg::MinitroneState>(
      "minitrone/state", 10,
      [this](const hih_interfaces::msg::MinitroneState::SharedPtr msg) {
        Eigen::Vector3d incoming(
          static_cast<double>(msg->rpy[0]),
          static_cast<double>(msg->rpy[1]),
          static_cast<double>(msg->rpy[2]));
        if (!incoming.allFinite()) {
          return;
        }
        rpy_ = incoming;
        state_stamp_ = SteadyClock::now();
        have_state_ = true;
      });

    offset_pub_ = create_publisher<hih_interfaces::msg::Cmd>(
      "minitrone/normal_force_position_offset", 10);
    normal_force_pub_ = create_publisher<hih_interfaces::msg::NormalForce>(
      "minitrone/normal_force", 10);

    last_tick_ = SteadyClock::now();
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(period_ns),
      std::bind(&LeaderNormalForce::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Leader normal-force P controller ready: enabled=%s Fdes=%.2f N "
      "Kp=%.4f m/(N*s) vlim=%.3f m/s xlim=%.3f m",
      enabled_ ? "true" : "false", desired_force_n_,
      kp_mps_per_n_, velocity_limit_mps_, offset_limit_m_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  double positive(const char * name, double fallback)
  {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
    return value;
  }

  double nonnegative(const char * name, double fallback)
  {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
    }
    return value;
  }

  static Eigen::Matrix3d rotationWorldFromBody(const Eigen::Vector3d & rpy)
  {
    const double r = rpy.x();
    const double p = rpy.y();
    const double y = rpy.z();
    const double sr = std::sin(r), cr = std::cos(r);
    const double sp = std::sin(p), cp = std::cos(p);
    const double sy = std::sin(y), cy = std::cos(y);

    Eigen::Matrix3d r_wb;
    r_wb <<
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp,     cp * sr,                cp * cr;
    return r_wb;
  }

  static double ageSeconds(
    SteadyClock::time_point stamp, SteadyClock::time_point now)
  {
    if (stamp == SteadyClock::time_point{}) {
      return INFINITY;
    }
    return std::chrono::duration<double>(now - stamp).count();
  }

  void publishOffset(const Eigen::Vector3d & offset_world)
  {
    hih_interfaces::msg::Cmd msg;
    msg.pos_cmd[0] = static_cast<float>(offset_world.x());
    msg.pos_cmd[1] = static_cast<float>(offset_world.y());
    msg.pos_cmd[2] = static_cast<float>(offset_world.z());
    offset_pub_->publish(msg);
  }

  void tick()
  {
    const auto now = SteadyClock::now();
    double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
      dt = 1.0 / control_rate_hz_;
    }

    const bool force_fresh =
      have_force_ && ageSeconds(force_stamp_, now) <= force_timeout_sec_;
    const bool state_fresh =
      have_state_ && ageSeconds(state_stamp_, now) <= state_timeout_sec_;

    Eigen::Vector3d offset_world = Eigen::Vector3d::Zero();
    double raw_error = desired_force_n_ - normal_force_n_;
    double effective_error = 0.0;
    double velocity_cmd = 0.0;

    if (!enabled_) {
      // Disabled means zero trim.  Keep the node publishing so the Leader
      // wrench controller never has to infer state from topic silence.
      armed_ = false;
      normal_offset_m_ = 0.0;
      publishOffset(offset_world);
      return;
    }

    if (!armed_) {
      // Do not "search" for the payload in free space.  The ordinary approach
      // controller establishes contact; this force controller takes over only after
      // a local compressive reaction is already present.
      if (force_fresh && state_fresh && normal_force_n_ >= contact_threshold_n_) {
        armed_ = true;
        normal_offset_m_ = 0.0;
        RCLCPP_INFO(
          get_logger(),
          "normal-force controller armed at Fn=%.3f N (target %.3f N)",
          normal_force_n_, desired_force_n_);
      }
      publishOffset(offset_world);
      return;
    }

    if (force_fresh && state_fresh) {
      if (std::abs(raw_error) > force_deadband_n_) {
        effective_error =
          raw_error - std::copysign(force_deadband_n_, raw_error);
      }

      velocity_cmd = std::clamp(
        kp_mps_per_n_ * effective_error,
        -velocity_limit_mps_, velocity_limit_mps_);

      normal_offset_m_ += velocity_cmd * dt;
      normal_offset_m_ = std::clamp(
        normal_offset_m_, -offset_limit_m_, offset_limit_m_);
    } else {
      // Freeze the last trim if the estimator/state stream is temporarily
      // stale.  If this whole node dies, leader_wrench_controller independently
      // drops the offset after its own freshness timeout.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "normal-force controller input stale: force_fresh=%d state_fresh=%d; offset frozen",
        force_fresh ? 1 : 0, state_fresh ? 1 : 0);
    }

    if (have_state_) {
      const Eigen::Vector3d normal_world =
        rotationWorldFromBody(rpy_) * Eigen::Vector3d::UnitX();
      offset_world = normal_offset_m_ * normal_world;
    }

    publishOffset(offset_world);
  }

  bool enabled_{true};
  bool have_force_{false};
  bool have_state_{false};
  bool armed_{false};

  double desired_force_n_{10.0};
  double kp_mps_per_n_{0.01};
  double force_deadband_n_{0.20};
  double velocity_limit_mps_{0.03};
  double offset_limit_m_{0.05};
  double contact_threshold_n_{0.8};
  double force_timeout_sec_{0.15};
  double state_timeout_sec_{0.15};
  double control_rate_hz_{100.0};

  double normal_force_n_{0.0};
  double normal_offset_m_{0.0};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};

  SteadyClock::time_point force_stamp_{};
  SteadyClock::time_point state_stamp_{};
  SteadyClock::time_point last_tick_{};

  rclcpp::Subscription<hih_interfaces::msg::Wrench>::SharedPtr force_sub_;
  rclcpp::Subscription<hih_interfaces::msg::MinitroneState>::SharedPtr state_sub_;
  rclcpp::Publisher<hih_interfaces::msg::Cmd>::SharedPtr offset_pub_;
  rclcpp::Publisher<hih_interfaces::msg::NormalForce>::SharedPtr normal_force_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderNormalForce>());
  rclcpp::shutdown();
  return 0;
}

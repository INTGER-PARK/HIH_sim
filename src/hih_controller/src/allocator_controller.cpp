// Shared control allocator, instantiated once in /leader and once in /follower.
//
// Input:  minitrone/wrench_cmd, ordered as Body moment xyz then Body force xyz.
// State:  measured servo angles from minitrone/state.
// Output: minitrone/input = four motor speeds followed by four servo angles.
//
// Allocation is deliberately split into three steps in onWrench(): seed motor
// thrust at the measured tilt, solve the lateral/yaw tilt request, then solve
// motor thrust again at the commanded tilt. Keep the saturation checks when
// changing geometry or limits; the wrench controllers use that feedback for
// anti-windup.

#include <rclcpp/rclcpp.hpp>
#include <hih_interfaces/msg/wrench.hpp>
#include <hih_interfaces/msg/input.hpp>
#include <hih_interfaces/msg/minitrone_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <limits>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

class AllocatorController : public rclcpp::Node 
{
public:
  static constexpr double inv_sqrt2   = 1.0 / 1.4142135623730951;
  static constexpr size_t buffer_size = 10;
  static constexpr double deg_to_rad  = M_PI / 180.0;

  static constexpr double zeta = 0.02;        // reaction torque coefficient
  static constexpr double k_thrust = 0.02;    // thrust coefficient in thrust = k_thrust * w^2
  static constexpr double arm_xy = 0.120913386; 
  static constexpr double arm_r = sqrt(2)*arm_xy;
  static constexpr double r_z = 0.126394928;    // prop site z relative to body CoM
  
  static constexpr double max_motor_thrust = 17;
  static constexpr double max_motor_speed = std::sqrt(max_motor_thrust / k_thrust);
  
  static constexpr double tauz_lpf_cutoff_rad_s = 1; // Yaw torque LPF cutoff angular frequency
  static constexpr double yaw_trim_limit = 2.0 * zeta * max_motor_thrust;// Maximum yaw trim supplied by motor reaction torque.
  
  static constexpr double servo_limit_degree = 60.0; //max=65.4, 0.5max = 32.7
  static constexpr double servo_limit_rad = servo_limit_degree * deg_to_rad; //1.0472;
 static inline const double servo_limit_sin = std::sin(servo_limit_rad);

  AllocatorController() : rclcpp::Node("hih_allocator_controller")
  {
    // Enables the verbose "allocator limit: ..." saturation diagnostics.
    // It is disabled by default and can be changed while the node is running.
    this->declare_parameter<bool>("enable_saturation_debug", false);

    const auto input_wrench_topic = this->declare_parameter<std::string>(
      "input_wrench_topic", "minitrone/wrench_cmd");
    sub_wrench_ = this->create_subscription<hih_interfaces::msg::Wrench>(
      input_wrench_topic, 10,
      std::bind(&AllocatorController::onWrench, this, std::placeholders::_1));
    sub_state_ = this->create_subscription<hih_interfaces::msg::MinitroneState>("minitrone/state", 10, std::bind(&AllocatorController::onState, this, std::placeholders::_1));

    pub_input_ = this->create_publisher<hih_interfaces::msg::Input>("minitrone/input", 10);
    pub_saturated_ = this->create_publisher<std_msgs::msg::Bool>(
      "minitrone/allocator_saturated", 10);

    dt_buffer_.assign(buffer_size, 0.01);
    dt_sum_ = 0.01 * static_cast<double>(buffer_size);

    Pc_ << 0.0, 0.0, r_z; //Pc_.setZero();
  }

private:
  void onState(const hih_interfaces::msg::MinitroneState::SharedPtr msg) 
  {
    C2_mea_(0) = static_cast<double>(msg->servo[0]) * deg_to_rad;
    C2_mea_(1) = static_cast<double>(msg->servo[1]) * deg_to_rad;
    C2_mea_(2) = static_cast<double>(msg->servo[2]) * deg_to_rad;
    C2_mea_(3) = static_cast<double>(msg->servo[3]) * deg_to_rad;
  }

  void onWrench(const hih_interfaces::msg::Wrench::SharedPtr msg)
  {
    rclcpp::Time current_callback_time = this->now();
    double dt = (last_callback_time_.nanoseconds() > 0) ? (current_callback_time - last_callback_time_).seconds() : 0.01;
    last_callback_time_ = current_callback_time;
    if (dt <= 0.0 || dt > 0.2) dt = 0.01;

    dt_sum_ = dt_sum_ - dt_buffer_[buffer_index_] + dt;
    dt_buffer_[buffer_index_] = dt;
    buffer_index_ = (buffer_index_ + 1) % buffer_size;
    double avg_dt = dt_sum_ / static_cast<double>(buffer_size);
    filtered_frequency_ = 1.0 / std::max(avg_dt, 1e-4);

    Eigen::Matrix<double,6,1> Wrench;
    Wrench << static_cast<double>(msg->moment[0]), static_cast<double>(msg->moment[1]), static_cast<double>(msg->moment[2]),
               static_cast<double>(msg->force[0]),  static_cast<double>(msg->force[1]),  static_cast<double>(msg->force[2]);
    // Desired yaw torque [Nm]
    double tauz_des = Wrench(2);

    // First-order LPF coefficient from cutoff angular frequency [rad/s]
    // alpha = 1 - exp(-wc * dt)
    const double alpha_tauz = 1.0 - std::exp(-tauz_lpf_cutoff_rad_s * dt);

    // Low-frequency yaw torque component
    // Allocate the low-frequency yaw torque through servo tilt.
    tauz_bar_ += alpha_tauz * (tauz_des - tauz_bar_);

    // High-frequency residual yaw torque
    // Allocate the high-frequency yaw torque through motor reaction trim.
    double tauz_r = tauz_des - tauz_bar_;

    // Saturate yaw trimming authority
    double tauz_r_sat = std::clamp(tauz_r, -yaw_trim_limit, yaw_trim_limit);

    // Remaining yaw torque goes to tilt-based allocation
    double tauz_t = tauz_bar_ + (tauz_r - tauz_r_sat);

    const Eigen::Vector4d B1(Wrench(0), Wrench(1), tauz_r_sat, Wrench(5));
    const Eigen::Vector4d B2(Wrench(3), Wrench(4), tauz_t, 0.0);

    // ----------------------------------------------------
    // Step 1: compute preliminary thrust at the measured servo angles.
    // ----------------------------------------------------
    Eigen::Matrix4d A1_mea = calc_A1(C2_mea_);
    Eigen::Vector4d C1_raw = solve4x4(A1_mea, B1);
    // Use physically attainable thrust when constructing A2.
    Eigen::Vector4d C1_seed =
      C1_raw
        .cwiseMax(0.0)
        .cwiseMin(max_motor_thrust);

    // ----------------------------------------------------
    // Step 2: allocate servo tilt for lateral force and tilt-induced yaw.
    // ----------------------------------------------------
    Eigen::Matrix4d A2 = calc_A2(C1_seed, C2_mea_);

    Eigen::Vector4d S_des = solve4x4(A2, B2);

    C2_des_ = sinToServoAngle(S_des);

    Eigen::Vector4d S_cmd =
      S_des
        .cwiseMax(-servo_limit_sin)
        .cwiseMin(servo_limit_sin);

    Eigen::Vector4d C2_cmd =
      sinToServoAngle(S_cmd)
        .cwiseMax(-servo_limit_rad)
        .cwiseMin(servo_limit_rad);

    // ----------------------------------------------------
    // Step 3: recompute final motor thrust at the selected servo angles.
    // ----------------------------------------------------
    Eigen::Matrix4d A1_cmd = calc_A1(C2_cmd);

    Eigen::Vector4d C1_final_raw = solve4x4(A1_cmd, B1);

    C1_ = C1_final_raw
        .cwiseMax(0.0)
        .cwiseMin(max_motor_thrust);
    std_msgs::msg::Bool saturated;
    saturated.data = (C1_ - C1_final_raw).cwiseAbs().maxCoeff() > 1e-6 ||
      (C2_des_ - C2_cmd).cwiseAbs().maxCoeff() > 1e-6 ||
      std::abs(tauz_r - tauz_r_sat) > 1e-6;
    pub_saturated_->publish(saturated);

    hih_interfaces::msg::Input out;

    double motor_speed[4];
    motor_speed[0] = std::sqrt(std::max(0.0, C1_(0)/k_thrust));
    motor_speed[1] = std::sqrt(std::max(0.0, C1_(1)/k_thrust));
    motor_speed[2] = std::sqrt(std::max(0.0, C1_(2)/k_thrust));
    motor_speed[3] = std::sqrt(std::max(0.0, C1_(3)/k_thrust));

    out.u[0] = std::clamp(motor_speed[0], 0.0, max_motor_speed);
    out.u[1] = std::clamp(motor_speed[1], 0.0, max_motor_speed);
    out.u[2] = std::clamp(motor_speed[2], 0.0, max_motor_speed);
    out.u[3] = std::clamp(motor_speed[3], 0.0, max_motor_speed);
    out.u[4] = C2_cmd(0); out.u[5] = C2_cmd(1); out.u[6] = C2_cmd(2); out.u[7] = C2_cmd(3);
    pub_input_->publish(out);

    if (this->get_parameter("enable_saturation_debug").as_bool()) {
      maybeLogAllocatorLimit(
        B1, B2, C1_raw, S_des, C2_des_, C2_cmd, C1_,
        conditionNumber(A1_mea), conditionNumber(A2), conditionNumber(A1_cmd));
    }
  }

  Eigen::Vector4d solve4x4(const Eigen::Matrix4d& A, const Eigen::Vector4d& b) const
  {
    Eigen::FullPivLU<Eigen::Matrix4d> lu(A);
    if (lu.isInvertible()) {
      return lu.solve(b);
    }
    return (A.transpose()*A + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A.transpose()*b);
  }

  Eigen::Vector4d sinToServoAngle(const Eigen::Vector4d& sin_value) const
  {
    Eigen::Vector4d angle;
    for (int i = 0; i < 4; ++i) {
      angle(i) = std::asin(std::clamp(sin_value(i), -1.0, 1.0));
    }
    return angle;
  }

  double conditionNumber(const Eigen::Matrix4d& A) const
  {
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A);
    const auto& singular_values = svd.singularValues();
    const double sigma_max = singular_values.maxCoeff();
    const double sigma_min = singular_values.minCoeff();
    if (sigma_min <= 1e-12) {
      return std::numeric_limits<double>::infinity();
    }
    return sigma_max / sigma_min;
  }

  void maybeLogAllocatorLimit(
    const Eigen::Vector4d& b1,
    const Eigen::Vector4d& b2,
    const Eigen::Vector4d& c1_raw,
    const Eigen::Vector4d& s_des,
    const Eigen::Vector4d& c2_des,
    const Eigen::Vector4d& c2_cmd,
    const Eigen::Vector4d& c1_cmd,
    double cond_a1_mea,
    double cond_a2,
    double cond_a1_cmd)
  {
    const bool servo_limited = (c2_des - c2_cmd).cwiseAbs().maxCoeff() > 1e-6;
    const bool thrust_limited =
      ((c1_cmd.array() < 1e-6) || (c1_cmd.array() > max_motor_thrust - 1e-6)).any();
    const bool raw_thrust_invalid =
      ((c1_raw.array() < 0.0) || (c1_raw.array() > max_motor_thrust)).any();
    const bool sin_solution_limited =
      ((s_des.array() < -servo_limit_sin) || (s_des.array() > servo_limit_sin)).any();

    if (!(servo_limited || thrust_limited || raw_thrust_invalid || sin_solution_limited)) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (last_limit_log_time_.nanoseconds() > 0 &&
        (now - last_limit_log_time_).seconds() < 0.2) {
      return;
    }
    last_limit_log_time_ = now;

    const auto to_deg = [](const Eigen::Vector4d& v) {
      return (v.array() * (180.0 / M_PI)).matrix();
    };

    RCLCPP_WARN(
      this->get_logger(),
      "allocator limit: B1=[%.2f %.2f %.2f %.2f] B2=[%.2f %.2f %.2f %.2f] "
      "cond=[A1_mea %.2e A2 %.2e A1_cmd %.2e] "
      "thrust_raw=[%.2f %.2f %.2f %.2f] s_des=[%.3f %.3f %.3f %.3f] "
      "servo_des_deg=[%.2f %.2f %.2f %.2f] servo_cmd_deg=[%.2f %.2f %.2f %.2f] "
      "thrust_cmd=[%.2f %.2f %.2f %.2f]",
      b1(0), b1(1), b1(2), b1(3),
      b2(0), b2(1), b2(2), b2(3),
      cond_a1_mea, cond_a2, cond_a1_cmd,
      c1_raw(0), c1_raw(1), c1_raw(2), c1_raw(3),
      s_des(0), s_des(1), s_des(2), s_des(3),
      to_deg(c2_des)(0), to_deg(c2_des)(1), to_deg(c2_des)(2), to_deg(c2_des)(3),
      to_deg(c2_cmd)(0), to_deg(c2_cmd)(1), to_deg(c2_cmd)(2), to_deg(c2_cmd)(3),
      c1_cmd(0), c1_cmd(1), c1_cmd(2), c1_cmd(3));
  }

  Eigen::Matrix4d calc_A1(const Eigen::Vector4d& C2) 
  {
    Eigen::Matrix4d A1;

    double s1 = std::sin(C2(0)); double s2 = std::sin(C2(1));
    double s3 = std::sin(C2(2)); double s4 = std::sin(C2(3));
    double c1 = std::cos(C2(0)); double c2 = std::cos(C2(1));
    double c3 = std::cos(C2(2)); double c4 = std::cos(C2(3));

    double pcx = Pc_(0), pcy = Pc_(1), pcz = Pc_(2);

    A1(0,0) = inv_sqrt2 * ( zeta +  r_z - pcz) * s1 + ( +arm_r/sqrt(2) - pcy) * c1; //
    A1(0,1) = inv_sqrt2 * (-zeta -  r_z + pcz) * s2 + ( +arm_r/sqrt(2) - pcy) * c2;
    A1(0,2) = inv_sqrt2 * (-zeta -  r_z + pcz) * s3 + ( -arm_r/sqrt(2) - pcy) * c3;
    A1(0,3) = inv_sqrt2 * ( zeta +  r_z - pcz) * s4 + ( -arm_r/sqrt(2) - pcy) * c4;

    A1(1,0) = inv_sqrt2 * (-zeta +  r_z - pcz) * s1 + ( -arm_r/sqrt(2) + pcx) * c1;
    A1(1,1) = inv_sqrt2 * (-zeta +  r_z - pcz) * s2 + ( +arm_r/sqrt(2) + pcx) * c2;
    A1(1,2) = inv_sqrt2 * ( zeta -  r_z + pcz) * s3 + ( +arm_r/sqrt(2) + pcx) * c3;
    A1(1,3) = inv_sqrt2 * ( zeta -  r_z + pcz) * s4 + ( -arm_r/sqrt(2) + pcx) * c4;

    A1(2,0) =  zeta * c1;
    A1(2,1) = -zeta * c2;
    A1(2,2) =  zeta * c3;
    A1(2,3) = -zeta * c4;

    A1(3,0) = c1;
    A1(3,1) = c2;
    A1(3,2) = c3;
    A1(3,3) = c4;

    return A1;
  }

  Eigen::Matrix4d calc_A2(const Eigen::Vector4d& C1, const Eigen::Vector4d& C2)
  {
    Eigen::Matrix4d A2;

    double pcx = Pc_(0), pcy = Pc_(1);
    double s1 = std::sin(C2(0)); double s2 = std::sin(C2(1));
    double s3 = std::sin(C2(2)); double s4 = std::sin(C2(3));

    double f1 = C1(0), f2 = C1(1), f3 = C1(2), f4 = C1(3);

    A2(0,0) =  inv_sqrt2 * f1;
    A2(0,1) =  inv_sqrt2 * f2;
    A2(0,2) = -inv_sqrt2 * f3;
    A2(0,3) = -inv_sqrt2 * f4;

    A2(1,0) = -inv_sqrt2 * f1;
    A2(1,1) =  inv_sqrt2 * f2;
    A2(1,2) =  inv_sqrt2 * f3;
    A2(1,3) = -inv_sqrt2 * f4;

    A2(2,0) = inv_sqrt2 * ( +pcx + pcy) * s1 + (-(+arm_r) ) * f1;
    A2(2,1) = inv_sqrt2 * ( -pcx + pcy) * s2 + (-(+arm_r) ) * f2;
    A2(2,2) = inv_sqrt2 * ( -pcx - pcy) * s3 + ((-arm_r) ) * f3;
    A2(2,3) = inv_sqrt2 * ( +pcx - pcy) * s4 + ((-arm_r)) * f4;

    A2(3,0) = +1 * ( -(+arm_r) - (+arm_r) ) * f1;
    A2(3,1) = -1* ( +(-arm_r) - (+arm_r) ) * f2;
    A2(3,2) = +1 * ( +(-arm_r) + (-arm_r) ) * f3;
    A2(3,3) = -1 * ( -(+arm_r) + (-arm_r) ) * f4;

    return A2;
  }

  rclcpp::Subscription<hih_interfaces::msg::Wrench>::SharedPtr          sub_wrench_;
  rclcpp::Subscription<hih_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Publisher<hih_interfaces::msg::Input>::SharedPtr              pub_input_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                           pub_saturated_;

  rclcpp::Time last_callback_time_;
  std::vector<double> dt_buffer_;
  size_t buffer_index_{0};
  double dt_sum_{0.0};
  double filtered_frequency_{0.0};
  rclcpp::Time last_limit_log_time_;

  double tauz_bar_{0.0};
  Eigen::Vector4d C1_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d C2_mea_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d C2_des_{Eigen::Vector4d::Zero()};
  Eigen::Vector3d Pc_{Eigen::Vector3d::Zero()};
};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllocatorController>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <cmath>
#include <algorithm>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cstdlib>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#define PI M_PI

class ControllerNode : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_subscription_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr propeller_speeds_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

  double kx, kv, kr, komega, T_MAX_RATIO;
  double m, g;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;

  Eigen::Vector3d x, v, omega;
  Eigen::Matrix3d R;

  Eigen::Vector3d xd, vd, ad;
  double yawd;
  int64_t hz;

  bool yaw_initialized_    = false;
  bool target_initialized_ = false;
  double xd_z_ramp_        = 0.0;
  static constexpr double CLIMB_RATE = 0.1; // m/s

  static Eigen::Vector3d Vee(const Eigen::Matrix3d &in) {
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

public:
  ControllerNode()
  : Node("controller_node"),
    xd(0, 0, 2.0), vd(0, 0, 0), ad(0, 0, 0),
    yawd(0.0), e3(0, 0, 1), hz(100)
  {
    x.setZero(); v.setZero(); omega.setZero(); R.setIdentity();

    // Declare parameters with safe defaults so the node doesn't crash
    // if gains are not passed — override at launch with -p kx:=4.0 etc.
    declare_parameter<double>("kx",     1.5);
    declare_parameter<double>("kv",     2.0);
    declare_parameter<double>("kr",     1.5);
    declare_parameter<double>("komega", 0.5);
    declare_parameter<double>("T_MAX_RATIO", 3.06);

    get_parameter("kx",     kx);
    get_parameter("kv",     kv);
    get_parameter("kr",     kr);
    get_parameter("komega", komega);
    get_parameter("T_MAX_RATIO", T_MAX_RATIO);

    // ---------------------------------------------------------------
    // Subscriber QoS: match MAVROS odometry publisher (BestEffort)
    // ---------------------------------------------------------------
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                     .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                     .durability(rclcpp::DurabilityPolicy::Volatile);

    current_state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", sub_qos,
      std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));

    // ---------------------------------------------------------------
    // FIX: publish to /mavros/setpoint_raw/attitude, NOT
    //      /mavros/setpoint_attitude/attitude.
    //
    //  - setpoint_attitude/attitude expects geometry_msgs/PoseStamped
    //    and splits thrust onto a separate topic — it does NOT accept
    //    AttitudeTarget and ignores type_mask entirely.
    //
    //  - setpoint_raw/attitude accepts mavros_msgs/AttitudeTarget
    //    with unified attitude + thrust + type_mask in one message,
    //    which maps directly to MAVLink SET_ATTITUDE_TARGET.
    //    This is the correct injection point for SE(3).
    //
    // Publisher QoS: use Reliable so MAVROS does not silently drop
    // messages when its internal queue is briefly busy.
    // ---------------------------------------------------------------
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                     .reliability(rclcpp::ReliabilityPolicy::Reliable)
                     .durability(rclcpp::DurabilityPolicy::Volatile);

    propeller_speeds_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", pub_qos);

    debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/se3/debug",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / hz),
      std::bind(&ControllerNode::controlLoop, this));

    m = 1.6;
    g = 9.81;
    J << 0.0347563, 0.0,       0.0,
         0.0,       0.0458929, 0.0,
         0.0,       0.0,       0.0796;

    RCLCPP_INFO(this->get_logger(),
      "SE3 Controller started | kx=%.2f kv=%.2f kr=%.2f komega=%.2f T_MAX_RATIO=%.2f",
      kx, kv, kr, komega, T_MAX_RATIO);
    RCLCPP_INFO(this->get_logger(),
      "Publishing to /mavros/setpoint_raw/attitude — waiting for first odometry...");
  }

  // -------------------------------------------------------------------
  void onCurrentState(const nav_msgs::msg::Odometry &cur_state) {
    x << cur_state.pose.pose.position.x,
         cur_state.pose.pose.position.y,
         cur_state.pose.pose.position.z;
    
    Eigen::Vector3d v_body;
    v_body << cur_state.twist.twist.linear.x,
         cur_state.twist.twist.linear.y,
         cur_state.twist.twist.linear.z;

    v = R * v_body;

    Eigen::Quaterniond q;
    tf2::fromMsg(cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();

    omega << cur_state.twist.twist.angular.x,
             cur_state.twist.twist.angular.y,
             cur_state.twist.twist.angular.z;

    if (!yaw_initialized_) {
      tf2::Quaternion q_tf;
      tf2::fromMsg(cur_state.pose.pose.orientation, q_tf);
      yawd = tf2::getYaw(q_tf);
      yaw_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Yaw initialized to: %.3f rad", yawd);
    }
  }

  // -------------------------------------------------------------------
  void controlLoop() {
    // Block until at least one odometry message has been received
    if (!yaw_initialized_) return;

    // Latch XY target at current position on first run; start Z ramp
    if (!target_initialized_) {
      xd_z_ramp_ = std::max(x.z(), 0.0);
      xd.x()     = x.x();
      xd.y()     = x.y();
      target_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Target initialized at [%.2f, %.2f, %.2f]",
        xd.x(), xd.y(), xd_z_ramp_);
    }

    // Ramp altitude up to 2 m at CLIMB_RATE m/s
    if (xd_z_ramp_ < 2.0) {
      xd_z_ramp_ = std::min(
        xd_z_ramp_ + CLIMB_RATE / static_cast<double>(hz), 2.0);
    }

    Eigen::Vector3d xd_now(xd.x(), xd.y(), xd_z_ramp_);
    Eigen::Vector3d ex = x - xd_now;
    Eigen::Vector3d ev = v - vd;

    // NEW: scale z error
    Eigen::Vector3d ex_scaled = ex;
    ex_scaled.z() *= 0.5;

    // ------------------------------------------------------------------
    // SE(3) Step 1 — Desired force vector
    // Fdes = -kx*ex - kv*ev + m*ad + m*g*e3
    // This replaces AC_PosControl's cascaded position+velocity PIDs.
    // ------------------------------------------------------------------
    // Eigen::Vector3d F_corr = -kx * ex - kv * ev + m * ad;
    Eigen::Vector3d F_corr = -kx * ex_scaled - kv * ev + m * ad;

    // double max_corr = 0.3 * m * g;
    // if (F_corr.norm() > max_corr)
    //   F_corr = F_corr * (max_corr / F_corr.norm());
    double max_corr = 0.8 * m * g;
    if (F_corr.norm() > max_corr)
      F_corr = F_corr * (max_corr / F_corr.norm());

    Eigen::Vector3d F_des = F_corr + m * g * e3;

    double F_norm = F_des.norm();
    if (F_norm < 1e-6) return;

    // ------------------------------------------------------------------
    // SE(3) Step 2 — Desired orientation Rd on SO(3)
    // b3d is the desired body-z (thrust) direction in world frame.
    // ------------------------------------------------------------------
    Eigen::Vector3d b3d = F_des / F_norm;
    Eigen::Vector3d b1d(std::cos(yawd), std::sin(yawd), 0.0);
    Eigen::Vector3d b2d = b3d.cross(b1d).normalized();
    b1d = b2d.cross(b3d);

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;

    Eigen::Vector3d eR_vec = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);

    // Angular rate error (omega_d = 0 for hover/constant yaw)
    Eigen::Vector3d eOmega = omega;  // since omega_d = 0

    // SE3 moment command in body frame
    Eigen::Vector3d tau = -kr * eR_vec
                          - komega * eOmega
                          + omega.cross(J * omega);  // gyroscopic feedforward

    // Convert moment to desired angular acceleration, then to rate setpoint
    // For AttitudeTarget, we send the desired body rate (feedforward term)
    // Rd^T * omega_d_world — for hover this simplifies to:
    Eigen::Vector3d omega_des = Rd.transpose() * (Eigen::Vector3d::Zero()); // omega_d = 0


    // Collective thrust: projection of desired force onto current body-z.
    // Clamped to [0.1, 0.85] in normalised [0,1] throttle range.
    double f         = std::max(F_des.dot(R * e3), 0.0);
    double thrust_n = std::clamp(f / (T_MAX_RATIO * m * g), 0.0, 0.75); 
    // double thrust_n = std::clamp(f / (2.0 * m * g), 0.0, 0.75);
    // double thrust_n = std::clamp(f / (2.0 * m * g), 0.0, 0.75);

    // ------------------------------------------------------------------
    // Build AttitudeTarget message
    //
    // type_mask bit field (mavros_msgs/AttitudeTarget):
    //   bit 0 = ignore body roll rate
    //   bit 1 = ignore body pitch rate
    //   bit 2 = ignore body yaw rate
    //   bit 7 = ignore attitude (only rates + thrust)
    //
    // type_mask = 0b00000111 = 7  →  ignore body rates, use attitude+thrust
    // This tells AC_AttitudeControl to track Rd directly.
    // ------------------------------------------------------------------
    mavros_msgs::msg::AttitudeTarget msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.type_mask       = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE  |
                      mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                      mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

    Eigen::Quaterniond quat_des(Rd);
    quat_des.normalize();
    msg.orientation.x = quat_des.x();
    msg.orientation.y = quat_des.y();
    msg.orientation.z = quat_des.z();
    msg.orientation.w = quat_des.w();

    msg.body_rate.x = 0.0;
    msg.body_rate.y = 0.0;
    msg.body_rate.z = 0.0;

    // msg.body_rate.x = omega_des.x() - kr * eR_vec.x();
    // msg.body_rate.y = omega_des.y() - kr * eR_vec.y();
    // msg.body_rate.z = omega_des.z() - kr * eR_vec.z();

    msg.thrust = thrust_n;

    propeller_speeds_publisher_->publish(msg);

    

    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {
      ex.x(),      ex.y(),      ex.z(),       // [0,1,2]  position error
      ev.x(),      ev.y(),      ev.z(),       // [3,4,5]  velocity error
      F_des.x(),   F_des.y(),   F_des.z(),   // [6,7,8]  desired force
      F_norm,                                 // [9]
      f,                                      // [10]     raw thrust (Newtons)
      msg.thrust,                             // [11]     normalized thrust sent
      eR_vec.x(),  eR_vec.y(),  eR_vec.z(), // [12,13,14] attitude error
      omega.x(),   omega.y(),   omega.z(),  // [15,16,17] angular rates
      xd_z_ramp_,                            // [18]     altitude target
      b3d.x(),     b3d.y(),     b3d.z(),    // [19,20,21] desired thrust direction
      x.z(),                                 // [22]     actual z (for easy comparison)
    };
    debug_pub_->publish(dbg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
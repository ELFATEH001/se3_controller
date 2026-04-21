#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <cmath>
#include <algorithm>
// #include <mav_msgs/msg/actuators.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  16.485 - Fall 2024  - Lab 3 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a C++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf2 provides useful functions to convert tf2
// messages to eigen types and vice versa.
#include <tf2_eigen/tf2_eigen.hpp>

// FOR exit(1) FOR DEBUGGING
#include <cstdlib>

class ControllerNode_b : public rclcpp::Node {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop
  //
  // ~~~~ begin solution
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_subscription_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr propeller_speeds_publisher_;

  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;            // mass of the UAV
  double g;            // gravity acceleration
  // double d;            // distance from the center of propellers to the c.o.m.
  // double cf,           // Propeller lift coefficient
  //     cd;              // Propeller drag coefficient
  Eigen::Matrix3d J;   // Inertia Matrix
  Eigen::Vector3d e3;  // [0,0,1]
  // Eigen::Matrix4d F2W; // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x; // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v; // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R; // current orientation of the UAV
  Eigen::Vector3d
      omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd; // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd; // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;      // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd; // desired yaw angle

  int64_t hz; // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d &in) {
    Eigen::Vector3d out;
    out << in(2, 1), in(0, 2), in(1, 0);
    return out;
  }

  bool yaw_initialized_ = false;

public:
  ControllerNode_b() : Node("controller_node"), xd(0,0 ,2.0), vd(0,0 ,0), ad(0,0 ,0), yawd(0.0), e3(0, 0, 1), hz(100) {
    // declare ROS parameters
    declare_parameter<double>("kx");
    declare_parameter<double>("kv");
    declare_parameter<double>("kr");
    declare_parameter<double>("komega");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 2 |  Initialize ROS callback handlers
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to initialize your handlers from part 1.
    // Specifically:
    //  - bind ControllerNode_b::onDesiredState() to the topic "desired_state"
    //  - bind ControllerNode_b::onCurrentState() to the topic "current_state"
    //  - bind ControllerNode_b::controlLoop() to the created timer, at frequency
    //    given by the "hz" variable
    //
    // Hints:
    //  - make sure you start your timer with reset()
    //
    // ~~~~ begin solution
    // desired_state_subscription_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
    //   "desired_state", 10, std::bind(&ControllerNode_b::onDesiredState , this, std::placeholders::_1));
    // current_state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/mavros/local_position/odom", 10, std::bind(&ControllerNode_b::onCurrentState , this, std::placeholders::_1));
    auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10))
                    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                    .durability(rclcpp::DurabilityPolicy::Volatile);

    propeller_speeds_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "/mavros/setpoint_attitude/attitude", qos_pub);
    //
    propeller_speeds_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_attitude/attitude", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000/hz), std::bind(&ControllerNode_b::controlLoop, this));
    
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (!yaw_initialized_) {
      tf2::Quaternion q;
      tf2::fromMsg(cur_state.pose.pose.orientation, q);
      yawd = tf2::getYaw(q);
      yaw_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Yaw initialized to: %.3f rad", yawd);
    }
    if (!(get_parameter("kx", kx) && get_parameter("kv", kv) &&
          get_parameter("kr", kr) && get_parameter("komega", komega))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get controller gains from parameter server");
      exit(1);
    }

    // Initialize constants
    m = 1.6;
    g = 9.81;
    J << 0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0796;
    // double a = cf * d / sqrt(2);
    // F2W << cf, cf, cf, cf, a, a, -a, -a, -a, a, a, -a, cd, -cd, cd, -cd;
  }

  // void onDesiredState(
  //     const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &des_state) {

  //   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //   //  PART 3 | Objective: fill in xd, vd, ad, yawd
  //   // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //   //
  //   // 3.1 Get the desired position, velocity and acceleration from the in-
  //   //     coming ROS message and fill in the class member variables xd, vd
  //   //     and ad accordingly. You can ignore the angular acceleration.
  //   //
  //   // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.
  //   //
  //   xd << des_state.transforms[0].translation.x,
  //       des_state.transforms[0].translation.y,
  //       des_state.transforms[0].translation.z;
    
  //   vd << des_state.velocities[0].linear.x,
  //       des_state.velocities[0].linear.y,
  //       des_state.velocities[0].linear.z;
        
  //   ad << des_state.accelerations[0].linear.x,
  //       des_state.accelerations[0].linear.y,
  //       des_state.accelerations[0].linear.z;
    
  //   // this is here to surpress an "unused variable compiler warning"
  //   // you can remove it when you start writing your answer
  //   // des_state == des_state;

  //   //
  //   // 3.2 Extract the yaw component from the quaternion in the incoming ROS
  //   //     message and store in the yawd class member variable
  //   //
  //   //  Hints:
  //   //    - look into the functions tf2::getYaw(...) and tf2::fromMsg
  //   //
  //   tf2::Quaternion quati_des;
  //   tf2::fromMsg(des_state.transforms[0].rotation, quati_des);
  //   yawd = tf2::getYaw(quati_des);
  //   //
  //   // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //   //                                 end part 3
  //   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // }

  void onCurrentState(const nav_msgs::msg::Odometry &cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 4 | Objective: fill in x, v, R and omega
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // Get the current position and velocity from the incoming ROS message and
    // fill in the class member variables x, v, R and omega accordingly.
    //
    //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
    //          needs to be in the body frame!
    //
    x << cur_state.pose.pose.position.x,
        cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,
        cur_state.twist.twist.linear.y,
        cur_state.twist.twist.linear.z;
    
    Eigen::Quaterniond quati_cur;
    tf2::fromMsg(cur_state.pose.pose.orientation, quati_cur);
    R = (quati_cur).toRotationMatrix();
    Eigen::Vector3d omega_world;
    omega_world << cur_state.twist.twist.angular.x,
        cur_state.twist.twist.angular.y,
        cur_state.twist.twist.angular.z;
    omega = R.transpose() * omega_world;

    
    // this is here to surpress an "unused variable compiler warning"
    // you can remove it when you start writing your answer
    // cur_state == cur_state;

    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 4
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void controlLoop() {
    if (!yaw_initialized_) return;
    Eigen::Vector3d ex, ev, er, eomega;
    Eigen::Matrix3d Rd;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 5 | Objective: Implement the controller!
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    //  Hint: [1], eq. (6), (7)
    //
    ex = x - xd;
    ev = v - vd;
    // 5.2 Compute the Rd matrix.
    //
    //  Hint: break it down in 3 parts:
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //
    //  CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - remember to normalize your axes!
    //
    // Build b3d vector
    Eigen::Vector3d b1d, b2d, b3d;

    Eigen::Vector3d F_des = -kx * ex - kv * ev + m * g * e3 + m * ad;
    b3d = F_des / F_des.norm();
    b1d << cos(yawd), sin(yawd), 0;
    b2d = b3d.cross(b1d);
    b2d.normalize();
    b1d = b2d.cross(b3d);
    
    

    Rd << b1d, b2d, b3d;

    //
    // 5.3 Compute the orientation error (er) and the rotation-rate error
    // (eomega)
    //  Hints:
    //     - [1] eq. (10) and (11)
    //     - you can use the Vee() static method implemented above
    //  
    //  CAVEAT: feel free to ignore the second addend in eq (11), since it
    //          requires numerical differentiation of Rd and it has negligible
    //          effects on the closed-loop dynamics.
    //
    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
    eomega = omega ; //- R.transpose() * Rd * Omega_d 
    //
    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    //  Hints:
    //     - [1] eq. (15), (16)
    // CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term
    //        ii) the overall sign (in front of the bracket) in equation (15)
    //            of the paper
    //
    //    - feel free to ignore all the terms involving \Omega_d and its time
    //      derivative as they are of the second order and have negligible
    //      effects on the closed-loop dynamics.
    //
    double f;
    Eigen::Vector3d tau;

    // Eigen::Vector3d F_des = -kx * ex - kv * ev - m * g * e3 + m * ad;
    f = F_des.dot(R * e3);  // Project desired force onto body z-axis
    tau = -kr * er - komega * eomega + omega.cross(J * omega);

    // f = ((- kx * ex - kv * ev + m * g * e3 + m * ad).transpose() * R).dot(e3);
    // tau = -kr * er - komega * eomega + omega.cross(J * omega);

    // 5.5 Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)
    //
    // CAVEATs:
    //     - we have different conventions for the arodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab 3 handout with Fig. 1 in
    //     the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.9 in the
    //       lecture notes!
    //
    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds -
    //       although uncommon - should be a possible outcome of the controller
    //       when appropriate. Note that this is the case in unity but not in
    //       real life, where propellers are aerodynamically optimized to spin
    //       in one direction!
    //
    // double angle45 = PI / 4.0;
    // Eigen::Vector4d rotor_speeds_squared;
    
    // Eigen::Vector4d rotor_speeds;
    // Eigen::Vector4d wrench;
    mavros_msgs::msg::AttitudeTarget prop_speeds_msg;
    Eigen::Quaterniond quat_des(Rd);
    
    prop_speeds_msg.header.stamp = this->get_clock()->now();
    prop_speeds_msg.type_mask = 0b00000111; 
    prop_speeds_msg.orientation.x = quat_des.x();
    prop_speeds_msg.orientation.y = quat_des.y();
    prop_speeds_msg.orientation.z = quat_des.z();
    prop_speeds_msg.orientation.w = quat_des.w(); 

    // More robust: normalize so hover = 0.5 throttle
    const double max_thrust = 2.0 * m * g;  // hover at 0.5 throttle
    prop_speeds_msg.thrust = std::clamp(f / max_thrust, 0.0, 1.0);

    // prop_speeds_msg.thrust = std::clamp(f / (2.0 * m * g), 0.0, 1.0); 

    //
    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).
    //

    // publish the message
    propeller_speeds_publisher_->publish(prop_speeds_msg);
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //           end part 5, congrats! Start tuning your gains (part 6)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv); // Initialize the ROS 2 system
  rclcpp::spin(std::make_shared<ControllerNode_b>()); // Spin the node so it
                                                    // processes callbacks
  rclcpp::shutdown(); // Shutdown the ROS 2 system when done
  return 0;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 6 [NOTE: save this for last] |  Tune your gains!
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// Live the life of a control engineer! Tune these parameters for a fast
// and accurate controller.
//
// Modify the gains kx, kv, kr, komega in controller_pkg/config/params.yaml
// and re-run the controller.
//
// Can you get the drone to do stable flight?
//
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  You made it! Congratulations! You are now a control engineer!
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"

#include "arena_swerve_controller/swerve_controller.hpp"

namespace
{
constexpr double kWheelRadius = 0.1125;
constexpr double kTolerance = 1e-4;
constexpr std::size_t kWheelCount = 4;

const std::vector<std::string> kWheelJoints = {"wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl"};
const std::vector<std::string> kSteerJoints = {"steer_fr", "steer_fl", "steer_rr", "steer_rl"};

class SwerveControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    wheel_cmd_vals_.assign(kWheelCount, 0.0);
    steer_cmd_vals_.assign(kWheelCount, 0.0);
    wheel_state_vals_.assign(kWheelCount, 0.0);
    steer_state_vals_.assign(kWheelCount, 0.0);

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      wheel_cmd_handles_.emplace_back(
        kWheelJoints[i], hardware_interface::HW_IF_VELOCITY, &wheel_cmd_vals_[i]);
      steer_cmd_handles_.emplace_back(
        kSteerJoints[i], hardware_interface::HW_IF_POSITION, &steer_cmd_vals_[i]);
      wheel_state_handles_.emplace_back(
        kWheelJoints[i], hardware_interface::HW_IF_VELOCITY, &wheel_state_vals_[i]);
      steer_state_handles_.emplace_back(
        kSteerJoints[i], hardware_interface::HW_IF_POSITION, &steer_state_vals_[i]);
    }
  }

  controller_interface::return_type init_controller(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    controller_ = std::make_shared<arena_swerve_controller::SwerveController>();

    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("wheel_joint_names", kWheelJoints),
      rclcpp::Parameter("steering_joint_names", kSteerJoints),
      rclcpp::Parameter("wheel_radius", kWheelRadius),
      rclcpp::Parameter(
        "wheel_positions_x", std::vector<double>{0.38, 0.38, -0.38, -0.38}),
      rclcpp::Parameter(
        "wheel_positions_y", std::vector<double>{-0.23725, 0.23725, -0.23725, 0.23725}),
      rclcpp::Parameter("max_steering_angle", 2.8),
      rclcpp::Parameter("allow_reverse_drive", true),
      rclcpp::Parameter("cmd_vel_timeout", 0.5),
      rclcpp::Parameter("use_stamped_vel", false),
      rclcpp::Parameter("publish_odom", false),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("odom_frame_id", std::string("odom")),
      rclcpp::Parameter("base_frame_id", std::string("base_link")),
      rclcpp::Parameter(
        "pose_covariance_diagonal",
        std::vector<double>{0.001, 0.001, 1.0e+9, 1.0e+9, 1.0e+9, 0.01}),
      rclcpp::Parameter(
        "twist_covariance_diagonal",
        std::vector<double>{0.001, 0.001, 1.0e+9, 1.0e+9, 1.0e+9, 0.01}),
    };
    for (const auto & p : overrides) {
      auto it = std::find_if(
        params.begin(), params.end(),
        [&](const auto & x) {return x.get_name() == p.get_name();});
      if (it != params.end()) {*it = p;} else {params.push_back(p);}
    }

    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);

    // Jazzy signature: (name, urdf, cm_update_rate, namespace, options).
    return controller_->init("test_swerve_controller", "", 100, "", opts);
  }

  void spin_for(rclcpp::Node::SharedPtr extra, std::chrono::milliseconds budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
      if (extra) {rclcpp::spin_some(extra);}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void configure_and_activate()
  {
    auto node = controller_->get_node();
    ASSERT_EQ(
      node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::vector<hardware_interface::LoanedCommandInterface> cmd_ifaces;
    for (auto & h : wheel_cmd_handles_) {cmd_ifaces.emplace_back(h);}
    for (auto & h : steer_cmd_handles_) {cmd_ifaces.emplace_back(h);}
    std::vector<hardware_interface::LoanedStateInterface> state_ifaces;
    for (auto & h : wheel_state_handles_) {state_ifaces.emplace_back(h);}
    for (auto & h : steer_state_handles_) {state_ifaces.emplace_back(h);}
    controller_->assign_interfaces(std::move(cmd_ifaces), std::move(state_ifaces));

    ASSERT_EQ(
      node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  std::shared_ptr<arena_swerve_controller::SwerveController> controller_;
  std::vector<double> wheel_cmd_vals_;
  std::vector<double> steer_cmd_vals_;
  std::vector<double> wheel_state_vals_;
  std::vector<double> steer_state_vals_;
  std::vector<hardware_interface::CommandInterface> wheel_cmd_handles_;
  std::vector<hardware_interface::CommandInterface> steer_cmd_handles_;
  std::vector<hardware_interface::StateInterface> wheel_state_handles_;
  std::vector<hardware_interface::StateInterface> steer_state_handles_;
};

// Forward-only Twist (vx=0.5): all wheels spin at vx/R, all steering targets ~0.
TEST_F(SwerveControllerTest, ForwardOnlyUpdateSetsCorrectCommands)
{
  ASSERT_EQ(init_controller(), controller_interface::return_type::OK);
  configure_and_activate();

  auto pub_node = rclcpp::Node::make_shared("test_publisher");
  auto cmd_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>(
    "/test_swerve_controller/cmd_vel", rclcpp::QoS(10).reliable());

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  cmd_pub->publish(cmd);

  spin_for(nullptr, std::chrono::milliseconds(200));

  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);

  const double expected_omega = 0.5 / kWheelRadius;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    EXPECT_NEAR(wheel_cmd_vals_[i], expected_omega, kTolerance)
      << "wheel[" << i << "]";
    EXPECT_NEAR(steer_cmd_vals_[i], 0.0, kTolerance)
      << "steer[" << i << "]";
  }
}

// No cmd ever published: the zero stamp from on_activate ages past cmd_vel_timeout,
// so update emits a zero twist and the wheels stay at zero.
TEST_F(SwerveControllerTest, ZeroAfterTimeoutLeavesWheelsAtZero)
{
  ASSERT_EQ(init_controller(), controller_interface::return_type::OK);
  configure_and_activate();

  const rclcpp::Time start = controller_->get_node()->now();
  const rclcpp::Time late = start + rclcpp::Duration::from_seconds(10.0);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(late, dt), controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    EXPECT_NEAR(wheel_cmd_vals_[i], 0.0, kTolerance);
  }
}

// Backward Twist with current steering at 0: target heading is pi, which is more than pi/2
// away from current, so the controller flips wheel direction and rotates the steer target
// back to 0 instead of physically turning the wheels around.
TEST_F(SwerveControllerTest, ReverseDriveFlipsWheelDirection)
{
  ASSERT_EQ(init_controller(), controller_interface::return_type::OK);
  configure_and_activate();

  auto pub_node = rclcpp::Node::make_shared("test_publisher");
  auto cmd_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>(
    "/test_swerve_controller/cmd_vel", rclcpp::QoS(10).reliable());

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.5;
  cmd_pub->publish(cmd);

  spin_for(nullptr, std::chrono::milliseconds(200));

  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);

  const double expected_omega = -0.5 / kWheelRadius;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    EXPECT_NEAR(wheel_cmd_vals_[i], expected_omega, kTolerance)
      << "wheel[" << i << "]";
    EXPECT_NEAR(steer_cmd_vals_[i], 0.0, kTolerance)
      << "steer[" << i << "]";
  }
}

// max_steering_angle = 1.0 with cmd (vx=-0.5, vy=1.0): target_steer ~ 2.03,
// flipped (target - pi) ~ -1.11 also outside +-1.0. The controller must NOT commit to
// the flip in that case, otherwise it would drive backward toward a wrong heading.
TEST_F(SwerveControllerTest, ReverseFlipSkippedWhenFlippedTargetOutOfRange)
{
  ASSERT_EQ(
    init_controller({rclcpp::Parameter("max_steering_angle", 1.0)}),
    controller_interface::return_type::OK);
  configure_and_activate();

  auto pub_node = rclcpp::Node::make_shared("test_publisher");
  auto cmd_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>(
    "/test_swerve_controller/cmd_vel", rclcpp::QoS(10).reliable());

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.5;
  cmd.linear.y = 1.0;
  cmd_pub->publish(cmd);

  spin_for(nullptr, std::chrono::milliseconds(200));

  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);

  // Steering still has to travel 1.0 rad, so traction is scaled by cos(1.0).
  const double expected_speed = std::hypot(-0.5, 1.0);
  const double expected_omega = expected_speed * std::cos(1.0) / kWheelRadius;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    EXPECT_NEAR(wheel_cmd_vals_[i], expected_omega, kTolerance)
      << "wheel[" << i << "]";
    EXPECT_NEAR(steer_cmd_vals_[i], 1.0, kTolerance)
      << "steer[" << i << "]";
  }
}

// Pure lateral Twist (vy=0.5) with all wheels at 0: the target is pi/2, both drive senses
// cost the same so the wheel stays forward, and traction is cut until the wheel has turned.
// Once the steering state reaches the target the full speed is commanded.
TEST_F(SwerveControllerTest, LateralCommandGatesTractionUntilAligned)
{
  ASSERT_EQ(init_controller(), controller_interface::return_type::OK);
  configure_and_activate();

  auto pub_node = rclcpp::Node::make_shared("test_publisher");
  auto cmd_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>(
    "/test_swerve_controller/cmd_vel", rclcpp::QoS(10).reliable());

  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = 0.5;
  cmd_pub->publish(cmd);

  spin_for(nullptr, std::chrono::milliseconds(200));

  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    EXPECT_NEAR(wheel_cmd_vals_[i], 0.0, kTolerance) << "wheel[" << i << "]";
    EXPECT_NEAR(steer_cmd_vals_[i], M_PI_2, kTolerance) << "steer[" << i << "]";
  }

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    steer_state_vals_[i] = M_PI_2;
  }
  EXPECT_EQ(controller_->update(t0 + dt, dt), controller_interface::return_type::OK);

  const double expected_omega = 0.5 / kWheelRadius;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    EXPECT_NEAR(wheel_cmd_vals_[i], expected_omega, kTolerance) << "wheel[" << i << "]";
    EXPECT_NEAR(steer_cmd_vals_[i], M_PI_2, kTolerance) << "steer[" << i << "]";
  }
}

// Wheels sitting at 1.2 rad get a heading of -1.15 rad: reaching it forward costs 2.35 rad,
// reversed only 0.79 rad, so the wheel reverses toward 1.99 rad. As the steering state
// closes in on that target the reversed sense must hold, and traction ramps with cos(error).
TEST_F(SwerveControllerTest, FlipDecisionHoldsWhileSteeringConverges)
{
  ASSERT_EQ(init_controller(), controller_interface::return_type::OK);
  configure_and_activate();

  auto pub_node = rclcpp::Node::make_shared("test_publisher");
  auto cmd_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>(
    "/test_swerve_controller/cmd_vel", rclcpp::QoS(10).reliable());

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.36;
  cmd.linear.y = -0.8;
  cmd_pub->publish(cmd);

  spin_for(nullptr, std::chrono::milliseconds(200));

  const double heading = std::atan2(-0.8, 0.36);
  const double flipped = heading + M_PI;
  const double speed = std::hypot(0.36, 0.8);
  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);

  const std::vector<double> states = {1.2, 1.5, 1.8, flipped};
  for (std::size_t k = 0; k < states.size(); ++k) {
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      steer_state_vals_[i] = states[k];
    }
    EXPECT_EQ(
      controller_->update(t0 + dt * static_cast<double>(k), dt),
      controller_interface::return_type::OK);
    const double err = std::abs(flipped - states[k]);
    const double expected_omega = (err >= 1.0471975511965976 ? 0.0 : -speed * std::cos(err)) /
      kWheelRadius;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      EXPECT_NEAR(steer_cmd_vals_[i], flipped, kTolerance)
        << "step " << k << " steer[" << i << "]";
      EXPECT_NEAR(wheel_cmd_vals_[i], expected_omega, kTolerance)
        << "step " << k << " wheel[" << i << "]";
    }
  }
}

// Drive the state to mimic a steady 0.5 m/s forward motion: each wheel spinning at vx/R,
// all steers at 0. Inverse kinematics on the wheel state should recover (vx=0.5, vy=0, wz=0)
// and the integrated odom should advance by vx*dt along x.
TEST_F(SwerveControllerTest, OdometryReportsForwardTwist)
{
  ASSERT_EQ(
    init_controller({rclcpp::Parameter("publish_odom", true)}),
    controller_interface::return_type::OK);
  configure_and_activate();

  const double wheel_omega = 0.5 / kWheelRadius;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    wheel_state_vals_[i] = wheel_omega;
    steer_state_vals_[i] = 0.0;
  }

  auto sub_node = rclcpp::Node::make_shared("test_subscriber");
  nav_msgs::msg::Odometry::SharedPtr received;
  auto sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
    "/test_swerve_controller/odometry", rclcpp::SystemDefaultsQoS(),
    [&](nav_msgs::msg::Odometry::SharedPtr msg) {received = msg;});

  // Let publisher and subscriber discover each other before the update fires the publish.
  spin_for(sub_node, std::chrono::milliseconds(200));

  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);

  for (int i = 0; i < 50 && !received; ++i) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);

  EXPECT_NEAR(received->twist.twist.linear.x, 0.5, kTolerance);
  EXPECT_NEAR(received->twist.twist.linear.y, 0.0, kTolerance);
  EXPECT_NEAR(received->twist.twist.angular.z, 0.0, kTolerance);
  EXPECT_NEAR(received->pose.pose.position.x, 0.5 * 0.02, kTolerance);
  EXPECT_NEAR(received->pose.pose.position.y, 0.0, kTolerance);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

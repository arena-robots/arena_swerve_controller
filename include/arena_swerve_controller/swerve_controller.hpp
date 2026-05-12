// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#ifndef ARENA_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
#define ARENA_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <arena_swerve_controller/arena_swerve_controller_parameters.hpp>

namespace arena_swerve_controller
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  SwerveController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Indices into command_interfaces_ / state_interfaces_, in the order params_.wheel_joint_names
  // and params_.steering_joint_names declare them.
  std::vector<std::size_t> wheel_cmd_idx_;
  std::vector<std::size_t> steering_cmd_idx_;
  std::vector<std::size_t> wheel_state_idx_;
  std::vector<std::size_t> steering_state_idx_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>> cmd_vel_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_pub_;

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

  // Inverse-kinematics LHS: depends only on (x_i, y_i) which are static params, so the matrix and
  // its decomposition are built once in on_configure and reused every tick.
  Eigen::Matrix<double, 8, 3> ik_A_;
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 8, 3>> ik_qr_;
};

}  // namespace arena_swerve_controller

#endif  // ARENA_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

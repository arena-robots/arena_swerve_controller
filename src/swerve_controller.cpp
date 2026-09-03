// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#include "arena_swerve_controller/swerve_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr std::size_t WHEEL_COUNT = 4;

double wrap_to_pi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

namespace arena_swerve_controller
{

controller_interface::CallbackReturn SwerveController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "param init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SwerveController::command_interface_configuration() const
{
  std::vector<std::string> names;
  names.reserve(2 * WHEEL_COUNT);
  for (const auto & j : params_.wheel_joint_names) {
    names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & j : params_.steering_joint_names) {
    names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
SwerveController::state_interface_configuration() const
{
  std::vector<std::string> names;
  names.reserve(2 * WHEEL_COUNT);
  for (const auto & j : params_.wheel_joint_names) {
    names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & j : params_.steering_joint_names) {
    names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn
SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();

  const auto cmd_qos = rclcpp::QoS(10).reliable();
  if (params_.use_stamped_vel) {
    cmd_vel_stamped_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel", cmd_qos,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        cmd_vel_buffer_.writeFromNonRT(msg);
      });
  } else {
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        auto stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
        stamped->header.stamp = get_node()->now();
        stamped->twist = *msg;
        cmd_vel_buffer_.writeFromNonRT(stamped);
      });
  }

  if (params_.publish_odom) {
    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
    rt_odom_pub_->msg_.header.frame_id = params_.odom_frame_id;
    rt_odom_pub_->msg_.child_frame_id = params_.base_frame_id;
    for (std::size_t i = 0; i < 6; ++i) {
      rt_odom_pub_->msg_.pose.covariance[i * 6 + i] = params_.pose_covariance_diagonal[i];
      rt_odom_pub_->msg_.twist.covariance[i * 6 + i] = params_.twist_covariance_diagonal[i];
    }
  }
  if (params_.publish_tf) {
    tf_pub_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
    rt_tf_pub_->msg_.transforms.resize(1);
    rt_tf_pub_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
    rt_tf_pub_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
  }

  // Cache the static LHS of the inverse-kinematics LSQ and its QR decomposition.
  ik_A_.setZero();
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    const double x = params_.wheel_positions_x[i];
    const double y = params_.wheel_positions_y[i];
    ik_A_(2 * i, 0) = 1.0;
    ik_A_(2 * i, 2) = -y;
    ik_A_(2 * i + 1, 1) = 1.0;
    ik_A_(2 * i + 1, 2) = x;
  }
  ik_qr_.compute(ik_A_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  auto find_cmd = [&](const std::string & full_name, std::size_t & out) {
      auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
        [&](const auto & h) {return h.get_name() == full_name;});
      if (it == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing command interface: %s", full_name.c_str());
        return false;
      }
      out = static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it));
      return true;
    };
  auto find_state = [&](const std::string & full_name, std::size_t & out) {
      auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto & h) {return h.get_name() == full_name;});
      if (it == state_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing state interface: %s", full_name.c_str());
        return false;
      }
      out = static_cast<std::size_t>(std::distance(state_interfaces_.begin(), it));
      return true;
    };

  wheel_cmd_idx_.assign(WHEEL_COUNT, 0);
  steering_cmd_idx_.assign(WHEEL_COUNT, 0);
  wheel_state_idx_.assign(WHEEL_COUNT, 0);
  steering_state_idx_.assign(WHEEL_COUNT, 0);

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    if (!find_cmd(
        params_.wheel_joint_names[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        wheel_cmd_idx_[i])) {return controller_interface::CallbackReturn::ERROR;}
    if (!find_cmd(
        params_.steering_joint_names[i] + "/" + hardware_interface::HW_IF_POSITION,
        steering_cmd_idx_[i])) {return controller_interface::CallbackReturn::ERROR;}
    if (!find_state(
        params_.wheel_joint_names[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        wheel_state_idx_[i])) {return controller_interface::CallbackReturn::ERROR;}
    if (!find_state(
        params_.steering_joint_names[i] + "/" + hardware_interface::HW_IF_POSITION,
        steering_state_idx_[i])) {return controller_interface::CallbackReturn::ERROR;}
  }

  auto zero = std::make_shared<geometry_msgs::msg::TwistStamped>();
  zero->header.stamp = get_node()->now();
  cmd_vel_buffer_.writeFromNonRT(zero);

  reversed_.assign(WHEEL_COUNT, false);
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto i : wheel_cmd_idx_) {
    (void)command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
SwerveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto cmd_ptr = *cmd_vel_buffer_.readFromRT();
  double vx = 0.0, vy = 0.0, wz = 0.0;
  if (cmd_ptr) {
    const double age = (time - cmd_ptr->header.stamp).seconds();
    if (age < params_.cmd_vel_timeout) {
      vx = cmd_ptr->twist.linear.x;
      vy = cmd_ptr->twist.linear.y;
      wz = cmd_ptr->twist.angular.z;
    }
  }

  // Forward kinematics: per-wheel velocity vector at hub position r_i, split into steering
  // heading and traction speed. Sticky reversed sense with flip_hysteresis, traction scaled
  // by cos(steering error) and cut beyond traction_gate_angle.
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    const double x = params_.wheel_positions_x[i];
    const double y = params_.wheel_positions_y[i];
    const double vx_i = vx - wz * y;
    const double vy_i = vy + wz * x;

    const double speed = std::hypot(vx_i, vy_i);
    const double current_steer =
      state_interfaces_[steering_state_idx_[i]].get_optional().value_or(0.0);
    if (speed < 1e-6) {
      (void)command_interfaces_[wheel_cmd_idx_[i]].set_value(0.0);
      (void)command_interfaces_[steering_cmd_idx_[i]].set_value(current_steer);
      continue;
    }

    const double forward = std::atan2(vy_i, vx_i);
    const double flipped = wrap_to_pi(forward + M_PI);
    const bool forward_ok = std::abs(forward) <= params_.max_steering_angle;
    const bool flipped_ok =
      params_.allow_reverse_drive && std::abs(flipped) <= params_.max_steering_angle;
    const double cost_forward = std::abs(wrap_to_pi(forward - current_steer));
    const double cost_flipped = std::abs(wrap_to_pi(flipped - current_steer));

    bool reversed = reversed_[i];
    if (!flipped_ok) {
      reversed = false;
    } else if (!forward_ok) {
      reversed = true;
    } else if (reversed) {
      reversed = !(cost_forward + params_.flip_hysteresis < cost_flipped);
    } else {
      reversed = cost_flipped + params_.flip_hysteresis < cost_forward;
    }
    reversed_[i] = reversed;

    const double target_steer = std::clamp(
      reversed ? flipped : forward,
      -params_.max_steering_angle, params_.max_steering_angle);
    const double steer_error = std::abs(wrap_to_pi(target_steer - current_steer));
    const double traction =
      steer_error >= params_.traction_gate_angle ? 0.0 : std::cos(steer_error);
    const double wheel_omega = (reversed ? -speed : speed) * traction / params_.wheel_radius;

    (void)command_interfaces_[wheel_cmd_idx_[i]].set_value(wheel_omega);
    (void)command_interfaces_[steering_cmd_idx_[i]].set_value(target_steer);
  }

  // Inverse kinematics for odometry: each wheel contributes two equations
  //   vx - wz*y_i = wheel_speed_i * cos(steer_i)
  //   vy + wz*x_i = wheel_speed_i * sin(steer_i)
  // LHS is static, decomposed once in on_configure; only b is rebuilt per tick.
  Eigen::Matrix<double, 8, 1> b;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    const double meas_speed =
      state_interfaces_[wheel_state_idx_[i]].get_optional().value_or(0.0) * params_.wheel_radius;
    const double meas_steer =
      state_interfaces_[steering_state_idx_[i]].get_optional().value_or(0.0);
    b(2 * i) = meas_speed * std::cos(meas_steer);
    b(2 * i + 1) = meas_speed * std::sin(meas_steer);
  }
  const Eigen::Vector3d twist_est = ik_qr_.solve(b);
  const double vx_est = twist_est(0);
  const double vy_est = twist_est(1);
  const double wz_est = twist_est(2);

  const double dt = period.seconds();
  const double cy = std::cos(odom_yaw_);
  const double sy = std::sin(odom_yaw_);
  odom_x_ += (vx_est * cy - vy_est * sy) * dt;
  odom_y_ += (vx_est * sy + vy_est * cy) * dt;
  odom_yaw_ = wrap_to_pi(odom_yaw_ + wz_est * dt);

  if (rt_odom_pub_ && rt_odom_pub_->trylock()) {
    auto & msg = rt_odom_pub_->msg_;
    msg.header.stamp = time;
    msg.pose.pose.position.x = odom_x_;
    msg.pose.pose.position.y = odom_y_;
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = vx_est;
    msg.twist.twist.linear.y = vy_est;
    msg.twist.twist.angular.z = wz_est;
    rt_odom_pub_->unlockAndPublish();
  }
  if (rt_tf_pub_ && rt_tf_pub_->trylock()) {
    auto & tf = rt_tf_pub_->msg_.transforms[0];
    tf.header.stamp = time;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    rt_tf_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arena_swerve_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arena_swerve_controller::SwerveController, controller_interface::ControllerInterface)

// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

// Verifies the plugin XML is correctly exported and pluginlib can resolve the class.
TEST(TestLoadSwerveController, PluginIsRegistered)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");

  std::shared_ptr<controller_interface::ControllerInterface> controller;
  ASSERT_NO_THROW(
    controller = loader.createSharedInstance("arena_swerve_controller/SwerveController"));
  ASSERT_NE(controller, nullptr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
// Explicit controlled-experiment pose seeding. This tool never commands motion
// and is not used by the perception node or the later autonomous mission.
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<rclcpp::Node>("validation_initial_pose");
    const auto acknowledge = node->declare_parameter<bool>("acknowledge_validation", false);
    const auto missing = std::numeric_limits<double>::quiet_NaN();
    const double x = node->declare_parameter<double>("x", missing);
    const double y = node->declare_parameter<double>("y", missing);
    const double yaw = node->declare_parameter<double>("yaw", missing);
    const double position_variance = node->declare_parameter<double>(
      "position_variance", 0.0025);
    const double yaw_variance = node->declare_parameter<double>("yaw_variance", 0.0004);
    if (!acknowledge) {
      throw std::runtime_error(
              "Explicit controlled-validation use requires -p acknowledge_validation:=true");
    }
    if (!node->get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Run validation_initial_pose with -p use_sim_time:=true");
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      !std::isfinite(position_variance) || position_variance <= 0 ||
      !std::isfinite(yaw_variance) || yaw_variance <= 0)
    {
      throw std::runtime_error("Supply finite x/y/yaw and positive finite pose variances");
    }
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).reliable().transient_local());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
      (node->now().nanoseconds() <= 0 || publisher->get_subscription_count() == 0))
    {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    if (!rclcpp::ok() || node->now().nanoseconds() <= 0 ||
      publisher->get_subscription_count() == 0)
    {
      throw std::runtime_error("Stopped/timed out waiting for /clock and /initialpose subscriber");
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node->now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.pose.orientation.w = std::cos(yaw / 2);
    pose.pose.covariance[0] = position_variance;
    pose.pose.covariance[7] = position_variance;
    pose.pose.covariance[35] = yaw_variance;
    RCLCPP_WARN(node->get_logger(),
      "CONTROLLED VALIDATION ONLY: publishing one initialpose map=(%.9f,%.9f) yaw=%.9f "
      "variance_xy=%.9f variance_yaw=%.9f; no motion command",
      x, y, yaw, position_variance, yaw_variance);
    publisher->publish(pose);
    const auto deliver_until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deliver_until) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    result = rclcpp::ok() ? 0 : 2;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("validation_initial_pose"), "%s", error.what());
  }
  rclcpp::shutdown();
  return result;
}

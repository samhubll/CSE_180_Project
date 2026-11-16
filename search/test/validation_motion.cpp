// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
// Manual Gazebo validation helper. Every pose/angle comes from CLI parameters;
// this executable contains no autonomous route or person-location knowledge.
#include <chrono>
#include <cmath>
#include <csignal>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "navigation/navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

using Deadline = std::chrono::steady_clock::time_point;

bool remaining(const Deadline & deadline)
{
  return !stop_requested && rclcpp::ok() && std::chrono::steady_clock::now() < deadline;
}

template<typename Action>
bool wait_for_server(
  const std::shared_ptr<Navigator> & navigator, const std::string & action,
  const Deadline & deadline)
{
  auto client = rclcpp_action::create_client<Action>(navigator, action);
  while (remaining(deadline)) {
    if (client->wait_for_action_server(std::chrono::milliseconds(200))) {
      return true;
    }
  }
  return false;
}

int run(const std::shared_ptr<Navigator> & navigator)
{
  if (!navigator->get_parameter("use_sim_time").as_bool()) {
    throw std::runtime_error("validation_motion requires -p use_sim_time:=true");
  }
  const double missing = std::numeric_limits<double>::quiet_NaN();
  const auto mode = navigator->declare_parameter<std::string>("mode", "");
  const double x = navigator->declare_parameter<double>("goal_x", missing);
  const double y = navigator->declare_parameter<double>("goal_y", missing);
  const double yaw = navigator->declare_parameter<double>("goal_yaw", missing);
  const double angle = navigator->declare_parameter<double>("spin_angle", missing);
  const double timeout = navigator->declare_parameter<double>("timeout_sec", 120.0);
  if (!std::isfinite(timeout) || timeout <= 0.0 || timeout > 600.0) {
    throw std::runtime_error("timeout_sec must be finite, positive and at most 600");
  }
  if (mode != "navigate" && mode != "spin") {
    throw std::runtime_error("set mode:=navigate or mode:=spin explicitly");
  }
  if ((mode == "navigate" && (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(yaw))) || (mode == "spin" && (!std::isfinite(angle) || angle == 0)))
  {
    throw std::runtime_error(
            "navigate requires finite goal_x/goal_y/goal_yaw; spin requires finite nonzero spin_angle");
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout));
  // Receive /clock before Navigator stamps a goal. Navigator owns all spinning;
  // do not add this node to a separate executor.
  while (remaining(deadline) && navigator->now().nanoseconds() == 0) {
    rclcpp::spin_some(navigator);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  if (!remaining(deadline)) {
    RCLCPP_ERROR(navigator->get_logger(), "Stopped/timed out waiting for simulation clock");
    return 2;
  }
  const bool available = mode == "navigate" ?
    wait_for_server<nav2_msgs::action::NavigateToPose>(navigator, "navigate_to_pose", deadline) :
    wait_for_server<nav2_msgs::action::Spin>(navigator, "spin", deadline);
  if (!available) {
    RCLCPP_ERROR(navigator->get_logger(), "Stopped/timed out waiting for %s server", mode.c_str());
    return 2;
  }

  bool accepted;
  if (mode == "navigate") {
    auto pose = std::make_shared<geometry_msgs::msg::Pose>();
    pose->position.x = x;
    pose->position.y = y;
    pose->orientation.z = std::sin(yaw / 2);
    pose->orientation.w = std::cos(yaw / 2);
    RCLCPP_INFO(navigator->get_logger(), "Manual validation navigate map=(%.3f,%.3f) yaw=%.3f",
      x, y, yaw);
    accepted = navigator->GoToPose(pose);
  } else {
    RCLCPP_INFO(navigator->get_logger(), "Manual validation spin angle=%.3frad", angle);
    accepted = navigator->Spin(angle);
  }
  if (!accepted) {
    RCLCPP_ERROR(navigator->get_logger(), "Validation goal rejected");
    return 3;
  }
  bool complete = false;
  while (remaining(deadline)) {
    if (navigator->IsTaskComplete()) {
      complete = true;
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  if (!complete) {
    RCLCPP_WARN(navigator->get_logger(), "Stopped/timed out; canceling validation goal");
    navigator->CancelTask();
  }
  const auto result = navigator->GetResult();
  RCLCPP_INFO(navigator->get_logger(), "Validation %s result=%d complete=%s",
    mode.c_str(), static_cast<int>(result), complete ? "yes" : "no");
  return complete && result == rclcpp_action::ResultCode::SUCCEEDED ? 0 : 4;
}
}  // namespace

int main(int argc, char ** argv)
{
  // Keep the ROS context alive while handling Ctrl+C so cancellation can still
  // reach Nav2. Use an outer shell timeout with a kill-after bound as well:
  // MRTP's goal acceptance and CancelTask internals have no timeout argument.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int result = 1;
  auto navigator = std::make_shared<Navigator>(false, false);
  try {
    result = run(navigator);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(navigator->get_logger(), "Validation motion: %s", error.what());
  }
  navigator.reset();
  rclcpp::shutdown();
  return result;
}

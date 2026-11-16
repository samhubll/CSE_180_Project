// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
// Read-only Nav2 planning checks. This never sends a robot motion goal.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "search/coverage_plan.hpp"

int main(int argc, char ** argv)
{
  using Action = nav2_msgs::action::ComputePathToPose;
  using Json = nlohmann::json;
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  int failures = 0;
  try {
    auto node = rclcpp::Node::make_shared("check_coverage_paths");
    if (!node->get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Pass use_sim_time:=true");
    }
    const auto missing = std::numeric_limits<double>::quiet_NaN();
    double x = node->declare_parameter("start_x", missing);
    double y = node->declare_parameter("start_y", missing);
    double yaw = node->declare_parameter("start_yaw", missing);
    const auto output_path = node->declare_parameter<std::string>("output_path", "");
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      throw std::runtime_error("Supply finite map start_x/start_y/start_yaw");
    }
    std::ofstream output;
    if (!output_path.empty()) {
      output.open(output_path);
      if (!output) {
        throw std::runtime_error("Cannot write output_path");
      }
    }
    auto emit = [&output](const Json & row) {
        std::cout << row.dump() << std::endl;
        if (output) {
          output << row.dump() << std::endl;
        }
      };
    auto client = rclcpp_action::create_client<Action>(node, "/compute_path_to_pose");
    if (!client->wait_for_action_server(10s)) {
      throw std::runtime_error("Nav2 path planner unavailable");
    }
    const auto clock_deadline = std::chrono::steady_clock::now() + 10s;
    while (rclcpp::ok() && node->now().nanoseconds() == 0 &&
      std::chrono::steady_clock::now() < clock_deadline)
    {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(10ms);
    }
    if (node->now().nanoseconds() == 0) {
      throw std::runtime_error("Simulation clock unavailable");
    }
    const auto & route = search_coverage::warehouse_waypoints();
    emit({{"type", "begin"}, {"motion_goals_sent", false}, {"waypoints", route.size()},
        {"planning_range_m", search_coverage::kUsableRangeM}});
    for (size_t i = 0; i < route.size() && rclcpp::ok(); ++i) {
      const auto & target = route[i];
      Action::Goal request;
      request.use_start = true;
      request.start.header.frame_id = request.goal.header.frame_id = "map";
      request.start.header.stamp = request.goal.header.stamp = node->now();
      request.start.pose.position.x = x;
      request.start.pose.position.y = y;
      request.start.pose.orientation.z = std::sin(yaw / 2);
      request.start.pose.orientation.w = std::cos(yaw / 2);
      request.goal.pose.position.x = target.x;
      request.goal.pose.position.y = target.y;
      request.goal.pose.orientation.z = std::sin(target.yaw / 2);
      request.goal.pose.orientation.w = std::cos(target.yaw / 2);
      Json row{{"type", "leg"}, {"waypoint", i + 1}, {"start", {x, y, yaw}},
        {"goal", {target.x, target.y, target.yaw}},
        {"position_tolerance_m", target.position_tolerance_m}, {"success", false}};
      const auto began = std::chrono::steady_clock::now();
      auto accepted = client->async_send_goal(request);
      if (rclcpp::spin_until_future_complete(node, accepted, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        row["reason"] = "goal_acceptance_timeout";
      } else if (!accepted.get()) {
        row["reason"] = "goal_rejected";
      } else {
        auto handle = accepted.get();
        auto future = client->async_get_result(handle);
        if (rclcpp::spin_until_future_complete(node, future, 5s) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
          row["reason"] = "planning_timeout";
          auto cancel = client->async_cancel_goal(handle);
          rclcpp::spin_until_future_complete(node, cancel, 2s);
        } else {
          const auto result = future.get();
          const auto & path = result.result->path.poses;
          row["action_result"] = static_cast<int>(result.code);
          row["error_code"] = result.result->error_code;
          row["error_msg"] = result.result->error_msg;
          row["poses"] = path.size();
          double length = 0;
          for (size_t j = 1; j < path.size(); ++j) {
            length += std::hypot(path[j].pose.position.x - path[j - 1].pose.position.x,
                path[j].pose.position.y - path[j - 1].pose.position.y);
          }
          row["path_length_m"] = length;
          double endpoint_error = std::numeric_limits<double>::infinity();
          if (!path.empty()) {
            const auto & end = path.back().pose.position;
            row["path_endpoint_xy"] = {end.x, end.y};
            endpoint_error = std::hypot(end.x - target.x, end.y - target.y);
            row["endpoint_error_m"] = endpoint_error;
          }
          row["success"] = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->error_code == 0 && !path.empty() &&
            endpoint_error <= std::min(0.10, target.position_tolerance_m);
          if (!path.empty() && endpoint_error > std::min(0.10, target.position_tolerance_m)) {
            row["reason"] = "planned_endpoint_outside_checked_tolerance";
          }
        }
      }
      row["wall_sec"] = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - began).count();
      if (!row["success"].get<bool>()) {
        ++failures;
      }
      emit(row);
      x = target.x;
      y = target.y;
      yaw = target.yaw;
    }
    emit({{"type", "summary"}, {"failures", failures}, {"waypoints", route.size()},
        {"motion_goals_sent", false}});
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    failures = 1;
  }
  rclcpp::shutdown();
  return failures == 0 ? 0 : 1;
}

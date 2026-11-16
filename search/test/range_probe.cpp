// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
// Validation-only collector. Truth poses are supplied at runtime, never used
// by search. All geometry uses the processed RGB image's nonzero timestamp.
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "search/perception.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace
{
using Json = nlohmann::json;
namespace core = search_perception;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

struct Truth
{
  double x, y, z, yaw;
};

struct Pending
{
  vision_msgs::msg::Detection2DArray::ConstSharedPtr message;
  int64_t arrival_sim_ns;
  Clock::time_point arrival_wall;
};

Json point_json(const geometry_msgs::msg::Point & point)
{
  return Json::array({point.x, point.y, point.z});
}

Json distribution(std::vector<double> values)
{
  if (values.empty()) {
    return nullptr;
  }
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  const double median = n % 2 ? values[n / 2] :
    (values[n / 2 - 1] + values[n / 2]) / 2;
  return {{"min", values.front()}, {"median", median}, {"max", values.back()}};
}

class RangeProbe : public rclcpp::Node
{
public:
  RangeProbe()
  : Node("range_probe"), depth_(160, 8000000000LL), buffer_(get_clock()),
    listener_(buffer_, this, false), begin_(Clock::now())
  {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("range_probe requires use_sim_time:=true");
    }
    label_ = declare_parameter<std::string>("trial_label", "trial");
    output_path_ = declare_parameter<std::string>("output_path", "");
    snapshot_dir_ = declare_parameter<std::string>("snapshot_dir", "");
    optical_frame_ = declare_parameter<std::string>(
      "optical_frame", "oakd_rgb_camera_optical_frame");
    const double missing = std::numeric_limits<double>::quiet_NaN();
    truths_.push_back({declare_parameter("target_x", missing),
        declare_parameter("target_y", missing), declare_parameter("target_z", 1.0),
        declare_parameter("target_yaw", missing)});
    const auto additional = declare_parameter<std::vector<double>>(
      "additional_truth_xyz", std::vector<double>{});
    if (additional.size() % 3 != 0) {
      throw std::runtime_error("additional_truth_xyz must contain x,y,z triples");
    }
    for (size_t i = 0; i < additional.size(); i += 3) {
      truths_.push_back({additional[i], additional[i + 1], additional[i + 2], 0});
    }
    sample_goal_ = declare_parameter<int>("sample_count", 10);
    warmup_ = declare_parameter("warmup_sim_sec", 7.0);
    min_warmup_wall_ = declare_parameter("warmup_wall_sec", 10.0);
    max_wall_ = declare_parameter("max_wall_sec", 120.0);
    confidence_ = declare_parameter("min_confidence", 0.5);
    tolerance_ = declare_parameter("depth_tolerance_sec", 0.02);
    max_age_ = declare_parameter("max_detection_age_sec", 5.0);
    error_limit_ = declare_parameter("validation_error_m", 0.5);
    if (label_.empty() || sample_goal_ <= 0 || sample_goal_ > 10000 ||
      !std::isfinite(warmup_) || warmup_ < 0 || !std::isfinite(min_warmup_wall_) ||
      min_warmup_wall_ < 0 || !std::isfinite(max_wall_) || max_wall_ <= 0 ||
      !std::isfinite(confidence_) || confidence_ < 0 || confidence_ > 1 ||
      !std::isfinite(tolerance_) || tolerance_ < 0 || tolerance_ > 0.1 ||
      !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 7 ||
      !std::isfinite(error_limit_) || error_limit_ <= 0)
    {
      throw std::runtime_error("Invalid trial limits/settings");
    }
    for (const auto & truth : truths_) {
      if (!std::isfinite(truth.x) || !std::isfinite(truth.y) ||
        !std::isfinite(truth.z) || !std::isfinite(truth.yaw))
      {
        throw std::runtime_error("Supply finite validation truth x/y/yaw parameters");
      }
    }
    if (!output_path_.empty()) {
      const auto parent = std::filesystem::path(output_path_).parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
      output_.open(output_path_, std::ios::app);
      if (!output_) {
        throw std::runtime_error("Cannot open output_path");
      }
    }
    if (!snapshot_dir_.empty()) {
      std::filesystem::create_directories(snapshot_dir_);
    }
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        rgb_.push_back(message);
        while (rgb_.size() > 160 || (!rgb_.empty() &&
        core::stamp_ns(message->header.stamp) - core::stamp_ns(rgb_.front()->header.stamp) >
        8000000000LL))
        {
          rgb_.pop_front();
        }
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/depth_image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        const auto stamp = core::stamp_ns(message->header.stamp);
        const double age = (now().nanoseconds() - stamp) * 1e-9;
        if (stamp > 0 && age >= -0.1 && age <= 8) {
          depth_.push(message);
        }
      });
    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/rgbd_camera/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {camera_ = message;});
    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/people/detections", rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr message) {
        const auto stamp = core::stamp_ns(message->header.stamp);
        if (!ready_ || stamp < ready_sim_ns_ || stamp <= newest_) {
          ++discarded_warmup_or_duplicate_;
          return;
        }
        newest_ = stamp;
        if (count_ + static_cast<int>(pending_.size()) < sample_goal_) {
          pending_.push_back({message, now().nanoseconds(), Clock::now()});
        }
      });
    emit({{"type", "trial_begin"}, {"sample_goal", sample_goal_},
        {"warmup_sim_sec", warmup_}, {"max_wall_sec", max_wall_},
        {"warmup_wall_sec", min_warmup_wall_},
        {"min_confidence", confidence_}, {"depth_tolerance_sec", tolerance_},
        {"max_detection_age_sec", max_age_}, {"validation_error_m", error_limit_},
        {"localization_is_independent_core_replay", true},
        {"unassociated_boxes_require_manual_false_positive_review", true},
        {"target_truth", Json::array({truths_[0].x, truths_[0].y, truths_[0].z, truths_[0].yaw})}});
  }

  bool tick()
  {
    const double wall = std::chrono::duration<double>(Clock::now() - begin_).count();
    const auto current = now().nanoseconds();
    if (start_sim_ns_ == 0 && current > 0) {
      start_sim_ns_ = current;
    }
    if (!ready_ && start_sim_ns_ > 0 && (current - start_sim_ns_) * 1e-9 >= warmup_ &&
      wall >= min_warmup_wall_)
    {
      ready_ = true;
      ready_sim_ns_ = current;
      emit({{"type", "trial_ready"}, {"source_floor_ns", ready_sim_ns_},
          {"warmup_wall_sec", wall}, {"depth_buffer_frames", depth_.size()}});
    }
    while (!pending_.empty()) {
      const auto & pending = pending_.front();
      const auto source = core::stamp_ns(pending.message->header.stamp);
      const double age = (current - source) * 1e-9;
      auto depth = depth_.match(source, static_cast<int64_t>(tolerance_ * 1e9),
        pending.message->header.frame_id);
      geometry_msgs::msg::TransformStamped transform;
      std::string failure;
      if (source <= 0 || pending.message->header.frame_id != optical_frame_) {
        failure = "source_header";
      } else if (age < -0.1) {
        failure = "future_image";
      } else if (age > max_age_) {
        failure = !camera_ ? "expired_missing_calibration" :
          (!depth ? "expired_unmatched_depth" : "expired_missing_image_time_tf");
      } else if (!camera_ || !depth) {
        break;
      } else {
        try {
          transform = buffer_.lookupTransform("map", optical_frame_,
              rclcpp::Time(pending.message->header.stamp));
        } catch (const tf2::TransformException &) {
          break;
        }
      }
      record(pending, depth, transform, failure);
      pending_.pop_front();
      if (count_ >= sample_goal_) {
        break;
      }
    }
    if (count_ >= sample_goal_ || wall >= max_wall_) {
      const double source_span = last_source_ > first_source_ ?
        (last_source_ - first_source_) * 1e-9 : 0;
      const double arrival_span = count_ > 1 ?
        std::chrono::duration<double>(last_arrival_ - first_arrival_).count() : 0;
      emit({{"type", "trial_summary"}, {"complete", count_ >= sample_goal_},
          {"wall_sec", wall}, {"distinct_processed_images", count_},
          {"target_detection_images", target_images_},
          {"target_projection_ok_images", projection_images_},
          {"target_usable_localization_images", usable_images_},
          {"unassociated_boxes", unassociated_}, {"frame_rejection_reasons", frame_rejections_},
          {"box_rejection_reasons", box_rejections_},
          {"map_tf_horizontal_distance_m", distribution(distances_)},
          {"map_tf_bearing_rad", distribution(bearings_)},
          {"map_tf_person_aspect_rad", distribution(aspects_)},
          {"latency_sim_sec", distribution(latencies_)},
          {"discarded_warmup_or_duplicate_arrays", discarded_warmup_or_duplicate_},
          {"source_processing_hz", source_span > 0 ? (count_ - 1) / source_span : 0},
          {"wall_processing_hz", arrival_span > 0 ? (count_ - 1) / arrival_span : 0}});
      complete = count_ >= sample_goal_;
      return false;
    }
    return true;
  }

  bool complete = false;

private:
  void emit(Json row)
  {
    row["trial"] = label_;
    if (output_) {
      output_ << row.dump() << '\n';
      output_.flush();
    }
    std::cout << row.dump() << std::endl;
  }

  std::string snapshot(sensor_msgs::msg::Image::ConstSharedPtr rgb, int64_t stamp)
  {
    if (!rgb || snapshot_dir_.empty() || (saved_snapshot_ && count_ + 1 != sample_goal_)) {
      return "";
    }
    if ((rgb->encoding != "rgb8" && rgb->encoding != "bgr8") ||
      rgb->step < static_cast<uint64_t>(rgb->width) * 3 ||
      rgb->data.size() < static_cast<uint64_t>(rgb->height) * rgb->step)
    {
      return "";
    }
    std::string safe_label = label_;
    for (char & c : safe_label) {
      if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_')) {
        c = '_';
      }
    }
    const auto path = (std::filesystem::path(snapshot_dir_) /
      (safe_label + "-" + std::to_string(stamp) + ".ppm")).string();
    std::ofstream file(path, std::ios::binary);
    file << "P6\n" << rgb->width << ' ' << rgb->height << "\n255\n";
    for (uint32_t v = 0; v < rgb->height; ++v) {
      const auto * row = rgb->data.data() + static_cast<size_t>(v) * rgb->step;
      for (uint32_t u = 0; u < rgb->width; ++u) {
        const size_t offset = static_cast<size_t>(u) * 3;
        const char pixel[] = {static_cast<char>(row[offset + (rgb->encoding == "rgb8" ? 0 : 2)]),
          static_cast<char>(row[offset + 1]),
          static_cast<char>(row[offset + (rgb->encoding == "rgb8" ? 2 : 0)])};
        file.write(pixel, 3);
      }
    }
    file.close();
    if (!file) {
      return "";
    }
    saved_snapshot_ = true;
    return path;
  }

  void record(
    const Pending & pending, sensor_msgs::msg::Image::ConstSharedPtr depth,
    const geometry_msgs::msg::TransformStamped & transform, const std::string & failure)
  {
    const auto & array = *pending.message;
    const auto source = core::stamp_ns(array.header.stamp);
    auto rgb = std::find_if(rgb_.begin(), rgb_.end(), [&array, source](const auto & image) {
          return image->header.frame_id == array.header.frame_id &&
                 core::stamp_ns(image->header.stamp) == source;
      });
    Json row{{"type", "image"}, {"source_ns", source},
      {"arrival_sim_ns", pending.arrival_sim_ns},
      {"latency_sim_sec", (pending.arrival_sim_ns - source) * 1e-9},
      {"arrival_wall_sec", std::chrono::duration<double>(pending.arrival_wall - begin_).count()},
      {"source_frame", array.header.frame_id}, {"source_rgb_seen", rgb != rgb_.end()},
      {"frame_rejection", failure}, {"boxes", Json::array()}};
    if (rgb != rgb_.end()) {
      row["rgb_width"] = (*rgb)->width;
      row["rgb_height"] = (*rgb)->height;
      row["rgb_encoding"] = (*rgb)->encoding;
      const auto saved = snapshot(*rgb, source);
      if (!saved.empty()) {
        row["source_matched_snapshot"] = saved;
      }
    }
    if (depth) {
      row["depth_dt_sec"] = (core::stamp_ns(depth->header.stamp) - source) * 1e-9;
      row["depth_encoding"] = depth->encoding;
    }
    tf2::Transform map_from_optical;
    std::vector<tf2::Vector3> optical_truth;
    if (failure.empty()) {
      tf2::fromMsg(transform.transform, map_from_optical);
      const auto & truth = truths_[0];
      const auto camera = map_from_optical.getOrigin();
      const double dx = truth.x - camera.x(), dy = truth.y - camera.y();
      const auto forward = map_from_optical.getBasis() * tf2::Vector3(0, 0, 1);
      row["map_tf_camera_xyz"] = Json::array({camera.x(), camera.y(), camera.z()});
      row["map_tf_camera_heading_rad"] = std::atan2(forward.y(), forward.x());
      row["map_tf_horizontal_distance_m"] = std::hypot(dx, dy);
      row["map_tf_body_point_distance_m"] = std::sqrt(dx * dx + dy * dy +
          (truth.z - camera.z()) * (truth.z - camera.z()));
      row["map_tf_bearing_rad"] = std::remainder(std::atan2(dy, dx) -
          std::atan2(forward.y(), forward.x()), 2 * pi);
      row["map_tf_person_aspect_rad"] = std::remainder(
        std::atan2(camera.y() - truth.y, camera.x() - truth.x) - truth.yaw, 2 * pi);
      distances_.push_back(row["map_tf_horizontal_distance_m"].get<double>());
      bearings_.push_back(row["map_tf_bearing_rad"].get<double>());
      aspects_.push_back(row["map_tf_person_aspect_rad"].get<double>());
      try {
        const auto base = buffer_.lookupTransform("map", "base_link",
            rclcpp::Time(array.header.stamp));
        row["map_tf_base_xyz"] = Json::array({base.transform.translation.x,
              base.transform.translation.y, base.transform.translation.z});
      } catch (const tf2::TransformException &) {
        row["map_tf_base_xyz"] = nullptr;
      }
      for (const auto & item : truths_) {
        optical_truth.push_back(map_from_optical.inverse() * tf2::Vector3(item.x, item.y, item.z));
      }
      const auto & target = optical_truth[0];
      if (target.z() > 0 && camera_) {
        row["target_projected_uv"] = Json::array({camera_->k[0] * target.x() / target.z() +
              camera_->k[2], camera_->k[4] * target.y() / target.z() + camera_->k[5]});
        row["target_optical_depth_m"] = target.z();
        if (depth) {
          vision_msgs::msg::BoundingBox2D target_box;
          target_box.center.position.x = row["target_projected_uv"][0].get<double>();
          target_box.center.position.y = row["target_projected_uv"][1].get<double>();
          target_box.size_x = target_box.size_y = 3;
          core::DepthOptions target_options;
          target_options.roi_fraction = 1;
          target_options.min_valid_pixels = 1;
          const auto target_depth = core::sample_depth(*depth, target_box, target_options);
          if (target_depth) {
            row["target_ray_depth_m"] = target_depth->depth_m;
            row["target_ray_foreground_gap_m"] = target.z() - target_depth->depth_m;
          }
        }
      }
    } else {
      ++frame_rejections_[failure];
    }
    bool detected = false, projection = false, usable = false;
    for (const auto & detection : array.detections) {
      const auto & box = detection.bbox;
      Json item{{"cx", box.center.position.x}, {"cy", box.center.position.y},
        {"width", box.size_x}, {"height", box.size_y},
        {"header_matches", detection.header == array.header}, {"hypotheses", Json::array()}};
      double score = -1;
      for (const auto & result : detection.results) {
        item["hypotheses"].push_back({{"class", result.hypothesis.class_id},
            {"confidence", result.hypothesis.score}});
        if (result.hypothesis.class_id == "person" && std::isfinite(result.hypothesis.score)) {
          score = std::max(score, result.hypothesis.score);
        }
      }
      int truth_index = -1;
      if (camera_) {
        for (size_t i = 0; i < optical_truth.size(); ++i) {
          const auto & point = optical_truth[i];
          if (point.z() <= 0) {
            continue;
          }
          const double u = camera_->k[0] * point.x() / point.z() + camera_->k[2];
          const double v = camera_->k[4] * point.y() / point.z() + camera_->k[5];
          if (std::abs(u - box.center.position.x) <= box.size_x / 2 &&
            std::abs(v - box.center.position.y) <= box.size_y / 2)
          {
            truth_index = static_cast<int>(i);
            break;
          }
        }
      }
      item["validation_truth_index"] = truth_index;
      if (truth_index < 0) {
        ++unassociated_;
      }
      detected = detected || (truth_index == 0 && score >= confidence_);
      std::string reason = failure;
      if (reason.empty() && detection.header != array.header) {
        reason = "box_header";
      }
      if (reason.empty() && (score < confidence_ || score > 1)) {
        reason = "class_or_confidence";
      }
      std::optional<core::DepthEstimate> estimate;
      std::optional<geometry_msgs::msg::Point> optical;
      if (reason.empty()) {
        estimate = core::sample_depth(*depth, box, {}, &reason);
        if (!estimate) {
          // This second call diagnoses support only. It cannot accept a point
          // that the production 25-pixel/25%-support policy rejected.
          core::DepthOptions diagnostic_options;
          diagnostic_options.min_valid_pixels = 1;
          diagnostic_options.min_valid_fraction = std::numeric_limits<double>::min();
          const auto support = core::sample_depth(*depth, box, diagnostic_options);
          if (support) {
            item["valid_depth_pixels"] = support->valid_pixels;
            item["roi_pixels"] = support->total_pixels;
            item["diagnostic_median_depth_m"] = support->depth_m;
          }
        }
      }
      if (estimate) {
        item["valid_depth_pixels"] = estimate->valid_pixels;
        item["roi_pixels"] = estimate->total_pixels;
        item["median_depth_m"] = estimate->depth_m;
        optical = core::project(*estimate, *camera_, *depth, &reason);
      }
      if (optical) {
        geometry_msgs::msg::PointStamped source_point, map_point;
        source_point.header = array.header;
        source_point.point = *optical;
        tf2::doTransform(source_point, map_point, transform);
        const double error = std::hypot(map_point.point.x - truths_[0].x,
          map_point.point.y - truths_[0].y);
        item["map_point"] = point_json(map_point.point);
        item["target_horizontal_error_m"] = error;
        item["within_validation_tolerance"] = error <= error_limit_;
        projection = projection || truth_index == 0;
        usable = usable || (truth_index == 0 && error <= error_limit_);
      }
      item["localization_rejection"] = reason;
      item["projection_ok"] = optical.has_value();
      if (!reason.empty()) {
        ++box_rejections_[reason];
      }
      row["boxes"].push_back(item);
    }
    row["target_detected"] = detected;
    row["recognition_association_unknown"] = !failure.empty();
    row["target_projection_ok"] = projection;
    row["target_usable_localization"] = usable;
    if (count_++ == 0) {
      first_source_ = source;
      first_arrival_ = pending.arrival_wall;
    }
    last_source_ = source;
    last_arrival_ = pending.arrival_wall;
    target_images_ += detected;
    projection_images_ += projection;
    usable_images_ += usable;
    latencies_.push_back((pending.arrival_sim_ns - source) * 1e-9);
    emit(row);
  }

  std::string label_, output_path_, snapshot_dir_, optical_frame_;
  std::ofstream output_;
  bool ready_ = false, saved_snapshot_ = false;
  int sample_goal_, count_ = 0, target_images_ = 0, projection_images_ = 0, usable_images_ = 0;
  size_t discarded_warmup_or_duplicate_ = 0, unassociated_ = 0;
  double warmup_, min_warmup_wall_, max_wall_, confidence_, tolerance_, max_age_, error_limit_;
  int64_t start_sim_ns_ = 0, ready_sim_ns_ = 0, newest_ = 0, first_source_ = 0, last_source_ = 0;
  std::vector<Truth> truths_;
  std::vector<double> distances_, bearings_, aspects_, latencies_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> rgb_;
  std::deque<Pending> pending_;
  std::map<std::string, size_t> frame_rejections_, box_rejections_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_;
  core::DepthHistory depth_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  Clock::time_point begin_, first_arrival_, last_arrival_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<RangeProbe>();
    rclcpp::WallRate rate(100);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (!node->tick()) {
        break;
      }
      rate.sleep();
    }
    result = node->complete ? 0 : 2;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("range_probe"), "%s", error.what());
  }
  rclcpp::shutdown();
  return result;
}

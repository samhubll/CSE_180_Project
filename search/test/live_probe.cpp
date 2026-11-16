// Copyright 2026 CSE180 student
// Licensed under the Apache License, Version 2.0.
// Read-only, wall-time-bounded evidence collector. It never commands motion.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}

double forward_heading(const geometry_msgs::msg::Quaternion & q)
{
  // Heading of optical +z, which points forward; optical +x is image-right.
  return std::atan2(2.0 * (q.y * q.z - q.w * q.x),
    2.0 * (q.x * q.z + q.w * q.y));
}

struct Samples
{
  size_t count = 0;
  int64_t first = 0;
  int64_t last = 0;
  std::string frame;
  std::deque<int64_t> stamps;

  void add(int64_t stamp)
  {
    if (count++ == 0) {
      first = stamp;
    }
    last = stamp;
    stamps.push_back(stamp);
    while (stamps.size() > 1000) {
      stamps.pop_front();
    }
  }

  double sim_rate() const
  {
    return count > 1 && last > first ?
           (count - 1) * 1e9 / static_cast<double>(last - first) : 0.0;
  }
};

class LiveProbe : public rclcpp::Node
{
public:
  LiveProbe()
  : Node("search_live_probe"), buffer_(get_clock(), tf2::durationFromSec(30.0)),
    listener_(buffer_, this, false)
  {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Live probe requires use_sim_time:=true");
    }
    duration = declare_parameter("duration_sec", 30.0);
    if (!std::isfinite(duration) || duration <= 0.0) {
      throw std::runtime_error("duration_sec must be positive and finite");
    }
    snapshot_path_ = declare_parameter<std::string>("snapshot_path", "");
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        image(*msg, rgb_, "RGB");
        save_snapshot(*msg);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/depth_image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {image(*msg, depth_, "DEPTH");});
    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/rgbd_camera/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (camera_count_++ == 0) {
          std::cout << "CALIBRATION frame=" << msg->header.frame_id << " size=" <<
            msg->width << 'x' << msg->height << " fx=" << msg->k[0] << " fy=" <<
            msg->k[4] << " cx=" << msg->k[2] << " cy=" << msg->k[5] << '\n';
        }
      });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/people/detections", rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {detection(*msg);});
    marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/people/markers", rclcpp::QoS(10).reliable(),
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {markers(*msg);});
    std::cout << std::fixed << std::setprecision(6) << "PROBE_BEGIN use_sim_time=" <<
      (get_parameter("use_sim_time").as_bool() ? "true" : "false") <<
      " duration_wall_sec=" << duration << std::endl;
  }

  double duration;

  void summary(double wall_sec)
  {
    std::cout << "PROBE_SUMMARY wall_sec=" << wall_sec << " rgb=" << rgb_.count <<
      " depth=" << depth_.count << " camera_info=" << camera_count_ <<
      " detections=" << detections_.count << " nonempty_detections=" << nonempty_ <<
      " source_headers_match=" << headers_match_ << " source_headers_missing=" <<
      headers_missing_ << " detection_headers_mismatch=" << box_header_mismatch_ <<
      " marker_arrays=" << marker_arrays_ << " unique_marker_ids=" << marker_points_.size() <<
      " nonfinite_markers=" << nonfinite_markers_ << " rgb_sim_hz=" << rgb_.sim_rate() <<
      " depth_sim_hz=" << depth_.sim_rate() << " detections_sim_hz=" <<
      detections_.sim_rate() << " rgb_wall_hz=" << rgb_.count / wall_sec <<
      " depth_wall_hz=" << depth_.count / wall_sec << " detections_wall_hz=" <<
      detections_.count / wall_sec << " timestamp_tf_success=" << tf_success_ <<
      " timestamp_tf_failure=" << tf_failure_ << std::endl;
  }

private:
  void save_snapshot(const sensor_msgs::msg::Image & msg)
  {
    // This is the first valid RGB received after starting this probe, not a
    // detection-matched frame. The source timestamp is recorded explicitly.
    if (snapshot_path_.empty() || snapshot_attempted_) {
      return;
    }
    if ((msg.encoding != "rgb8" && msg.encoding != "bgr8") || msg.width == 0 ||
      msg.height == 0 || msg.step < static_cast<uint64_t>(msg.width) * 3 ||
      msg.data.size() < static_cast<uint64_t>(msg.step) * msg.height)
    {
      return;
    }
    snapshot_attempted_ = true;
    std::ofstream file(snapshot_path_, std::ios::binary);
    file << "P6\n" << msg.width << ' ' << msg.height << "\n255\n";
    for (uint32_t v = 0; v < msg.height; ++v) {
      const auto * row = msg.data.data() + static_cast<size_t>(v) * msg.step;
      if (msg.encoding == "rgb8") {
        file.write(reinterpret_cast<const char *>(row), static_cast<size_t>(msg.width) * 3);
      } else {
        for (uint32_t u = 0; u < msg.width; ++u) {
          const char pixel[] = {static_cast<char>(row[u * 3 + 2]),
            static_cast<char>(row[u * 3 + 1]), static_cast<char>(row[u * 3])};
          file.write(pixel, sizeof(pixel));
        }
      }
    }
    file.close();
    std::cout << "SNAPSHOT path=" << snapshot_path_ << " saved=" << !file.fail() <<
      " stamp=" << stamp_ns(msg.header.stamp) * 1e-9 << " frame=" << msg.header.frame_id <<
      " first_received_rgb=true detection_matched=false" << std::endl;
  }

  void image(const sensor_msgs::msg::Image & msg, Samples & samples, const char * label)
  {
    if (samples.count == 0) {
      std::cout << label << " frame=" << msg.header.frame_id << " size=" << msg.width <<
        'x' << msg.height << " encoding=" << msg.encoding << " step=" << msg.step <<
        " bigendian=" << int(msg.is_bigendian) << std::endl;
    }
    samples.add(stamp_ns(msg.header.stamp));
    samples.frame = msg.header.frame_id;
  }

  void detection(const vision_msgs::msg::Detection2DArray & msg)
  {
    const int64_t stamp = stamp_ns(msg.header.stamp);
    detections_.add(stamp);
    const bool matched = msg.header.frame_id == rgb_.frame &&
      std::find(rgb_.stamps.begin(), rgb_.stamps.end(), stamp) != rgb_.stamps.end();
    matched ? ++headers_match_ : ++headers_missing_;
    if (!msg.detections.empty()) {
      ++nonempty_;
    }
    int64_t nearest_delta = std::numeric_limits<int64_t>::max();
    for (const auto value : depth_.stamps) {
      if (std::abs(value - stamp) < std::abs(nearest_delta)) {
        nearest_delta = value - stamp;
      }
    }
    std::cout << "DETECTION stamp=" << stamp * 1e-9 << " frame=" << msg.header.frame_id <<
      " boxes=" << msg.detections.size() << " source_rgb_seen=" << matched <<
      " age_sim_sec=" << (now().nanoseconds() - stamp) * 1e-9 <<
      " nearest_depth_delta_sec=" << nearest_delta * 1e-9 << std::endl;
    for (const auto & box : msg.detections) {
      if (stamp_ns(box.header.stamp) != stamp || box.header.frame_id != msg.header.frame_id) {
        ++box_header_mismatch_;
      }
      std::cout << "BOX stamp=" << stamp * 1e-9 << " cx=" << box.bbox.center.position.x <<
        " cy=" << box.bbox.center.position.y << " w=" << box.bbox.size_x << " h=" <<
        box.bbox.size_y;
      if (!box.results.empty()) {
        std::cout << " label=" << box.results.front().hypothesis.class_id <<
          " score=" << box.results.front().hypothesis.score;
      }
      std::cout << std::endl;
    }
    if (msg.detections.empty()) {
      return;
    }
    try {
      const auto at_image = buffer_.lookupTransform(
        "map", msg.header.frame_id, rclcpp::Time(msg.header.stamp));
      const auto latest = buffer_.lookupTransform("map", msg.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PointStamped ray, image_point, latest_point;
      ray.header = msg.header;
      ray.point.z = 3.0;
      tf2::doTransform(ray, image_point, at_image);
      tf2::doTransform(ray, latest_point, latest);
      const double dx = latest_point.point.x - image_point.point.x;
      const double dy = latest_point.point.y - image_point.point.y;
      const double dz = latest_point.point.z - image_point.point.z;
      const double delta_yaw = std::remainder(
        forward_heading(latest.transform.rotation) -
        forward_heading(at_image.transform.rotation), 2.0 * M_PI);
      std::cout << "TF_COMPARISON stamp=" << stamp * 1e-9 << " latest_stamp=" <<
        stamp_ns(latest.header.stamp) * 1e-9 << " yaw_delta_rad=" << delta_yaw <<
        " hypothetical_center_ray_3m_displacement_m=" << std::sqrt(dx * dx + dy * dy + dz * dz) <<
        std::endl;
      ++tf_success_;
    } catch (const tf2::TransformException & error) {
      ++tf_failure_;
      std::cout << "TF_UNAVAILABLE stamp=" << stamp * 1e-9 << " reason=" << error.what() <<
        std::endl;
    }
  }

  void markers(const visualization_msgs::msg::MarkerArray & msg)
  {
    ++marker_arrays_;
    for (const auto & marker : msg.markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD) {
        continue;
      }
      const auto & point = marker.pose.position;
      const bool finite = std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
      if (!finite) {
        ++nonfinite_markers_;
      }
      const std::string key = marker.ns + ":" + std::to_string(marker.id);
      const bool new_id = marker_points_.find(key) == marker_points_.end();
      // Log new IDs and changed positions; repeated timer publications stay quiet.
      const auto previous = marker_points_.find(key);
      if (new_id || previous->second.x != point.x || previous->second.y != point.y ||
        previous->second.z != point.z)
      {
        std::cout << "MARKER key=" << key << " frame=" << marker.header.frame_id <<
          " stamp=" << stamp_ns(marker.header.stamp) * 1e-9 << " new_id=" << new_id <<
          " finite=" << finite << " xyz=" << point.x << ',' << point.y << ',' << point.z <<
          " text=" << marker.text << std::endl;
      }
      marker_points_[key] = point;
    }
  }

  Samples rgb_, depth_, detections_;
  std::string snapshot_path_;
  bool snapshot_attempted_ = false;
  size_t camera_count_ = 0, nonempty_ = 0, headers_match_ = 0, headers_missing_ = 0;
  size_t box_header_mismatch_ = 0, marker_arrays_ = 0, nonfinite_markers_ = 0;
  size_t tf_success_ = 0, tf_failure_ = 0;
  std::map<std::string, geometry_msgs::msg::Point> marker_points_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto probe = std::make_shared<LiveProbe>();
  const auto begin = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(100);
  while (rclcpp::ok() &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() <
    probe->duration)
  {
    rclcpp::spin_some(probe);
    rate.sleep();
  }
  probe->summary(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count());
  probe.reset();
  rclcpp::shutdown();
  return 0;
}

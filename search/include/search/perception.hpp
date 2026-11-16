// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
#ifndef SEARCH__PERCEPTION_HPP_
#define SEARCH__PERCEPTION_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"

namespace search_perception
{

// Returns -1 for malformed ROS timestamps. Zero is deliberately not a usable
// image time: TF2 treats it as a request for its latest transform.
int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp);

struct DepthOptions
{
  double roi_fraction = 0.5;
  std::size_t min_valid_pixels = 25;
  double min_valid_fraction = 0.25;
  double min_depth_m = 0.3;
  double max_depth_m = 100.0;
};

struct DepthEstimate
{
  double depth_m;
  double u;
  double v;
  std::size_t valid_pixels;
  std::size_t total_pixels;
};

// Central 50% of each box dimension, clipped to the image. u,v are the center
// of the resulting discrete pixel region, and depth is its valid-pixel median.
std::optional<DepthEstimate> sample_depth(
  const sensor_msgs::msg::Image & image, const vision_msgs::msg::BoundingBox2D & box,
  const DepthOptions & options = {}, std::string * reason = nullptr);

bool valid_calibration(
  const sensor_msgs::msg::CameraInfo & camera, const sensor_msgs::msg::Image & image,
  std::string * reason = nullptr);

std::optional<geometry_msgs::msg::Point> project(
  const DepthEstimate & sample, const sensor_msgs::msg::CameraInfo & camera,
  const sensor_msgs::msg::Image & image, std::string * reason = nullptr);

class DepthHistory
{
public:
  explicit DepthHistory(std::size_t max_frames = 160, int64_t max_age_ns = 8000000000LL);
  void push(sensor_msgs::msg::Image::ConstSharedPtr image);
  sensor_msgs::msg::Image::ConstSharedPtr match(
    int64_t source_ns, int64_t tolerance_ns, const std::string & frame) const;
  void clear();
  std::size_t size() const {return images_.size();}

private:
  std::size_t max_frames_;
  int64_t max_age_ns_;
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images_;
};

struct Track
{
  int32_t id;
  geometry_msgs::msg::Point position;
  std::size_t observations;
  int64_t last_source_ns;
  bool confirmed;
};

class TrackStore
{
public:
  explicit TrackStore(
    double merge_radius_m = 0.6, std::size_t confirmation_count = 2,
    std::size_t max_tracks = 256);
  // Image arrays must be processed chronologically. A repeated/older image
  // near an existing track never increments its count or creates another ID.
  // Association uses horizontal map distance; height is still averaged.
  std::optional<Track> observe(const geometry_msgs::msg::Point & point, int64_t source_ns);
  const std::vector<Track> & tracks() const {return tracks_;}
  void clear();

private:
  double merge_radius_m_;
  std::size_t confirmation_count_;
  std::size_t max_tracks_;
  int32_t next_id_ = 0;
  std::vector<Track> tracks_;
};

}  // namespace search_perception

#endif  // SEARCH__PERCEPTION_HPP_

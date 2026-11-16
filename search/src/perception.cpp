// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
#include "search/perception.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace search_perception
{
namespace
{
void explain(std::string * reason, const std::string & message)
{
  if (reason) {
    *reason = message;
  }
}

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// Assemble numeric bits explicitly so both endiannesses and padded rows work.
uint32_t read_bits(const uint8_t * bytes, std::size_t count, bool big_endian)
{
  uint32_t bits = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t index = big_endian ? i : count - i - 1;
    bits = (bits << 8) | bytes[index];
  }
  return bits;
}
}  // namespace

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return -1;
  }
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

std::optional<DepthEstimate> sample_depth(
  const sensor_msgs::msg::Image & image, const vision_msgs::msg::BoundingBox2D & box,
  const DepthOptions & options, std::string * reason)
{
  const bool float_depth = image.encoding == "32FC1";
  const std::size_t bytes_per_pixel = float_depth ? 4 : 2;
  if (!float_depth && image.encoding != "16UC1") {
    explain(reason, "depth encoding must be 32FC1 metres or 16UC1 millimetres");
    return std::nullopt;
  }
  if (image.width == 0 || image.height == 0 || image.is_bigendian > 1 ||
    image.step < static_cast<uint64_t>(image.width) * bytes_per_pixel ||
    static_cast<uint64_t>(image.step) * image.height > image.data.size())
  {
    explain(reason, "invalid depth image dimensions, row stride or data length");
    return std::nullopt;
  }
  if (!std::isfinite(options.roi_fraction) || options.roi_fraction <= 0.0 ||
    options.roi_fraction > 1.0 || options.min_valid_pixels == 0 ||
    !std::isfinite(options.min_valid_fraction) || options.min_valid_fraction <= 0.0 ||
    options.min_valid_fraction > 1.0 || !std::isfinite(options.min_depth_m) ||
    !std::isfinite(options.max_depth_m) || options.min_depth_m <= 0.0 ||
    options.max_depth_m <= options.min_depth_m)
  {
    explain(reason, "invalid depth sampling options");
    return std::nullopt;
  }
  const double center_u = box.center.position.x;
  const double center_v = box.center.position.y;
  if (!std::isfinite(center_u) || !std::isfinite(center_v) ||
    !std::isfinite(box.size_x) || !std::isfinite(box.size_y) ||
    box.size_x <= 0.0 || box.size_y <= 0.0 || !std::isfinite(box.center.theta) ||
    std::abs(box.center.theta) > 1e-9)
  {
    explain(reason, "invalid box or unsupported rotated box");
    return std::nullopt;
  }
  const double half_width = box.size_x * options.roi_fraction * 0.5;
  const double half_height = box.size_y * options.roi_fraction * 0.5;
  const double left = center_u - half_width;
  const double right = center_u + half_width;
  const double top = center_v - half_height;
  const double bottom = center_v + half_height;
  if (!std::isfinite(left) || !std::isfinite(right) ||
    !std::isfinite(top) || !std::isfinite(bottom))
  {
    explain(reason, "box extent overflow");
    return std::nullopt;
  }
  // Clamp before converting to integers; an extreme but finite box must not
  // trigger an out-of-range floating-point to integer conversion.
  const auto x0 = static_cast<uint32_t>(std::floor(std::clamp(left, 0.0,
      static_cast<double>(image.width))));
  const auto x1 = static_cast<uint32_t>(std::ceil(std::clamp(right, 0.0,
      static_cast<double>(image.width))));
  const auto y0 = static_cast<uint32_t>(std::floor(std::clamp(top, 0.0,
      static_cast<double>(image.height))));
  const auto y1 = static_cast<uint32_t>(std::ceil(std::clamp(bottom, 0.0,
      static_cast<double>(image.height))));
  if (x1 <= x0 || y1 <= y0) {
    explain(reason, "box central region is outside the depth image");
    return std::nullopt;
  }

  const std::size_t total = static_cast<std::size_t>(x1 - x0) * (y1 - y0);
  std::vector<double> valid;
  valid.reserve(total);
  for (uint32_t v = y0; v < y1; ++v) {
    for (uint32_t u = x0; u < x1; ++u) {
      const auto * bytes = image.data.data() + static_cast<std::size_t>(v) * image.step +
        static_cast<std::size_t>(u) * bytes_per_pixel;
      const uint32_t bits = read_bits(bytes, bytes_per_pixel, image.is_bigendian != 0);
      double depth;
      if (float_depth) {
        float value;
        static_assert(sizeof(value) == sizeof(bits), "32FC1 requires four-byte floats");
        std::memcpy(&value, &bits, sizeof(value));
        depth = value;
      } else {
        depth = static_cast<double>(bits) * 0.001;
      }
      if (std::isfinite(depth) && depth >= options.min_depth_m &&
        depth <= options.max_depth_m)
      {
        valid.push_back(depth);
      }
    }
  }
  if (valid.size() < options.min_valid_pixels ||
    static_cast<double>(valid.size()) / total < options.min_valid_fraction)
  {
    explain(reason, "insufficient valid depth support in central box region");
    return std::nullopt;
  }
  const auto middle = valid.begin() + static_cast<std::ptrdiff_t>(valid.size() / 2);
  std::nth_element(valid.begin(), middle, valid.end());
  double median = *middle;
  if (valid.size() % 2 == 0) {
    median = (*std::max_element(valid.begin(), middle) + median) / 2.0;
  }
  return DepthEstimate{median, (static_cast<double>(x0) + x1 - 1) / 2.0,
    (static_cast<double>(y0) + y1 - 1) / 2.0, valid.size(), total};
}

bool valid_calibration(
  const sensor_msgs::msg::CameraInfo & camera, const sensor_msgs::msg::Image & image,
  std::string * reason)
{
  if (image.width == 0 || image.height == 0 || camera.width != image.width ||
    camera.height != image.height || camera.header.frame_id.empty() ||
    camera.header.frame_id != image.header.frame_id)
  {
    explain(reason, "camera calibration dimensions/frame do not match depth");
    return false;
  }
  if (camera.binning_x > 1 || camera.binning_y > 1 || camera.roi.x_offset != 0 ||
    camera.roi.y_offset != 0 || camera.roi.do_rectify ||
    (camera.roi.width != 0 && camera.roi.width != camera.width) ||
    (camera.roi.height != 0 && camera.roi.height != camera.height))
  {
    explain(reason, "cropped, binned or rectification-ROI calibration is unsupported");
    return false;
  }
  if ((camera.distortion_model != "" && camera.distortion_model != "plumb_bob") ||
    std::any_of(camera.d.begin(), camera.d.end(), [](double d) {
      return !std::isfinite(d) || std::abs(d) > 1e-9;
    }))
  {
    explain(reason, "pinhole projection requires zero distortion");
    return false;
  }
  if (std::any_of(camera.k.begin(), camera.k.end(), [](double k) {return !std::isfinite(k);}) ||
    camera.k[0] <= 0.0 || camera.k[4] <= 0.0 || camera.k[2] < 0.0 ||
    camera.k[2] >= camera.width || camera.k[5] < 0.0 || camera.k[5] >= camera.height ||
    std::abs(camera.k[1]) > 1e-9 || std::abs(camera.k[3]) > 1e-9 ||
    std::abs(camera.k[6]) > 1e-9 || std::abs(camera.k[7]) > 1e-9 ||
    std::abs(camera.k[8] - 1.0) > 1e-9)
  {
    explain(reason, "camera intrinsic matrix is not a finite canonical pinhole matrix");
    return false;
  }
  return true;
}

std::optional<geometry_msgs::msg::Point> project(
  const DepthEstimate & sample, const sensor_msgs::msg::CameraInfo & camera,
  const sensor_msgs::msg::Image & image, std::string * reason)
{
  if (!valid_calibration(camera, image, reason)) {
    return std::nullopt;
  }
  if (!std::isfinite(sample.depth_m) || sample.depth_m <= 0.0 ||
    !std::isfinite(sample.u) || !std::isfinite(sample.v) || sample.u < 0.0 || sample.v < 0.0 ||
    sample.u >= image.width || sample.v >= image.height)
  {
    explain(reason, "invalid projected pixel or depth");
    return std::nullopt;
  }
  geometry_msgs::msg::Point point;
  point.x = (sample.u - camera.k[2]) * sample.depth_m / camera.k[0];
  point.y = (sample.v - camera.k[5]) * sample.depth_m / camera.k[4];
  point.z = sample.depth_m;
  if (!finite_point(point)) {
    explain(reason, "nonfinite projected optical point");
    return std::nullopt;
  }
  return point;
}

DepthHistory::DepthHistory(std::size_t max_frames, int64_t max_age_ns)
: max_frames_(max_frames), max_age_ns_(max_age_ns)
{
  if (max_frames == 0 || max_age_ns <= 0) {
    throw std::invalid_argument("depth history limits must be positive");
  }
}

void DepthHistory::push(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  if (!image || image->header.frame_id.empty() || stamp_ns(image->header.stamp) <= 0) {
    return;
  }
  const int64_t stamp = stamp_ns(image->header.stamp);
  auto it = std::lower_bound(images_.begin(), images_.end(), stamp,
      [](const auto & stored, int64_t value) {return stamp_ns(stored->header.stamp) < value;});
  // One depth stream is expected. Replace transport duplicates at the same time.
  if (it != images_.end() && stamp_ns((*it)->header.stamp) == stamp) {
    *it = std::move(image);
  } else {
    images_.insert(it, std::move(image));
  }
  const int64_t oldest_allowed = stamp_ns(images_.back()->header.stamp) - max_age_ns_;
  while (!images_.empty() && (images_.size() > max_frames_ ||
    stamp_ns(images_.front()->header.stamp) < oldest_allowed))
  {
    images_.erase(images_.begin());
  }
}

sensor_msgs::msg::Image::ConstSharedPtr DepthHistory::match(
  int64_t source_ns, int64_t tolerance_ns, const std::string & frame) const
{
  if (source_ns <= 0 || tolerance_ns < 0 || frame.empty()) {
    return nullptr;
  }
  sensor_msgs::msg::Image::ConstSharedPtr best;
  int64_t best_delta = std::numeric_limits<int64_t>::max();
  for (const auto & image : images_) {
    if (image->header.frame_id != frame) {
      continue;
    }
    const int64_t delta = std::abs(stamp_ns(image->header.stamp) - source_ns);
    if (delta <= tolerance_ns && delta < best_delta) {
      best = image;
      best_delta = delta;
    }
  }
  return best;
}

void DepthHistory::clear()
{
  images_.clear();
}

TrackStore::TrackStore(
  double merge_radius_m, std::size_t confirmation_count, std::size_t max_tracks)
: merge_radius_m_(merge_radius_m), confirmation_count_(confirmation_count), max_tracks_(max_tracks)
{
  if (!std::isfinite(merge_radius_m) || merge_radius_m <= 0.0 || confirmation_count < 2 ||
    max_tracks == 0 || max_tracks > static_cast<std::size_t>(std::numeric_limits<int32_t>::max()))
  {
    throw std::invalid_argument("invalid track association/confirmation limits");
  }
}

std::optional<Track> TrackStore::observe(
  const geometry_msgs::msg::Point & point, int64_t source_ns)
{
  if (!finite_point(point) || source_ns <= 0) {
    return std::nullopt;
  }
  Track * nearest = nullptr;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (auto & track : tracks_) {
    const double distance = std::hypot(point.x - track.position.x, point.y - track.position.y);
    if (distance <= merge_radius_m_ && distance < nearest_distance) {
      nearest = &track;
      nearest_distance = distance;
    }
  }
  if (nearest) {
    if (source_ns <= nearest->last_source_ns ||
      nearest->observations == std::numeric_limits<std::size_t>::max())
    {
      return std::nullopt;
    }
    ++nearest->observations;
    const double weight = 1.0 / nearest->observations;
    nearest->position.x = nearest->position.x * (1.0 - weight) + point.x * weight;
    nearest->position.y = nearest->position.y * (1.0 - weight) + point.y * weight;
    nearest->position.z = nearest->position.z * (1.0 - weight) + point.z * weight;
    nearest->last_source_ns = source_ns;
    nearest->confirmed = nearest->observations >= confirmation_count_;
    return *nearest;
  }
  if (tracks_.size() >= max_tracks_) {
    return std::nullopt;
  }
  tracks_.push_back(Track{next_id_++, point, 1, source_ns, false});
  return tracks_.back();
}

void TrackStore::clear()
{
  tracks_.clear();
  next_id_ = 0;
}

}  // namespace search_perception

// Stationary perception slice. The autonomous Navigator mission is a later milestone.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "navigation/navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "search/perception.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perception = search_perception;

class Search : public rclcpp::Node
{
public:
  Search()
  : Node("search"), depth_history_(160, 8000000000LL), tracks_(0.6, 2)
  {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::runtime_error("Run search with --ros-args -p use_sim_time:=true");
    }
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/rgbd_camera/depth_image");
    const auto camera_topic = declare_parameter<std::string>(
      "camera_info_topic", "/rgbd_camera/camera_info");
    const auto detections_topic = declare_parameter<std::string>(
      "detections_topic", "/people/detections");
    // These defaults describe the verified single Gazebo RGB-D sensor. Recheck
    // registration before remapping topics; dimensions alone are insufficient.
    optical_frame_ = declare_parameter<std::string>(
      "optical_frame", "oakd_rgb_camera_optical_frame");
    match_tolerance_ = declare_parameter<double>("depth_tolerance_sec", 0.02);
    max_age_ = declare_parameter<double>("max_detection_age_sec", 5.0);
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);
    log_measurements_ = declare_parameter<bool>("log_measurements", false);
    depth_options_.min_depth_m = declare_parameter<double>("min_depth_m", 0.3);
    depth_options_.max_depth_m = declare_parameter<double>("max_depth_m", 100.0);
    if (optical_frame_.empty() || !std::isfinite(match_tolerance_) ||
      match_tolerance_ < 0 || match_tolerance_ > 0.1 ||
      !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 7 ||
      !std::isfinite(min_confidence_) || min_confidence_ < 0 || min_confidence_ > 1 ||
      !std::isfinite(depth_options_.min_depth_m) || depth_options_.min_depth_m <= 0 ||
      !std::isfinite(depth_options_.max_depth_m) ||
      depth_options_.max_depth_m <= depth_options_.min_depth_m)
    {
      throw std::runtime_error("Invalid localization parameters (age must be <=7 s)");
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // One executor owns all perception callbacks, including /tf. Lookups below
    // never block it: unavailable image-time TF is retried by the timer.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
        const auto current = now().nanoseconds();
        const auto source = perception::stamp_ns(image->header.stamp);
        const double age = (current - source) * 1e-9;
        if (image->header.frame_id == optical_frame_ &&
        current > 0 && source > 0 && age >= -0.1 && age <= 8.0)
        {
          depth_history_.push(image);
        } else {
          reject("depth_header_or_age");
        }
      });
    camera_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        const auto source = perception::stamp_ns(info->header.stamp);
        const auto current = now().nanoseconds();
        const double age = (current - source) * 1e-9;
        sensor_msgs::msg::Image geometry;
        geometry.width = info->width;
        geometry.height = info->height;
        geometry.header = info->header;
        if (info->header.frame_id != optical_frame_ || current <= 0 || source <= 0 ||
        source < newest_camera_source_ || age < -0.1 || age > 8.0 ||
        !perception::valid_calibration(*info, geometry))
        {
          reject("invalid_stale_or_reordered_camera_info");
          return;
        }
        if (camera_ && (info->width != camera_->width || info->height != camera_->height ||
        info->k != camera_->k || info->d != camera_->d || info->r != camera_->r ||
        info->p != camera_->p || info->distortion_model != camera_->distortion_model ||
        info->binning_x != camera_->binning_x || info->binning_y != camera_->binning_y ||
        info->roi != camera_->roi))
        {
          reset("camera calibration changed");
          calibration_epoch_ = source;
        }
        if (!camera_) {
          RCLCPP_INFO(get_logger(), "Camera %ux%u %s fx=%.6f fy=%.6f cx=%.3f cy=%.3f",
            info->width, info->height, info->header.frame_id.c_str(),
            info->k[0], info->k[4], info->k[2], info->k[5]);
        }
        camera_ = info;
        newest_camera_source_ = source;
      });
    detections_subscription_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr detections) {
        const auto stamp = perception::stamp_ns(detections->header.stamp);
        if (detections->header.frame_id != optical_frame_ || stamp <= 0 ||
        stamp < calibration_epoch_)
        {
          reject("source_header");
          log_rejection(detections->header, "source_header");
          return;
        }
        const auto current = now().nanoseconds();
        const double age = (current - stamp) * 1e-9;
        if (current <= 0 || age > max_age_ || age < -0.1) {
          reject("stale_future_or_clock_not_ready");
          log_rejection(detections->header, "stale_future_or_clock_not_ready");
          return;
        }
        // Enforce chronological source images. A replay or repeated array must
        // not count as new evidence, even when it contains duplicate boxes.
        if (stamp <= newest_source_) {
          reject("duplicate_or_out_of_order_image");
          log_rejection(detections->header, "duplicate_or_out_of_order_image");
          return;
        }
        newest_source_ = stamp;
        ++images_;
        if (!detections->detections.empty()) {
          if (pending_.size() >= 32) {
            pending_.pop_front();
            reject("pending_overflow");
          }
          pending_.push_back(detections);
        }
      });
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/people/markers", rclcpp::QoS(1).reliable().transient_local());
    // Clear retained markers from an earlier search process on startup.
    clear_markers();
    process_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {process();});
    status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {status();});
    RCLCPP_INFO(get_logger(),
      "Stationary perception only; no motion goals. depth tolerance=%.3fs, max age=%.1fs; "
      "depth history <=160 frames/8s; merge=0.6m, confirm=2 distinct images",
      match_tolerance_, max_age_);
  }

private:
  void log_rejection(const std_msgs::msg::Header & header, const std::string & reason)
  {
    if (log_measurements_) {
      RCLCPP_INFO(get_logger(), "Localization rejection source=%d.%09u reason=%s",
        header.stamp.sec, header.stamp.nanosec, reason.c_str());
    }
  }

  void reject(const std::string & reason)
  {
    ++rejections_[reason];
  }

  void clear_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    marker_publisher_->publish(markers);
  }

  void reset(const char * reason)
  {
    RCLCPP_WARN(get_logger(), "Clearing perception: %s", reason);
    depth_history_.clear();
    pending_.clear();
    tracks_.clear();
    camera_.reset();
    newest_source_ = 0;
    newest_camera_source_ = 0;
    calibration_epoch_ = 0;
    clear_markers();
  }

  void process()
  {
    const auto current = now().nanoseconds();
    if (last_now_ > current) {
      reset("simulation clock moved backwards");
    }
    last_now_ = current;
    for (auto it = pending_.begin(); it != pending_.end(); ) {
      const auto & detections = **it;
      const auto source = perception::stamp_ns(detections.header.stamp);
      const double age = (current - source) * 1e-9;
      if (age > max_age_ || age < -0.1) {
        reject(age > max_age_ ? "stale_image" : "future_image");
        std::string waiting_for = "stale_image";
        if (!camera_) {
          waiting_for = "missing_calibration";
        } else if (!depth_history_.match(
            source, static_cast<int64_t>(match_tolerance_ * 1e9), optical_frame_))
        {
          waiting_for = "unmatched_depth";
        } else if (!tf_buffer_->canTransform(
            "map", optical_frame_, rclcpp::Time(detections.header.stamp)))
        {
          waiting_for = "unavailable_image_time_tf";
        }
        log_rejection(detections.header, age < -0.1 ? "future_image" : waiting_for);
        it = pending_.erase(it);
        continue;
      }
      if (!camera_) {
        break;
      }
      auto depth = depth_history_.match(
        source, static_cast<int64_t>(match_tolerance_ * 1e9), optical_frame_);
      if (!depth) {
        // Depth and detector callbacks can arrive in either order. Retry until
        // max_detection_age_sec, without substituting a newer depth frame.
        break;
      }
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_->lookupTransform(
          "map", optical_frame_, rclcpp::Time(detections.header.stamp));
      } catch (const tf2::TransformException &) {
        break;
      }
      for (const auto & detection : detections.detections) {
        if (detection.header != detections.header) {
          reject("box_header");
          log_rejection(detections.header, "box_header");
          continue;
        }
        const bool person = std::any_of(
          detection.results.begin(), detection.results.end(), [this](const auto & result) {
            return result.hypothesis.class_id == "person" &&
                   std::isfinite(result.hypothesis.score) &&
                   result.hypothesis.score >= min_confidence_ && result.hypothesis.score <= 1;
          });
        if (!person) {
          reject("class_or_confidence");
          log_rejection(detections.header, "class_or_confidence");
          continue;
        }
        std::string reason;
        auto estimate = perception::sample_depth(*depth, detection.bbox, depth_options_, &reason);
        if (!estimate) {
          reject("depth_" + reason);
          log_rejection(detections.header, "depth_" + reason);
          continue;
        }
        auto point = perception::project(*estimate, *camera_, *depth, &reason);
        if (!point) {
          reject("projection_" + reason);
          log_rejection(detections.header, "projection_" + reason);
          continue;
        }
        geometry_msgs::msg::PointStamped optical_point;
        optical_point.header = detections.header;
        optical_point.point = *point;
        geometry_msgs::msg::PointStamped map_point;
        tf2::doTransform(optical_point, map_point, transform);
        auto track = tracks_.observe(map_point.point, source);
        if (!track) {
          reject("duplicate_box_or_invalid_track");
          log_rejection(detections.header, "duplicate_box_or_invalid_track");
          continue;
        }
        ++observations_;
        markers_dirty_ = true;
        RCLCPP_INFO(get_logger(),
          "Observation source=%d.%09u depth_dt_ms=%.3f age=%.3fs support=%zu/%zu "
          "map=(%.3f,%.3f,%.3f) id=%d images=%zu confirmed=%s",
          detections.header.stamp.sec, detections.header.stamp.nanosec,
          (perception::stamp_ns(depth->header.stamp) - source) * 1e-6, age,
          estimate->valid_pixels, estimate->total_pixels,
          map_point.point.x, map_point.point.y, map_point.point.z,
          static_cast<int>(track->id), static_cast<size_t>(track->observations),
          track->confirmed ? "yes" : "no");
      }
      it = pending_.erase(it);
    }
    if (markers_dirty_) {
      publish_markers();
      markers_dirty_ = false;
    }
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    // Every retained message is a complete snapshot, including deletions. A
    // reconnecting RViz can recover even if it missed a reset publication.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (const auto & track : tracks_.tracks()) {
      if (!track.confirmed) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.ns = "confirmed_people";
      marker.id = static_cast<int>(track.id);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = track.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
      marker.color.r = 0.15f;
      marker.color.g = 0.9f;
      marker.color.b = 0.3f;
      marker.color.a = 1.0f;
      marker.text = "person " + std::to_string(track.id) + " (" +
        std::to_string(track.observations) + " images)";
      markers.markers.push_back(marker);
    }
    marker_publisher_->publish(markers);
  }

  void status()
  {
    publish_markers();
    const auto confirmed = std::count_if(
      tracks_.tracks().begin(), tracks_.tracks().end(), [](const auto & t) {return t.confirmed;});
    RCLCPP_INFO(get_logger(),
      "Perception: images=%zu observations=%zu confirmed=%zu pending=%zu depth_buffer=%zu",
      images_, observations_, static_cast<size_t>(confirmed), pending_.size(),
      depth_history_.size());
    for (const auto & entry : rejections_) {
      RCLCPP_INFO(get_logger(), "Rejected %s: %zu", entry.first.c_str(), entry.second);
    }
    if (!pending_.empty()) {
      RCLCPP_INFO(get_logger(), "Waiting for calibration, matching depth or source-time map TF");
    }
  }

  std::string optical_frame_;
  double match_tolerance_, max_age_, min_confidence_;
  int64_t newest_source_{0}, newest_camera_source_{0}, calibration_epoch_{0}, last_now_{0};
  size_t images_{0}, observations_{0};
  bool markers_dirty_{false};
  bool log_measurements_{false};
  perception::DepthOptions depth_options_;
  perception::DepthHistory depth_history_;
  perception::TrackStore tracks_;
  std::map<std::string, size_t> rejections_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_;
  std::deque<vision_msgs::msg::Detection2DArray::ConstSharedPtr> pending_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_subscription_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr process_timer_, status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<Search>();
    // Keep MRTP linked and configured on simulation time. It owns its internal
    // spin calls, so it is deliberately outside the perception executor. No
    // Navigator methods send goals in this milestone.
    auto navigator = std::make_shared<Navigator>(false, false);
    navigator->set_parameter(rclcpp::Parameter("use_sim_time", true));
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("search"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

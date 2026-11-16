// Project starter: verify sensor, detector, Nav2-library and TF connections.
#include <chrono>
#include <memory>
#include <string>

#include "navigation/navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

rclcpp::Node::SharedPtr nodeh;
std::shared_ptr<Navigator> navigator;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription;
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_subscription;
rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_subscription;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
rclcpp::TimerBase::SharedPtr status_timer;

sensor_msgs::msg::Image::ConstSharedPtr latest_depth;
sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera;
vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_detections;
int previous_status = -1;

void depth_callback(sensor_msgs::msg::Image::ConstSharedPtr message)
{
  latest_depth = message;
}

void camera_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
{
  if (!latest_camera) {
    RCLCPP_INFO(
      nodeh->get_logger(),
      "Camera: %ux%u, frame=%s; fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
      message->width, message->height, message->header.frame_id.c_str(),
      message->k[0], message->k[4], message->k[2], message->k[5]);
  }
  latest_camera = message;
}

void detections_callback(vision_msgs::msg::Detection2DArray::ConstSharedPtr message)
{
  latest_detections = message;

  // Next project step: match depth to message->header.stamp, take a median
  // depth in each box, project with camera K, and transform to map at that
  // same stamp. Merge observations within 0.6 m and require repeated sightings.
}

void status_callback()
{
  bool map_tf_ready = false;
  if (latest_camera) {
    try {
      // Latest TF is appropriate for this readiness check only. Person
      // localization must instead use the corresponding image's timestamp.
      tf_buffer->lookupTransform("map", latest_camera->header.frame_id, tf2::TimePointZero);
      map_tf_ready = true;
    } catch (const tf2::TransformException &) {
      map_tf_ready = false;
    }
  }

  const int status = (latest_depth ? 1 : 0) | (latest_camera ? 2 : 0) |
    (latest_detections ? 4 : 0) | (map_tf_ready ? 8 : 0);
  if (status != previous_status) {
    RCLCPP_INFO(
      nodeh->get_logger(), "Starter connections: depth=%s camera_info=%s detections=%s map_TF=%s",
      latest_depth ? "ready" : "waiting", latest_camera ? "ready" : "waiting",
      latest_detections ? "ready" : "waiting", map_tf_ready ? "ready" : "waiting");
    previous_status = status;
  }

  // The topic is ready for RViz. Add confirmed-person markers here when the
  // localization/merge logic is implemented; this starter creates none.
  marker_publisher->publish(visualization_msgs::msg::MarkerArray());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  nodeh = rclcpp::Node::make_shared("search");
  const auto depth_topic = nodeh->declare_parameter<std::string>(
    "depth_topic", "/rgbd_camera/depth_image");
  const auto camera_topic = nodeh->declare_parameter<std::string>(
    "camera_info_topic", "/rgbd_camera/camera_info");
  const auto detections_topic = nodeh->declare_parameter<std::string>(
    "detections_topic", "/people/detections");

  // MRTP Navigator is linked and available. This starter sends no motion
  // goals. Add SetInitialPose/WaitUntilNav2Active/GoToPose/Spin/IsTaskComplete
  // calls and a return-home phase when implementing the waypoint mission.
  navigator = std::make_shared<Navigator>(false, false);
  tf_buffer = std::make_shared<tf2_ros::Buffer>(nodeh->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nodeh, true);
  depth_subscription = nodeh->create_subscription<sensor_msgs::msg::Image>(
    depth_topic, rclcpp::SensorDataQoS(), depth_callback);
  camera_subscription = nodeh->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_topic, rclcpp::SensorDataQoS(), camera_callback);
  detections_subscription = nodeh->create_subscription<vision_msgs::msg::Detection2DArray>(
    detections_topic, rclcpp::SensorDataQoS(), detections_callback);
  marker_publisher = nodeh->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/people/markers", 10);
  status_timer = nodeh->create_wall_timer(std::chrono::seconds(1), status_callback);

  RCLCPP_INFO(
    nodeh->get_logger(),
    "Search starter running. Sensor/TF checks only; autonomous search is not implemented yet.");
  rclcpp::spin(nodeh);

  status_timer.reset();
  depth_subscription.reset();
  camera_subscription.reset();
  detections_subscription.reset();
  tf_listener.reset();
  tf_buffer.reset();
  navigator.reset();
  marker_publisher.reset();
  nodeh.reset();
  rclcpp::shutdown();
  return 0;
}

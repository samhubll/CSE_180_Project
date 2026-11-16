// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "search/perception.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"

namespace sp = search_perception;

namespace
{
sensor_msgs::msg::Image image32(bool big_endian = false, uint32_t padding = 0)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera_optical";
  image.header.stamp.sec = 1;
  image.width = 20;
  image.height = 20;
  image.encoding = "32FC1";
  image.is_bigendian = big_endian;
  image.step = image.width * 4 + padding;
  image.data.resize(image.step * image.height);
  return image;
}

void pixel(sensor_msgs::msg::Image & image, uint32_t u, uint32_t v, double metres)
{
  uint32_t bits;
  const std::size_t bytes = image.encoding == "32FC1" ? 4 : 2;
  if (bytes == 4) {
    const float value = metres;
    std::memcpy(&bits, &value, sizeof(bits));
  } else {
    bits = static_cast<uint16_t>(std::round(metres * 1000.0));
  }
  auto * destination = image.data.data() + image.step * v + bytes * u;
  for (std::size_t i = 0; i < bytes; ++i) {
    const std::size_t shift = image.is_bigendian ? bytes - i - 1 : i;
    destination[i] = (bits >> (8 * shift)) & 0xff;
  }
}

void fill(sensor_msgs::msg::Image & image, double metres)
{
  for (uint32_t v = 0; v < image.height; ++v) {
    for (uint32_t u = 0; u < image.width; ++u) {
      pixel(image, u, v, metres);
    }
  }
}

vision_msgs::msg::BoundingBox2D box(double u = 10, double v = 10, double size = 20)
{
  vision_msgs::msg::BoundingBox2D result;
  result.center.position.x = u;
  result.center.position.y = v;
  result.size_x = size;
  result.size_y = size;
  return result;
}

sensor_msgs::msg::CameraInfo calibration()
{
  sensor_msgs::msg::CameraInfo camera;
  camera.header.frame_id = "camera_optical";
  camera.width = 20;
  camera.height = 20;
  camera.distortion_model = "plumb_bob";
  camera.d = {0, 0, 0, 0, 0};
  camera.k = {10, 0, 10, 0, 20, 10, 0, 0, 1};
  return camera;
}

sensor_msgs::msg::Image::ConstSharedPtr timed_image(int64_t time)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>(image32());
  image->header.stamp.sec = static_cast<int32_t>(time / 1000000000LL);
  image->header.stamp.nanosec = static_cast<uint32_t>(time % 1000000000LL);
  return image;
}

geometry_msgs::msg::Point point(double x, double y = 0, double z = 1)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}
}  // namespace

TEST(Depth, MedianRejectsInvalidValuesAndOutliers)
{
  auto image = image32();
  fill(image, 2);
  pixel(image, 5, 5, 99);
  pixel(image, 6, 5, std::numeric_limits<double>::quiet_NaN());
  pixel(image, 7, 5, std::numeric_limits<double>::infinity());
  pixel(image, 8, 5, 0);
  pixel(image, 9, 5, -1);
  pixel(image, 10, 5, 101);
  const auto sample = sp::sample_depth(image, box());
  ASSERT_TRUE(sample);
  EXPECT_DOUBLE_EQ(sample->depth_m, 2);
  EXPECT_EQ(sample->valid_pixels, 95U);
  EXPECT_EQ(sample->total_pixels, 100U);
  EXPECT_DOUBLE_EQ(sample->u, 9.5);
  EXPECT_DOUBLE_EQ(sample->v, 9.5);
}

TEST(Depth, EvenMedianAveragesCentralValues)
{
  auto image = image32();
  fill(image, 2);
  for (uint32_t v = 10; v < 15; ++v) {
    for (uint32_t u = 5; u < 15; ++u) {
      pixel(image, u, v, 4);
    }
  }
  const auto sample = sp::sample_depth(image, box());
  ASSERT_TRUE(sample);
  EXPECT_DOUBLE_EQ(sample->depth_m, 3);
}

TEST(Depth, HandlesPaddedBigEndianFloatAndMillimetreImages)
{
  for (bool big_endian : {false, true}) {
    auto image = image32(big_endian, 7);
    fill(image, 2.5);
    auto sample = sp::sample_depth(image, box());
    ASSERT_TRUE(sample);
    EXPECT_DOUBLE_EQ(sample->depth_m, 2.5);
    image.encoding = "16UC1";
    image.step = 2 * image.width + 7;
    image.data.resize(image.step * image.height);
    fill(image, 2.5);
    sample = sp::sample_depth(image, box());
    ASSERT_TRUE(sample);
    EXPECT_DOUBLE_EQ(sample->depth_m, 2.5);
  }
}

TEST(Depth, ClipsCentralRegionAndRejectsDisjointSmallOrRotatedBoxes)
{
  auto image = image32();
  fill(image, 2);
  auto sample = sp::sample_depth(image, box(0, 0));
  ASSERT_TRUE(sample);
  EXPECT_EQ(sample->valid_pixels, 25U);
  EXPECT_DOUBLE_EQ(sample->u, 2);
  EXPECT_DOUBLE_EQ(sample->v, 2);
  EXPECT_FALSE(sp::sample_depth(image, box(-100, 10)));
  EXPECT_FALSE(sp::sample_depth(image, box(10, 10, 2)));
  auto rotated = box();
  rotated.center.theta = 0.5;
  EXPECT_FALSE(sp::sample_depth(image, rotated));
  auto invalid = box();
  invalid.size_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sp::sample_depth(image, invalid));
  invalid = box();
  invalid.size_y = -10;
  EXPECT_FALSE(sp::sample_depth(image, invalid));
}

TEST(Depth, RequiresAbsoluteAndFractionalSupport)
{
  auto image = image32();
  fill(image, 0);
  for (uint32_t u = 5; u < 15; ++u) {
    pixel(image, u, 5, 2);
    pixel(image, u, 6, 2);
  }
  EXPECT_FALSE(sp::sample_depth(image, box()));
  sp::DepthOptions options;
  options.min_valid_pixels = 1;
  EXPECT_FALSE(sp::sample_depth(image, box(), options));
  options.min_valid_fraction = 0.2;
  EXPECT_TRUE(sp::sample_depth(image, box(), options));
  options.min_valid_fraction = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sp::sample_depth(image, box(), options));
}

TEST(Depth, RejectsMalformedLayoutEncodingAndAllInvalidPixels)
{
  auto image = image32();
  EXPECT_FALSE(sp::sample_depth(image, box()));
  fill(image, 2);
  image.data.pop_back();
  EXPECT_FALSE(sp::sample_depth(image, box()));
  image = image32();
  image.step = 1;
  EXPECT_FALSE(sp::sample_depth(image, box()));
  image = image32();
  image.encoding = "rgb8";
  EXPECT_FALSE(sp::sample_depth(image, box()));
  image = image32();
  image.width = 0;
  EXPECT_FALSE(sp::sample_depth(image, box()));
}

TEST(Projection, PinholeAxesAndMetricUnits)
{
  const auto result = sp::project({2, 15, 5, 100, 100}, calibration(), image32());
  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->x, 1);
  EXPECT_DOUBLE_EQ(result->y, -0.5);
  EXPECT_DOUBLE_EQ(result->z, 2);
}

TEST(Projection, RejectsInvalidPixelsIntrinsicsFramesDistortionAndBinning)
{
  auto camera = calibration();
  auto image = image32();
  const sp::DepthEstimate sample{2, 15, 5, 100, 100};
  std::string reason;
  camera.k[0] = 0;
  EXPECT_FALSE(sp::project(sample, camera, image, &reason));
  EXPECT_FALSE(reason.empty());
  camera = calibration();
  camera.k[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sp::project(sample, camera, image));
  camera = calibration();
  camera.k[1] = 0.1;
  EXPECT_FALSE(sp::project(sample, camera, image));
  camera = calibration();
  camera.header.frame_id = "another_camera";
  EXPECT_FALSE(sp::project(sample, camera, image));
  camera = calibration();
  camera.width = 40;
  EXPECT_FALSE(sp::project(sample, camera, image));
  camera = calibration();
  camera.d[0] = 0.1;
  EXPECT_FALSE(sp::project(sample, camera, image));
  camera = calibration();
  camera.binning_x = 2;
  EXPECT_FALSE(sp::project(sample, camera, image));
  camera = calibration();
  camera.roi.x_offset = 1;
  EXPECT_FALSE(sp::project(sample, camera, image));
  EXPECT_FALSE(sp::project({-1, 15, 5, 1, 1}, calibration(), image));
  EXPECT_FALSE(sp::project({2, 20, 5, 1, 1}, calibration(), image));
  EXPECT_FALSE(sp::project(
      {2, 15, std::numeric_limits<double>::infinity(), 1, 1}, calibration(), image));
}

TEST(Projection, NonidentityOpticalToMapTransformPreservesSourceStamp)
{
  const auto optical_point = sp::project({3, 15, 5, 100, 100}, calibration(), image32());
  ASSERT_TRUE(optical_point);
  geometry_msgs::msg::PointStamped source;
  source.header.frame_id = "camera_optical";
  source.header.stamp.sec = 10;
  source.header.stamp.nanosec = 123;
  source.point = *optical_point;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = source.header.stamp;
  transform.child_frame_id = "camera_optical";
  transform.transform.translation.x = 10;
  transform.transform.translation.y = 20;
  transform.transform.translation.z = 30;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);
  geometry_msgs::msg::PointStamped mapped;
  tf2::doTransform(source, mapped, transform);
  EXPECT_NEAR(mapped.point.x, 10.75, 1e-12);
  EXPECT_NEAR(mapped.point.y, 21.5, 1e-12);
  EXPECT_NEAR(mapped.point.z, 33, 1e-12);
  EXPECT_EQ(mapped.header.frame_id, "map");
  EXPECT_EQ(mapped.header.stamp, source.header.stamp);
}

TEST(Timestamps, MalformedOrZeroNeverRequestsLatestTransform)
{
  builtin_interfaces::msg::Time stamp;
  EXPECT_EQ(sp::stamp_ns(stamp), 0);
  stamp.sec = -1;
  EXPECT_EQ(sp::stamp_ns(stamp), -1);
  stamp.sec = 1;
  stamp.nanosec = 1000000000;
  EXPECT_EQ(sp::stamp_ns(stamp), -1);
  sp::DepthHistory history;
  history.push(timed_image(0));
  EXPECT_EQ(history.size(), 0U);
  EXPECT_FALSE(history.match(0, 10000000, "camera_optical"));
}

TEST(Timestamps, HistoricalRotatingTransformDiffersFromLatestAndMissingTimeThrows)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_optical";
  transform.header.stamp.sec = 10;
  transform.transform.rotation.w = 1;
  ASSERT_TRUE(buffer.setTransform(transform, "test", false));
  transform.header.stamp.sec = 12;
  transform.transform.translation.x = 2;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);
  ASSERT_TRUE(buffer.setTransform(transform, "test", false));
  geometry_msgs::msg::PointStamped source;
  source.header.frame_id = "camera_optical";
  source.header.stamp.sec = 11;
  source.point = point(1, 0, 2);
  const auto historical = buffer.lookupTransform(
    "map", source.header.frame_id, rclcpp::Time(source.header.stamp));
  const auto latest = buffer.lookupTransform(
    "map", source.header.frame_id, tf2::TimePointZero);
  geometry_msgs::msg::PointStamped mapped;
  tf2::doTransform(source, mapped, historical);
  EXPECT_NEAR(mapped.point.x, 1 + std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(mapped.point.y, std::sqrt(0.5), 1e-12);
  EXPECT_EQ(mapped.header.stamp, source.header.stamp);
  geometry_msgs::msg::PointStamped wrongly_latest;
  tf2::doTransform(source, wrongly_latest, latest);
  EXPECT_NEAR(wrongly_latest.point.x, 2, 1e-12);
  EXPECT_NEAR(wrongly_latest.point.y, 1, 1e-12);
  EXPECT_GT(std::hypot(mapped.point.x - wrongly_latest.point.x,
    mapped.point.y - wrongly_latest.point.y), 0.4);
  EXPECT_THROW(buffer.lookupTransform("map", "camera_optical", rclcpp::Time(9, 0)),
    tf2::TransformException);
}

TEST(Timestamps, NearestMatchIncludesToleranceBoundaryAndUsesEarlierTie)
{
  sp::DepthHistory history;
  history.push(timed_image(1000000000));
  history.push(timed_image(1020000000));
  const auto match = history.match(1010000000, 10000000, "camera_optical");
  ASSERT_TRUE(match);
  EXPECT_EQ(sp::stamp_ns(match->header.stamp), 1000000000);
  EXPECT_FALSE(history.match(1010000000, 9999999, "camera_optical"));
  EXPECT_FALSE(history.match(1010000000, 10000000, "wrong_frame"));
  EXPECT_FALSE(history.match(1010000000, -1, "camera_optical"));
}

TEST(Timestamps, HistoryBoundsAgeAndCountEvenForOutOfOrderArrival)
{
  sp::DepthHistory history(2, 100000000);
  history.push(timed_image(1000000000));
  history.push(timed_image(1100000000));
  history.push(timed_image(1090000000));
  EXPECT_EQ(history.size(), 2U);
  EXPECT_FALSE(history.match(1000000000, 0, "camera_optical"));
  history.push(timed_image(900000000));
  EXPECT_EQ(history.size(), 2U);
  EXPECT_FALSE(history.match(900000000, 0, "camera_optical"));
  history.push(timed_image(2000000000));
  EXPECT_EQ(history.size(), 1U);
  history.push(timed_image(2000000000));
  EXPECT_EQ(history.size(), 1U);
  history.clear();
  EXPECT_EQ(history.size(), 0U);
}

TEST(Tracks, DistinctImagesConfirmRunningMeanAndKeepStableId)
{
  sp::TrackStore store;
  const auto first = store.observe(point(1, 2, 0.5), 100);
  ASSERT_TRUE(first);
  EXPECT_FALSE(first->confirmed);
  EXPECT_EQ(first->observations, 1U);
  const auto second = store.observe(point(1.2, 2.2, 1.5), 200);
  ASSERT_TRUE(second);
  EXPECT_EQ(first->id, second->id);
  EXPECT_TRUE(second->confirmed);
  EXPECT_EQ(second->observations, 2U);
  EXPECT_DOUBLE_EQ(second->position.x, 1.1);
  EXPECT_DOUBLE_EQ(second->position.y, 2.1);
  EXPECT_DOUBLE_EQ(second->position.z, 1);
}

TEST(Tracks, SameOrOlderImageCannotConfirmOrCreateDuplicateTrack)
{
  sp::TrackStore store;
  ASSERT_TRUE(store.observe(point(0), 100));
  EXPECT_FALSE(store.observe(point(0.1), 100));
  EXPECT_FALSE(store.observe(point(0.2), 90));
  ASSERT_EQ(store.tracks().size(), 1U);
  EXPECT_EQ(store.tracks().front().observations, 1U);
  EXPECT_FALSE(store.tracks().front().confirmed);
  EXPECT_DOUBLE_EQ(store.tracks().front().position.x, 0);
  EXPECT_TRUE(store.observe(point(2), 100));
  EXPECT_EQ(store.tracks().size(), 2U);
}

TEST(Tracks, MergeBoundaryIsInclusiveInHorizontalMapPlane)
{
  sp::TrackStore at_boundary;
  const auto first = at_boundary.observe(point(0), 100);
  const auto second = at_boundary.observe(point(0.6, 0, 5), 200);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_EQ(first->id, second->id);
  EXPECT_EQ(at_boundary.tracks().size(), 1U);
  sp::TrackStore outside;
  ASSERT_TRUE(outside.observe(point(0), 100));
  ASSERT_TRUE(outside.observe(point(0.600001), 200));
  EXPECT_EQ(outside.tracks().size(), 2U);
}

TEST(Tracks, NearestAssociationAndBoundedCapacityRejectBadInput)
{
  sp::TrackStore store(0.6, 2, 2);
  const auto first = store.observe(point(0), 100);
  const auto second = store.observe(point(1), 100);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  const auto third = store.observe(point(0.55), 200);
  ASSERT_TRUE(third);
  EXPECT_EQ(third->id, second->id);
  EXPECT_FALSE(store.observe(point(3), 300));
  EXPECT_FALSE(store.observe(point(std::numeric_limits<double>::quiet_NaN()), 300));
  EXPECT_FALSE(store.observe(point(3), 0));
  EXPECT_EQ(store.tracks().size(), 2U);
  store.clear();
  EXPECT_EQ(store.tracks().size(), 0U);
  EXPECT_EQ(store.observe(point(0), 400)->id, 0);
}

TEST(Configuration, InvalidMemoryAndAssociationLimitsAreRejected)
{
  EXPECT_THROW(sp::DepthHistory(0, 100), std::invalid_argument);
  EXPECT_THROW(sp::DepthHistory(2, 0), std::invalid_argument);
  EXPECT_THROW(sp::TrackStore(-0.6), std::invalid_argument);
  EXPECT_THROW(sp::TrackStore(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(sp::TrackStore(0.6, 1), std::invalid_argument);
  EXPECT_THROW(sp::TrackStore(0.6, 2, 0), std::invalid_argument);
}

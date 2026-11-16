// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
#include <cstdio>
#include <fstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "search/coverage_plan.hpp"

// Exercise the same offline ray/clearance implementation used for the artifact.
// This compilation unit excludes the tool's command-line entry point.
#define SEARCH_COVERAGE_MAP_NO_MAIN
#include "coverage_map.cpp"

namespace
{
class SyntheticMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pixels.assign(31 * 31, 254);
    for (int i = 0; i < 31; ++i) {
      pixels[i] = pixels[30 * 31 + i] = pixels[i * 31] = pixels[i * 31 + 30] = 0;
    }
  }
  void TearDown() override {std::remove(path.c_str());}
  Grid save()
  {
    std::ofstream out(path, std::ios::binary);
    out << "P5\n# synthetic geometry; no simulation or person data\n31 31\n255\n";
    out.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
    out.close();
    return Grid(path);
  }
  const std::string path = "/tmp/cse180-search-coverage-unit-map.pgm";
  std::vector<uint8_t> pixels;
};
}  // namespace

TEST(CoveragePlan, EightViewsHaveNoAngularBlindSector)
{
  // Include both sides of the -pi/pi seam and an arbitrary arrival yaw.
  for (double yaw : {-3.12, -0.7, 0.0, 1.83, 3.12}) {
    for (int degree = -180; degree <= 180; ++degree) {
      const double bearing = degree * sc::kPi / 180.0;
      bool visible = false;
      for (double offset : sc::kScanOffsetsRad) {
        visible = visible || sc::in_view(bearing, yaw + offset);
      }
      EXPECT_TRUE(visible) << "yaw=" << yaw << " bearing=" << degree;
    }
  }
  EXPECT_TRUE(sc::in_view(-sc::kPi + 0.05, sc::kPi - 0.05));
  EXPECT_FALSE(sc::in_view(sc::kPi, 0));
  // Alternating opposite errors create the largest adjacent-heading gaps.
  for (double error_sign : {-1.0, 1.0}) {
    for (int degree = -180; degree <= 180; ++degree) {
      bool visible = false;
      for (std::size_t h = 0; h < sc::kScanOffsetsRad.size(); ++h) {
        const double error = (h % 2 ? error_sign : -error_sign) * sc::kMaximumHeadingErrorRad;
        visible = visible || sc::in_view(degree * sc::kPi / 180.0, sc::kScanOffsetsRad[h] + error);
      }
      EXPECT_TRUE(visible) << "bounded heading errors leave bearing=" << degree;
    }
  }
}

TEST(CoveragePlan, RouteHasDistinctFiniteStopsAndRequiresTwoImages)
{
  std::set<std::pair<double, double>> positions;
  ASSERT_GT(sc::warehouse_waypoints().size(), 1U);
  for (const auto & w : sc::warehouse_waypoints()) {
    EXPECT_TRUE(std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.yaw));
    EXPECT_TRUE(std::isfinite(w.position_tolerance_m) && w.position_tolerance_m > 0.0 &&
      w.position_tolerance_m < sc::kPlanningClearanceM);
    EXPECT_TRUE(positions.emplace(w.x, w.y).second);
  }
  EXPECT_GE(sc::kRequiredImagesPerHeading, 2U);
}

TEST_F(SyntheticMap, RangeAndShelfOcclusionAreBothRequired)
{
  Grid open = save();
  const Point a = open.point(15 * 31 + 5), b = open.point(15 * 31 + 25);
  EXPECT_TRUE(open.visible(a, b, 0.61));
  EXPECT_FALSE(open.visible(a, b, 0.59));
  for (int y = 1; y < 30; ++y) {
    pixels[y * 31 + 15] = 0;
  }
  Grid wall = save();
  EXPECT_FALSE(wall.visible(a, b, 1.0));
  // A connected floor around an occluder does not imply line of sight.
  pixels[2 * 31 + 15] = 254;
  Grid doorway = save();
  auto component = doorway.connected(a, 0.001);
  EXPECT_TRUE(component[doorway.index(b)]);
  EXPECT_FALSE(doorway.visible(a, b, 1.0));
}

TEST_F(SyntheticMap, UnknownCellsOccludeAndCornerTouchIsConservative)
{
  Grid open = save();
  const Point a = open.point(10 * 31 + 10), b = open.point(20 * 31 + 20);
  pixels[15 * 31 + 15] = 205;
  EXPECT_FALSE(save().visible(a, b, 1.0));
  pixels[15 * 31 + 15] = 254;
  pixels[14 * 31 + 15] = 0;
  EXPECT_FALSE(save().visible(a, b, 1.0));
}

TEST_F(SyntheticMap, ClearanceErosionRejectsNarrowPassages)
{
  for (int y = 1; y < 30; ++y) {
    pixels[y * 31 + 15] = 0;
  }
  pixels[15 * 31 + 15] = 254;
  Grid grid = save();
  const Point a = grid.point(15 * 31 + 8), b = grid.point(15 * 31 + 22);
  EXPECT_TRUE(grid.connected(a, 0.001)[grid.index(b)]);
  EXPECT_FALSE(grid.connected(a, 0.04)[grid.index(b)]);
  // Exact center distance minus the blocked-cell half diagonal is conservative.
  EXPECT_NEAR(grid.clearances[10 * 31 + 8], 0.21 - 0.03 / std::sqrt(2.0), 1e-10);
}

TEST_F(SyntheticMap, RayAlongCellBoundaryChecksBothSides)
{
  Grid grid = save();
  const double boundary_x = grid.origin_x + 10 * grid.resolution;
  const Point a{boundary_x, grid.point(5 * 31 + 10).y};
  const Point b{boundary_x, grid.point(25 * 31 + 10).y};
  EXPECT_TRUE(grid.visible(a, b, 1.0));
  pixels[15 * 31 + 9] = 0;
  EXPECT_FALSE(save().visible(a, b, 1.0));
  pixels[15 * 31 + 9] = 254;
  pixels[15 * 31 + 10] = 205;
  EXPECT_FALSE(save().visible(a, b, 1.0));
}

TEST_F(SyntheticMap, PersonRangeAnnulusIsSeparateFromDepthClip)
{
  const int side = 2 * static_cast<int>(std::ceil((sc::kUsableRangeM + 2) / 0.03)) + 1;
  std::vector<uint8_t> open_pixels(side * side, 254);
  std::ofstream out(path, std::ios::binary);
  out << "P5\n" << side << " " << side << "\n255\n";
  out.write(reinterpret_cast<const char *>(open_pixels.data()), open_pixels.size());
  out.close();
  Grid grid(path);
  const Point stop = grid.point((side / 2) * side + side / 2);
  const Point too_near{stop.x + sc::kMinimumUsableRangeM - 0.2, stop.y};
  const Point usable{stop.x + (sc::kMinimumUsableRangeM + sc::kUsableRangeM) / 2, stop.y};
  const Point too_far{stop.x + sc::kUsableRangeM + 0.2, stop.y};
  EXPECT_TRUE(grid.visible(stop, too_near, sc::kUsableRangeM));
  EXPECT_FALSE(view_visible(grid, stop, 0.0, too_near, sc::kUsableRangeM));
  EXPECT_TRUE(view_visible(grid, stop, 0.0, usable, sc::kUsableRangeM));
  EXPECT_FALSE(view_visible(grid, stop, 0.0, too_far, sc::kUsableRangeM));
}

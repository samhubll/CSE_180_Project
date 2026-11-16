// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
#ifndef SEARCH__COVERAGE_PLAN_HPP_
#define SEARCH__COVERAGE_PLAN_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace search_coverage
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHorizontalFovRad = 1.25;
constexpr double kPlanningClearanceM = 0.45;
constexpr double kMinimumUsableRangeM = 3.2;
constexpr double kUsableRangeM = 8.0;  // Conditional measured bound; see detector-range.md.
constexpr std::size_t kRequiredImagesPerHeading = 2;
constexpr double kHeadingSettleSimSec = 0.5;
constexpr double kMinimumViewDwellSimSec = 2.0;
constexpr double kMaximumHeadingErrorRad = 10.0 * kPi / 180.0;  // map <- base_link yaw.
constexpr std::array<double, 8> kScanOffsetsRad = {
  0.0, kPi / 4, kPi / 2, 3 * kPi / 4, kPi, 5 * kPi / 4, 3 * kPi / 2, 7 * kPi / 4};

struct Waypoint
{
  double x;
  double y;
  double yaw;
  double position_tolerance_m = 0.25;  // Actual map xy gate before future scan credit.
};

inline double wrap_angle(double angle)
{
  return std::remainder(angle, 2 * kPi);
}

inline bool in_view(double bearing, double heading, double fov = kHorizontalFovRad)
{
  return std::isfinite(bearing) && std::isfinite(heading) && std::isfinite(fov) &&
         fov > 0.0 && fov <= 2 * kPi && std::abs(wrap_angle(bearing - heading)) <= fov / 2;
}

// Planning data only. The perception executable does not send these goals.
inline const std::vector<Waypoint> & warehouse_waypoints()
{
  static const std::vector<Waypoint> waypoints = {
    {2.500, -21.500, 0.000},
    {8.500, -21.500, -0.785},
    {10.500, -23.500, 1.429},
    {11.500, -16.500, 2.820},
    {8.500, -15.500, 1.406},
    {9.500, -9.500, 1.751},
    {7.500, 1.500, 1.816},
    {6.500, 5.500, 0.278},
    {13.500, 7.500, 1.816},
    {12.500, 11.500, 1.249},
    {13.500, 14.500, 2.498},
    {9.500, 17.500, -2.820},
    {-11.500, 10.500, 2.356},
    {-13.500, 12.500, 1.571},
    {-13.500, 13.500, 1.571},
    {-13.500, 17.500, 1.571},
    {-13.500, 22.500, 0.464},
    {-11.500, 23.500, 0.000},
    {-10.250, 24.700, kPi / 8, 0.05},
    {-8.500, 23.500, -1.571},
    {-8.500, 20.500, 0.000},
    {-7.500, 20.500, 0.000},
    {-6.500, 20.500, 0.000},
    {-6.350, 19.750, 0.000, 0.05},
    {-5.500, 20.500, 1.571},
    {-5.500, 23.500, 0.000},
    {-3.350, 24.550, 0.000, 0.05},
    {-2.500, 23.500, 0.000},
    {2.500, 23.500, -0.197},
    {7.500, 22.500, -2.531},
    {-2.500, 15.500, -0.785},
    {1.500, 11.500, 3.142},
    {-2.500, 11.500, 2.820},
    {-5.500, 12.500, -2.111},
    {-8.500, 7.500, -1.681},
    {-9.500, -1.500, -1.951},
    {-13.500, -11.500, -1.571},
    {-13.500, -13.500, -1.571},
    {-13.500, -18.500, -1.571},
    {-13.500, -23.500, 0.000},
    {-7.500, -23.500, 0.588},
    {-4.500, -21.500, 1.571},
    {-4.500, -17.500, 1.571},
    {-4.500, -12.500, 1.571},
    {-4.500, -3.500, 2.897},
    {-8.500, -2.500, 1.190},
    {-6.500, 2.500, -0.219},
    {2.500, 0.500, -0.540},
    {7.500, -2.500, -1.989},
    {3.500, -11.500, 3.142},
    {0.500, -11.500, -1.571},
    {0.500, -16.500, -1.245},
  };
  return waypoints;
}
}  // namespace search_coverage

#endif  // SEARCH__COVERAGE_PLAN_HPP_

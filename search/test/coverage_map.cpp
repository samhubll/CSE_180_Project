// Copyright 2026 CSE180 student
// SPDX-License-Identifier: Apache-2.0
// Offline warehouse geometry inspection: no ROS, no robot commands, no person data.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "search/coverage_plan.hpp"

namespace sc = search_coverage;
namespace
{
struct Point {double x; double y;};
struct FloorPoint {Point p; int index; bool navigable;};

std::string token(std::istream & stream)
{
  std::string value;
  while (stream >> value) {
    if (value[0] != '#') {return value;}
    std::string comment;
    std::getline(stream, comment);
  }
  throw std::runtime_error("truncated PGM header");
}

// Exact separable squared Euclidean distance transform, cell-center distances.
std::vector<double> distance_transform_1d(const std::vector<double> & f)
{
  const int n = static_cast<int>(f.size());
  std::vector<int> v(n);
  std::vector<double> z(n + 1), d(n);
  int k = 0;
  v[0] = 0;
  z[0] = -1e30;
  z[1] = 1e30;
  for (int q = 1; q < n; ++q) {
    double s;
    do {
      const int p = v[k];
      s = ((f[q] + q * q) - (f[p] + p * p)) / (2.0 * (q - p));
      if (s <= z[k]) {--k;} else {break;}
    } while (k >= 0);
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e30;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    const double delta = q - v[k];
    d[q] = delta * delta + f[v[k]];
  }
  return d;
}

class Grid
{
public:
  explicit Grid(const std::string & path)
  {
    std::ifstream stream(path, std::ios::binary);
    if (!stream || token(stream) != "P5") {throw std::runtime_error("expected P5 map");}
    width = std::stoi(token(stream));
    height = std::stoi(token(stream));
    if (std::stoi(token(stream)) != 255) {throw std::runtime_error("expected 8-bit map");}
    stream.get();
    pixels.resize(width * height);
    stream.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
    if (!stream) {throw std::runtime_error("truncated map pixels");}
    clearances.resize(pixels.size());
    std::vector<double> row(width), col(height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {row[x] = free(x, y) ? 1e12 : 0.0;}
      auto d = distance_transform_1d(row);
      for (int x = 0; x < width; ++x) {clearances[y * width + x] = d[x];}
    }
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {col[y] = clearances[y * width + x];}
      auto d = distance_transform_1d(col);
      for (int y = 0; y < height; ++y) {
        // Subtract the blocked-cell half diagonal: clearance is to its area.
        const double boundary = std::min(
          {x + 0.5, width - x - 0.5,
            y + 0.5, height - y - 0.5}) * resolution;
        clearances[y * width + x] = std::max(
          0.0, std::min(
            boundary,
            std::sqrt(d[y]) * resolution - resolution / std::sqrt(2.0)));
      }
    }
  }

  int index(Point p) const
  {
    const int x = static_cast<int>(std::floor((p.x - origin_x) / resolution));
    const int y = height - 1 - static_cast<int>(std::floor((p.y - origin_y) / resolution));
    return x < 0 || x >= width || y < 0 || y >= height ? -1 : y * width + x;
  }
  Point point(int i) const
  {
    return {origin_x + (i % width + 0.5) * resolution,
      origin_y + (height - i / width - 0.5) * resolution};
  }
  bool free(int x, int y) const
  {
    // map_server trinary free_thresh=0.1, occupied_thresh=0.65, negate=0.
    return x >= 0 && y >= 0 && x < width && y < height && pixels[y * width + x] > 229;
  }
  bool safe(int i, double clearance) const
  {
    return i >= 0 && i < width * height && clearances[i] >= clearance;
  }
  double point_clearance(Point p) const
  {
    const int i = index(p);
    if (i < 0) {return 0.0;}
    const Point center = point(i);
    return std::max(0.0, clearances[i] - std::hypot(p.x - center.x, p.y - center.y));
  }
  std::vector<uint8_t> connected(Point start, double clearance) const
  {
    std::vector<uint8_t> visited(pixels.size());
    const int first = index(start);
    if (!safe(first, clearance)) {throw std::runtime_error("seed lacks clearance");}
    std::queue<int> queue;
    visited[first] = 1;
    queue.push(first);
    while (!queue.empty()) {
      const int i = queue.front();
      queue.pop();
      const int x = i % width, y = i / width;
      for (const auto & delta : std::array<std::pair<int, int>, 4>{
          {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}})
      {
        const int nx = x + delta.first, ny = y + delta.second;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
        const int next = ny * width + nx;
        if (!visited[next] && safe(next, clearance)) {
          visited[next] = 1;
          queue.push(next);
        }
      }
    }
    return visited;
  }

  bool visible(Point a, Point b, double range) const
  {
    const double dx = b.x - a.x, dy = b.y - a.y;
    if (std::hypot(dx, dy) > range) {return false;}
    // Supercover grid traversal: touching an unknown/occupied cell blocks sight.
    const int ai = index(a), bi = index(b);
    if (ai < 0 || bi < 0) {return false;}
    int x = ai % width, y = ai / width;
    const int bx = bi % width, by = bi / width;
    if (!free(x, y)) {return false;}
    const double gx = (a.x - origin_x) / resolution;
    const double gy = height - (a.y - origin_y) / resolution;
    const double vx = dx / resolution, vy = -dy / resolution;
    const auto on_boundary = [](double value) {
        return std::abs(value - std::round(value)) < 1e-9;
      };
    const auto endpoint_free = [this, &on_boundary](Point p) {
        const double px = (p.x - origin_x) / resolution;
        const double py = height - (p.y - origin_y) / resolution;
        const int xmin = on_boundary(px) ? static_cast<int>(std::round(px)) - 1 :
          static_cast<int>(std::floor(px));
        const int xmax = on_boundary(px) ? xmin + 1 : xmin;
        const int ymin = on_boundary(py) ? static_cast<int>(std::round(py)) - 1 :
          static_cast<int>(std::floor(py));
        const int ymax = on_boundary(py) ? ymin + 1 : ymin;
        return free(xmin, ymin) && free(xmin, ymax) && free(xmax, ymin) && free(xmax, ymax);
      };
    if (!endpoint_free(a) || !endpoint_free(b)) {return false;}
    const auto cell_free = [this, gx, gy, vx, vy, &on_boundary](int cx, int cy) {
        if (!free(cx, cy)) {return false;}
        if (vx == 0 && on_boundary(gx)) {
          const int boundary = static_cast<int>(std::round(gx));
          if (!free(boundary - 1, cy) || !free(boundary, cy)) {return false;}
        }
        if (vy == 0 && on_boundary(gy)) {
          const int boundary = static_cast<int>(std::round(gy));
          if (!free(cx, boundary - 1) || !free(cx, boundary)) {return false;}
        }
        return true;
      };
    const int sx = vx > 0 ? 1 : -1, sy = vy > 0 ? 1 : -1;
    const double tx_delta = vx == 0 ? 1e30 : 1.0 / std::abs(vx);
    const double ty_delta = vy == 0 ? 1e30 : 1.0 / std::abs(vy);
    double tx = vx == 0 ? 1e30 : ((sx > 0 ? x + 1 - gx : gx - x) / std::abs(vx));
    double ty = vy == 0 ? 1e30 : ((sy > 0 ? y + 1 - gy : gy - y) / std::abs(vy));
    while (x != bx || y != by) {
      if (std::abs(tx - ty) < 1e-10) {
        if (!cell_free(x + sx, y) || !cell_free(x, y + sy)) {return false;}
        x += sx;
        y += sy;
        tx += tx_delta;
        ty += ty_delta;
      } else if (tx < ty) {x += sx; tx += tx_delta;} else {y += sy; ty += ty_delta;}
      if (!cell_free(x, y)) {return false;}
    }
    return true;
  }

  int width = 0, height = 0;
  double resolution = 0.03, origin_x = -15.1, origin_y = -25.0;
  std::vector<uint8_t> pixels;
  std::vector<double> clearances;
};

double dist(Point a, Point b) {return std::hypot(a.x - b.x, a.y - b.y);}

bool view_visible(
  const Grid & grid, Point stop, double yaw, Point target, double range,
  double minimum_range = sc::kMinimumUsableRangeM)
{
  if (dist(stop, target) > range + 0.06) {return false;}
  for (double offset : sc::kScanOffsetsRad) {
    const double heading = yaw + offset;
    const Point camera{stop.x - 0.0596 * std::cos(heading),
      stop.y - 0.0596 * std::sin(heading)};
    const double bearing = std::atan2(target.y - camera.y, target.x - camera.x);
    const double optical_depth = (target.x - camera.x) * std::cos(heading) +
      (target.y - camera.y) * std::sin(heading);
    if (dist(camera, target) >= minimum_range && optical_depth >= 0.3 &&
      sc::in_view(bearing, heading) &&
      grid.visible(camera, target, range)) {return true;}
  }
  return false;
}

std::vector<std::vector<double>> path_distances(
  const Grid & grid, const std::vector<uint8_t> & component, const std::vector<Point> & points)
{
  std::vector<std::vector<double>> lengths(points.size(), std::vector<double>(points.size()));
  for (std::size_t p = 0; p < points.size(); ++p) {
    std::vector<int> steps(grid.pixels.size(), -1);
    std::vector<int> queue;
    queue.reserve(grid.pixels.size());
    const int first = grid.index(points[p]);
    queue.push_back(first);
    steps[first] = 0;
    for (std::size_t q = 0; q < queue.size(); ++q) {
      const int i = queue[q], x = i % grid.width, y = i / grid.width;
      for (const auto & delta : std::array<std::pair<int, int>, 4>{
          {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}})
      {
        const int nx = x + delta.first, ny = y + delta.second;
        if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {continue;}
        const int next = ny * grid.width + nx;
        if (component[next] && steps[next] < 0) {
          steps[next] = steps[i] + 1;
          queue.push_back(next);
        }
      }
    }
    for (std::size_t r = 0; r < points.size(); ++r) {
      lengths[p][r] = steps[grid.index(points[r])] * grid.resolution;
    }
  }
  return lengths;
}

[[maybe_unused]] std::vector<Point> generate(
  const Grid & grid, const std::vector<uint8_t> & component,
  const std::vector<FloorPoint> & floor, double range)
{
  std::vector<Point> candidates;
  for (double y = -23.5; y <= 24; y += 1.0) {
    for (double x = -13.5; x <= 14; x += 1.0) {
      const int i = grid.index({x, y});
      if (i >= 0 && component[i] && grid.point_clearance({x, y}) >= 0.70) {
        candidates.push_back({x, y});
      }
    }
  }
  std::vector<std::vector<int>> views(candidates.size());
  for (std::size_t c = 0; c < candidates.size(); ++c) {
    for (std::size_t f = 0; f < floor.size(); ++f) {
      if (view_visible(grid, candidates[c], 0.0, floor[f].p, range)) {
        views[c].push_back(static_cast<int>(f));
      }
    }
  }
  std::vector<bool> covered(floor.size());
  std::vector<Point> route;
  std::size_t count = 0;
  while (count < floor.size() * 0.995 && route.size() < 160) {
    std::size_t best = 0, gain = 0;
    for (std::size_t c = 0; c < candidates.size(); ++c) {
      const std::size_t unseen = std::count_if(
        views[c].begin(), views[c].end(),
        [&covered](int f) {return !covered[f];});
      if (unseen > gain) {gain = unseen; best = c;}
    }
    if (gain < 4) {break;}
    route.push_back(candidates[best]);
    for (int f : views[best]) {
      covered[f] = true;
    }
    count += gain;
  }
  // Order by shortest four-connected paths through the cleared map, so shelves
  // affect both candidate visibility and travel order.
  route.push_back({2.12, -21.3});
  const auto lengths = path_distances(grid, component, route);
  const std::size_t home = route.size() - 1;
  std::vector<std::size_t> remaining, order;
  for (std::size_t i = 0; i < home; ++i) {
    remaining.push_back(i);
  }
  std::size_t previous = home;
  while (!remaining.empty()) {
    auto closest = std::min_element(
      remaining.begin(), remaining.end(),
      [&lengths, previous](std::size_t a, std::size_t b) {
        return lengths[previous][a] < lengths[previous][b];
      });
    previous = *closest;
    order.push_back(previous);
    remaining.erase(closest);
  }
  for (int pass = 0; pass < 20; ++pass) {
    bool improved = false;
    for (std::size_t i = 0; i + 1 < order.size(); ++i) {
      for (std::size_t j = i + 1; j < order.size(); ++j) {
        const std::size_t before = i == 0 ? home : order[i - 1];
        const std::size_t after = j + 1 == order.size() ? home : order[j + 1];
        if (lengths[before][order[j]] + lengths[order[i]][after] + 0.01 <
          lengths[before][order[i]] + lengths[order[j]][after])
        {
          std::reverse(order.begin() + i, order.begin() + j + 1);
          improved = true;
        }
      }
    }
    if (!improved) {break;}
  }
  std::vector<Point> ordered;
  for (auto i : order) {
    ordered.push_back(route[i]);
  }
  return ordered;
}

[[maybe_unused]] void svg(
  const std::string & path, const Grid & grid, const std::vector<Point> & route,
  const std::vector<FloorPoint> & floor, const std::vector<bool> & covered, double range,
  int sample_step, const std::vector<double> & headings,
  const std::vector<double> & position_tolerances)
{
  std::ofstream out(path);
  const double scale = 22.0;
  const auto sx = [scale](double x) {return 50 + (x + 15.1) * scale;};
  const auto sy = [scale](double y) {return 65 + (25.22 - y) * scale;};
  out <<
    "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1040\" height=\"1230\" viewBox=\"0 0 1040 1230\">\n"
      <<
    "<defs><marker id=\"arrow\" markerWidth=\"6\" markerHeight=\"6\" refX=\"5\" refY=\"3\" orient=\"auto\"><path d=\"M0,0 L6,3 L0,6\" fill=\"#34669a\"/></marker></defs>\n"
      << "<rect width=\"1040\" height=\"1230\" fill=\"white\"/>"
      <<
    "<g font-family=\"sans-serif\" fill=\"#152c40\"><text x=\"48\" y=\"26\" font-size=\"21\">Warehouse coverage plan · map frame · "
      << route.size() << " stops</text>"
      << "<text x=\"48\" y=\"47\" font-size=\"13\">Static geometry only; " <<
    sc::kMinimumUsableRangeM << "–" << range <<
    " m annulus, 0.45 m goal clearance; no mission execution</text></g>\n";
  out << "<defs><clipPath id=\"map-clip\"><rect x=\"50\" y=\"65\" width=\"" <<
    grid.width * grid.resolution * scale << "\" height=\"" <<
    grid.height * grid.resolution * scale <<
    "\"/></clipPath></defs><g clip-path=\"url(#map-clip)\" shape-rendering=\"crispEdges\">";
  for (std::size_t f = 0; f < floor.size(); ) {
    std::size_t end = f + 1;
    while (end < floor.size() && covered[end] == covered[f] &&
      floor[end].navigable == floor[f].navigable &&
      floor[end].index / grid.width == floor[f].index / grid.width &&
      floor[end].index == floor[end - 1].index + sample_step) {++end;}
    const double side = sample_step * grid.resolution * scale;
    out << "<rect x=\"" << sx(floor[f].p.x) - side / 2 << "\" y=\"" << sy(floor[f].p.y) - side / 2
        << "\" width=\"" << side * (end - f) << "\" height=\"" << side << "\" fill=\""
        << (covered[f] ? (floor[f].navigable ? "#a6ddc2" : "#d2e9dd") :
    (floor[f].navigable ? "#ee7b70" : "#eab661")) << "\" opacity=\".72\"/>\n";
    f = end;
  }
  out << "</g>";
  // Compress raster rows into SVG runs, retaining occupied versus unknown distinction.
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ) {
      const auto category = [](uint8_t p) {return p > 229 ? 0 : (p < 90 ? 1 : 2);};
      const int kind = category(grid.pixels[y * grid.width + x]);
      int end = x + 1;
      while (end < grid.width && category(grid.pixels[y * grid.width + end]) == kind) {++end;}
      if (kind) {
        out << "<rect x=\"" << 50 + x * grid.resolution * scale << "\" y=\""
            << 65 + y * grid.resolution * scale << "\" width=\"" <<
          (end - x) * grid.resolution * scale
            << "\" height=\"" << grid.resolution * scale + 0.05 << "\" fill=\""
            << (kind == 1 ? "#283441" : "#b6bbc1") << "\"/>\n";
      }
      x = end;
    }
  }
  for (std::size_t i = 1; i < route.size(); ++i) {
    out << "<line x1=\"" << sx(route[i - 1].x) << "\" y1=\"" << sy(route[i - 1].y)
        << "\" x2=\"" << sx(route[i].x) << "\" y2=\"" << sy(route[i].y)
        <<
      "\" stroke=\"#34669a\" stroke-width=\"1.2\" stroke-dasharray=\"4 3\" opacity=\".65\" marker-end=\"url(#arrow)\"/>\n";
  }
  for (std::size_t i = 0; i < route.size(); ++i) {
    const double x = sx(route[i].x), y = sy(route[i].y);
    const double yaw = headings[i];
    if (position_tolerances[i] <= 0.05) {
      out << "<circle cx=\"" << x << "\" cy=\"" << y <<
        "\" r=\"11\" fill=\"none\" stroke=\"#7b3fac\" stroke-width=\"2\"/>";
    }
    out << "<circle cx=\"" << x << "\" cy=\"" << y <<
      "\" r=\"8.5\" fill=\"#fff\" stroke=\"#175b9b\" stroke-width=\"1.4\"/>"
        << "<text x=\"" << x << "\" y=\"" << y + 3 <<
      "\" font-family=\"sans-serif\" font-size=\"8\" text-anchor=\"middle\" fill=\"#103f72\">" <<
      i + 1 << "</text>"
        << "<line x1=\"" << x + 10 * std::cos(yaw) << "\" y1=\"" << y - 10 * std::sin(yaw)
        << "\" x2=\"" << x + 21 * std::cos(yaw) << "\" y2=\"" << y - 21 * std::sin(yaw)
        << "\" stroke=\"#175b9b\" stroke-width=\"1.4\" marker-end=\"url(#arrow)\"/>\n";
  }
  out << "<g font-family=\"sans-serif\" fill=\"#152c40\" font-size=\"13\">"
      << "<text x=\"746\" y=\"85\" font-size=\"17\">How to read this map</text>"
      << "<text x=\"746\" y=\"115\">Numbers: planned stop order</text>"
      << "<text x=\"746\" y=\"139\">Small arrows: first viewing yaw</text>"
      << "<text x=\"746\" y=\"163\">Each stop: 8 stationary views</text>"
      << "<text x=\"746\" y=\"184\">45° apart; 71.6° camera FOV</text>"
      << "<text x=\"746\" y=\"216\">Green: visible safe floor</text>"
      << "<text x=\"746\" y=\"238\">Pale green: visible edge floor</text>"
      << "<text x=\"746\" y=\"260\">Red: unseen safe-floor samples</text>"
      << "<text x=\"746\" y=\"282\">Amber: unseen low-clearance floor</text>"
      << "<text x=\"746\" y=\"304\">Gray: unknown; dark: occupied</text>"
      << "<text x=\"746\" y=\"320\">Dashed links show sequence;</text>"
      << "<text x=\"746\" y=\"341\">Nav2 must plan around shelves.</text>"
      << "<text x=\"746\" y=\"375\">No people or truth positions used.</text>"
      << "<text x=\"746\" y=\"405\">2D sampled visibility is a planning</text>"
      << "<text x=\"746\" y=\"426\">check, not recognition success.</text>"
      << "<text x=\"746\" y=\"928\">Metrics use exact planned headings.</text>"
      << "<text x=\"746\" y=\"950\">Future scan gate: actual yaw ±10°.</text>"
      << "<text x=\"746\" y=\"972\">0.5 s settle; 2 s dwell in sim time,</text>"
      << "<text x=\"746\" y=\"994\">plus ≥2 new processed images.</text>"
      << "<text x=\"746\" y=\"1020\">Purple rings: 0.05 m arrival gate.</text>"
      << "<text x=\"746\" y=\"1042\">Other stops: 0.25 m arrival gate.</text>"
      <<
    "<path d=\"M805,545 L805,470 L796,486 M805,470 L814,486\" fill=\"none\" stroke=\"#152c40\" stroke-width=\"2\"/>"
      << "<text x=\"796\" y=\"459\">+y</text><text x=\"784\" y=\"570\">North</text>"
      <<
    "<path d=\"M760,615 h110 m-110,-5 v10 m110,-10 v10\" stroke=\"#152c40\" stroke-width=\"2\"/>"
      << "<text x=\"804\" y=\"640\">5 m</text>"
      <<
    "<circle cx=\"820\" cy=\"780\" r=\"60\" stroke=\"#175b9b\" fill=\"none\" stroke-dasharray=\"3 3\"/>"
      <<
    "<path d=\"M820,780 L868.7,745.0 A60,60 0 0 1 868.7,815.0 Z\" fill=\"#a6ddc2\" opacity=\".8\"/>";
  for (double heading : sc::kScanOffsetsRad) {
    out << "<line x1=\"820\" y1=\"780\" x2=\"" << 820 + 60 * std::cos(heading)
        << "\" y2=\"" << 780 - 60 * std::sin(heading)
        << "\" stroke=\"#175b9b\" marker-end=\"url(#arrow)\"/>";
  }
  out << "<text x=\"752\" y=\"870\">One stop: eight viewing headings</text>"
      << "<text x=\"766\" y=\"892\">Green sector: one 71.6° view</text>";
  for (double x : {-10.0, -5.0, 0.0, 5.0, 10.0}) {
    out << "<text x=\"" << sx(x) <<
      "\" y=\"1188\" text-anchor=\"middle\" font-size=\"11\">" << x << "</text>";
  }
  for (double y : {-20.0, -10.0, 0.0, 10.0, 20.0}) {
    out << "<text x=\"44\" y=\"" << sy(y) + 4 <<
      "\" text-anchor=\"end\" font-size=\"11\">" << y << "</text>";
  }
  out << "<text x=\"655\" y=\"1188\" font-size=\"11\">x (m)</text>"
      << "<text x=\"12\" y=\"76\" font-size=\"11\">y (m)</text>"
      << "<text x=\"48\" y=\"1205\">Sampling: " << sample_step * grid.resolution
      <<
    " m grid of ALL known-free floor; robot targets require 0.45 m clearance; unknown/occupied occlude.</text></g></svg>\n";
}
}  // namespace

#ifndef SEARCH_COVERAGE_MAP_NO_MAIN
int main(int argc, char ** argv)
{
  try {
    const std::string map_path = argc > 1 ? argv[1] :
      "/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.pgm";
    const double range = argc > 2 ? std::stod(argv[2]) : sc::kUsableRangeM;
    if (!std::isfinite(range) || range <= sc::kMinimumUsableRangeM) {
      throw std::runtime_error("maximum range must exceed minimum usable person range");
    }
    const bool make_route = argc > 3 && std::string(argv[3]) == "--generate";
    const std::string svg_path = argc > 4 ? argv[4] : "";
    const int sample_step = argc > 5 ? std::stoi(argv[5]) : 10;
    if (sample_step < 1 || sample_step > 50) {throw std::runtime_error("invalid sample step");}
    const double sample_area = std::pow(sample_step * 0.03, 2);
    Grid grid(map_path);
    auto component = grid.connected({2.12, -21.3}, sc::kPlanningClearanceM);
    std::vector<FloorPoint> floor;
    std::size_t free_count = 0, occupied_count = 0, unknown_count = 0, safe_count = 0;
    for (std::size_t i = 0; i < grid.pixels.size(); ++i) {
      if (grid.pixels[i] > 229) {++free_count;} else if (grid.pixels[i] < 90) {
        ++occupied_count;
      } else {++unknown_count;}
      if (grid.safe(static_cast<int>(i), sc::kPlanningClearanceM)) {++safe_count;}
    }
    for (int y = sample_step / 2; y < grid.height; y += sample_step) {
      for (int x = sample_step / 2; x < grid.width; x += sample_step) {
        const int i = y * grid.width + x;
        if (grid.free(x, y)) {floor.push_back({grid.point(i), i, component[i] != 0});}
      }
    }
    std::vector<Point> route;
    if (make_route) {route = generate(grid, component, floor, range);} else {
      for (const auto & w : sc::warehouse_waypoints()) {
        route.push_back({w.x, w.y});
      }
    }
    std::vector<bool> covered(floor.size());
    std::vector<double> headings;
    std::vector<double> position_tolerances;
    std::size_t valid = 0;
    double min_clearance = 1000;
    for (std::size_t r = 0; r < route.size(); ++r) {
      const Point w = route[r];
      const Point next = r + 1 < route.size() ? route[r + 1] : Point{2.12, -21.3};
      const double yaw = make_route ? std::atan2(next.y - w.y, next.x - w.x) :
        sc::warehouse_waypoints()[r].yaw;
      headings.push_back(yaw);
      position_tolerances.push_back(make_route ? 0.25 :
        sc::warehouse_waypoints()[r].position_tolerance_m);
      const int i = grid.index(w);
      if (i >= 0 && component[i] && grid.point_clearance(w) >= sc::kPlanningClearanceM) {
        ++valid; min_clearance = std::min(min_clearance, grid.point_clearance(w));
      }
      for (std::size_t f = 0; f < floor.size(); ++f) {
        if (!covered[f] && view_visible(grid, w, yaw, floor[f].p, range)) {covered[f] = true;}
      }
    }
    const std::size_t seen = std::count(covered.begin(), covered.end(), true);
    std::vector<uint8_t> near_only_gap(grid.pixels.size());
    std::size_t near_gap_count = 0;
    for (std::size_t f = 0; f < floor.size(); ++f) {
      if (covered[f]) {continue;}
      for (std::size_t r = 0; r < route.size(); ++r) {
        if (view_visible(grid, route[r], headings[r], floor[f].p, range, 0.0)) {
          near_only_gap[floor[f].index] = 1;
          ++near_gap_count;
          break;
        }
      }
    }
    std::size_t safe_samples = 0, safe_seen = 0;
    for (std::size_t f = 0; f < floor.size(); ++f) {
      if (floor[f].navigable) {++safe_samples; if (covered[f]) {++safe_seen;}}
    }
    const std::size_t component_count = std::count(component.begin(), component.end(), uint8_t{1});
    Point component_low{1000, 1000}, component_high{-1000, -1000};
    for (std::size_t i = 0; i < component.size(); ++i) {
      if (!component[i]) {continue;}
      const Point p = grid.point(static_cast<int>(i));
      component_low.x = std::min(component_low.x, p.x);
      component_low.y = std::min(component_low.y, p.y);
      component_high.x = std::max(component_high.x, p.x);
      component_high.y = std::max(component_high.y, p.y);
    }
    std::cout << std::fixed << std::setprecision(3)
              << "map=" << grid.width << "x" << grid.height << " resolution=" << grid.resolution
              << " free_m2=" << free_count * 0.0009 << " occupied_m2=" << occupied_count * 0.0009
              << " unknown_m2=" << unknown_count * 0.0009 << "\n"
              << "safe_m2=" << safe_count * 0.0009 << " connected_m2=" << component_count * 0.0009
              << " disconnected_safe_m2=" << (safe_count - component_count) * 0.0009 << "\n"
              << "connected_bbox=[" << component_low.x << "," << component_low.y << ","
              << component_high.x << "," << component_high.y << "]\n"
              << "min_range=" << sc::kMinimumUsableRangeM << " max_range=" << range
              << " waypoints=" << route.size() << " valid=" << valid
              << " min_clearance=" << min_clearance << "\n"
              << "all_free_covered=" << seen << "/" << floor.size()
              << " percent=" << 100.0 * seen / floor.size() << " gap_m2_approx="
              << (floor.size() - seen) * sample_area << "\n"
              << "reachable_safe_covered=" << safe_seen << "/" << safe_samples
              << " percent=" << 100.0 * safe_seen / safe_samples << " gap_m2_approx="
              << (safe_samples - safe_seen) * sample_area << "\n"
              << "edge_floor_covered=" << seen - safe_seen << "/" << floor.size() - safe_samples
              << " percent=" << 100.0 * (seen - safe_seen) / (floor.size() - safe_samples)
              << " gap_m2_approx=" <<
      ((floor.size() - safe_samples) - (seen - safe_seen)) * sample_area << "\n"
              << "gap_removed_if_near_cutoff_ignored=" << near_gap_count
              << " area_approx=" << near_gap_count * sample_area << "\n";
    // Connected clusters of the sampled residuals, useful for honest gap notes.
    std::vector<uint8_t> gaps(grid.pixels.size());
    for (std::size_t f = 0; f < floor.size(); ++f) {
      if (!covered[f]) {
        gaps[floor[f].index] = 1;
      }
    }
    std::size_t cluster_id = 0;
    for (std::size_t f = 0; f < floor.size(); ++f) {
      const int first = floor[f].index;
      if (!gaps[first]) {continue;}
      std::vector<int> cluster{first};
      std::size_t safe_gap_samples = 0;
      std::size_t near_gap_samples = 0;
      gaps[first] = 0;
      Point low = grid.point(first), high = low;
      for (std::size_t q = 0; q < cluster.size(); ++q) {
        const int i = cluster[q];
        if (component[i]) {++safe_gap_samples;}
        if (near_only_gap[i]) {++near_gap_samples;}
        const Point p = grid.point(i);
        low.x = std::min(low.x, p.x); low.y = std::min(low.y, p.y);
        high.x = std::max(high.x, p.x); high.y = std::max(high.y, p.y);
        for (int dy : {-sample_step, 0, sample_step}) {
          for (int dx : {-sample_step, 0, sample_step}) {
            const int x = i % grid.width + dx, y = i / grid.width + dy;
            if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {continue;}
            const int next = y * grid.width + x;
            if (gaps[next]) {gaps[next] = 0; cluster.push_back(next);}
          }
        }
      }
      std::cout << "gap_cluster=" << ++cluster_id << " samples=" << cluster.size()
                << " safe_samples=" << safe_gap_samples
                << " near_cutoff_samples=" << near_gap_samples
                << " area_approx=" << cluster.size() * sample_area << " bbox=[" << low.x << ","
                << low.y << "," << high.x << "," << high.y << "]\n";
    }
    if (make_route) {
      for (std::size_t i = 0; i < route.size(); ++i) {
        const Point next = i + 1 < route.size() ? route[i + 1] : Point{2.12, -21.3};
        std::cout << "    {" << route[i].x << ", " << route[i].y << ", "
                  << std::atan2(next.y - route[i].y, next.x - route[i].x) << "},\n";
      }
    }
    if (!svg_path.empty()) {
      svg(svg_path, grid, route, floor, covered, range, sample_step, headings, position_tolerances);
    }
    return valid == route.size() ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n"; return 2;
  }
}
#endif  // SEARCH_COVERAGE_MAP_NO_MAIN

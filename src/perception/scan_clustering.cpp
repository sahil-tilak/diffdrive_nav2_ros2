#include "diffdrive_nav2_ros2/perception/scan_clustering.hpp"

#include <algorithm>
#include <cmath>

namespace diffdrive_nav2_ros2::perception
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Pt
{
  std::size_t beam;
  double r, x, y;
};

bool joined(const Pt & a, const Pt & b, const Params & p)
{
  return std::hypot(a.x - b.x, a.y - b.y) <= p.gap_base + p.gap_ratio * std::min(a.r, b.r);
}
}  // namespace

std::vector<Cluster> findClusters(
  const std::vector<float> & ranges, double angle_min, double angle_increment, const Params & p)
{
  std::vector<Cluster> out;
  std::size_t n = ranges.size();
  if (n == 0 || angle_increment == 0.0) {
    return out;
  }

  // Full circle: the span from beam 0 to beam n-1 plus one step reaches 2*pi (within half a step).
  const double span = std::abs(angle_increment) * static_cast<double>(n);
  const bool full_circle = span >= 2.0 * kPi - 0.5 * std::abs(angle_increment);
  // Step 2: duplicate last beam (angle of beam n-1 equals angle of beam 0 + 2*pi).
  if (full_circle && n > 1) {
    const double last_angle = static_cast<double>(n - 1) * std::abs(angle_increment);
    if (std::abs(last_angle - 2.0 * kPi) <= 0.5 * std::abs(angle_increment)) {
      --n;
    }
  }

  auto valid = [&](std::size_t i) {
      const double r = ranges[i];
      return std::isfinite(r) && r >= p.range_min && r <= p.range_max;
    };

  // Steps 1 and 3
  std::vector<std::vector<Pt>> groups;
  for (std::size_t i = 0; i < n; ++i) {
    if (!valid(i)) {
      continue;
    }
    const double r = ranges[i];
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    Pt pt{i, r, r * std::cos(a), r * std::sin(a)};
    if (groups.empty() || !joined(groups.back().back(), pt, p)) {
      groups.emplace_back();
    }
    groups.back().push_back(pt);
  }
  if (groups.empty()) {
    return out;
  }

  // Step 4: wrap-around merge, only if the last and first valid points are really neighbours
  // (no valid-looking beam between them is not required; invalid beams are skipped like in step 3).
  if (full_circle && groups.size() > 1 && joined(groups.back().back(), groups.front().front(), p)) {
    auto & last = groups.back();
    last.insert(last.end(), groups.front().begin(), groups.front().end());
    groups.front() = std::move(last);
    groups.pop_back();
  }

  // Step 5
  for (const auto & g : groups) {
    if (g.size() < p.min_points) {
      continue;
    }
    double length = 0.0;
    for (const Pt * end : {&g.front(), &g.back()}) {
      for (const auto & q : g) {
        length = std::max(length, std::hypot(end->x - q.x, end->y - q.y));
      }
    }
    if (length > p.max_length) {
      continue;
    }
    // occlusion: beam just outside each end (indices wrap on a full circle)
    const double near_r = std::min(g.front().r, g.back().r);
    bool occluded = false;
    for (int side = 0; side < 2 && !occluded; ++side) {
      const long ni = side == 0 ? static_cast<long>(g.front().beam) - 1
                                : static_cast<long>(g.back().beam) + 1;
      long idx = ni;
      if (full_circle) {
        idx = (ni % static_cast<long>(n) + static_cast<long>(n)) % static_cast<long>(n);
      } else if (ni < 0 || ni >= static_cast<long>(n)) {
        continue;
      }
      if (valid(static_cast<std::size_t>(idx)) &&
        ranges[static_cast<std::size_t>(idx)] < near_r - p.occlusion_margin)
      {
        occluded = true;
      }
    }
    if (occluded) {
      continue;
    }
    Cluster c;
    for (const auto & q : g) {
      c.x += q.x;
      c.y += q.y;
    }
    c.x /= static_cast<double>(g.size());
    c.y /= static_cast<double>(g.size());
    c.n_points = g.size();
    c.length = length;
    c.first_beam = g.front().beam;
    c.last_beam = g.back().beam;
    out.push_back(c);
  }
  return out;
}

}  // namespace diffdrive_nav2_ros2::perception

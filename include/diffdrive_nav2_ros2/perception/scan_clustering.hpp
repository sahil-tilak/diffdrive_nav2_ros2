// Pure C++17 clustering of a 2D laser scan. No ROS types, so it can move to its own package.
#pragma once

#include <cstddef>
#include <vector>

namespace diffdrive_nav2_ros2::perception
{

struct Params
{
  double range_min = 0.15;         // m, ignore beams closer than this
  double range_max = 8.0;          // m, ignore beams farther than this
  double gap_base = 0.03;          // m, constant part of the split threshold
  double gap_ratio = 0.112;        // m per m of range, range-dependent part
  std::size_t min_points = 3;      // clusters with fewer points are noise
  double max_length = 0.6;         // m, longer clusters are walls
  double occlusion_margin = 0.3;   // m, see findClusters step 5
};

struct Cluster
{
  double x = 0.0;                  // centroid, sensor frame
  double y = 0.0;
  std::size_t n_points = 0;
  double length = 0.0;             // largest distance from an end point to any point
  std::size_t first_beam = 0;      // beam index of the first point (in scan order)
  std::size_t last_beam = 0;       // beam index of the last point
};

// Steps:
//  1. valid beam = finite and range_min <= r <= range_max; others are skipped
//  2. full circle + last beam at the same angle as beam 0 -> last beam is a duplicate, dropped
//  3. split when the gap between neighbouring valid points > gap_base + gap_ratio * min(r_a, r_b)
//  4. full circle: merge the last cluster into the first if the wrap-around gap passes the same test
//  5. drop: < min_points; length > max_length; occluded = the beam just outside either end is valid
//     and shorter than min(r_first, r_last) - occlusion_margin
//
// KNOWN BIAS (not fixed): the centroid is the mean of the visible points, so it sits on the visible
// face of an object, up to ~0.15 m in front of the true centre of a 0.3 m box.
// KNOWN LIMIT: a real small object partly hidden behind something closer is dropped (step 5).
std::vector<Cluster> findClusters(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  const Params & p);

}  // namespace diffdrive_nav2_ros2::perception

#include "diffdrive_nav2_ros2/prediction/obstacle_prediction.hpp"

#include <algorithm>
#include <cmath>

namespace diffdrive_nav2_ros2::prediction
{

// ============================================================================
// EMPTY BLOCK 1: predictPositions     Checked by: Predict.* tests in
//                                                 test/test_obstacle_prediction.cpp
//
// Idea: constant velocity. After t seconds the obstacle is at
//         position = start position + velocity * t
//
// Steps:
//   1. Bad input: if step <= 0, or t_end < t_start, return an empty vector.
//   2. Count the points. The last point must be at t_end (inclusive), so
//        n = floor((t_end - t_start) / step) + 1.
//      WATCH OUT: a division can land a hair below the true whole number. Example:
//      0.3 / 0.1 is 2.9999999999999996 in double, so floor gives 2, not 3. Add a tiny
//      tolerance (1e-9) to the division BEFORE you floor it, or you lose the last point.
//   3. For each i from 0 to n - 1, compute t = t_start + i * step.
//      Do NOT build t by adding step again and again: the rounding error piles up.
//   4. For each t, build one PredictedPoint: x = s.x + s.vx * t, y = s.y + s.vy * t,
//      and store t in the point (t is measured from the message stamp, as in the header).
//   5. Return all points in order of increasing t.
//
// Sizes: with t_start 0, t_end 1.5, step 0.1 you get exactly 16 points.
//
// Library calls you may need (list only): std::floor, std::vector::reserve,
// std::vector::push_back (or emplace_back).
// ============================================================================
std::vector<PredictedPoint> predictPositions(
  [[maybe_unused]] const ObstacleState & s,
  [[maybe_unused]] double t_start,
  [[maybe_unused]] double t_end,
  [[maybe_unused]] double step)
{
  return {};
}

// ============================================================================
// EMPTY BLOCK 2: cellsInDisc          Checked by: Disc.* tests in
//                                                 test/test_obstacle_prediction.cpp
//
// Idea: a cell is "in the disc" if the CENTRE of the cell is within `radius` of the point
// (wx, wy). Cell centre: origin + (index + 0.5) * resolution (see GridInfo).
// Do not test every cell of the grid (the grid can be 60 x 60 or more). Only look at the
// small square that contains the circle.
//
// Steps:
//   1. Bad input: if radius < 0, or grid.resolution <= 0, return an empty vector.
//   2. Take the square around the point: from (wx - radius, wy - radius)
//      to (wx + radius, wy + radius).
//   3. Turn the four edges into cell indices: (world - origin) / resolution, then FLOOR.
//      Use floor, not a plain cast to int: a cast rounds -4.5 to -4, floor gives -5.
//   4. Keep the indices as SIGNED integers (int) for now. An unsigned number can never be
//      negative: -3 turns into about 4 billion. Clamp the four indices to the grid:
//      the lowest allowed index is 0, the highest is size - 1.
//   5. If the clamped range is empty (low index > high index in x or y), the circle is
//      completely outside the grid: return an empty vector.
//   6. Loop over the cells in that range. For each cell:
//        centre = origin + (index + 0.5) * resolution      (x and y separately)
//        keep the cell if dx*dx + dy*dy <= radius*radius + 1e-9
//      Compare squared distances (no sqrt needed). The 1e-9 matters: a cell centre that
//      lies exactly on the edge of the circle can come out a tiny bit too far in floating
//      point. Without it, PlusShapeForRadiusOneCell and NegativeOriginAndNonSquareGrid
//      each lose 2 cells (checked with a scratch script, not with your code).
//   7. Only now convert the index to unsigned int (the Cell struct) and store it.
//      Each cell is visited once in the loop, so there are no duplicates.
//
// Sizes: radius 0.2 at resolution 0.05, circle on a cell centre: 49 cells.
//
// Library calls you may need (list only): std::floor, std::max, std::min,
// static_cast<int>, std::vector::reserve, std::vector::push_back.
// ============================================================================
std::vector<Cell> cellsInDisc(
  [[maybe_unused]] double wx,
  [[maybe_unused]] double wy,
  [[maybe_unused]] double radius,
  [[maybe_unused]] const GridInfo & grid)
{
  return {};
}

// GIVEN (not a block): see the header for the rule and the plan for why 252 and 140.
unsigned char predictedCost(
  double time_ahead, double horizon, double max_cost, double min_cost)
{
  double fraction = 0.0;
  if (horizon > 0.0) {
    fraction = std::clamp(time_ahead / horizon, 0.0, 1.0);
  }
  const double cost = max_cost - (max_cost - min_cost) * fraction;
  return static_cast<unsigned char>(std::clamp(std::lround(cost), 0L, 255L));
}

}  // namespace diffdrive_nav2_ros2::prediction

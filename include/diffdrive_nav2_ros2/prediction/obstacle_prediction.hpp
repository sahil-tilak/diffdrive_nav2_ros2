// Maths for the predictive costmap layer. Pure C++17, no ROS types.
//
// Two blocks are left EMPTY on purpose (see obstacle_prediction.cpp):
//   predictPositions   (block 1: where will the obstacle be?)
//   cellsInDisc        (block 2: which grid cells does a circle cover?)
// predictedCost is given (it is the cost-value rule from the plan).
#pragma once

#include <vector>

namespace diffdrive_nav2_ros2::prediction
{

// Obstacle state at the stamp of the tracker message (world frame, metres, m/s).
struct ObstacleState
{
  double x;
  double y;
  double vx;
  double vy;
};

// A guessed position. t = seconds AFTER THE MESSAGE STAMP (not after "now").
struct PredictedPoint
{
  double x;
  double y;
  double t;
};

// The part of a Nav2 grid that block 2 needs.
// Cell (mx, my) has its CENTRE at (origin_x + (mx + 0.5) * resolution,
//                                  origin_y + (my + 0.5) * resolution).
struct GridInfo
{
  double origin_x;            // world x of the lower-left CORNER of cell (0, 0)
  double origin_y;
  double resolution;          // m per cell
  unsigned int size_x;        // cells
  unsigned int size_y;
};

struct Cell
{
  unsigned int mx;
  unsigned int my;
};

// EMPTY BLOCK 1 (yours). Constant-velocity prediction.
// Returns points at t = t_start, t_start + step, ... up to and including t_end.
// Position at time t is the state position plus velocity times t.
std::vector<PredictedPoint> predictPositions(
  const ObstacleState & s, double t_start, double t_end, double step);

// EMPTY BLOCK 2 (yours). Cells of `grid` whose CENTRE is within `radius` of (wx, wy).
// Only cells inside the grid are returned, each one once.
std::vector<Cell> cellsInDisc(double wx, double wy, double radius, const GridInfo & grid);

// GIVEN. Cost of a predicted point, from how far ahead it is:
//   max_cost at time_ahead = 0, falling in a straight line to min_cost at time_ahead = horizon.
// time_ahead is clamped to [0, horizon]. horizon <= 0 gives max_cost.
// The result is rounded to the nearest whole number.
unsigned char predictedCost(
  double time_ahead, double horizon, double max_cost, double min_cost);

}  // namespace diffdrive_nav2_ros2::prediction

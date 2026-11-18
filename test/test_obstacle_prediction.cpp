// Tests written against the spec in include/.../prediction/obstacle_prediction.hpp.
// Predict.* and Disc.* FAIL until the two empty blocks in src/prediction/obstacle_prediction.cpp
// are filled in. Cost.* passes already (predictedCost is given).
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <utility>
#include <vector>

#include "diffdrive_nav2_ros2/prediction/obstacle_prediction.hpp"

using namespace diffdrive_nav2_ros2::prediction;  // NOLINT

namespace
{
using CellSet = std::set<std::pair<unsigned int, unsigned int>>;

CellSet toSet(const std::vector<Cell> & cells)
{
  CellSet s;
  for (const auto & c : cells) {
    s.insert({c.mx, c.my});
  }
  return s;
}

double centreX(const GridInfo & g, unsigned int mx)
{
  return g.origin_x + (mx + 0.5) * g.resolution;
}

double centreY(const GridInfo & g, unsigned int my)
{
  return g.origin_y + (my + 0.5) * g.resolution;
}

// 3 m x 3 m local costmap, as in nav2_params.yaml (60 x 60 cells of 0.05 m).
const GridInfo kGrid{0.0, 0.0, 0.05, 60, 60};
}  // namespace

// ---------------------------------------------------------------- Block 1

TEST(Predict, StationaryStaysPut)
{
  const auto pts = predictPositions({1.0, 2.0, 0.0, 0.0}, 0.0, 1.0, 0.5);
  ASSERT_EQ(pts.size(), 3u);
  for (const auto & p : pts) {
    EXPECT_NEAR(p.x, 1.0, 1e-9);
    EXPECT_NEAR(p.y, 2.0, 1e-9);
  }
}

TEST(Predict, MovesByVelocityTimesT)
{
  const auto pts = predictPositions({0.0, 0.0, 1.0, 2.0}, 0.0, 1.0, 0.5);
  ASSERT_EQ(pts.size(), 3u);
  EXPECT_NEAR(pts[0].t, 0.0, 1e-9);
  EXPECT_NEAR(pts[0].x, 0.0, 1e-9);
  EXPECT_NEAR(pts[0].y, 0.0, 1e-9);
  EXPECT_NEAR(pts[1].t, 0.5, 1e-9);
  EXPECT_NEAR(pts[1].x, 0.5, 1e-9);
  EXPECT_NEAR(pts[1].y, 1.0, 1e-9);
  EXPECT_NEAR(pts[2].t, 1.0, 1e-9);
  EXPECT_NEAR(pts[2].x, 1.0, 1e-9);
  EXPECT_NEAR(pts[2].y, 2.0, 1e-9);
}

TEST(Predict, StartsAtLatencyOffset)
{
  // The message is 0.2 s old: the first guess is where the obstacle is NOW (t = 0.2).
  const auto pts = predictPositions({1.0, -1.0, 0.5, 0.0}, 0.2, 1.7, 0.5);
  ASSERT_EQ(pts.size(), 4u);
  EXPECT_NEAR(pts.front().t, 0.2, 1e-9);
  EXPECT_NEAR(pts.front().x, 1.1, 1e-9);
  EXPECT_NEAR(pts.front().y, -1.0, 1e-9);
  EXPECT_NEAR(pts.back().t, 1.7, 1e-9);
  EXPECT_NEAR(pts.back().x, 1.85, 1e-9);
}

TEST(Predict, SixteenPointsForOnePointFiveSecondsAtTenthStep)
{
  const auto pts = predictPositions({0.0, 0.0, 0.25, 0.0}, 0.0, 1.5, 0.1);
  ASSERT_EQ(pts.size(), 16u);  // the last point (t = 1.5) must not be lost to rounding
  EXPECT_NEAR(pts.back().t, 1.5, 1e-9);
  EXPECT_NEAR(pts.back().x, 0.375, 1e-9);
}

TEST(Predict, FourPointsForPointThreeSecondsAtTenthStep)
{
  // 0.3 / 0.1 is 2.9999999999999996 in double: a plain floor loses the last point.
  const auto pts = predictPositions({0.0, 0.0, 1.0, 0.0}, 0.0, 0.3, 0.1);
  ASSERT_EQ(pts.size(), 4u);
  EXPECT_NEAR(pts.back().t, 0.3, 1e-9);
  EXPECT_NEAR(pts.back().x, 0.3, 1e-9);
}

TEST(Predict, SinglePointWhenStartEqualsEnd)
{
  const auto pts = predictPositions({0.0, 0.0, 1.0, 0.0}, 0.3, 0.3, 0.1);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 0.3, 1e-9);
}

TEST(Predict, BadInputGivesEmptyButValidInputDoesNot)
{
  const ObstacleState s{0.0, 0.0, 1.0, 0.0};
  EXPECT_EQ(predictPositions(s, 0.0, 1.0, 0.5).size(), 3u);  // valid case
  EXPECT_TRUE(predictPositions(s, 0.0, 1.0, 0.0).empty());   // step 0
  EXPECT_TRUE(predictPositions(s, 0.0, 1.0, -0.1).empty());  // negative step
  EXPECT_TRUE(predictPositions(s, 1.0, 0.5, 0.1).empty());   // t_end < t_start
}

// ---------------------------------------------------------------- Block 2

TEST(Disc, RadiusZeroOnCellCentreIsThatCell)
{
  const auto cells = cellsInDisc(centreX(kGrid, 30), centreY(kGrid, 30), 0.0, kGrid);
  ASSERT_EQ(cells.size(), 1u);
  EXPECT_EQ(cells[0].mx, 30u);
  EXPECT_EQ(cells[0].my, 30u);
}

TEST(Disc, FortyNineCellsForRadius0p2AtResolution0p05)
{
  // Cell centres with dx^2 + dy^2 <= 4^2 (in cell units): 49 of them. 4 of those lie exactly
  // on the edge: (4, 0), (-4, 0), (0, 4), (0, -4).
  const auto cells = cellsInDisc(centreX(kGrid, 30), centreY(kGrid, 30), 0.2, kGrid);
  EXPECT_EQ(cells.size(), 49u);
  const CellSet s = toSet(cells);
  EXPECT_EQ(s.count({34, 30}), 1u);  // on the edge: inside
  EXPECT_EQ(s.count({26, 30}), 1u);
  EXPECT_EQ(s.count({30, 34}), 1u);
  EXPECT_EQ(s.count({30, 26}), 1u);
  EXPECT_EQ(s.count({33, 32}), 1u);  // 3^2 + 2^2 = 13 <= 16: inside
  EXPECT_EQ(s.count({33, 33}), 0u);  // 3^2 + 3^2 = 18 > 16: outside
  EXPECT_EQ(s.count({34, 34}), 0u);  // corner of the bounding square: outside
}

TEST(Disc, PlusShapeForRadiusOneCell)
{
  // Resolution 0.1, radius 0.1, centred on cell (10, 10): the cell and its 4 direct neighbours.
  const GridInfo g{0.0, 0.0, 0.1, 20, 20};
  const auto cells = cellsInDisc(centreX(g, 10), centreY(g, 10), 0.1, g);
  const CellSet expected{{10, 10}, {9, 10}, {11, 10}, {10, 9}, {10, 11}};
  EXPECT_EQ(toSet(cells), expected);
  EXPECT_EQ(cells.size(), 5u);
}

TEST(Disc, EveryCellInsideGridInsideRadiusAndUnique)
{
  const double wx = 1.43;
  const double wy = 0.97;
  const double r = 0.3;
  const auto cells = cellsInDisc(wx, wy, r, kGrid);
  // Area check: pi * (r / res)^2 = about 113 cells.
  EXPECT_GT(cells.size(), 100u);
  EXPECT_LT(cells.size(), 125u);
  EXPECT_EQ(toSet(cells).size(), cells.size());  // no duplicates
  for (const auto & c : cells) {
    ASSERT_LT(c.mx, kGrid.size_x);
    ASSERT_LT(c.my, kGrid.size_y);
    const double dx = centreX(kGrid, c.mx) - wx;
    const double dy = centreY(kGrid, c.my) - wy;
    EXPECT_LE(dx * dx + dy * dy, r * r + 1e-9);
  }
}

TEST(Disc, ClippedAtGridCorner)
{
  // Centre of cell (0, 0): only the quarter of the 49-cell disc that lies inside the grid.
  const auto cells = cellsInDisc(centreX(kGrid, 0), centreY(kGrid, 0), 0.2, kGrid);
  EXPECT_EQ(cells.size(), 17u);
  for (const auto & c : cells) {
    EXPECT_LE(c.mx, 4u);
    EXPECT_LE(c.my, 4u);
  }
}

TEST(Disc, CentreOutsideLowEdgeStillCoversInsideCells)
{
  // Centre (-0.1, 0.025), radius 0.2: reaches into columns 0 and 1 only.
  // Column 0: rows 0..3 (4 cells). Column 1: rows 0..1 (2 cells).
  const auto cells = cellsInDisc(-0.1, 0.025, 0.2, kGrid);
  const CellSet expected{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 0}, {1, 1}};
  EXPECT_EQ(toSet(cells), expected);
  EXPECT_EQ(cells.size(), 6u);
}

TEST(Disc, NegativeOriginAndNonSquareGrid)
{
  // Like a rolling window that moved to negative world coordinates, 20 x 10 cells.
  const GridInfo g{-1.0, -0.5, 0.1, 20, 10};
  const auto cells = cellsInDisc(centreX(g, 7), centreY(g, 3), 0.1, g);
  const CellSet expected{{7, 3}, {6, 3}, {8, 3}, {7, 2}, {7, 4}};
  EXPECT_EQ(toSet(cells), expected);
  EXPECT_EQ(cells.size(), 5u);
}

TEST(Disc, FullyOutsideIsEmptyButInsideIsNot)
{
  EXPECT_FALSE(cellsInDisc(1.5, 1.5, 0.2, kGrid).empty());   // valid case
  EXPECT_TRUE(cellsInDisc(-1.0, 1.5, 0.2, kGrid).empty());   // far left of the grid
  EXPECT_TRUE(cellsInDisc(1.5, -1.0, 0.2, kGrid).empty());   // far below
  EXPECT_TRUE(cellsInDisc(5.0, 1.5, 0.2, kGrid).empty());    // far right
  EXPECT_TRUE(cellsInDisc(1.5, 5.0, 0.2, kGrid).empty());    // far above
  EXPECT_TRUE(cellsInDisc(1.5, 1.5, -0.1, kGrid).empty());   // negative radius
}

// ---------------------------------------------------------------- Given (passes now)

TEST(Cost, EndpointsAndMiddle)
{
  EXPECT_EQ(predictedCost(0.0, 1.5, 252, 140), 252);
  EXPECT_EQ(predictedCost(1.5, 1.5, 252, 140), 140);
  EXPECT_EQ(predictedCost(0.1, 1.5, 252, 140), 245);
  EXPECT_EQ(predictedCost(0.5, 1.5, 252, 140), 215);
  EXPECT_EQ(predictedCost(1.0, 1.5, 252, 140), 177);
  EXPECT_EQ(predictedCost(1.3, 1.5, 252, 140), 155);
}

TEST(Cost, ClampsTimeAndHandlesBadHorizon)
{
  EXPECT_EQ(predictedCost(-0.5, 1.5, 252, 140), 252);  // before now: no more than max
  EXPECT_EQ(predictedCost(9.0, 1.5, 252, 140), 140);   // beyond the horizon: no less than min
  EXPECT_EQ(predictedCost(0.5, 0.0, 252, 140), 252);   // horizon 0: max
}

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "diffdrive_nav2_ros2/perception/scan_clustering.hpp"

using namespace diffdrive_nav2_ros2::perception;  // NOLINT

namespace
{
constexpr double kPi = 3.14159265358979323846;
const float kInf = std::numeric_limits<float>::infinity();
const float kNaN = std::numeric_limits<float>::quiet_NaN();

struct Seg { double ax, ay, bx, by; };

struct Scan
{
  std::vector<float> ranges;
  double angle_min, inc;
};

// Axis-aligned square with centre (cx, cy) and side s, as 4 segments.
std::vector<Seg> box(double cx, double cy, double s)
{
  const double h = s / 2;
  return {
    {cx - h, cy - h, cx + h, cy - h}, {cx + h, cy - h, cx + h, cy + h},
    {cx + h, cy + h, cx - h, cy + h}, {cx - h, cy + h, cx - h, cy - h}};
}

// Ray from the origin, nearest hit among segments. No noise.
double castRay(double ang, const std::vector<Seg> & segs)
{
  const double dx = std::cos(ang), dy = std::sin(ang);
  double best = std::numeric_limits<double>::infinity();
  for (const auto & s : segs) {
    const double ex = s.bx - s.ax, ey = s.by - s.ay;
    const double den = dx * ey - dy * ex;
    if (std::abs(den) < 1e-12) {continue;}
    const double t = (s.ax * ey - s.ay * ex) / den;   // along the ray
    const double u = (s.ax * dy - s.ay * dx) / den;   // along the segment
    if (t > 0 && u >= 0 && u <= 1 && t < best) {best = t;}
  }
  return best;
}

// Same layout as the simulated lidar: 360 beams, -3.14159 .. +3.14159 (last beam duplicates the first)
Scan makeScan(const std::vector<Seg> & segs, bool duplicate_last = true)
{
  Scan sc;
  sc.angle_min = -3.14159;
  sc.inc = duplicate_last ? 6.28318 / 359.0 : 2.0 * kPi / 360.0;
  for (int i = 0; i < 360; ++i) {
    const double r = castRay(sc.angle_min + i * sc.inc, segs);
    sc.ranges.push_back(std::isfinite(r) && r <= 10.0 ? static_cast<float>(r) : kInf);
  }
  return sc;
}

std::vector<Cluster> run(const Scan & sc, const Params & p = Params())
{
  return findClusters(sc.ranges, sc.angle_min, sc.inc, p);
}

std::vector<Seg> cat(std::vector<Seg> a, const std::vector<Seg> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}
}  // namespace

TEST(ScanClustering, BoxAt1p5mGivesOneCluster)
{
  auto c = run(makeScan(box(1.5, 0.0, 0.3)));
  ASSERT_EQ(c.size(), 1u);
  EXPECT_LT(std::hypot(c[0].x - 1.5, c[0].y), 0.16);   // known bias: on the visible face
  EXPECT_GE(c[0].n_points, 8u);
}

TEST(ScanClustering, LongWallIsRejected)
{
  EXPECT_TRUE(run(makeScan({{2.0, -2.0, 2.0, 2.0}})).empty());
}

TEST(ScanClustering, SingleNoisePointIsRejected)
{
  Scan sc;
  sc.angle_min = -3.14159;
  sc.inc = 6.28318 / 359.0;
  sc.ranges.assign(360, kInf);
  sc.ranges[100] = 2.0f;
  EXPECT_TRUE(run(sc).empty());
}

TEST(ScanClustering, TwoBoxesWithClearGapGiveTwoClusters)
{
  auto c = run(makeScan(cat(box(1.5, -0.3, 0.3), box(1.5, 0.3, 0.3))));
  EXPECT_EQ(c.size(), 2u);
}

TEST(ScanClustering, NonFiniteAndOutOfRangeBeamsAreSkipped)
{
  auto sc = makeScan(box(1.5, 0.0, 0.3));
  // beams around +x are 179/180 (angle_min + i*inc ~ 0 at i ~ 179.5)
  sc.ranges[0] = kNaN;
  sc.ranges[10] = -kInf;
  sc.ranges[20] = 0.05f;      // below range_min
  sc.ranges[30] = 20.0f;      // above range_max
  sc.ranges[179] = kNaN;      // one beam inside the box face
  auto c = run(sc);
  ASSERT_EQ(c.size(), 1u);
  EXPECT_LT(std::hypot(c[0].x - 1.5, c[0].y), 0.16);
}

TEST(ScanClustering, BoxAcrossWrapAroundIsOneCluster)
{
  for (bool dup : {true, false}) {
    auto sc = makeScan(box(-1.5, 0.0, 0.3), dup);
    // the box must really straddle the end of the array
    ASSERT_TRUE(std::isfinite(sc.ranges.front())) << "dup=" << dup;
    ASSERT_TRUE(std::isfinite(sc.ranges.back())) << "dup=" << dup;
    auto c = run(sc);
    ASSERT_EQ(c.size(), 1u) << "dup=" << dup;
    EXPECT_LT(std::hypot(c[0].x + 1.5, c[0].y), 0.16) << "dup=" << dup;
  }
}

TEST(ScanClustering, BoxInFrontOfWallKeepsOnlyTheBox)
{
  // Short wall behind the box; the box shadow leaves two ~0.3 m pieces of wall.
  auto segs = cat(box(1.5, 0.0, 0.3), {{3.0, -0.6, 3.0, 0.6}});
  auto sc = makeScan(segs);
  auto c = run(sc);
  ASSERT_EQ(c.size(), 1u);
  EXPECT_LT(std::hypot(c[0].x - 1.5, c[0].y), 0.16);

  // Sanity: without the occlusion step the wall pieces would be reported too.
  Params off;
  off.occlusion_margin = 100.0;
  EXPECT_GT(run(sc, off).size(), 1u);
}

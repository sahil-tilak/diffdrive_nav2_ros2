// Tests written against the spec. Tests 1-6 FAIL until the three empty blocks in
// src/tracking/kalman_tracker.cpp are filled in. The static-filter test passes already.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "diffdrive_nav2_ros2/tracking/kalman_tracker.hpp"

using namespace diffdrive_nav2_ros2::tracking;  // NOLINT

namespace
{
constexpr double kQ = 0.1;
constexpr double kR = 0.02;
constexpr double kVelVar = 1.0;

ConstantVelocityKF makeKf(double x, double y)
{
  return ConstantVelocityKF(Vec2(x, y), kQ, kR, kVelVar);
}

Track makeTrack(std::uint32_t id, double x, double y)
{
  return Track{id, makeKf(x, y), 0};
}

using Pairs = std::set<std::pair<std::size_t, std::size_t>>;

Pairs toSet(const Association & a)
{
  return Pairs(a.matches.begin(), a.matches.end());
}
}  // namespace

// Block 1
TEST(Predict, MovesByVdtAndGrowsP)
{
  auto kf = makeKf(0.0, 0.0);
  kf.setState(Vec4(0.0, 0.0, 1.0, 2.0));
  kf.setCovariance(Mat4::Identity());
  const double trace_before = kf.P().trace();
  kf.predict(0.5);
  EXPECT_NEAR(kf.x()(0), 0.5, 1e-9);
  EXPECT_NEAR(kf.x()(1), 1.0, 1e-9);
  EXPECT_NEAR(kf.x()(2), 1.0, 1e-9);  // velocity unchanged
  EXPECT_NEAR(kf.x()(3), 2.0, 1e-9);
  EXPECT_GT(kf.P().trace(), trace_before);
}

// Block 2
TEST(Update, MovesTowardMeasurementAndShrinksP)
{
  auto kf = makeKf(0.0, 0.0);
  kf.setState(Vec4::Zero());
  kf.setCovariance(Mat4::Identity());
  const Vec2 z(1.0, 1.0);
  const double dist_before = (kf.position() - z).norm();
  const double trace_before = kf.P().trace();
  kf.update(z);
  EXPECT_LT((kf.position() - z).norm(), dist_before);
  EXPECT_GT(kf.position().x(), 0.0);
  EXPECT_LE(kf.position().x(), 1.0);  // does not overshoot the measurement
  EXPECT_LT(kf.P().trace(), trace_before);
}

// Blocks 1 + 2 together. Tolerance and noise settings are UNVERIFIED until the blocks exist.
TEST(Filter, VelocityConverges)
{
  const double dt = 0.1;
  const Vec2 v_true(0.5, 0.2);
  const Vec2 p0(1.0, -1.0);
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.02);

  auto measure = [&](int k) {
      return Vec2(p0 + v_true * (dt * k) + Vec2(noise(rng), noise(rng)));
    };

  auto kf = ConstantVelocityKF(measure(0), kQ, kR, kVelVar);
  Vec2 v_sum = Vec2::Zero();
  int n = 0;
  for (int k = 1; k <= 20; ++k) {
    kf.predict(dt);
    kf.update(measure(k));
    if (k >= 16) {
      v_sum += kf.velocity();
      ++n;
    }
  }
  const Vec2 v_mean = v_sum / n;
  EXPECT_NEAR(v_mean.x(), v_true.x(), 0.1);
  EXPECT_NEAR(v_mean.y(), v_true.y(), 0.1);
}

// Block 3
TEST(Associate, NearestInsideGate)
{
  std::vector<Track> tracks{makeTrack(1, 0.0, 0.0)};
  std::vector<Vec2> dets{Vec2(1.0, 0.0), Vec2(0.3, 0.0)};  // nearest is index 1
  const auto a = associate(tracks, dets, 0.5);
  EXPECT_EQ(toSet(a), (Pairs{{0, 1}}));
  EXPECT_EQ(a.unmatched_detections, (std::vector<std::size_t>{0}));
  EXPECT_TRUE(a.unmatched_tracks.empty());
}

// Block 3
TEST(Associate, NoMatchOutsideGate)
{
  std::vector<Track> tracks{makeTrack(1, 0.0, 0.0)};
  std::vector<Vec2> dets{Vec2(2.0, 0.0)};
  const auto a = associate(tracks, dets, 0.5);
  EXPECT_TRUE(a.matches.empty());
  EXPECT_EQ(a.unmatched_detections, (std::vector<std::size_t>{0}));
  EXPECT_EQ(a.unmatched_tracks, (std::vector<std::size_t>{0}));
}

// Block 3. Distances: T0-D0 0.22, T0-D1 0.25, T1-D0 0.18, T1-D1 0.65 (gate 0.7).
// Correct: T1<->D0 (smallest pair) then T0<->D1. A per-track loop gives T0->D0 and T1->D1.
TEST(Associate, TwoTracksNotSwapped)
{
  std::vector<Track> tracks{makeTrack(1, 0.0, 0.0), makeTrack(2, 0.4, 0.0)};
  std::vector<Vec2> dets{Vec2(0.22, 0.0), Vec2(-0.25, 0.0)};
  const auto a = associate(tracks, dets, 0.7);
  EXPECT_EQ(toSet(a), (Pairs{{0, 1}, {1, 0}}));
  EXPECT_TRUE(a.unmatched_detections.empty());
  EXPECT_TRUE(a.unmatched_tracks.empty());
}

// Our code: should pass now.
TEST(StaticFilter, StaticTrackNotPublished)
{
  std::vector<Track> tracks{makeTrack(1, 0.0, 0.0), makeTrack(2, 1.0, 1.0)};
  tracks[0].kf.setState(Vec4(0.0, 0.0, 0.01, 0.0));  // speed 0.01, static
  tracks[1].kf.setState(Vec4(1.0, 1.0, 0.3, 0.0));   // speed 0.3, moving
  const auto out = filterMoving(tracks, 0.1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 2u);
}

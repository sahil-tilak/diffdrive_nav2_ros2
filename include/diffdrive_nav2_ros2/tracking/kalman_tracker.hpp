// Constant-velocity Kalman tracker for 2D obstacle positions. Pure C++17 + Eigen, no ROS types.
// State per track: [x, y, vx, vy]. Measurement: [x, y].
//
// Three blocks are left EMPTY on purpose (see kalman_tracker.cpp):
//   ConstantVelocityKF::predict, ConstantVelocityKF::update, associate.
#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace diffdrive_nav2_ros2::tracking
{

using Vec2 = Eigen::Vector2d;
using Vec4 = Eigen::Vector4d;
using Mat2 = Eigen::Matrix2d;
using Mat4 = Eigen::Matrix4d;
using Mat24 = Eigen::Matrix<double, 2, 4>;

// --- model matrices (given) ---
Mat4 makeF(double dt);                        // state transition, constant velocity
Mat4 makeQ(double dt, double q_accel);        // white-noise acceleration, q_accel in m^2/s^3
Mat24 makeH();                                // measurement matrix, picks [x, y]
Mat2 makeR(double r_meas);                    // measurement noise, r_meas is a std in metres

class ConstantVelocityKF
{
public:
  // Starts at [z.x, z.y, 0, 0]; P = diag(r^2, r^2, init_vel_var, init_vel_var).
  ConstantVelocityKF(const Vec2 & z, double q_accel, double r_meas, double init_vel_var);

  void predict(double dt);          // EMPTY BLOCK 1 (yours)
  void update(const Vec2 & z);      // EMPTY BLOCK 2 (yours)

  const Vec4 & x() const {return x_;}
  const Mat4 & P() const {return P_;}
  void setState(const Vec4 & x) {x_ = x;}
  void setCovariance(const Mat4 & P) {P_ = P;}

  Vec2 position() const {return x_.head<2>();}
  Vec2 velocity() const {return x_.tail<2>();}
  double speed() const {return velocity().norm();}

private:
  Vec4 x_;
  Mat4 P_;
  double q_accel_;
  double r_meas_;
};

struct Track
{
  std::uint32_t id;
  ConstantVelocityKF kf;
  int missed = 0;                   // scans in a row without a match
};

struct Association
{
  std::vector<std::pair<std::size_t, std::size_t>> matches;  // (track index, detection index)
  std::vector<std::size_t> unmatched_detections;
  std::vector<std::size_t> unmatched_tracks;
};

// EMPTY BLOCK 3 (yours). Greedy nearest neighbour with a Euclidean gate.
Association associate(
  const std::vector<Track> & tracks, const std::vector<Vec2> & detections, double gate);

struct TrackerParams
{
  double q_accel = 0.5;        // m^2/s^3, process noise: allows turns without lag
  double r_meas = 0.05;        // m, measurement std: centroid jitter of a few cm
  double gate = 0.5;           // m, max match distance: well above 0.025 m per scan motion
  int max_missed = 5;          // scans, 0.5 s at 10 Hz
  double min_speed = 0.1;      // m/s, tracks slower than this are not published
  double init_vel_var = 1.0;   // (m/s)^2, velocity unknown at birth
};

class TrackManager
{
public:
  explicit TrackManager(const TrackerParams & p)
  : p_(p) {}

  // dt > 0: predict all tracks first. dt <= 0: skip predict (first scan or bad stamps).
  // Then associate, update matched, age unmatched (delete when missed > max_missed),
  // and create a new track for each unmatched detection.
  void step(const std::vector<Vec2> & detections, double dt);

  const std::vector<Track> & tracks() const {return tracks_;}

private:
  TrackerParams p_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_ = 1;
};

// Static filter: keep only tracks with speed >= min_speed.
std::vector<Track> filterMoving(const std::vector<Track> & tracks, double min_speed);

}  // namespace diffdrive_nav2_ros2::tracking

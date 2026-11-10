#include "diffdrive_nav2_ros2/tracking/kalman_tracker.hpp"

#include <algorithm>
#include <tuple>

namespace diffdrive_nav2_ros2::tracking
{

Mat4 makeF(double dt)
{
  Mat4 F = Mat4::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

Mat4 makeQ(double dt, double q_accel)
{
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  Mat4 Q = Mat4::Zero();
  Q(0, 0) = q_accel * dt3 / 3.0;
  Q(0, 2) = Q(2, 0) = q_accel * dt2 / 2.0;
  Q(2, 2) = q_accel * dt;
  Q(1, 1) = q_accel * dt3 / 3.0;
  Q(1, 3) = Q(3, 1) = q_accel * dt2 / 2.0;
  Q(3, 3) = q_accel * dt;
  return Q;
}

Mat24 makeH()
{
  Mat24 H = Mat24::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  return H;
}

Mat2 makeR(double r_meas)
{
  return Mat2::Identity() * (r_meas * r_meas);
}

ConstantVelocityKF::ConstantVelocityKF(
  const Vec2 & z, double q_accel, double r_meas, double init_vel_var)
: x_(Vec4::Zero()), P_(Mat4::Zero()), q_accel_(q_accel), r_meas_(r_meas)
{
  x_.head<2>() = z;
  P_(0, 0) = P_(1, 1) = r_meas * r_meas;
  P_(2, 2) = P_(3, 3) = init_vel_var;
}

// ============================================================================
// EMPTY BLOCK 1: predict(dt)          Checked by: Predict.MovesByVdtAndGrowsP,
//                                                 Filter.VelocityConverges
//
// Formulas:   x = F x          P = F P F^T + Q
//
// Steps:
//   1. Build F for this dt (helper: makeF).                      F is 4x4
//   2. Build Q for this dt (helper: makeQ, uses member q_accel_). Q is 4x4
//   3. New state: F times the old state (member x_).             x_ is 4x1
//   4. New covariance: F times P_ times F transposed, plus Q.    P_ is 4x4
//   5. Store both results back into x_ and P_.
//
// Sizes: F 4x4, Q 4x4, x_ 4x1, P_ 4x4. Every intermediate is 4x4 or 4x1.
//
// Eigen calls you will need (list only):
//   operator*, operator+, .transpose()
//
// Common pitfall: building F and Q once (e.g. in the constructor). dt changes
// every scan, so both must be built again on every call.
// ============================================================================
void ConstantVelocityKF::predict(double dt)
{
  Mat4 F = makeF(dt);
  Mat4 Q = makeQ(dt, q_accel_);
  
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

// ============================================================================
// EMPTY BLOCK 2: update(z)            Checked by: Update.MovesTowardMeasurementAndShrinksP,
//                                                 Filter.VelocityConverges
//
// Formulas:   y = z - H x        S = H P H^T + R        K = P H^T S^-1
//             x = x + K y        P = (I - K H) P
//
// Steps:
//   1. Get H (helper: makeH) and R (helper: makeR, uses member r_meas_).
//   2. Innovation y: the measurement z minus the predicted measurement (H times x_).
//   3. Innovation covariance S: H times P_ times H transposed, plus R.
//   4. Kalman gain K: P_ times H transposed times the inverse of S.
//   5. Correct the state: x_ plus K times y.
//   6. Correct the covariance: identity minus K times H, all times P_.
//
// Sizes: z 2x1, H 2x4, R 2x2, y 2x1, S 2x2, K 4x2, identity 4x4,
//        x_ 4x1, P_ 4x4.
//
// Eigen calls you will need (list only):
//   operator*, operator+, operator-, .transpose(), .inverse(), Mat4::Identity()
//
// Common pitfall: wrong multiplication order when building K. Check the sizes:
// (4x4)(4x2)(2x2) gives 4x2. If you get a shape that does not match, Eigen
// will not compile it. Also use the P_ from BEFORE you change it in step 6.
// ============================================================================
void ConstantVelocityKF::update(const Vec2 & z)
{
  Mat24 H = makeH();
  Mat2 R = makeR(r_meas_);

  Vec2 y = z - H*x_;
  Mat2 S = H*P_*H.transpose() + R;
  Eigen::Matrix<double, 4, 2> K = P_*H.transpose()*S.inverse();
  x_ = x_ + K*y;
  P_ = (Mat4::Identity() - K*H)*P_;
}

// ============================================================================
// EMPTY BLOCK 3: associate(tracks, detections, gate)
//                                     Checked by: Associate.NearestInsideGate,
//                                                 Associate.NoMatchOutsideGate,
//                                                 Associate.TwoTracksNotSwapped
//
// Greedy nearest neighbour with a Euclidean gate. Steps:
//   1. Distance table: for every pair (track i, detection j), the Euclidean
//      distance from the track's position (kf.position()) to the detection.
//      The table is N_tracks x N_detections.
//   2. Keep only pairs with distance <= gate.
//   3. Repeatedly pick the smallest remaining pair whose track AND detection
//      are both still unused.
//   4. Add that pair (track index, detection index) to matches, and mark the
//      track and the detection as used.
//   5. Stop when no valid pair is left.
//   6. Leftover detections go to unmatched_detections, leftover tracks go to
//      unmatched_tracks. Every index ends up in exactly one list.
//
// Helpers / calls (list only):
//   kf.position() (a Vec2), (a - b).norm(), std::vector<bool> for the used flags,
//   std::sort on a list of (distance, track index, detection index)
//
// Common pitfall: looping track by track and giving each track its nearest
// detection. An early track can take a detection that is closer to a later
// track, and the pairs come out swapped. Always pick the global smallest
// remaining pair.
// ============================================================================
Association associate(
  const std::vector<Track> & tracks, const std::vector<Vec2> & detections, double gate)
{
  Association result;
  std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
  for(std::size_t i = 0; i < tracks.size(); ++i){
    for(std::size_t j = 0; j < detections.size(); ++j){
      double d = (tracks[i].kf.position() - detections[j]).norm();
      if(d <= gate){
        pairs.push_back({d, i, j});
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());

  std::vector<bool> track_used(tracks.size(), false);
  std::vector<bool> det_used(detections.size(), false);
  
  for(const auto & [d, i, j] : pairs){
    if(track_used[i] || det_used[j]){
      continue;
    }
    result.matches.push_back({i, j});
    track_used[i] = true;
    det_used[j] = true;
  }

  for(std::size_t i = 0; i < tracks.size(); ++i){
    if (!track_used[i]){
      result.unmatched_tracks.push_back(i);
    }
  }

  for (std::size_t j = 0; j < detections.size(); ++j){
    if(!det_used[j]){
      result.unmatched_detections.push_back(j);
    }
  }
  return result;
}

void TrackManager::step(const std::vector<Vec2> & detections, double dt)
{
  if (dt > 0.0) {
    for (auto & t : tracks_) {
      t.kf.predict(dt);
    }
  }

  const Association a = associate(tracks_, detections, p_.gate);

  for (const auto & [ti, di] : a.matches) {
    tracks_[ti].kf.update(detections[di]);
    tracks_[ti].missed = 0;
  }
  for (const std::size_t ti : a.unmatched_tracks) {
    ++tracks_[ti].missed;
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & t) {return t.missed > p_.max_missed;}),
    tracks_.end());

  for (const std::size_t di : a.unmatched_detections) {
    tracks_.push_back(
      Track{next_id_++,
        ConstantVelocityKF(detections[di], p_.q_accel, p_.r_meas, p_.init_vel_var), 0});
  }
}

std::vector<Track> filterMoving(const std::vector<Track> & tracks, double min_speed)
{
  std::vector<Track> out;
  for (const auto & t : tracks) {
    if (t.kf.speed() >= min_speed) {
      out.push_back(t);
    }
  }
  return out;
}

}  // namespace diffdrive_nav2_ros2::tracking

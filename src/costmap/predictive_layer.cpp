#include "diffdrive_nav2_ros2/costmap/predictive_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "diffdrive_nav2_ros2/prediction/obstacle_prediction.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace pred = diffdrive_nav2_ros2::prediction;

namespace diffdrive_nav2_ros2
{

void PredictiveLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PredictiveLayer: failed to lock the costmap node");
  }

  // Parameters live under "<layer name>." (for example predictive_layer.horizon).
  // Note: doubles must be written with a decimal point in YAML (2.0, not 2), ints without.
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/tracked_obstacles")));
  declareParameter("horizon", rclcpp::ParameterValue(1.5));
  declareParameter("time_step", rclcpp::ParameterValue(0.1));
  declareParameter("obstacle_radius", rclcpp::ParameterValue(0.22));
  declareParameter("extra_margin", rclcpp::ParameterValue(0.0));
  declareParameter("max_cost", rclcpp::ParameterValue(252));
  declareParameter("min_cost", rclcpp::ParameterValue(140));
  declareParameter("max_msg_age", rclcpp::ParameterValue(0.5));
  declareParameter("max_tracks", rclcpp::ParameterValue(10));

  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic_);
  node->get_parameter(getFullName("horizon"), horizon_);
  node->get_parameter(getFullName("time_step"), time_step_);
  node->get_parameter(getFullName("obstacle_radius"), obstacle_radius_);
  node->get_parameter(getFullName("extra_margin"), extra_margin_);
  node->get_parameter(getFullName("max_cost"), max_cost_);
  node->get_parameter(getFullName("min_cost"), min_cost_);
  node->get_parameter(getFullName("max_msg_age"), max_msg_age_);
  node->get_parameter(getFullName("max_tracks"), max_tracks_);

  // Sanity limits. 253 and 254 would be a collision for MPPI's CostCritic, and a prediction
  // is only a guess, so the cost is never allowed above 252.
  if (max_cost_ > nav2_costmap_2d::MAX_NON_OBSTACLE) {
    RCLCPP_WARN(
      logger_, "%s: max_cost %d is lethal for MPPI, using %d", name_.c_str(), max_cost_,
      static_cast<int>(nav2_costmap_2d::MAX_NON_OBSTACLE));
    max_cost_ = nav2_costmap_2d::MAX_NON_OBSTACLE;
  }
  max_cost_ = std::max(max_cost_, 0);
  min_cost_ = std::clamp(min_cost_, 0, max_cost_);
  if (horizon_ <= 0.0) {
    RCLCPP_WARN(logger_, "%s: horizon must be > 0, using 1.5", name_.c_str());
    horizon_ = 1.5;
  }
  if (time_step_ < 0.01) {  // CPU guard: at most 100+ points per track
    RCLCPP_WARN(logger_, "%s: time_step below 0.01 s, using 0.01", name_.c_str());
    time_step_ = 0.01;
  }
  obstacle_radius_ = std::max(obstacle_radius_, 0.0);
  max_tracks_ = std::max(max_tracks_, 0);

  // Tell the user early if the two empty blocks are not filled in yet.
  const auto test_points = pred::predictPositions({0.0, 0.0, 1.0, 0.0}, 0.0, 1.0, 0.5);
  const pred::GridInfo test_grid{0.0, 0.0, 0.05, 2, 2};
  const auto test_cells = pred::cellsInDisc(0.025, 0.025, 0.01, test_grid);
  blocks_ready_ = !test_points.empty() && !test_cells.empty();
  if (!blocks_ready_) {
    RCLCPP_WARN(
      logger_, "%s: prediction blocks not filled; layer marks nothing", name_.c_str());
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node->create_subscription<TrackedArray>(
    topic_, rclcpp::QoS(1),
    [this](TrackedArray::ConstSharedPtr msg) {onTracks(std::move(msg));}, options);

  // A missing tracker must never make the costmap "not current" (that would block navigation).
  current_ = true;
  RCLCPP_INFO(
    logger_, "%s: topic %s, horizon %.2f s, step %.2f s, cost %d..%d, enabled %d",
    name_.c_str(), topic_.c_str(), horizon_, time_step_, max_cost_, min_cost_, enabled_);
}

void PredictiveLayer::onTracks(TrackedArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(msg);
}

void PredictiveLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  TrackedArray::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = latest_;
  }

  // Radius of one marked disc: the obstacle, plus the robot (MPPI checks the robot CENTRE only
  // and the inflation layer ignores our cells), plus an optional margin.
  const double radius =
    obstacle_radius_ + layered_costmap_->getInscribedRadius() + extra_margin_;

  std::vector<Mark> marks;
  if (blocks_ready_ && msg) {
    const rclcpp::Time now = clock_->now();
    const double age = (now - rclcpp::Time(msg->header.stamp, now.get_clock_type())).seconds();
    // Takes a predicted point from the track frame to the costmap frame. Identity if equal.
    tf2::Transform to_costmap;
    to_costmap.setIdentity();
    // Fills to_costmap. false = no TF at all, so nothing can be marked.
    const auto lookup_to_costmap = [this, &msg, &to_costmap]() {
        const std::string target = layered_costmap_->getGlobalFrameID();
        const std::string & source = msg->header.frame_id;
        if (source == target) {
          return true;  // no TF needed
        }
        // No timeout on either lookup: the map update thread must never wait for TF.
        geometry_msgs::msg::TransformStamped transform;
        try {
          transform = tf_->lookupTransform(target, source, tf2_ros::fromMsg(msg->header.stamp));
          RCLCPP_INFO_THROTTLE(
            logger_, *clock_, 5000, "%s: TF %s <- %s, path: stamp",
            name_.c_str(), target.c_str(), source.c_str());
        } catch (const tf2::TransformException & stamp_error) {
          // map -> odom changes slowly: the latest one is close to the one at the stamp.
          try {
            transform = tf_->lookupTransform(target, source, tf2::TimePointZero);
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 5000, "%s: TF %s <- %s, path: latest (fallback), at stamp: %s",
              name_.c_str(), target.c_str(), source.c_str(), stamp_error.what());
          } catch (const tf2::TransformException & latest_error) {
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 5000, "%s: no TF %s <- %s, marking nothing: %s",
              name_.c_str(), target.c_str(), source.c_str(), latest_error.what());
            return false;
          }
        }
        tf2::fromMsg(transform.transform, to_costmap);
        return true;
      };
    if (age > max_msg_age_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "%s: latest tracks are %.2f s old (max %.2f s), marking nothing",
        name_.c_str(), age, max_msg_age_);
    } else if (age < -0.05) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "%s: tracks are stamped %.2f s in the future, marking nothing",
        name_.c_str(), -age);
    } else if (lookup_to_costmap()) {
      // The tracks describe the obstacles at the message stamp. The first guess to mark is
      // "now" (t = age), the last one is horizon seconds later.
      const double t_start = std::max(age, 0.0);
      const std::size_t count =
        std::min(msg->obstacles.size(), static_cast<std::size_t>(max_tracks_));
      for (std::size_t i = 0; i < count; ++i) {
        const auto & o = msg->obstacles[i];
        if (!std::isfinite(o.x) || !std::isfinite(o.y) ||
          !std::isfinite(o.vx) || !std::isfinite(o.vy))
        {
          continue;
        }
        const auto points = pred::predictPositions(
          {o.x, o.y, o.vx, o.vy}, t_start, t_start + horizon_, time_step_);
        for (const auto & p : points) {
          const tf2::Vector3 q = to_costmap * tf2::Vector3(p.x, p.y, 0.0);
          marks.push_back(
            {q.x(), q.y(), pred::predictedCost(p.t - t_start, horizon_, max_cost_, min_cost_)});
        }
      }
    }
  }

  // Area covered by the new marks.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  double new_min_x = kInf;
  double new_min_y = kInf;
  double new_max_x = -kInf;
  double new_max_y = -kInf;
  for (const auto & m : marks) {
    new_min_x = std::min(new_min_x, m.x - radius);
    new_min_y = std::min(new_min_y, m.y - radius);
    new_max_x = std::max(new_max_x, m.x + radius);
    new_max_y = std::max(new_max_y, m.y + radius);
  }

  // Report new area AND last cycle's area, so old marks are reset by the master grid.
  if (have_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }
  if (!marks.empty()) {
    *min_x = std::min(*min_x, new_min_x);
    *min_y = std::min(*min_y, new_min_y);
    *max_x = std::max(*max_x, new_max_x);
    *max_y = std::max(*max_y, new_max_y);
  }
  have_last_bounds_ = !marks.empty();
  last_min_x_ = new_min_x;
  last_min_y_ = new_min_y;
  last_max_x_ = new_max_x;
  last_max_y_ = new_max_y;

  std::lock_guard<std::mutex> lock(mutex_);
  marks_ = std::move(marks);
  disc_radius_ = radius;
}

void PredictiveLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::vector<Mark> marks;
  double radius = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    marks = marks_;
    radius = disc_radius_;
  }
  if (marks.empty()) {
    return;
  }

  const pred::GridInfo grid{
    master_grid.getOriginX(), master_grid.getOriginY(), master_grid.getResolution(),
    master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY()};
  unsigned char * data = master_grid.getCharMap();

  for (const auto & m : marks) {
    for (const auto & c : pred::cellsInDisc(m.x, m.y, radius, grid)) {
      const int i = static_cast<int>(c.mx);
      const int j = static_cast<int>(c.my);
      if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
        continue;  // outside the area Nav2 is updating this cycle
      }
      unsigned char & cell = data[master_grid.getIndex(c.mx, c.my)];
      // Keep the higher cost. NO_INFORMATION (255) counts as "nothing known": overwrite it.
      if (cell == nav2_costmap_2d::NO_INFORMATION || cell < m.cost) {
        cell = m.cost;
      }
    }
  }
}

void PredictiveLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  marks_.clear();
}

}  // namespace diffdrive_nav2_ros2

PLUGINLIB_EXPORT_CLASS(diffdrive_nav2_ros2::PredictiveLayer, nav2_costmap_2d::Layer)

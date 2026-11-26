// Nav2 costmap layer: marks where tracked moving obstacles WILL be in the next ~1-2 s.
//
// Plumbing only. The maths is in prediction/obstacle_prediction.hpp (predictPositions,
// cellsInDisc, predictedCost). Works in the local and the global costmap: if the costmap frame
// (map) differs from the frame of the tracks (odom), predicted points are transformed with TF.
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diffdrive_nav2_ros2/msg/tracked_obstacle_array.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_nav2_ros2
{

class PredictiveLayer : public nav2_costmap_2d::Layer
{
public:
  PredictiveLayer() = default;
  ~PredictiveLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override {return false;}

private:
  using TrackedArray = diffdrive_nav2_ros2::msg::TrackedObstacleArray;

  // One predicted position with the cost it offers to the cells around it.
  struct Mark
  {
    double x;
    double y;
    unsigned char cost;
  };

  void onTracks(TrackedArray::ConstSharedPtr msg);

  // Parameters (read once in onInitialize).
  std::string topic_;
  double horizon_ = 1.5;
  double time_step_ = 0.1;
  double obstacle_radius_ = 0.22;
  double extra_margin_ = 0.0;
  int max_cost_ = 252;
  int min_cost_ = 140;
  double max_msg_age_ = 0.5;
  int max_tracks_ = 10;

  rclcpp::Subscription<TrackedArray>::SharedPtr sub_;

  // Shared between the subscription thread and the map-update thread.
  std::mutex mutex_;
  TrackedArray::ConstSharedPtr latest_;
  std::vector<Mark> marks_;      // computed in updateBounds, used in updateCosts
  double disc_radius_ = 0.0;     // obstacle + robot (inscribed) + margin, set in updateBounds

  // Area marked in the previous update: it must be reset in the next one.
  bool have_last_bounds_ = false;
  double last_min_x_ = 0.0;
  double last_min_y_ = 0.0;
  double last_max_x_ = 0.0;
  double last_max_y_ = 0.0;

  bool blocks_ready_ = false;    // false while the two empty blocks are still empty
};

}  // namespace diffdrive_nav2_ros2

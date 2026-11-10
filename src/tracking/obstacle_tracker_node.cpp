// Thin ROS 2 wrapper around tracking::TrackManager.
// /detected_obstacles (PoseArray, odom) -> tracks -> /tracked_obstacles + /tracked_obstacles/markers
// dt = difference of header stamps of two consecutive messages. Only moving tracks are published.
#include <memory>
#include <vector>

#include "diffdrive_nav2_ros2/msg/tracked_obstacle.hpp"
#include "diffdrive_nav2_ros2/msg/tracked_obstacle_array.hpp"
#include "diffdrive_nav2_ros2/tracking/kalman_tracker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace trk = diffdrive_nav2_ros2::tracking;
using diffdrive_nav2_ros2::msg::TrackedObstacle;
using diffdrive_nav2_ros2::msg::TrackedObstacleArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class ObstacleTracker : public rclcpp::Node
{
public:
  ObstacleTracker()
  : Node("obstacle_tracker")
  {
    trk::TrackerParams p;
    p.q_accel = declare_parameter<double>("q_accel", p.q_accel);
    p.r_meas = declare_parameter<double>("r_meas", p.r_meas);
    p.gate = declare_parameter<double>("gate", p.gate);
    p.max_missed = declare_parameter<int>("max_missed", p.max_missed);
    p.min_speed = declare_parameter<double>("min_speed", p.min_speed);
    p.init_vel_var = declare_parameter<double>("init_vel_var", p.init_vel_var);
    min_speed_ = p.min_speed;
    manager_ = std::make_unique<trk::TrackManager>(p);

    pub_ = create_publisher<TrackedObstacleArray>("tracked_obstacles", 10);
    marker_pub_ = create_publisher<MarkerArray>("tracked_obstacles/markers", 10);
    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "detected_obstacles", 10,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {onDetections(*msg);});
    RCLCPP_INFO(
      get_logger(), "ready: q_accel=%.3f r_meas=%.3f gate=%.2f max_missed=%d min_speed=%.2f",
      p.q_accel, p.r_meas, p.gate, p.max_missed, p.min_speed);
  }

private:
  void onDetections(const geometry_msgs::msg::PoseArray & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp);
    double dt = 0.0;  // first message: no predict
    if (have_last_) {
      dt = (stamp - last_stamp_).seconds();
      if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "non-positive dt=%.4f, skipping predict", dt);
        dt = 0.0;
      }
    }
    last_stamp_ = stamp;
    have_last_ = true;

    std::vector<trk::Vec2> dets;
    dets.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      dets.emplace_back(pose.position.x, pose.position.y);
    }
    manager_->step(dets, dt);
    const auto moving = trk::filterMoving(manager_->tracks(), min_speed_);

    TrackedObstacleArray out;
    out.header = msg.header;
    MarkerArray markers;
    Marker clear;
    clear.header = msg.header;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (const auto & t : moving) {
      const auto & x = t.kf.x();
      TrackedObstacle o;
      o.id = t.id;
      o.x = x(0);
      o.y = x(1);
      o.vx = x(2);
      o.vy = x(3);
      out.obstacles.push_back(o);

      Marker m;
      m.header = msg.header;
      m.ns = "tracked_velocity";
      m.id = static_cast<int>(t.id);
      m.type = Marker::ARROW;
      m.action = Marker::ADD;
      geometry_msgs::msg::Point a, b;
      a.x = o.x;
      a.y = o.y;
      b.x = o.x + o.vx;  // arrow length = velocity * 1 s
      b.y = o.y + o.vy;
      m.points = {a, b};
      m.scale.x = 0.03;  // shaft diameter
      m.scale.y = 0.06;  // head diameter
      m.scale.z = 0.06;  // head length
      m.color.r = 1.0f;
      m.color.g = 0.5f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      markers.markers.push_back(m);
    }
    pub_->publish(out);
    marker_pub_->publish(markers);
  }

  double min_speed_ = 0.1;
  bool have_last_ = false;
  rclcpp::Time last_stamp_{0, 0, RCL_SYSTEM_TIME};
  std::unique_ptr<trk::TrackManager> manager_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleTracker>());
  rclcpp::shutdown();
  return 0;
}

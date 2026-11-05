// Thin ROS 2 wrapper around perception::findClusters.
// /scan -> clusters -> centroids transformed to target_frame at the scan stamp -> /detected_obstacles
// Publishes one PoseArray per scan, also when it is empty.
// Centroid bias (not fixed): the point is on the visible face, up to 0.15 m in front of a 0.3 m box centre.
#include <memory>
#include <string>

#include "diffdrive_nav2_ros2/perception/scan_clustering.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

namespace perc = diffdrive_nav2_ros2::perception;
using sensor_msgs::msg::LaserScan;

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    params_.range_min = declare_parameter<double>("range_min", 0.15);
    params_.range_max = declare_parameter<double>("range_max", 8.0);
    params_.gap_base = declare_parameter<double>("gap_base", 0.03);
    params_.gap_ratio = declare_parameter<double>("gap_ratio", 0.112);
    params_.min_points = static_cast<std::size_t>(declare_parameter<int>("min_points", 3));
    params_.max_length = declare_parameter<double>("max_length", 0.6);
    params_.occlusion_margin = declare_parameter<double>("occlusion_margin", 0.3);
  }

  // Needs shared_from_this(), so it cannot run in the constructor.
  void init()
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // MessageFilter calls waitForTransform, which needs a timer interface on the buffer.
    tf_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("detected_obstacles", 10);
    sub_.subscribe(this, "scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    filter_ = std::make_unique<tf2_ros::MessageFilter<LaserScan>>(
      sub_, *tf_buffer_, target_frame_, 10, get_node_logging_interface(),
      get_node_clock_interface());
    filter_->registerCallback(&ObstacleDetector::onScan, this);
    RCLCPP_INFO(
      get_logger(), "ready: target_frame=%s gap=%.3f+%.3f*r min_points=%zu max_length=%.2f",
      target_frame_.c_str(), params_.gap_base, params_.gap_ratio, params_.min_points,
      params_.max_length);
  }

private:
  void onScan(const LaserScan::ConstSharedPtr & msg)
  {
    auto clusters = perc::findClusters(msg->ranges, msg->angle_min, msg->angle_increment, params_);
    geometry_msgs::msg::PoseArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
    for (const auto & c : clusters) {
      geometry_msgs::msg::PoseStamped in;
      in.header = msg->header;
      in.pose.position.x = c.x;
      in.pose.position.y = c.y;
      in.pose.orientation.w = 1.0;
      try {
        out.poses.push_back(tf_buffer_->transform(in, target_frame_).pose);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "transform failed: %s", e.what());
      }
    }
    pub_->publish(out);
  }

  std::string target_frame_;
  perc::Params params_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<LaserScan> sub_;
  std::unique_ptr<tf2_ros::MessageFilter<LaserScan>> filter_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetector>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

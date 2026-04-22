#ifndef AUTONOMY_ROS2_WRAPPER__CONVERSIONS_HPP_
#define AUTONOMY_ROS2_WRAPPER__CONVERSIONS_HPP_

#include <string>

#include "autonomy_contracts/types.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros2_wrapper
{

autonomy_contracts::Pose2D poseFromRos(
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp);

geometry_msgs::msg::Pose poseToRos(const autonomy_contracts::Pose2D & pose);

autonomy_contracts::Twist2D twistFromRos(const geometry_msgs::msg::Twist & twist);

geometry_msgs::msg::Twist twistToRos(const autonomy_contracts::Twist2D & twist);

autonomy_contracts::Path2D pathFromRos(const nav_msgs::msg::Path & path);

nav_msgs::msg::Path pathToRos(
  const autonomy_contracts::Path2D & path,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

std::vector<autonomy_contracts::Obstacle2D> obstaclesFromMarkers(
  const visualization_msgs::msg::MarkerArray & markers);

visualization_msgs::msg::MarkerArray markersFromPerception(
  const autonomy_contracts::PerceptionResult2D & result,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

}  // namespace autonomy_ros2_wrapper

#endif  // AUTONOMY_ROS2_WRAPPER__CONVERSIONS_HPP_

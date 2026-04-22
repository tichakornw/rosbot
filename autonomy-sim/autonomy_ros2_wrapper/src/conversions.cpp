#include "autonomy_ros2_wrapper/conversions.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace autonomy_ros2_wrapper
{

autonomy_contracts::Pose2D poseFromRos(
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp)
{
  autonomy_contracts::Pose2D output;
  output.x = pose.position.x;
  output.y = pose.position.y;
  output.yaw = tf2::getYaw(pose.orientation);
  output.stamp_sec = stamp.seconds();
  return output;
}

geometry_msgs::msg::Pose poseToRos(const autonomy_contracts::Pose2D & pose)
{
  geometry_msgs::msg::Pose output;
  output.position.x = pose.x;
  output.position.y = pose.y;
  output.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  output.orientation = tf2::toMsg(q);
  return output;
}

autonomy_contracts::Twist2D twistFromRos(const geometry_msgs::msg::Twist & twist)
{
  autonomy_contracts::Twist2D output;
  output.linear_x = twist.linear.x;
  output.linear_y = twist.linear.y;
  output.angular_z = twist.angular.z;
  return output;
}

geometry_msgs::msg::Twist twistToRos(const autonomy_contracts::Twist2D & twist)
{
  geometry_msgs::msg::Twist output;
  output.linear.x = twist.linear_x;
  output.linear.y = twist.linear_y;
  output.angular.z = twist.angular_z;
  return output;
}

autonomy_contracts::Path2D pathFromRos(const nav_msgs::msg::Path & path)
{
  autonomy_contracts::Path2D output;
  output.poses.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    output.poses.push_back(poseFromRos(pose.pose, rclcpp::Time(pose.header.stamp)));
  }
  return output;
}

nav_msgs::msg::Path pathToRos(
  const autonomy_contracts::Path2D & path,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path output;
  output.header.frame_id = frame_id;
  output.header.stamp = stamp;
  output.poses.reserve(path.poses.size());

  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header = output.header;
    stamped_pose.pose = poseToRos(pose);
    output.poses.push_back(stamped_pose);
  }

  return output;
}

std::vector<autonomy_contracts::Obstacle2D> obstaclesFromMarkers(
  const visualization_msgs::msg::MarkerArray & markers)
{
  std::vector<autonomy_contracts::Obstacle2D> obstacles;
  obstacles.reserve(markers.markers.size());

  for (const auto & marker : markers.markers) {
    if (
      marker.action == visualization_msgs::msg::Marker::DELETE ||
      marker.action == visualization_msgs::msg::Marker::DELETEALL)
    {
      continue;
    }

    autonomy_contracts::Obstacle2D obstacle;
    obstacle.pose = poseFromRos(marker.pose, rclcpp::Time(marker.header.stamp));
    obstacle.radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
    obstacle.cost = marker.color.r > 0.0F ? marker.color.r : 1.0F;
    obstacles.push_back(obstacle);
  }

  return obstacles;
}

visualization_msgs::msg::MarkerArray markersFromPerception(
  const autonomy_contracts::PerceptionResult2D & result,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  for (const auto & obstacle : result.obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "autonomy_wrapper_obstacles";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = poseToRos(obstacle.pose);
    marker.scale.x = std::max(0.01, obstacle.radius * 2.0);
    marker.scale.y = std::max(0.01, obstacle.radius * 2.0);
    marker.scale.z = 0.1;
    marker.color.r = obstacle.cost;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 0.45F;
    markers.markers.push_back(marker);
  }

  for (const auto & detection : result.detections) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "autonomy_wrapper_detections";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = poseToRos(detection.pose);
    marker.scale.x = std::max(0.01, detection.size_x);
    marker.scale.y = std::max(0.01, detection.size_y);
    marker.scale.z = 0.1;
    marker.color.r = 0.0F;
    marker.color.g = 0.4F;
    marker.color.b = 1.0F;
    marker.color.a = static_cast<float>(std::max(0.1, detection.confidence));
    marker.text = detection.class_name;
    markers.markers.push_back(marker);
  }

  return markers;
}

}  // namespace autonomy_ros2_wrapper

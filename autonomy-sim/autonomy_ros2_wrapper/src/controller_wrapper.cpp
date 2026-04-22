#include "autonomy_ros2_wrapper/controller_wrapper.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "autonomy_ros2_wrapper/conversions.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::geometry_utils::euclidean_distance;

namespace autonomy_ros2_wrapper
{

namespace
{

template<typename Iter, typename Getter>
Iter minBy(Iter begin, Iter end, Getter get_compare_value)
{
  if (begin == end) {
    return end;
  }

  auto lowest = get_compare_value(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto value = get_compare_value(*it);
    if (value < lowest) {
      lowest = value;
      lowest_it = it;
    }
  }
  return lowest_it;
}

std::string joinNames(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  for (const auto & name : names) {
    stream << name << " ";
  }
  return stream.str();
}

}  // namespace

void ContractController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".dynamic_obstacles_topic",
    rclcpp::ParameterValue("autonomy_wrapper/dynamic_obstacles"));
  node->get_parameter(name_ + ".dynamic_obstacles_topic", dynamic_obstacles_topic_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  double transform_tolerance = 0.1;
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  auto & registry = autonomy_contracts::ComponentRegistry::instance();
  controller_ = registry.createController(controller_component_name_);
  if (!controller_) {
    const auto available = joinNames(registry.availableControllers());
    throw std::runtime_error(
      "Failed to create controller component '" + controller_component_name_ +
      "'. Available controllers: " + available);
  }

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

  RCLCPP_INFO(
    logger_,
    "Configured contract controller '%s' with component '%s'",
    name_.c_str(),
    controller_component_name_.c_str());
}

void ContractController::cleanup()
{
  controller_.reset();
  obstacle_sub_.reset();
  global_pub_.reset();
}

void ContractController::activate()
{
  auto node = node_.lock();
  obstacle_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    dynamic_obstacles_topic_,
    rclcpp::QoS(10),
    std::bind(&ContractController::obstacleMarkersCallback, this, std::placeholders::_1));
  global_pub_->on_activate();
}

void ContractController::deactivate()
{
  obstacle_sub_.reset();
  global_pub_->on_deactivate();
}

void ContractController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  (void)speed_limit;
  (void)percentage;
}

geometry_msgs::msg::TwistStamped ContractController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (!controller_) {
    RCLCPP_ERROR(logger_, "Controller component is not configured");
    return cmd_vel;
  }

  const auto transformed_plan = transformGlobalPlan(pose);

  autonomy_contracts::ControlInput2D input;
  input.robot_pose = autonomy_contracts::Pose2D{};
  input.robot_velocity = twistFromRos(velocity);
  input.reference_path = pathFromRos(transformed_plan);
  input.dynamic_obstacles = currentObstacles();
  input.dt_sec = 0.1;

  const auto output = controller_->computeCommand(input);
  if (!output.success) {
    RCLCPP_WARN(
      logger_,
      "Controller component '%s' failed: %s",
      controller_component_name_.c_str(),
      output.error_message.c_str());
    return cmd_vel;
  }

  cmd_vel.twist = twistToRos(output.command);
  return cmd_vel;
}

void ContractController::setPlan(const nav_msgs::msg::Path & path)
{
  if (global_pub_) {
    global_pub_->publish(path);
  }
  global_plan_ = path;
}

nav_msgs::msg::Path ContractController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan frame");
  }

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double dist_threshold =
    std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  auto transformation_begin = minBy(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & plan_pose) {
      return euclidean_distance(robot_pose, plan_pose);
    });

  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose;
      geometry_msgs::msg::PoseStamped transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin,
    transformation_end,
    std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);
  if (global_pub_) {
    global_pub_->publish(transformed_plan);
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting transformed plan has zero poses");
  }

  return transformed_plan;
}

bool ContractController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException &) {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      return false;
    }
    tf2::doTransform(in_pose, out_pose, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    return false;
  }
}

void ContractController::obstacleMarkersCallback(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacle_markers_ = *msg;
}

std::vector<autonomy_contracts::Obstacle2D> ContractController::currentObstacles() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  std::vector<autonomy_contracts::Obstacle2D> obstacles;
  obstacles.reserve(latest_obstacle_markers_.markers.size());
  const auto target_frame = costmap_ros_->getBaseFrameID();

  for (const auto & marker : latest_obstacle_markers_.markers) {
    if (
      marker.action == visualization_msgs::msg::Marker::DELETE ||
      marker.action == visualization_msgs::msg::Marker::DELETEALL)
    {
      continue;
    }

    geometry_msgs::msg::PoseStamped marker_pose;
    marker_pose.header = marker.header;
    marker_pose.pose = marker.pose;

    try {
      if (!marker.header.frame_id.empty() && marker.header.frame_id != target_frame) {
        marker_pose = tf_->transform(marker_pose, target_frame, tf2::durationFromSec(0.1));
      }

      autonomy_contracts::Obstacle2D obstacle;
      obstacle.pose = poseFromRos(marker_pose.pose, rclcpp::Time(marker_pose.header.stamp));
      obstacle.radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
      obstacle.cost = marker.color.r > 0.0F ? marker.color.r : 1.0F;
      obstacles.push_back(obstacle);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_,
        "Skipping dynamic obstacle marker that cannot transform to %s: %s",
        target_frame.c_str(),
        ex.what());
    }
  }

  return obstacles;
}

}  // namespace autonomy_ros2_wrapper

PLUGINLIB_EXPORT_CLASS(autonomy_ros2_wrapper::ContractController, nav2_core::Controller)

#include "autonomy_ros2_wrapper/planner_wrapper.hpp"

#include <algorithm>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "autonomy_ros2_wrapper/conversions.hpp"
#include "autonomy_reference_components/reference_components.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace autonomy_ros2_wrapper
{

namespace
{

std::string joinNames(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  for (const auto & name : names) {
    stream << name << " ";
  }
  return stream.str();
}

}  // namespace

void ContractPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  autonomy_reference_components::ensureReferenceComponentsLinked();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".planner_component", rclcpp::ParameterValue("bfs_grid_planner"));
  node_->get_parameter(name_ + ".planner_component", planner_component_name_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".dynamic_obstacles_topic",
    rclcpp::ParameterValue("autonomy_wrapper/dynamic_obstacles"));
  node_->get_parameter(name_ + ".dynamic_obstacles_topic", dynamic_obstacles_topic_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  auto & registry = autonomy_contracts::ComponentRegistry::instance();
  planner_ = registry.createPlanner(planner_component_name_);
  if (!planner_) {
    const auto available = joinNames(registry.availablePlanners());
    throw std::runtime_error(
      "Failed to create planner component '" + planner_component_name_ +
      "'. Available planners: " + available);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured contract planner '%s' with component '%s'",
    name_.c_str(),
    planner_component_name_.c_str());
}

void ContractPlanner::cleanup()
{
  planner_.reset();
  obstacle_sub_.reset();
}

void ContractPlanner::activate()
{
  obstacle_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    dynamic_obstacles_topic_,
    rclcpp::QoS(10),
    std::bind(&ContractPlanner::obstacleMarkersCallback, this, std::placeholders::_1));
}

void ContractPlanner::deactivate()
{
  obstacle_sub_.reset();
}

nav_msgs::msg::Path ContractPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = global_frame_;
  empty_path.header.stamp = node_->now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ContractPlanner requires start and goal in global frame '%s'",
      global_frame_.c_str());
    return empty_path;
  }

  if (!planner_) {
    RCLCPP_ERROR(node_->get_logger(), "Planner component is not configured");
    return empty_path;
  }

  autonomy_contracts::PlanningInput2D input;
  input.map = gridFromCostmap();
  input.start = poseFromRos(start.pose, rclcpp::Time(start.header.stamp));
  input.goal = poseFromRos(goal.pose, rclcpp::Time(goal.header.stamp));
  input.dynamic_obstacles = currentObstacles();

  const auto output = planner_->plan(input);
  if (!output.success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Planner component '%s' failed: %s",
      planner_component_name_.c_str(),
      output.error_message.c_str());
    return empty_path;
  }

  return pathToRos(output.path, global_frame_, node_->now());
}

autonomy_contracts::GridMap2D ContractPlanner::gridFromCostmap() const
{
  autonomy_contracts::GridMap2D grid;
  grid.width = costmap_->getSizeInCellsX();
  grid.height = costmap_->getSizeInCellsY();
  grid.resolution = costmap_->getResolution();
  grid.origin.x = costmap_->getOriginX();
  grid.origin.y = costmap_->getOriginY();
  grid.origin.yaw = 0.0;

  const size_t cell_count = static_cast<size_t>(grid.width) * grid.height;
  grid.occupancy.resize(cell_count, 0);
  grid.cost.resize(cell_count, 0.0F);

  for (uint32_t y = 0; y < grid.height; ++y) {
    for (uint32_t x = 0; x < grid.width; ++x) {
      const auto cost = costmap_->getCost(x, y);
      const size_t idx = static_cast<size_t>(y) * grid.width + x;
      grid.cost[idx] = static_cast<float>(cost);

      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        grid.occupancy[idx] = allow_unknown_ ? 0 : 255;
      } else if (
        cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        grid.occupancy[idx] = 100;
      } else {
        grid.occupancy[idx] = 0;
      }
    }
  }

  return grid;
}

void ContractPlanner::obstacleMarkersCallback(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacle_markers_ = *msg;
}

std::vector<autonomy_contracts::Obstacle2D> ContractPlanner::currentObstacles() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  std::vector<autonomy_contracts::Obstacle2D> obstacles;
  obstacles.reserve(latest_obstacle_markers_.markers.size());

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
      if (!marker.header.frame_id.empty() && marker.header.frame_id != global_frame_) {
        marker_pose = tf_->transform(marker_pose, global_frame_, tf2::durationFromSec(0.1));
      }

      autonomy_contracts::Obstacle2D obstacle;
      obstacle.pose = poseFromRos(marker_pose.pose, rclcpp::Time(marker_pose.header.stamp));
      obstacle.radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
      obstacle.cost = marker.color.r > 0.0F ? marker.color.r : 1.0F;
      obstacles.push_back(obstacle);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping dynamic obstacle marker that cannot transform to %s: %s",
        global_frame_.c_str(),
        ex.what());
    }
  }

  return obstacles;
}

}  // namespace autonomy_ros2_wrapper

PLUGINLIB_EXPORT_CLASS(autonomy_ros2_wrapper::ContractPlanner, nav2_core::GlobalPlanner)

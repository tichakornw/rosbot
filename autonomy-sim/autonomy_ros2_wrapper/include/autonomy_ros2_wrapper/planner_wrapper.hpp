#ifndef AUTONOMY_ROS2_WRAPPER__PLANNER_WRAPPER_HPP_
#define AUTONOMY_ROS2_WRAPPER__PLANNER_WRAPPER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy_contracts/registry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros2_wrapper
{

class ContractPlanner : public nav2_core::GlobalPlanner
{
public:
  ContractPlanner() = default;
  ~ContractPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  autonomy_contracts::GridMap2D gridFromCostmap() const;
  void obstacleMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  std::vector<autonomy_contracts::Obstacle2D> currentObstacles() const;

  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_ {nullptr};
  std::string name_;
  std::string global_frame_;

  std::string planner_component_name_ {"bfs_grid_planner"};
  std::string dynamic_obstacles_topic_ {"autonomy_wrapper/dynamic_obstacles"};
  bool allow_unknown_ {true};

  std::unique_ptr<autonomy_contracts::IPlannerComponent> planner_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_sub_;

  mutable std::mutex obstacles_mutex_;
  visualization_msgs::msg::MarkerArray latest_obstacle_markers_;
};

}  // namespace autonomy_ros2_wrapper

#endif  // AUTONOMY_ROS2_WRAPPER__PLANNER_WRAPPER_HPP_

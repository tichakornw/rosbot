#ifndef AUTONOMY_ROS2_WRAPPER__CONTROLLER_WRAPPER_HPP_
#define AUTONOMY_ROS2_WRAPPER__CONTROLLER_WRAPPER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy_contracts/registry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros2_wrapper
{

class ContractController : public nav2_core::Controller
{
public:
  ContractController() = default;
  ~ContractController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

private:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance) const;

  void obstacleMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  std::vector<autonomy_contracts::Obstacle2D> currentObstacles() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ContractController")};
  rclcpp::Clock::SharedPtr clock_;

  std::string controller_component_name_ {"pure_pursuit_controller"};
  std::string dynamic_obstacles_topic_ {"autonomy_wrapper/dynamic_obstacles"};
  rclcpp::Duration transform_tolerance_ {0, 0};
  nav_msgs::msg::Path global_plan_;

  std::unique_ptr<autonomy_contracts::IControllerComponent> controller_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_sub_;

  mutable std::mutex obstacles_mutex_;
  visualization_msgs::msg::MarkerArray latest_obstacle_markers_;
};

}  // namespace autonomy_ros2_wrapper

#endif  // AUTONOMY_ROS2_WRAPPER__CONTROLLER_WRAPPER_HPP_

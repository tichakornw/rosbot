#ifndef AUTONOMY_ROS2_WRAPPER__PERCEPTION_WRAPPER_NODE_HPP_
#define AUTONOMY_ROS2_WRAPPER__PERCEPTION_WRAPPER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy_contracts/registry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autonomy_ros2_wrapper
{

class PerceptionWrapperNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PerceptionWrapperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  autonomy_contracts::PerceptionInput2D inputFromImage(
    const sensor_msgs::msg::Image & image) const;

  std::string perception_component_name_ {"noop_perception"};
  std::string image_topic_ {"camera/image"};
  std::string depth_topic_ {"camera/depth_image"};
  std::string output_obstacles_topic_ {"autonomy_wrapper/dynamic_obstacles"};
  std::string output_frame_ {"map"};

  std::unique_ptr<autonomy_contracts::IPerceptionComponent> perception_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    obstacles_pub_;

  mutable std::mutex depth_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
};

}  // namespace autonomy_ros2_wrapper

#endif  // AUTONOMY_ROS2_WRAPPER__PERCEPTION_WRAPPER_NODE_HPP_

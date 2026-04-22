#include "autonomy_ros2_wrapper/perception_wrapper_node.hpp"

#include <cstring>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "autonomy_ros2_wrapper/conversions.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace autonomy_ros2_wrapper
{

PerceptionWrapperNode::PerceptionWrapperNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("perception_wrapper", options)
{
  declare_parameter("image_topic", "camera/image");
  declare_parameter("depth_topic", "camera/depth_image");
  declare_parameter("output_obstacles_topic", "autonomy_wrapper/dynamic_obstacles");
  declare_parameter("output_frame", "map");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionWrapperNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  get_parameter("image_topic", image_topic_);
  get_parameter("depth_topic", depth_topic_);
  get_parameter("output_obstacles_topic", output_obstacles_topic_);
  get_parameter("output_frame", output_frame_);

  auto & registry = autonomy_contracts::ComponentRegistry::instance();
  perception_ = registry.createPerception(perception_component_name_);
  if (!perception_) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to create perception component '%s'",
      perception_component_name_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    output_obstacles_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "Configured perception wrapper with component '%s'",
    perception_component_name_.c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionWrapperNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  obstacles_pub_->on_activate();

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_,
    rclcpp::QoS(10),
    std::bind(&PerceptionWrapperNode::depthCallback, this, std::placeholders::_1));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::QoS(10),
    std::bind(&PerceptionWrapperNode::imageCallback, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionWrapperNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  image_sub_.reset();
  depth_sub_.reset();
  obstacles_pub_->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionWrapperNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  image_sub_.reset();
  depth_sub_.reset();
  obstacles_pub_.reset();
  perception_.reset();
  latest_depth_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PerceptionWrapperNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!perception_) {
    return;
  }

  auto input = inputFromImage(*msg);
  const auto output = perception_->process(input);
  if (!output.success) {
    RCLCPP_WARN(
      get_logger(),
      "Perception component '%s' failed: %s",
      perception_component_name_.c_str(),
      output.error_message.c_str());
    return;
  }

  const auto frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;
  obstacles_pub_->publish(markersFromPerception(output.result, frame_id, now()));
}

void PerceptionWrapperNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_ = msg;
}

autonomy_contracts::PerceptionInput2D PerceptionWrapperNode::inputFromImage(
  const sensor_msgs::msg::Image & image) const
{
  autonomy_contracts::PerceptionInput2D input;
  input.width = image.width;
  input.height = image.height;
  input.encoding = image.encoding;
  input.image_data = image.data;
  input.stamp_sec = rclcpp::Time(image.header.stamp).seconds();

  sensor_msgs::msg::Image::SharedPtr depth;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth = latest_depth_;
  }

  if (!depth) {
    return input;
  }

  if (depth->encoding == "32FC1") {
    const size_t float_count = depth->data.size() / sizeof(float);
    input.depth_data.resize(float_count);
    std::memcpy(input.depth_data.data(), depth->data.data(), float_count * sizeof(float));
  } else if (depth->encoding == "16UC1") {
    const size_t value_count = depth->data.size() / sizeof(uint16_t);
    input.depth_data.resize(value_count);
    for (size_t i = 0; i < value_count; ++i) {
      uint16_t raw_value = 0;
      std::memcpy(&raw_value, depth->data.data() + i * sizeof(uint16_t), sizeof(uint16_t));
      input.depth_data[i] = static_cast<float>(raw_value) / 1000.0F;
    }
  }

  return input;
}

}  // namespace autonomy_ros2_wrapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autonomy_ros2_wrapper::PerceptionWrapperNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

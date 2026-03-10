#ifndef NAV2_MULTICOST_PLANNER__NAV2_MULTICOST_PLANNER_HPP_
#define NAV2_MULTICOST_PLANNER__NAV2_MULTICOST_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <unordered_set>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "multicost_array.hpp"
#include "multicost_compute.hpp"
#include "multicost_graph.hpp"
#include "multicost_pathfind.hpp"
#include "multicost_pathfind_factory.hpp"
#include "grid_state.hpp"
#include "single_optimal_path_finder.hpp"
#include <opencv2/opencv.hpp>
#include <limits>

namespace nav2_multicost_planner
{

class MulticostPlanner : public nav2_core::GlobalPlanner
{
public:
  MulticostPlanner() = default;
  ~MulticostPlanner() = default;

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
  std::shared_ptr<tf2_ros::Buffer> tf_;

  nav2_util::LifecycleNode::SharedPtr node_;

  nav2_costmap_2d::Costmap2D * costmap_;

  std::string global_frame_, name_;

  // Multicost planner components
  std::unique_ptr<SingleOptimalPathFinder<GridState>> path_finder_;
  std::unique_ptr<IMulticostPathfind> pathfind_algorithm_;

  unsigned int current_grid_width_;
  unsigned int current_grid_height_;

  // Parameters
  double distance_weight_;
  double cost_weight_;
  bool use_obstacle_cost_;
  std::string algorithm_name_;
  
  // Minimum obstacle clearance parameter (in meters)
  double min_obstacle_clearance_;
  
  // YOLO integration parameters
  bool use_yolo_detections_;
  double yolo_detection_cost_weight_;
  double yolo_detection_inflation_radius_;
  std::string yolo_markers_topic_;

  // YOLO detection subscriber (using marker array since its mapped correctly)
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr yolo_markers_sub_;
  
  // Store latest marker detections
  visualization_msgs::msg::MarkerArray::SharedPtr latest_markers_;
  std::mutex markers_mutex_;
  
  // Track which cells have YOLO-detected objects
  std::unordered_map<unsigned int, int> yolo_cost_overlay_;
  std::mutex overlay_mutex_;

  // Helper methods
  void initializePathFinder();
  void loadAlgorithm();
  void updateGridFromCostmap();
  void updateYoloOverlay();
  void yoloMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void applyMinObstacleClearance();
  bool validatePathClearance(const std::vector<GridState>& path_states);
  
  // Cost computation functions
  static int compareDistanceCost(int c1, int c2);
  static int compareObstacleCost(int c1, int c2);
  static int addDistanceCost(int c1, int c2);
  static int addObstacleCost(int c1, int c2);
  static int computeDistanceCost(GridState& fromState, GridState& toState);
  static int computeObstacleCost(GridState& fromState, GridState& toState);
  
  // Debug methods
  void debugCostmap();
  void debugPath(const std::vector<GridState>& path);
  
  int worldToGridX(double wx);
  int worldToGridY(double wy);
  void gridToWorld(int gx, int gy, double & wx, double & wy);
  
  std::unordered_set<unsigned int> getCellsInRadius(
    int center_x, int center_y, double radius_meters);
};

}  

#endif 
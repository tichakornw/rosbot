#include "nav2_multicost_planner/nav2_multicost_planner.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>
#include <climits>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_multicost_planner
{

void MulticostPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(
    node_->get_logger(), 
    "Configuring plugin %s of type MulticostPlanner",
    name_.c_str());

  // Algorithm selection parameter
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".algorithm_name", 
    rclcpp::ParameterValue("iterated_dijkstra"));
  node_->get_parameter(name_ + ".algorithm_name", algorithm_name_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".distance_weight", 
    rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".distance_weight", distance_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_weight", 
    rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".cost_weight", cost_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_obstacle_cost", 
    rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".use_obstacle_cost", use_obstacle_cost_);

  // Minimum obstacle clearance parameter (in meters)
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_obstacle_clearance", 
    rclcpp::ParameterValue(0.2));
  node_->get_parameter(name_ + ".min_obstacle_clearance", min_obstacle_clearance_);

  // YOLO detection parameters 
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_yolo_detections", 
    rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".use_yolo_detections", use_yolo_detections_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".yolo_detection_cost_weight", 
    rclcpp::ParameterValue(300.0));
  node_->get_parameter(name_ + ".yolo_detection_cost_weight", yolo_detection_cost_weight_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".yolo_detection_inflation_radius", 
    rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".yolo_detection_inflation_radius", yolo_detection_inflation_radius_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".yolo_markers_topic", 
    rclcpp::ParameterValue("/" + std::string(std::getenv("ROBOT_NAMESPACE") ? std::getenv("ROBOT_NAMESPACE") : "rosbot2r") + "/yolo/dgb_bb_markers"));
  node_->get_parameter(name_ + ".yolo_markers_topic", yolo_markers_topic_);


  loadAlgorithm();

  initializePathFinder();

  current_grid_width_ = 0;
  current_grid_height_ = 0;

  RCLCPP_INFO(
    node_->get_logger(),
    "MulticostPlanner configured: algorithm=%s, distance_weight=%.1f, cost_weight=%.1f, min_clearance=%.2fm, use_yolo=%s",
    algorithm_name_.c_str(), distance_weight_, cost_weight_, min_obstacle_clearance_, use_yolo_detections_ ? "true" : "false");
}

void MulticostPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s", name_.c_str());
  path_finder_.reset();
  pathfind_algorithm_.reset();
  if (yolo_markers_sub_) {
    yolo_markers_sub_.reset();
  }
  latest_markers_.reset();
}

void MulticostPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());

  if (use_yolo_detections_) {
    yolo_markers_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      yolo_markers_topic_,
      rclcpp::QoS(10),
      std::bind(&MulticostPlanner::yoloMarkersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "Subscribed to YOLO markers: %s", yolo_markers_topic_.c_str());
  }
}

void MulticostPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
  if (yolo_markers_sub_) {
    yolo_markers_sub_.reset();
  }
}

void MulticostPlanner::yoloMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(markers_mutex_);
  latest_markers_ = msg;
  
  RCLCPP_DEBUG(node_->get_logger(), "Received %zu YOLO marker detections", msg->markers.size());
}

nav_msgs::msg::Path MulticostPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Start frame %s != global frame %s", 
      start.header.frame_id.c_str(), global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal frame %s != global frame %s",
      goal.header.frame_id.c_str(), global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  unsigned int current_width = costmap_->getSizeInCellsX();
  unsigned int current_height = costmap_->getSizeInCellsY();

  if (current_width != current_grid_width_ || 
      current_height != current_grid_height_) {
    
    RCLCPP_INFO(node_->get_logger(), "Grid size changed: %ux%u", current_width, current_height);
    
    current_grid_width_ = current_width;
    current_grid_height_ = current_height;
    
    GridState::GRID_WIDTH = current_width;
    GridState::GRID_HEIGHT = current_height;
    GridState::CELL_STATES.resize(current_width * current_height, false);
    
    path_finder_->clearGraph();
  }

  try {
    updateGridFromCostmap();
    
    // Apply minimum obstacle clearance inflation
    applyMinObstacleClearance();
    
    // Update YOLO overlay
    if (use_yolo_detections_) {
      updateYoloOverlay();
    }
    
    // debugCostmap();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error updating grid: %s", e.what());
    return global_path;
  }

  unsigned int start_mx, start_my, goal_mx, goal_my;
  
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    RCLCPP_ERROR(node_->get_logger(), "Start position out of bounds");
    return global_path;
  }
  
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal position out of bounds");
    return global_path;
  }

  // Check if start/goal are obstacles
  if (GridState::CELL_STATES[start_my * current_grid_width_ + start_mx]) {
    RCLCPP_ERROR(node_->get_logger(), "Start is in obstacle");
    return global_path;
  }

  if (GridState::CELL_STATES[goal_my * current_grid_width_ + goal_mx]) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is in obstacle");
    return global_path;
  }

  GridState start_state(start_mx, start_my);
  GridState goal_state(goal_mx, goal_my);

  std::vector<GridState> path_states;
  try {
    path_states = path_finder_->getOptimalPath(*pathfind_algorithm_, start_state, goal_state);
    
    // Validate that the path meets minimum clearance requirements
    if (!path_states.empty() && !validatePathClearance(path_states)) {
      RCLCPP_WARN(node_->get_logger(), 
        "Path violates minimum clearance of %.2fm - path rejected", 
        min_obstacle_clearance_);
      return global_path;
    }
    
    // debugPath(path_states);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error finding path: %s", e.what());
    return global_path;
  }

  if (path_states.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No path found!");
    return global_path;
  }

  for (const GridState& state : path_states) {
    double wx, wy;
    gridToWorld(state.x, state.y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    
    global_path.poses.push_back(pose);
  }

  if (!global_path.poses.empty()) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  RCLCPP_INFO(node_->get_logger(), "Path found with %zu waypoints", global_path.poses.size());

  return global_path;
}

void MulticostPlanner::loadAlgorithm()
{
  auto& factory = MulticostPathfindFactory::getInstance();
  
  // List available algorithms
  auto available = factory.getAvailableAlgorithms();
  std::string available_str;
  for (const auto& name : available) {
    available_str += name + " ";
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "Available pathfinding algorithms: %s", 
    available_str.c_str());

  // Create the selected algorithm
  pathfind_algorithm_ = factory.create(algorithm_name_);
  
  if (!pathfind_algorithm_) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Failed to create algorithm '%s'. Available algorithms: %s", 
      algorithm_name_.c_str(), available_str.c_str());
    throw std::runtime_error("Failed to create pathfinding algorithm: " + algorithm_name_);
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "Successfully loaded pathfinding algorithm: %s", 
    algorithm_name_.c_str());
}

void MulticostPlanner::initializePathFinder()
{
  constexpr unsigned int num_monoids = 2;

  std::array<int, num_monoids> identity = {0, 0};

  std::array<std::function<int(int a, int b)>, num_monoids> compares = {
    compareDistanceCost,
    compareObstacleCost,
  };

  std::array<std::function<int(int a, int b)>, num_monoids> binaryOperators = {
    addDistanceCost,
    addObstacleCost
  };

  std::array<std::function<int(GridState& a, GridState& b)>, num_monoids> computes = {
    computeDistanceCost,
    computeObstacleCost
  };

  path_finder_ = std::make_unique<SingleOptimalPathFinder<GridState>>(
    identity, compares, binaryOperators, computes);
  
  RCLCPP_INFO(node_->get_logger(), "Path finder initialized");
}

void MulticostPlanner::updateGridFromCostmap()
{
  unsigned int width = costmap_->getSizeInCellsX();
  unsigned int height = costmap_->getSizeInCellsY();

  // First pass: Mark obstacles from costmap
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      unsigned char cost = costmap_->getCost(x, y);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
          cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        GridState::CELL_STATES[y * width + x] = true;  // obstacle
      } else {
        GridState::CELL_STATES[y * width + x] = false;  // free
      }
    }
  }
}

void MulticostPlanner::applyMinObstacleClearance()
{
  if (min_obstacle_clearance_ <= 0.0) {
    return;  // No clearance inflation needed
  }

  unsigned int width = current_grid_width_;
  unsigned int height = current_grid_height_;
  double resolution = costmap_->getResolution();
  
  int clearance_cells = static_cast<int>(std::ceil(min_obstacle_clearance_ / resolution));
  
  std::vector<bool> original_obstacles = GridState::CELL_STATES;
  
  int inflated_cells = 0;
  
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      unsigned int idx = y * width + x;
      
      if (original_obstacles[idx]) {
        for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
          for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
            int nx = static_cast<int>(x) + dx;
            int ny = static_cast<int>(y) + dy;
            
            // Check bounds
            if (nx >= 0 && nx < static_cast<int>(width) &&
                ny >= 0 && ny < static_cast<int>(height)) {
              
              // Calculate actual distance
              double dist = std::sqrt(dx * dx + dy * dy) * resolution;
              
              if (dist <= min_obstacle_clearance_) {
                unsigned int neighbor_idx = ny * width + nx;
                if (!GridState::CELL_STATES[neighbor_idx]) {
                  GridState::CELL_STATES[neighbor_idx] = true;
                  inflated_cells++;
                }
              }
            }
          }
        }
      }
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), 
    "Applied min clearance of %.2fm (%d cells): inflated %d additional cells",
    min_obstacle_clearance_, clearance_cells, inflated_cells);
}

bool MulticostPlanner::validatePathClearance(const std::vector<GridState>& path_states)
{
  if (min_obstacle_clearance_ <= 0.0) {
    return true;  // No validation needed
  }

  unsigned int width = current_grid_width_;
  unsigned int height = current_grid_height_;
  double resolution = costmap_->getResolution();
  int clearance_cells = static_cast<int>(std::ceil(min_obstacle_clearance_ / resolution));


  for (const auto& state : path_states) {
    // Check if any obstacle is within the minimum clearance distance
    for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
      for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
        int nx = state.x + dx;
        int ny = state.y + dy;
        
        // Check bounds
        if (nx >= 0 && nx < static_cast<int>(width) &&
            ny >= 0 && ny < static_cast<int>(height)) {
          
          unsigned int idx = ny * width + nx;
          
          // Get the original costmap value (before inflation)
          unsigned char cost = costmap_->getCost(nx, ny);
          
          if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
            // Calculate actual distance to this obstacle
            double dist = std::sqrt(dx * dx + dy * dy) * resolution;
            
            if (dist < min_obstacle_clearance_) {
              RCLCPP_DEBUG(node_->get_logger(),
                "Path point (%d, %d) is %.2fm from obstacle (min: %.2fm)",
                state.x, state.y, dist, min_obstacle_clearance_);
              return false;
            }
          }
        }
      }
    }
  }
  
  return true;
}


void MulticostPlanner::updateYoloOverlay()
{
  std::lock_guard<std::mutex> markers_lock(markers_mutex_);
  std::lock_guard<std::mutex> overlay_lock(overlay_mutex_);
  
  yolo_cost_overlay_.clear();
  
  if (!latest_markers_ || latest_markers_->markers.empty()) {
    return;
  }

  unsigned int width = current_grid_width_;
  unsigned int height = current_grid_height_;
  int yolo_objects_processed = 0;

  for (const auto& marker : latest_markers_->markers) {
    // Skip delete markers
    if (marker.action == visualization_msgs::msg::Marker::DELETE ||
        marker.action == visualization_msgs::msg::Marker::DELETEALL) {
      continue;
    }

    try {
      // Transform marker position to map frame if required
      geometry_msgs::msg::PoseStamped marker_pose;
      marker_pose.header = marker.header;
      marker_pose.pose = marker.pose;

      if (marker.header.frame_id != global_frame_) {
        geometry_msgs::msg::PoseStamped marker_in_map;
        marker_in_map = tf_->transform(marker_pose, global_frame_, tf2::durationFromSec(0.1));
        marker_pose = marker_in_map;
      }

      int grid_x = worldToGridX(marker_pose.pose.position.x);
      int grid_y = worldToGridY(marker_pose.pose.position.y);

      if (grid_x < 0 || grid_x >= static_cast<int>(width) ||
          grid_y < 0 || grid_y >= static_cast<int>(height)) {
        RCLCPP_DEBUG(node_->get_logger(), 
          "YOLO detection at (%.2f, %.2f) -> grid (%d, %d) is out of bounds",
          marker_pose.pose.position.x, marker_pose.pose.position.y, grid_x, grid_y);
        continue;
      }

      double object_radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
      double total_radius = object_radius + yolo_detection_inflation_radius_;

      auto affected_cells = getCellsInRadius(grid_x, grid_y, total_radius);

      double center_x = static_cast<double>(grid_x);
      double center_y = static_cast<double>(grid_y);
      double resolution = costmap_->getResolution();
      double radius_cells = total_radius / resolution;
      
      RCLCPP_INFO(node_->get_logger(), 
        "Processing YOLO detection '%s' at grid (%d, %d), radius: %.2fm, affected cells: %zu",
        marker.text.c_str(), grid_x, grid_y, total_radius, affected_cells.size());

      for (unsigned int cell_id : affected_cells) {
        int cx = cell_id % width;
        int cy = cell_id / width;
        
        double dx = cx - center_x;
        double dy = cy - center_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // exponnential decay of cost with distance
        double dist_ratio = std::max(0.0, 1.0 - (dist / radius_cells));
        int yolo_cost = static_cast<int>(yolo_detection_cost_weight_ * dist_ratio);
        
        // Mark cells as obstacles if they're within the core radius of the object
        if (dist_ratio > 0.3) {  // Mark as obstacle if within 70% of radius
          GridState::CELL_STATES[cell_id] = true;
        }
        
        // Store overlay cost
        if (yolo_cost_overlay_.find(cell_id) != yolo_cost_overlay_.end()) {
          yolo_cost_overlay_[cell_id] = std::max(yolo_cost_overlay_[cell_id], yolo_cost);
        } else {
          yolo_cost_overlay_[cell_id] = yolo_cost;
        }
      }
      
      yolo_objects_processed++;
        
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "Transform failed for YOLO marker: %s", ex.what());
      continue;
    } catch (const std::exception& ex) {
      RCLCPP_WARN(node_->get_logger(), "Error processing YOLO marker: %s", ex.what());
      continue;
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "Updated YOLO overlay: %d objects processed, %zu cells affected",
    yolo_objects_processed, yolo_cost_overlay_.size());
}

std::unordered_set<unsigned int> MulticostPlanner::getCellsInRadius(
  int center_x, int center_y, double radius_meters)
{
  std::unordered_set<unsigned int> cells;
  
  double resolution = costmap_->getResolution();
  int radius_cells = static_cast<int>(std::ceil(radius_meters / resolution));
  
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      double distance = std::sqrt(dx * dx + dy * dy) * resolution;
      if (distance > radius_meters) continue;
      
      int x = center_x + dx;
      int y = center_y + dy;
      
      if (x >= 0 && x < static_cast<int>(current_grid_width_) &&
          y >= 0 && y < static_cast<int>(current_grid_height_)) {
        unsigned int cell_id = y * current_grid_width_ + x;
        cells.insert(cell_id);
      }
    }
  }
  
  return cells;
}

int MulticostPlanner::worldToGridX(double wx)
{
  return static_cast<int>((wx - costmap_->getOriginX()) / costmap_->getResolution());
}

int MulticostPlanner::worldToGridY(double wy)
{
  return static_cast<int>((wy - costmap_->getOriginY()) / costmap_->getResolution());
}

void MulticostPlanner::gridToWorld(int gx, int gy, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + (gx + 0.5) * costmap_->getResolution();
  wy = costmap_->getOriginY() + (gy + 0.5) * costmap_->getResolution();
}

int MulticostPlanner::compareDistanceCost(int c1, int c2) {
  return c1 - c2;
}

int MulticostPlanner::compareObstacleCost(int c1, int c2) {
  return c1 - c2;
}

int MulticostPlanner::addDistanceCost(int c1, int c2) {
  return c1 + c2;
}

int MulticostPlanner::addObstacleCost(int c1, int c2) {
  return c1 + c2;
}

int MulticostPlanner::computeDistanceCost(GridState& fromCell, GridState& toCell) {
  return 1;
}

int MulticostPlanner::computeObstacleCost(GridState& fromState, GridState& toState) {
  return fromState.numberOfNearbyObstacles() + toState.numberOfNearbyObstacles();
}

}  // namespace nav2_multicost_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_multicost_planner::MulticostPlanner, 
  nav2_core::GlobalPlanner)
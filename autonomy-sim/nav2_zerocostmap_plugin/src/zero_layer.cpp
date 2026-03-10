/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Zero Costmap Layer
 * 
 * Description: A minimal zero-cost costmap layer that does nothing
 *              and returns a completely free costmap.
 *********************************************************************/
#include "nav2_zerocostmap_plugin/zero_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

using nav2_costmap_2d::FREE_SPACE;

namespace nav2_zerocostmap_plugin
{

ZeroLayer::ZeroLayer()
{
}

// Called at the end of plugin initialization
void ZeroLayer::onInitialize()
{
  auto node = node_.lock(); 
  
  // Declare and get the enabled parameter
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  // Mark as current (always up to date since we do nothing)
  current_ = true;
  
  RCLCPP_INFO(
    rclcpp::get_logger("nav2_costmap_2d"), 
    "ZeroLayer '%s' initialized - this layer does no calculations", 
    name_.c_str());
}

// Called to determine which area of costmap needs updating
// We return without expanding bounds since we don't need updates
void ZeroLayer::updateBounds(
  double /*robot_x*/, 
  double /*robot_y*/, 
  double /*robot_yaw*/, 
  double * /*min_x*/,
  double * /*min_y*/, 
  double * /*max_x*/, 
  double * /*max_y*/)
{
  // Do nothing - we don't need to update any bounds
  // The costmap will handle bounds from other layers
}

// Called when the robot's footprint changes
void ZeroLayer::onFootprintChanged()
{
  // Do nothing - we don't care about footprint changes
  RCLCPP_DEBUG(
    rclcpp::get_logger("nav2_costmap_2d"), 
    "ZeroLayer::onFootprintChanged() - no action taken");
}

// Called when costmap recalculation is required
// This is where we set all cells to FREE_SPACE (cost = 0)
void ZeroLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, 
  int min_i, 
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // Get direct access to the costmap array
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  
  // Ensure bounds are valid
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Set all cells within bounds to FREE_SPACE (cost = 0)
  // This makes the entire costmap completely traversable
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      master_array[index] = FREE_SPACE;
    }
  }
}

}  // namespace nav2_zerocostmap_plugin

// Register this plugin with pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_zerocostmap_plugin::ZeroLayer, nav2_costmap_2d::Layer)
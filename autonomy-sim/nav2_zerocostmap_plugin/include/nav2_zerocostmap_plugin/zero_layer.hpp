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
#ifndef ZERO_LAYER_HPP_
#define ZERO_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_zerocostmap_plugin
{

/**
 * @class ZeroLayer
 * @brief A costmap layer that performs no calculations and returns free space
 * 
 * This layer is designed to be a minimal placeholder that satisfies Nav2's
 * requirement for costmap plugins while doing no actual obstacle detection
 * or cost calculation. All cells are set to FREE_SPACE (cost = 0).
 */
class ZeroLayer : public nav2_costmap_2d::Layer
{
public:
  /**
   * @brief Constructor
   */
  ZeroLayer();

  /**
   * @brief Initialization method called after plugin is created
   */
  virtual void onInitialize();

  /**
   * @brief Update the bounds of the costmap
   * @param robot_x X position of robot
   * @param robot_y Y position of robot
   * @param robot_yaw Yaw of robot
   * @param min_x Minimum x bound to update
   * @param min_y Minimum y bound to update
   * @param max_x Maximum x bound to update
   * @param max_y Maximum y bound to update
   * 
   * This implementation does nothing - it doesn't expand bounds
   */
  virtual void updateBounds(
    double robot_x, 
    double robot_y, 
    double robot_yaw, 
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  /**
   * @brief Update the costs in the costmap
   * @param master_grid The master costmap to update
   * @param min_i Minimum x index to update
   * @param min_j Minimum y index to update
   * @param max_i Maximum x index to update
   * @param max_j Maximum y index to update
   * 
   * This implementation sets all cells to FREE_SPACE
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Reset the layer (no-op)
   */
  virtual void reset()
  {
    // Nothing to reset
  }

  /**
   * @brief Called when the footprint changes
   */
  virtual void onFootprintChanged();

  /**
   * @brief Whether this layer can be cleared
   * @return false - this layer cannot be cleared as it has no data
   */
  virtual bool isClearable() 
  {
    return false;
  }
};

}  // namespace nav2_zerocostmap_plugin

#endif  // ZERO_LAYER_HPP_
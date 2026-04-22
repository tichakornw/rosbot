#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <string>
#include <vector>

#include "autonomy_contracts/autonomy_contracts.hpp"

namespace autonomy_reference_components
{

class GridBfsPlanner
{
public:
  autonomy_contracts::PlanningOutput2D plan(const autonomy_contracts::PlanningInput2D & input)
  {
    autonomy_contracts::PlanningOutput2D output;
    const auto & map = input.map;
    const uint32_t cell_count = map.width * map.height;

    if (map.width == 0 || map.height == 0 || map.resolution <= 0.0) {
      output.error_message = "Invalid grid map dimensions or resolution";
      return output;
    }

    if (map.occupancy.size() != cell_count) {
      output.error_message = "Grid occupancy size does not match width * height";
      return output;
    }

    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    if (!worldToGrid(map, input.start.x, input.start.y, start_x, start_y) ||
      !worldToGrid(map, input.goal.x, input.goal.y, goal_x, goal_y))
    {
      output.error_message = "Start or goal is outside the grid map";
      return output;
    }

    std::vector<bool> blocked(cell_count, false);
    for (uint32_t idx = 0; idx < cell_count; ++idx) {
      blocked[idx] = map.occupancy[idx] >= 100;
    }
    applyDynamicObstacles(map, input.dynamic_obstacles, blocked);

    const uint32_t start_idx = index(map, start_x, start_y);
    const uint32_t goal_idx = index(map, goal_x, goal_y);
    if (blocked[start_idx] || blocked[goal_idx]) {
      output.error_message = "Start or goal is blocked";
      return output;
    }

    std::vector<int32_t> parent(cell_count, -1);
    std::queue<uint32_t> frontier;
    frontier.push(start_idx);
    parent[start_idx] = static_cast<int32_t>(start_idx);

    static constexpr int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static constexpr int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!frontier.empty() && parent[goal_idx] == -1) {
      const uint32_t current = frontier.front();
      frontier.pop();
      const int cx = static_cast<int>(current % map.width);
      const int cy = static_cast<int>(current / map.width);

      for (size_t i = 0; i < 8; ++i) {
        const int nx = cx + DX[i];
        const int ny = cy + DY[i];
        if (!inBounds(map, nx, ny)) {
          continue;
        }

        const uint32_t next = index(map, nx, ny);
        if (blocked[next] || parent[next] != -1) {
          continue;
        }

        parent[next] = static_cast<int32_t>(current);
        frontier.push(next);
      }
    }

    if (parent[goal_idx] == -1) {
      output.error_message = "No grid path found";
      return output;
    }

    std::vector<uint32_t> reversed_indices;
    for (uint32_t at = goal_idx; at != start_idx; at = static_cast<uint32_t>(parent[at])) {
      reversed_indices.push_back(at);
    }
    reversed_indices.push_back(start_idx);
    std::reverse(reversed_indices.begin(), reversed_indices.end());

    output.path.poses.reserve(reversed_indices.size());
    for (const auto grid_index : reversed_indices) {
      const int gx = static_cast<int>(grid_index % map.width);
      const int gy = static_cast<int>(grid_index / map.width);
      output.path.poses.push_back(gridToWorld(map, gx, gy));
    }
    if (!output.path.poses.empty()) {
      output.path.poses.front().yaw = input.start.yaw;
      output.path.poses.back().yaw = input.goal.yaw;
    }

    output.success = true;
    return output;
  }

private:
  static bool inBounds(const autonomy_contracts::GridMap2D & map, int x, int y)
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(map.width) &&
           y < static_cast<int>(map.height);
  }

  static uint32_t index(const autonomy_contracts::GridMap2D & map, int x, int y)
  {
    return static_cast<uint32_t>(y) * map.width + static_cast<uint32_t>(x);
  }

  static bool worldToGrid(
    const autonomy_contracts::GridMap2D & map,
    double wx,
    double wy,
    int & gx,
    int & gy)
  {
    gx = static_cast<int>(std::floor((wx - map.origin.x) / map.resolution));
    gy = static_cast<int>(std::floor((wy - map.origin.y) / map.resolution));
    return inBounds(map, gx, gy);
  }

  static autonomy_contracts::Pose2D gridToWorld(
    const autonomy_contracts::GridMap2D & map,
    int gx,
    int gy)
  {
    autonomy_contracts::Pose2D pose;
    pose.x = map.origin.x + (static_cast<double>(gx) + 0.5) * map.resolution;
    pose.y = map.origin.y + (static_cast<double>(gy) + 0.5) * map.resolution;
    pose.yaw = 0.0;
    return pose;
  }

  static void applyDynamicObstacles(
    const autonomy_contracts::GridMap2D & map,
    const std::vector<autonomy_contracts::Obstacle2D> & obstacles,
    std::vector<bool> & blocked)
  {
    for (const auto & obstacle : obstacles) {
      int center_x = 0;
      int center_y = 0;
      if (!worldToGrid(map, obstacle.pose.x, obstacle.pose.y, center_x, center_y)) {
        continue;
      }

      const int radius_cells = static_cast<int>(std::ceil(obstacle.radius / map.resolution));
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int nx = center_x + dx;
          const int ny = center_y + dy;
          if (!inBounds(map, nx, ny)) {
            continue;
          }

          const double distance = std::sqrt(
            static_cast<double>(dx * dx + dy * dy)) * map.resolution;
          if (distance <= obstacle.radius) {
            blocked[index(map, nx, ny)] = true;
          }
        }
      }
    }
  }
};

}  // namespace autonomy_reference_components

REGISTER_PLANNER_COMPONENT("bfs_grid_planner", autonomy_reference_components::GridBfsPlanner)

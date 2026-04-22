#include <algorithm>
#include <cmath>
#include <string>

#include "autonomy_contracts/autonomy_contracts.hpp"

namespace autonomy_reference_components
{

class PurePursuitComponent
{
public:
  autonomy_contracts::ControlOutput2D computeCommand(
    const autonomy_contracts::ControlInput2D & input)
  {
    autonomy_contracts::ControlOutput2D output;
    if (input.reference_path.poses.empty()) {
      output.error_message = "Reference path is empty";
      return output;
    }

    auto target = input.reference_path.poses.back();
    for (const auto & pose : input.reference_path.poses) {
      const double distance = std::hypot(pose.x - input.robot_pose.x, pose.y - input.robot_pose.y);
      if (distance >= lookahead_dist_) {
        target = pose;
        break;
      }
    }

    const double dx = target.x - input.robot_pose.x;
    const double dy = target.y - input.robot_pose.y;
    const double heading = input.robot_pose.yaw;
    const double local_x = std::cos(heading) * dx + std::sin(heading) * dy;
    const double local_y = -std::sin(heading) * dx + std::cos(heading) * dy;

    if (local_x > 0.0) {
      const double curvature = 2.0 * local_y / std::max(0.001, local_x * local_x + local_y * local_y);
      output.command.linear_x = desired_linear_vel_;
      output.command.angular_z = clamp(desired_linear_vel_ * curvature, -max_angular_vel_, max_angular_vel_);
    } else {
      output.command.linear_x = 0.0;
      output.command.angular_z = max_angular_vel_;
    }

    output.success = true;
    return output;
  }

private:
  static double clamp(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  double desired_linear_vel_ {0.2};
  double lookahead_dist_ {0.4};
  double max_angular_vel_ {1.0};
};

}  // namespace autonomy_reference_components

REGISTER_CONTROLLER_COMPONENT(
  "pure_pursuit_controller",
  autonomy_reference_components::PurePursuitComponent)

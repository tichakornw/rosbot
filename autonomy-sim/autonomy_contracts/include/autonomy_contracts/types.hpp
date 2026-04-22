#ifndef AUTONOMY_CONTRACTS__TYPES_HPP_
#define AUTONOMY_CONTRACTS__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy_contracts
{

struct Pose2D
{
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
  double stamp_sec {0.0};
};

struct Twist2D
{
  double linear_x {0.0};
  double linear_y {0.0};
  double angular_z {0.0};
};

struct GridMap2D
{
  uint32_t width {0};
  uint32_t height {0};
  double resolution {0.0};
  Pose2D origin;
  std::vector<uint8_t> occupancy;
  std::vector<float> cost;
};

struct Path2D
{
  std::vector<Pose2D> poses;
};

struct Detection2D
{
  std::string class_name;
  int class_id {0};
  double confidence {0.0};
  Pose2D pose;
  double size_x {0.0};
  double size_y {0.0};
};

struct Obstacle2D
{
  Pose2D pose;
  double radius {0.0};
  float cost {0.0F};
};

struct PerceptionResult2D
{
  std::vector<Detection2D> detections;
  std::vector<Obstacle2D> obstacles;
};

struct PlanningInput2D
{
  GridMap2D map;
  Pose2D start;
  Pose2D goal;
  std::vector<Obstacle2D> dynamic_obstacles;
};

struct PlanningOutput2D
{
  bool success {false};
  Path2D path;
  std::string error_message;
};

struct ControlInput2D
{
  Pose2D robot_pose;
  Twist2D robot_velocity;
  Path2D reference_path;
  std::vector<Obstacle2D> dynamic_obstacles;
  double dt_sec {0.0};
};

struct ControlOutput2D
{
  bool success {false};
  Twist2D command;
  std::string error_message;
};

struct PerceptionInput2D
{
  uint32_t width {0};
  uint32_t height {0};
  std::string encoding;
  std::vector<uint8_t> image_data;
  std::vector<float> depth_data;
  double stamp_sec {0.0};
};

struct PerceptionOutput2D
{
  bool success {false};
  PerceptionResult2D result;
  std::string error_message;
};

}  // namespace autonomy_contracts

#endif  // AUTONOMY_CONTRACTS__TYPES_HPP_

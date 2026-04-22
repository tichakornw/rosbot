#include <cassert>
#include <memory>

#include "autonomy_contracts/autonomy_contracts.hpp"

namespace
{

class ValidPlanner
{
public:
  autonomy_contracts::PlanningOutput2D plan(
    const autonomy_contracts::PlanningInput2D & input)
  {
    (void)input;
    autonomy_contracts::PlanningOutput2D output;
    output.success = true;
    return output;
  }
};

class InvalidPlanner
{
public:
  void plan(const autonomy_contracts::PlanningInput2D & input)
  {
    (void)input;
  }
};

struct CustomPlanningInput
{
  int request_id {0};
};

struct CustomPlanningOutput
{
  bool accepted {false};
};

class CustomPlanner
{
public:
  CustomPlanningOutput plan(const CustomPlanningInput & input)
  {
    CustomPlanningOutput output;
    output.accepted = input.request_id > 0;
    return output;
  }
};

class CustomPlannerAdapter
{
public:
  autonomy_contracts::PlanningOutput2D plan(
    const autonomy_contracts::PlanningInput2D & input)
  {
    CustomPlanningInput custom_input;
    custom_input.request_id = input.goal.x > input.start.x ? 1 : 0;

    const auto custom_output = planner_.plan(custom_input);

    autonomy_contracts::PlanningOutput2D output;
    output.success = custom_output.accepted;
    output.path.poses.push_back(input.start);
    output.path.poses.push_back(input.goal);
    return output;
  }

private:
  CustomPlanner planner_;
};

class ValidController
{
public:
  autonomy_contracts::ControlOutput2D computeCommand(
    const autonomy_contracts::ControlInput2D & input)
  {
    (void)input;
    autonomy_contracts::ControlOutput2D output;
    output.success = true;
    return output;
  }
};

class ValidPerception
{
public:
  autonomy_contracts::PerceptionOutput2D process(
    const autonomy_contracts::PerceptionInput2D & input)
  {
    (void)input;
    autonomy_contracts::PerceptionOutput2D output;
    output.success = true;
    return output;
  }
};

}  // namespace

int main()
{
  static_assert(autonomy_contracts::is_planner_component<ValidPlanner>::value);
  static_assert(!autonomy_contracts::is_planner_component<InvalidPlanner>::value);
  static_assert(
    autonomy_contracts::is_planner_component<
      CustomPlanner,
      CustomPlanningInput,
      CustomPlanningOutput>::value);
  static_assert(autonomy_contracts::is_planner_component<CustomPlannerAdapter>::value);
  static_assert(autonomy_contracts::is_controller_component<ValidController>::value);
  static_assert(autonomy_contracts::is_perception_component<ValidPerception>::value);

  assert(autonomy_contracts::registerPlannerComponent<ValidPlanner>("test_planner"));
  assert(autonomy_contracts::registerPlannerComponent<CustomPlannerAdapter>("custom_adapter"));
  assert(autonomy_contracts::registerControllerComponent<ValidController>("test_controller"));
  assert(autonomy_contracts::registerPerceptionComponent<ValidPerception>("test_perception"));

  auto planner = autonomy_contracts::ComponentRegistry::instance().createPlanner("test_planner");
  auto custom_adapter =
    autonomy_contracts::ComponentRegistry::instance().createPlanner("custom_adapter");
  auto controller =
    autonomy_contracts::ComponentRegistry::instance().createController("test_controller");
  auto perception =
    autonomy_contracts::ComponentRegistry::instance().createPerception("test_perception");

  assert(planner);
  assert(custom_adapter);
  assert(controller);
  assert(perception);
  assert(!autonomy_contracts::ComponentRegistry::instance().createPlanner("missing_planner"));

  autonomy_contracts::PlanningInput2D input;
  input.start.x = 0.0;
  input.goal.x = 1.0;
  assert(custom_adapter->plan(input).success);

  return 0;
}

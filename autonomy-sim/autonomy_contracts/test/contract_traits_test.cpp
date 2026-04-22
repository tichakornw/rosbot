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
  static_assert(autonomy_contracts::is_controller_component<ValidController>::value);
  static_assert(autonomy_contracts::is_perception_component<ValidPerception>::value);

  assert(autonomy_contracts::registerPlannerComponent<ValidPlanner>("test_planner"));
  assert(autonomy_contracts::registerControllerComponent<ValidController>("test_controller"));
  assert(autonomy_contracts::registerPerceptionComponent<ValidPerception>("test_perception"));

  auto planner = autonomy_contracts::ComponentRegistry::instance().createPlanner("test_planner");
  auto controller =
    autonomy_contracts::ComponentRegistry::instance().createController("test_controller");
  auto perception =
    autonomy_contracts::ComponentRegistry::instance().createPerception("test_perception");

  assert(planner);
  assert(controller);
  assert(perception);
  assert(!autonomy_contracts::ComponentRegistry::instance().createPlanner("missing_planner"));

  return 0;
}

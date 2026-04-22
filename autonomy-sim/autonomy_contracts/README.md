# autonomy_contracts

`autonomy_contracts` is the ROS-free API for the planner boundary. In this
workspace, planning is the only external extension point. Control and perception
use common defaults inside `autonomy_ros2_wrapper`.

## Boundary Rule

The class registered with this wrapper must expose the standard planning method:

```cpp
autonomy_contracts::PlanningOutput2D plan(
  const autonomy_contracts::PlanningInput2D& input);
```

`PlanningOutput2D` must contain a usable `Path2D`, because Nav2 and the internal
controller consume that path.

Raw planner code may use custom types internally. Put an adapter at the registered
boundary:

```cpp
class MyRawPlanner
{
public:
  MyCustomOutput plan(const MyCustomInput& input);
};

class MyPlannerAdapter
{
public:
  autonomy_contracts::PlanningOutput2D plan(
    const autonomy_contracts::PlanningInput2D& input)
  {
    MyCustomInput custom_input = convertInput(input);
    MyCustomOutput custom_output = raw_planner_.plan(custom_input);

    autonomy_contracts::PlanningOutput2D output;
    output.success = true;
    output.path = convertToPath2D(custom_output);
    return output;
  }

private:
  MyRawPlanner raw_planner_;
};
```

Register the adapter, not the raw planner:

```cpp
#include "autonomy_contracts/autonomy_contracts.hpp"

REGISTER_PLANNER_COMPONENT("my_planner", MyPlannerAdapter)
```

## Where Things Go

- `include/my_planner/my_raw_planner.hpp`: your custom planner types and raw planner class.
- `src/my_planner_adapter.cpp`: the adapter class and the `REGISTER_PLANNER_COMPONENT(...)` call.
- `CMakeLists.txt`: build a library target that links `autonomy_contracts`.
- `autonomy-sim/external_components/component_manifest.cmake`: point the wrapper at that library target.
- `autonomy-sim/config/nav2_*_params.yaml`: set `planner_component: "my_planner"`.

## Templates

The planner helpers are templated for testing and local adapter code:

```cpp
static_assert(
  autonomy_contracts::is_planner_component<MyRawPlanner, MyCustomInput, MyCustomOutput>::value);
```

That proves the raw custom planner has the signature you expect. It does not make
that raw planner usable by the ROS wrapper. The registered adapter must still pass:

```cpp
static_assert(autonomy_contracts::is_planner_component<MyPlannerAdapter>::value);
```

## Verification

Registration runs compile-time checks from `traits.hpp`. The build fails if the
registered adapter does not:

- use `plan`
- accept `PlanningInput2D` by `const&`
- return `PlanningOutput2D`
- provide a default constructor

At runtime, the wrapper validates the selected YAML component name by asking the
registry to create it. Unknown names are reported as configuration errors.

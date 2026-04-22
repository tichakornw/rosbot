# external_components

Put plain C++ planner repositories here when developing them alongside this wrapper.
These repositories should not include ROS 2 or Nav2 code. They should depend only on
`autonomy_contracts`.

Planning is the only external extension point. Control and perception use the common
defaults built into `autonomy_ros2_wrapper`:

- `pure_pursuit_controller`
- `noop_perception`

## Required Shape

The registered component must be an adapter with this method:

```cpp
autonomy_contracts::PlanningOutput2D plan(
  const autonomy_contracts::PlanningInput2D& input);
```

If your raw planner uses custom input or output types, keep those inside your planner
package and convert at the adapter boundary:

```text
PlanningInput2D -> MyPlannerAdapter -> MyCustomInput
MyCustomOutput -> MyPlannerAdapter -> PlanningOutput2D with Path2D
```

Register only the adapter class.

## File Layout

A typical local planner repo should look like:

```text
external_components/my_planner/
  CMakeLists.txt
  include/my_planner/my_raw_planner.hpp
  src/my_planner_adapter.cpp
```

Put custom planner types and algorithm code in `include/my_planner/my_raw_planner.hpp`
or other private files.

Build the adapter source into a library target from `CMakeLists.txt`:

```cmake
add_library(my_planner_lib SHARED
  src/my_planner_adapter.cpp
)

target_include_directories(my_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_planner_lib
  autonomy_contracts
)
```

Put the wrapper-facing adapter and registration in `src/my_planner_adapter.cpp`:

```cpp
#include "autonomy_contracts/autonomy_contracts.hpp"
#include "my_planner/my_raw_planner.hpp"

namespace my_planner
{

class MyPlannerAdapter
{
public:
  autonomy_contracts::PlanningOutput2D plan(
    const autonomy_contracts::PlanningInput2D& input)
  {
    auto custom_input = convertInput(input);
    auto custom_output = raw_planner_.plan(custom_input);

    autonomy_contracts::PlanningOutput2D output;
    output.success = true;
    output.path = convertToPath2D(custom_output);
    return output;
  }

private:
  MyRawPlanner raw_planner_;
};

}  // namespace my_planner

REGISTER_PLANNER_COMPONENT("my_planner", my_planner::MyPlannerAdapter)
```

## Link The Planner

Edit `component_manifest.cmake` in this directory. It has one labeled section:

- `PLANNING COMPONENT LINK`

For a local repo:

```cmake
autonomy_fetch_component(
  NAME my_planner
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/my_planner"
  TARGET my_planner_lib
)
```

For a GitHub repo:

```cmake
autonomy_fetch_component(
  NAME my_planner
  GIT_REPOSITORY https://github.com/your-org/my-planner.git
  GIT_TAG main
  TARGET my_planner_lib
)
```

The target named in `TARGET` must be the library that contains
`REGISTER_PLANNER_COMPONENT`.

## Select The Planner

Set the registered name in Nav2 YAML:

```yaml
GridBased:
  plugin: "autonomy_ros2_wrapper/ContractPlanner"
  planner_component: "my_planner"
```

The included external reference planner is `bfs_grid_planner`.

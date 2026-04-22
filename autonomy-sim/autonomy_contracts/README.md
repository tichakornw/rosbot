# autonomy_contracts

`autonomy_contracts` is the ROS-free API that C++ planner libraries implement.
In this workspace, planning is the external extension point. Control and perception
use common defaults inside `autonomy_ros2_wrapper`.

## Component requirements

External planner libraries must provide default-constructible classes with this exact method:

```cpp
autonomy_contracts::PlanningOutput2D plan(
  const autonomy_contracts::PlanningInput2D& input);
```

The planner contract helpers are templated, so non-wrapper code can validate or adapt
other planning shapes:

```cpp
static_assert(
  autonomy_contracts::is_planner_component<MyPlanner, MyInput, MyOutput>::value);

autonomy_contracts::PlannerComponentModel<MyPlanner, MyInput, MyOutput> planner;
```

The ROS 2 wrapper instantiates that template with `PlanningInput2D` and
`PlanningOutput2D`, so external planners used by this workspace should keep the default
2D signature above.

Register the planner once in a `.cpp` file:

```cpp
#include "autonomy_contracts/autonomy_contracts.hpp"

REGISTER_PLANNER_COMPONENT("my_planner", MyPlanner)
```

The ROS 2 wrapper links planner libraries, then selects the registered planner name from YAML.

## Verification

Contracts are checked at compile time. The registration macros call C++17
`static_assert` checks from `traits.hpp`.

The build fails if a registered class does not:

- use the exact required method name
- accept the exact required input type by `const&`
- return the exact required output type
- provide a default constructor

You can also use the traits directly in external component tests:

```cpp
static_assert(autonomy_contracts::is_planner_component<MyPlanner>::value);
```

At runtime, the wrapper validates the selected YAML component name by asking the
registry to create it. Unknown names are reported as configuration errors.

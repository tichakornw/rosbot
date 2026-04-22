# autonomy_contracts

`autonomy_contracts` is the ROS-free API that external C++ component libraries implement.
Planning, control, and perception libraries should depend only on this package, not on ROS 2,
Nav2, lifecycle nodes, topics, or message headers.

## Component requirements

External libraries must provide default-constructible classes with these exact methods:

```cpp
autonomy_contracts::PlanningOutput2D plan(
  const autonomy_contracts::PlanningInput2D& input);

autonomy_contracts::ControlOutput2D computeCommand(
  const autonomy_contracts::ControlInput2D& input);

autonomy_contracts::PerceptionOutput2D process(
  const autonomy_contracts::PerceptionInput2D& input);
```

Register components once in a `.cpp` file:

```cpp
#include "autonomy_contracts/autonomy_contracts.hpp"

REGISTER_PLANNER_COMPONENT("my_planner", MyPlanner)
REGISTER_CONTROLLER_COMPONENT("my_controller", MyController)
REGISTER_PERCEPTION_COMPONENT("my_perception", MyPerception)
```

The ROS 2 wrapper links component libraries, then selects registered names from YAML.

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
static_assert(autonomy_contracts::is_controller_component<MyController>::value);
static_assert(autonomy_contracts::is_perception_component<MyPerception>::value);
```

At runtime, the wrapper validates the selected YAML component name by asking the
registry to create it. Unknown names are reported as configuration errors.

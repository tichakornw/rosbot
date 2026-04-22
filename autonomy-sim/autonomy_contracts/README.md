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

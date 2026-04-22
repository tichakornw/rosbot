# Integrating Plain C++ Autonomy Components

This repo treats ROS 2 as an internal wrapper layer. External planning,
control, and perception repositories should be plain C++ libraries that depend
only on `autonomy_contracts`.

## Current Contract Flow

1. External libraries implement exact C++ methods such as:

```cpp
autonomy_contracts::PlanningOutput2D plan(
  const autonomy_contracts::PlanningInput2D& input);
```

2. The library registers the class in a `.cpp` file:

```cpp
#include "autonomy_contracts/autonomy_contracts.hpp"

REGISTER_PLANNER_COMPONENT("my_planner", MyPlanner)
REGISTER_CONTROLLER_COMPONENT("my_controller", MyController)
REGISTER_PERCEPTION_COMPONENT("my_perception", MyPerception)
```

3. `autonomy_ros2_wrapper` adapts internal ROS 2/Nav2 data to the ROS-free
contract structs and calls the selected component.

4. Nav2 YAML selects among linked components:

```yaml
GridBased:
  plugin: "autonomy_ros2_wrapper/ContractPlanner"
  planner_component: "bfs_grid_planner"

FollowPath:
  plugin: "autonomy_ros2_wrapper/ContractController"
  controller_component: "pure_pursuit_controller"
```

Additional external component libraries can be linked into
`autonomy_ros2_wrapper` through the CMake cache variable
`AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES`.

For repos checked out beside this wrapper, put them under
`autonomy-sim/external_components/` and add their library targets to
`autonomy-sim/external_components/component_manifest.cmake`.

For GitHub-hosted component repos, use the labeled sections in
`component_manifest.cmake`:

```cmake
autonomy_fetch_component(
  NAME my_planner
  GIT_REPOSITORY https://github.com/your-org/my-planner.git
  GIT_TAG main
  TARGET my_planner_lib
)
```

The public component surface is documented in
`autonomy-sim/autonomy_contracts/README.md`.

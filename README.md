# cc01_autonomous_robots

### SETUP

The ROS-free component contract layer lives in `autonomy-sim/autonomy_contracts`.
Plain C++ planning, control, and perception libraries should implement those
contracts and register components by name. `autonomy-sim/autonomy_ros2_wrapper`
keeps ROS 2/Nav2 integration internal and adapts the selected components from
YAML.

Default example components live in `autonomy-sim/external_components/reference_components`.
Add external GitHub links in the labeled Planning, Control, and Perception sections of
`autonomy-sim/external_components/component_manifest.cmake`.

### Component Link Boxes

GitHub does not allow README pages to save real editable text boxes back into the
repo. The boxes below show exactly where each link belongs. To make a real change,
edit `autonomy-sim/external_components/component_manifest.cmake` and paste the
GitHub URL into the matching labeled section.

#### Planning Component Link

Controls global path planning from `GridMap2D + start/goal` to `Path2D`.

```cmake
# PLANNING COMPONENT LINK
autonomy_fetch_component(
  NAME my_planner
  GIT_REPOSITORY https://github.com/your-org/my-planner.git
  GIT_TAG main
  TARGET my_planner_lib
)
```

#### Control Component Link

Controls local velocity command generation from `Path2D + robot state`.

```cmake
# CONTROL COMPONENT LINK
autonomy_fetch_component(
  NAME my_controller
  GIT_REPOSITORY https://github.com/your-org/my-controller.git
  GIT_TAG main
  TARGET my_controller_lib
)
```

#### Perception Component Link

Controls image/depth processing into detections and dynamic obstacles.

```cmake
# PERCEPTION COMPONENT LINK
autonomy_fetch_component(
  NAME my_perception
  GIT_REPOSITORY https://github.com/your-org/my-perception.git
  GIT_TAG main
  TARGET my_perception_lib
)
```

Each linked repo must expose the `TARGET` named in the box and register one or
more components with `REGISTER_PLANNER_COMPONENT`, `REGISTER_CONTROLLER_COMPONENT`,
or `REGISTER_PERCEPTION_COMPONENT`.

### Contract Verification

External repositories are verified when this repository builds.

The registration macros call C++17 `static_assert` checks from
`autonomy_contracts`. A linked external repository fails compilation if a
registered component:

- does not have the exact required method name
- uses the wrong input type
- uses the wrong output type
- is not default constructible

Required signatures:

```cpp
autonomy_contracts::PlanningOutput2D plan(
  const autonomy_contracts::PlanningInput2D& input);

autonomy_contracts::ControlOutput2D computeCommand(
  const autonomy_contracts::ControlInput2D& input);

autonomy_contracts::PerceptionOutput2D process(
  const autonomy_contracts::PerceptionInput2D& input);
```

Runtime selection is also checked. If YAML requests a component name that was not
registered by any linked library, `autonomy_ros2_wrapper` logs the available
registered names and fails to create that component.

### External Repository Setup

External repositories should be plain C++ CMake projects. They should not include
ROS 2, Nav2, lifecycle nodes, topics, or ROS messages.

Recommended external repo layout:

```text
my_planner/
  CMakeLists.txt
  include/my_planner/my_planner.hpp
  src/my_planner.cpp
```

Minimal `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.15)
project(my_planner)

add_library(my_planner_lib SHARED
  src/my_planner.cpp
)

target_include_directories(my_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(my_planner_lib PUBLIC autonomy_contracts)
```

Minimal planner implementation:

```cpp
#include "autonomy_contracts/autonomy_contracts.hpp"

class MyPlanner
{
public:
  autonomy_contracts::PlanningOutput2D plan(
    const autonomy_contracts::PlanningInput2D& input)
  {
    autonomy_contracts::PlanningOutput2D output;
    // Fill output.path.poses here.
    output.success = true;
    return output;
  }
};

REGISTER_PLANNER_COMPONENT("my_planner", MyPlanner)
```

Then link the repo from `autonomy-sim/external_components/component_manifest.cmake`:

```cmake
autonomy_fetch_component(
  NAME my_planner
  GIT_REPOSITORY https://github.com/your-org/my-planner.git
  GIT_TAG main
  TARGET my_planner_lib
)
```

Finally select the registered component name in Nav2 YAML:

```yaml
GridBased:
  plugin: "autonomy_ros2_wrapper/ContractPlanner"
  planner_component: "my_planner"
```

### SIMULATION

The ros-autonomy sim contains the codebase for the gazebo simulation of the robot.

To start simulation, navigate to `ros-autonomy-sim` and run `just start-gazebo-sim`

> [!NOTE]
> [just](https://github.com/casey/just) is used to simplify execution 
>
> Install it with:
>
> ```bash
> curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
> ```

### Versions
- ROS 2 - Humble (Through the docker image `husarion/rosbot:humble-0.13.1-20240201`)
- Gazebo Simulation - Version 6.15.0 (Utilized through docker image `husarion/rosbot-gazebo:humble-0.13.0-20240115`)
- RViz version 11.2.6 (ROS 2) (Through docker image `husarion/rviz2:humble-11.2.6-20230809`)
- Base docker image for custom Navigation container - `husarion/navigation2:humble-1.1.12-20240123`

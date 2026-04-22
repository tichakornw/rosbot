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

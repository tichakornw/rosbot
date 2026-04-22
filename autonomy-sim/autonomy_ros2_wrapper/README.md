# autonomy_ros2_wrapper

`autonomy_ros2_wrapper` keeps ROS 2 internal while exposing ROS-free C++ component contracts.

- `ContractPlanner` is a Nav2 global planner plugin that calls a registered planner component.
- `ContractController` is a Nav2 controller plugin that uses the common pure-pursuit default.
- `perception_wrapper_node` is a lifecycle node that uses the common noop perception default and publishes internal obstacle markers.

The swappable planning reference lives outside the wrapper in
`external_components/reference_components` and compiles into `autonomy_reference_components`:

- `bfs_grid_planner`

Additional plain C++ planner libraries can be linked with the CMake cache variable
`AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES`. Those libraries must register a planner adapter
with `REGISTER_PLANNER_COMPONENT`.

For local sibling repos, place them under `autonomy-sim/external_components/`, put the
adapter registration in that repo's `.cpp` file, and wire its CMake target in
`external_components/component_manifest.cmake`.

The perception wrapper is available as an optional Docker Compose profile:

```bash
docker compose --profile contract_perception -f compose.sim.gazebo.yaml up
```

The current default keeps YOLO as an internal ROS perception source. Nav2 configs
consume its marker output at `/rosbot2r/yolo/dgb_bb_markers`. To consume the
contract perception wrapper instead, set `dynamic_obstacles_topic` in the
planner/controller YAML to `/rosbot2r/autonomy_wrapper/dynamic_obstacles`.

Nav2 YAML selects the planner component with:

```yaml
GridBased:
  plugin: "autonomy_ros2_wrapper/ContractPlanner"
  planner_component: "bfs_grid_planner"

FollowPath:
  plugin: "autonomy_ros2_wrapper/ContractController"
```

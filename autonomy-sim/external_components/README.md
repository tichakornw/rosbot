# external_components

Put plain C++ component repositories here when developing them alongside this wrapper.
These repositories should not include ROS 2 or Nav2 code. They should depend only on
`autonomy_contracts`, implement the exact planning/control/perception contract methods,
and register their components by name.

The default examples live in `reference_components`:

- `bfs_grid_planner`
- `pure_pursuit_controller`
- `noop_perception`

## GitHub link boxes

Edit `component_manifest.cmake`. It has three labeled sections:

- `PLANNING COMPONENT LINK`
- `CONTROL COMPONENT LINK`
- `PERCEPTION COMPONENT LINK`

Each section takes a GitHub URL, a Git tag/branch/commit, and the CMake target
exported by that external repo.

Example:

```cmake
autonomy_fetch_component(
  NAME my_planner
  GIT_REPOSITORY https://github.com/your-org/my-planner.git
  GIT_TAG main
  TARGET my_planner_lib
)
```

For local development, use `SOURCE_DIR` instead of `GIT_REPOSITORY`:

```cmake
autonomy_fetch_component(
  NAME my_planner
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/my_planner"
  TARGET my_planner_lib
)
```

Then select the registered name from Nav2 YAML:

```yaml
GridBased:
  plugin: "autonomy_ros2_wrapper/ContractPlanner"
  planner_component: "my_planner"
```

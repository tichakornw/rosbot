# external_components

Put plain C++ planner repositories here when developing them alongside this wrapper.
These repositories should not include ROS 2 or Nav2 code. They should depend only on
`autonomy_contracts`, implement the exact planning contract method, and register their
planner component by name.

The planning helpers in `autonomy_contracts` are templated for other C++ use cases, but
this ROS wrapper instantiates the external seam as `PlanningInput2D -> PlanningOutput2D`.

Planning is the only external extension point. The included external reference planner is:

- `bfs_grid_planner`

Control and perception use the common defaults built into `autonomy_ros2_wrapper`:

- `pure_pursuit_controller`
- `noop_perception`

## GitHub link boxes

Edit `component_manifest.cmake`. It has one labeled section:

- `PLANNING COMPONENT LINK`

It takes a GitHub URL, a Git tag/branch/commit, and the CMake target exported by
that external planner repo.

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

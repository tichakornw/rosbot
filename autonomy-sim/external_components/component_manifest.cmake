include("${CMAKE_CURRENT_LIST_DIR}/autonomy_fetch_component.cmake")

# ---------------------------------------------------------------------------
# PLANNING COMPONENT LINK
# Controls: Global path planning from GridMap2D + start/goal to Path2D.
# Put a planner GitHub URL in GIT_REPOSITORY and its exported CMake library in TARGET.
# ---------------------------------------------------------------------------
autonomy_fetch_component(
  NAME reference_components
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/reference_components"
  TARGET autonomy_reference_components
)

# Example planner GitHub link:
#
# autonomy_fetch_component(
#   NAME my_planner
#   GIT_REPOSITORY https://github.com/your-org/my-planner.git
#   GIT_TAG main
#   TARGET my_planner_lib
# )

# ---------------------------------------------------------------------------
# CONTROL COMPONENT LINK
# Controls: Local velocity command generation from Path2D + robot state.
# Put a controller GitHub URL in GIT_REPOSITORY and its exported CMake library in TARGET.
# ---------------------------------------------------------------------------
#
# autonomy_fetch_component(
#   NAME my_controller
#   GIT_REPOSITORY https://github.com/your-org/my-controller.git
#   GIT_TAG main
#   TARGET my_controller_lib
# )

# ---------------------------------------------------------------------------
# PERCEPTION COMPONENT LINK
# Controls: Image/depth processing into detections and dynamic obstacles.
# Put a perception GitHub URL in GIT_REPOSITORY and its exported CMake library in TARGET.
# ---------------------------------------------------------------------------
#
# autonomy_fetch_component(
#   NAME my_perception
#   GIT_REPOSITORY https://github.com/your-org/my-perception.git
#   GIT_TAG main
#   TARGET my_perception_lib
# )

# Requirements for every linked external component:
#
# Each library target appended to AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES must link
# autonomy_contracts and register components with:
#
# REGISTER_PLANNER_COMPONENT("my_planner", MyPlanner)
# REGISTER_CONTROLLER_COMPONENT("my_controller", MyController)
# REGISTER_PERCEPTION_COMPONENT("my_perception", MyPerception)

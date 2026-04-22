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

# Requirements for every linked external planner component:
#
# Each library target appended to AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES must link
# autonomy_contracts and register a planner with:
#
# REGISTER_PLANNER_COMPONENT("my_planner", MyPlanner)

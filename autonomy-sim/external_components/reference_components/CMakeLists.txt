add_library(autonomy_reference_components SHARED
  src/grid_bfs_planner.cpp
)

ament_target_dependencies(autonomy_reference_components
  autonomy_contracts
)

install(TARGETS autonomy_reference_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

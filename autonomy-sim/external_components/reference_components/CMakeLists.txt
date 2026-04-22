add_library(autonomy_reference_components SHARED
  src/grid_bfs_planner.cpp
  src/noop_perception.cpp
  src/pure_pursuit_component.cpp
  src/reference_components.cpp
)

target_include_directories(autonomy_reference_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(autonomy_reference_components
  autonomy_contracts
)

install(TARGETS autonomy_reference_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

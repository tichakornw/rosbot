cmake_minimum_required(VERSION 3.5)
project(autonomy_ros2_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy_contracts REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES "" CACHE STRING
  "Additional linked C++ component libraries that register autonomy_contracts components")
set(AUTONOMY_EXTERNAL_COMPONENT_MANIFEST
  "${CMAKE_CURRENT_SOURCE_DIR}/../external_components/component_manifest.cmake"
  CACHE FILEPATH
  "CMake manifest that adds external component repos and appends their library targets")

if(EXISTS "${AUTONOMY_EXTERNAL_COMPONENT_MANIFEST}")
  include("${AUTONOMY_EXTERNAL_COMPONENT_MANIFEST}")
endif()

set(dependencies
  autonomy_contracts
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  lifecycle_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pluginlib
  visualization_msgs
)

add_library(${PROJECT_NAME}_plugins SHARED
  src/controller_wrapper.cpp
  src/conversions.cpp
  src/planner_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_plugins
  ${dependencies}
)

target_compile_definitions(${PROJECT_NAME}_plugins
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(perception_wrapper_node
  src/perception_wrapper_node.cpp
)

target_link_libraries(perception_wrapper_node
  ${AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES}
)

ament_target_dependencies(perception_wrapper_node
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core autonomy_ros2_wrapper_plugins.xml)

install(TARGETS
  ${PROJECT_NAME}_plugins
  perception_wrapper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES autonomy_ros2_wrapper_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_plugins
)
ament_export_dependencies(${dependencies})

ament_package()

cmake_minimum_required(VERSION 3.15...4.0)

project(multicost_planning LANGUAGES C CXX)


# ------------------ Compile with GUI ------------------ #
find_package(raylib)
if(raylib_FOUND)
    message(STATUS "Raylib was found!")
else()
    message(FATAL_ERROR "Raylib not found!")
endif()


add_executable(multicost_planning)

set(CMAKE_CXX_STANDARD 17) # Set to C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Require C++17, prevent fallback to older standards
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

target_link_libraries(multicost_planning raylib)
target_compile_options(multicost_planning PRIVATE -O2)

target_sources(multicost_planning PRIVATE
    source/examples/example_setup.cpp
    source/state/grid_state.cpp
    source/search/iterated_dijkstra_propagation.cpp
    test/environment.cpp
)


# ------------------ Compile without GUI ------------------ #
# add_executable(multicost_planning)
# 
# set(CMAKE_CXX_STANDARD 17) # Set to C++17
# set(CMAKE_CXX_STANDARD_REQUIRED ON) # Require C++17, prevent fallback to older standards
# set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 
# target_compile_options(multicost_planning PRIVATE -O2)
# 
# target_sources(multicost_planning PRIVATE
#     source/examples/example_setup.cpp
#     source/state/grid_state.cpp
#     source/search/iterated_dijkstra_propagation.cpp
# )


# ------------------ PYTHON BINDINGS ------------------ #
# add_executable(multicost_pathfind_pybind)
# 
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# find_package(pybind11 REQUIRED)
# 
# message(STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS}")
# message(STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES}")
# 
# 
# target_link_libraries(multicost_pathfind_pybind PRIVATE pybind11::pybind11 ${Python3_LIBRARIES})
# target_sources(multicost_pathfind_pybind PRIVATE
#     source/examples/example_setup.cpp
#     source/state/grid_state.cpp
#     source/search/iterated_dijkstra_propagation.cpp
#     test/benchmark_bindings.cpp
# )
# 
# pybind11_add_module(multicost_pathfind_bindings
#     source/examples/example_setup.cpp
#     source/state/grid_state.cpp
#     source/search/iterated_dijkstra_propagation.cpp
#     test/benchmark_bindings.cpp
# )

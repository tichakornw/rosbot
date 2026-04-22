include(FetchContent)

function(autonomy_fetch_component)
  set(options)
  set(one_value_args NAME TARGET SOURCE_DIR GIT_REPOSITORY GIT_TAG)
  set(multi_value_args)
  cmake_parse_arguments(COMPONENT "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT COMPONENT_NAME)
    message(FATAL_ERROR "autonomy_fetch_component requires NAME")
  endif()

  if(NOT COMPONENT_TARGET)
    message(FATAL_ERROR "autonomy_fetch_component(${COMPONENT_NAME}) requires TARGET")
  endif()

  if(COMPONENT_SOURCE_DIR AND COMPONENT_GIT_REPOSITORY)
    message(FATAL_ERROR
      "autonomy_fetch_component(${COMPONENT_NAME}) must use SOURCE_DIR or GIT_REPOSITORY, not both")
  endif()

  if(COMPONENT_SOURCE_DIR)
    add_subdirectory(
      "${COMPONENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/external_components/${COMPONENT_NAME}"
    )
  elseif(COMPONENT_GIT_REPOSITORY)
    if(NOT COMPONENT_GIT_TAG)
      set(COMPONENT_GIT_TAG main)
    endif()

    FetchContent_Declare(
      ${COMPONENT_NAME}
      GIT_REPOSITORY "${COMPONENT_GIT_REPOSITORY}"
      GIT_TAG "${COMPONENT_GIT_TAG}"
    )
    FetchContent_MakeAvailable(${COMPONENT_NAME})
  else()
    message(FATAL_ERROR
      "autonomy_fetch_component(${COMPONENT_NAME}) requires SOURCE_DIR or GIT_REPOSITORY")
  endif()

  if(NOT TARGET ${COMPONENT_TARGET})
    message(FATAL_ERROR
      "autonomy_fetch_component(${COMPONENT_NAME}) expected CMake target '${COMPONENT_TARGET}'")
  endif()

  list(APPEND AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES ${COMPONENT_TARGET})
  set(AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES
    "${AUTONOMY_EXTERNAL_COMPONENT_LIBRARIES}"
    PARENT_SCOPE
  )
endfunction()

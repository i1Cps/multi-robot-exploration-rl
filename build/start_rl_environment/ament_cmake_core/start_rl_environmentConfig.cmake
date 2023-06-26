# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_start_rl_environment_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED start_rl_environment_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(start_rl_environment_FOUND FALSE)
  elseif(NOT start_rl_environment_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(start_rl_environment_FOUND FALSE)
  endif()
  return()
endif()
set(_start_rl_environment_CONFIG_INCLUDED TRUE)

# output package information
if(NOT start_rl_environment_FIND_QUIETLY)
  message(STATUS "Found start_rl_environment: 0.0.0 (${start_rl_environment_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'start_rl_environment' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${start_rl_environment_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(start_rl_environment_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${start_rl_environment_DIR}/${_extra}")
endforeach()

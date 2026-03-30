# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mpc-tracker_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mpc-tracker_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mpc-tracker_FOUND FALSE)
  elseif(NOT mpc-tracker_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mpc-tracker_FOUND FALSE)
  endif()
  return()
endif()
set(_mpc-tracker_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mpc-tracker_FIND_QUIETLY)
  message(STATUS "Found mpc-tracker: 0.0.0 (${mpc-tracker_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mpc-tracker' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT mpc-tracker_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mpc-tracker_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mpc-tracker_DIR}/${_extra}")
endforeach()

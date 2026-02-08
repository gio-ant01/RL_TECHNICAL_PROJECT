# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_escape_room_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED escape_room_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(escape_room_FOUND FALSE)
  elseif(NOT escape_room_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(escape_room_FOUND FALSE)
  endif()
  return()
endif()
set(_escape_room_CONFIG_INCLUDED TRUE)

# output package information
if(NOT escape_room_FIND_QUIETLY)
  message(STATUS "Found escape_room: 0.0.1 (${escape_room_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'escape_room' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${escape_room_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(escape_room_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${escape_room_DIR}/${_extra}")
endforeach()

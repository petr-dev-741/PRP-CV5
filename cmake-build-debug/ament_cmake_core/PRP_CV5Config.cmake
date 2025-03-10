# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_PRP_CV5_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED PRP_CV5_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(PRP_CV5_FOUND FALSE)
  elseif(NOT PRP_CV5_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(PRP_CV5_FOUND FALSE)
  endif()
  return()
endif()
set(_PRP_CV5_CONFIG_INCLUDED TRUE)

# output package information
if(NOT PRP_CV5_FIND_QUIETLY)
  message(STATUS "Found PRP_CV5: 0.0.0 (${PRP_CV5_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'PRP_CV5' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${PRP_CV5_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(PRP_CV5_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${PRP_CV5_DIR}/${_extra}")
endforeach()

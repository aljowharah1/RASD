# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rasd_elev_profile_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rasd_elev_profile_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rasd_elev_profile_FOUND FALSE)
  elseif(NOT rasd_elev_profile_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rasd_elev_profile_FOUND FALSE)
  endif()
  return()
endif()
set(_rasd_elev_profile_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rasd_elev_profile_FIND_QUIETLY)
  message(STATUS "Found rasd_elev_profile: 0.0.0 (${rasd_elev_profile_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rasd_elev_profile' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rasd_elev_profile_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rasd_elev_profile_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rasd_elev_profile_DIR}/${_extra}")
endforeach()

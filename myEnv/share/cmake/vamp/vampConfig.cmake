
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was vampConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Find required dependencies
find_dependency(Eigen3 REQUIRED)

# VAMP requires these dependencies but they're bundled
# No need to find_dependency for nigh, pdqsort, or SIMDxorshift
# as they're built as part of VAMP

# Include the exported targets
include("${CMAKE_CURRENT_LIST_DIR}/vamp_cppTargets.cmake")

# Check that the main target exists
if(NOT TARGET vamp::vamp)
    message(FATAL_ERROR "Expected target vamp::vamp not found!")
endif()

# Provide legacy target name for compatibility
if(NOT TARGET vamp_cpp)
    add_library(vamp_cpp ALIAS vamp::vamp)
endif()

# Set success
set(vamp_FOUND TRUE)

check_required_components(vamp)

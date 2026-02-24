#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ompl::ompl" for configuration "Release"
set_property(TARGET ompl::ompl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ompl::ompl PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libompl.so.2.0.0-beta"
  IMPORTED_SONAME_RELEASE "libompl.so.19"
  )

list(APPEND _cmake_import_check_targets ompl::ompl )
list(APPEND _cmake_import_check_files_for_ompl::ompl "${_IMPORT_PREFIX}/lib/libompl.so.2.0.0-beta" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

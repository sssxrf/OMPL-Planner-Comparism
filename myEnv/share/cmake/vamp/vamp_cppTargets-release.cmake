#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vamp::simdxorshift" for configuration "Release"
set_property(TARGET vamp::simdxorshift APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vamp::simdxorshift PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsimdxorshift.a"
  )

list(APPEND _cmake_import_check_targets vamp::simdxorshift )
list(APPEND _cmake_import_check_files_for_vamp::simdxorshift "${_IMPORT_PREFIX}/lib/libsimdxorshift.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

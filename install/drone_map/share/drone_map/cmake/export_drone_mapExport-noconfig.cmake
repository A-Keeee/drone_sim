#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drone_map::drone_map_core" for configuration ""
set_property(TARGET drone_map::drone_map_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(drone_map::drone_map_core PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdrone_map_core.so"
  IMPORTED_SONAME_NOCONFIG "libdrone_map_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_map::drone_map_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_map::drone_map_core "${_IMPORT_PREFIX}/lib/libdrone_map_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

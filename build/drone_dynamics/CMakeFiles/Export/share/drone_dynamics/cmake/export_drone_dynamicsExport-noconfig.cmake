#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drone_dynamics::drone_dynamics_core" for configuration ""
set_property(TARGET drone_dynamics::drone_dynamics_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(drone_dynamics::drone_dynamics_core PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdrone_dynamics_core.so"
  IMPORTED_SONAME_NOCONFIG "libdrone_dynamics_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_dynamics::drone_dynamics_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_dynamics::drone_dynamics_core "${_IMPORT_PREFIX}/lib/libdrone_dynamics_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

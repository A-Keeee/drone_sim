#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drone_controller::drone_controller_core" for configuration ""
set_property(TARGET drone_controller::drone_controller_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(drone_controller::drone_controller_core PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdrone_controller_core.so"
  IMPORTED_SONAME_NOCONFIG "libdrone_controller_core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS drone_controller::drone_controller_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_drone_controller::drone_controller_core "${_IMPORT_PREFIX}/lib/libdrone_controller_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

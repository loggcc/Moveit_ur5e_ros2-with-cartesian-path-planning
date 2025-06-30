#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robotiq_controllers::robotiq_controllers" for configuration ""
set_property(TARGET robotiq_controllers::robotiq_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robotiq_controllers::robotiq_controllers PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librobotiq_controllers.so"
  IMPORTED_SONAME_NOCONFIG "librobotiq_controllers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robotiq_controllers::robotiq_controllers )
list(APPEND _IMPORT_CHECK_FILES_FOR_robotiq_controllers::robotiq_controllers "${_IMPORT_PREFIX}/lib/librobotiq_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

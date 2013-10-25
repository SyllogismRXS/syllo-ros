# - Try to find Stage
# Once done this will define
#  Stage_FOUND - System has Stage
#  Stage_INCLUDE_DIRS - The Stage include directories
#  Stage_LIBRARIES - The libraries needed to use Stage
#  Stage_DEFINITIONS - Compiler switches required for using Stage

find_package(PkgConfig)
pkg_check_modules(PC_Stage QUIET Stage)
set(Stage_DEFINITIONS ${PC_Stage_CFLAGS_OTHER})

find_path(
  Stage_INCLUDE_DIR 
  NAMES Stage-4.1/stage.hh
  HINTS ${PC_Stage_INCLUDEDIR} ${PC_Stage_INCLUDE_DIRS}
  #PATH_SUFFIXES libstage
  )

#message("========> Stage_INCLUDE_DIR  <=========")
#message(${Stage_INCLUDE_DIR})

find_library(
  Stage_LIBRARY 
  NAMES libstage.so
  HINTS ${PC_Stage_LIBDIR} ${PC_Stage_LIBRARY_DIRS} /usr/local/lib /usr/local/lib64
  )

set(Stage_LIBRARIES ${Stage_LIBRARY} )
set(Stage_INCLUDE_DIRS ${Stage_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set Stage_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Stage  DEFAULT_MSG
                                  Stage_LIBRARY Stage_INCLUDE_DIR)

mark_as_advanced(Stage_INCLUDE_DIR Stage_LIBRARY )

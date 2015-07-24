# - Try to find opencv_workbench
# Once done, this will define
#
#  opencv_workbench_FOUND - system has opencv_workbench
#  opencv_workbench_INCLUDE_DIRS - the opencv_workbench include directories
#  opencv_workbench_LIBRARIES - link these to use opencv_workbench

include(LibFindMacros)

# Dependencies (forward, required or quietly)
#libfind_package(opencv_workbench_SIM opencv_workbench)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(opencv_workbench_PKGCONF opencv_workbench)

#message("==================================================")
#message("opencv_workbench_INCLUDE_DIR: ${opencv_workbench_INCLUDE_DIR}")
#message("opencv_workbench_PKGCONF_LIBRARY_DIRS: ${opencv_workbench_PKGCONF_LIBRARY_DIRS}")
#message("==================================================")

# Include dir
find_path(opencv_workbench_INCLUDE_DIR
  #NAMES opencv_workbench/motion/MotionModel.h - fix for vince, not creating include dir
  NAMES opencv_workbench/detector/Detector.h
  PATHS ${opencv_workbench_PKGCONF_INCLUDE_DIRS}
)

#find_path(opencv_workbench_INCLUDE_DIR_2
#  #NAMES opencv_workbench/motion/MotionModel.h - fix for vince, not creating include dir
#  NAMES OPENCV_WORKBENCH_Math.h
#  PATHS ${opencv_workbench_PKGCONF_INCLUDE_DIRS}
#)
#message("=======> ${opencv_workbench_INCLUDE_DIR_2}")

# Find all the relevant opencv_workbench libraries
find_library(plot_LIBRARY
  NAMES plot
  PATHS ${opencv_workbench_PKGCONF_LIBRARY_DIRS}
)

find_library(syllo_LIBRARY
  NAMES syllo
  PATHS ${opencv_workbench_PKGCONF_LIBRARY_DIRS}
)

find_library(track_LIBRARY
  NAMES track
  PATHS ${opencv_workbench_PKGCONF_LIBRARY_DIRS}
)

find_library(workbench-utils_LIBRARY
  NAMES workbench-utils
  PATHS ${opencv_workbench_PKGCONF_LIBRARY_DIRS}
)

find_library(blank_detector_LIBRARY
  NAMES blank_detector
  PATHS ${opencv_workbench_PKGCONF_LIBRARY_DIRS}
)

find_library(displace_detector_LIBRARY
  NAMES displace_detector
  PATHS ${opencv_workbench_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(opencv_workbench_PROCESS_INCLUDES opencv_workbench_INCLUDE_DIR)
set(opencv_workbench_PROCESS_LIBS 
  plot_LIBRARY
  syllo_LIBRARY
  track_LIBRARY
  workbench-utils_LIBRARY
  blank_detector_LIBRARY
  displace_detector_LIBRARY
)

libfind_process(opencv_workbench)

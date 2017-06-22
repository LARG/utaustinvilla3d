# - Find Rcssnet3d
# Find the native rcssnet includes and libraries
#
#  RCSSNET3D_INCLUDE_DIR     - where to find rcssnet include files
#  RCSSNET3D_LIBRARIES       - List of libraries when using rcssnet.
#  RCSSNET3D_FOUND           - True if rcssnet found.

SET(SPARK_DIR $ENV{SPARK_DIR} "C:/Program Files/simspark" "C:/Program Files (x86)/simspark" "C:/library/simspark")


# rcssnet3D
IF (RCSSNET3D_INCLUDE_DIR)
  # Already in cache, be silent
  SET(RCSSNET3D_FIND_QUIETLY TRUE)
ENDIF (RCSSNET3D_INCLUDE_DIR)

FIND_PATH(RCSSNET3D_INCLUDE_DIR rcssnet/addr.hpp
  HINTS ENV SPARK_DIR
  PATHS ${SPARK_DIR}
  PATH_SUFFIXES simspark include/simspark)

SET(RCSSNET3D_NAMES rcssnet3D rcssnet3D_debug)
FIND_LIBRARY(RCSSNET3D_LIBRARY NAMES ${RCSSNET3D_NAMES}
  HINTS ENV SPARK_DIR
  PATHS ${SPARK_DIR}
  PATH_SUFFIXES simspark lib/simspark)

# handle the QUIETLY and REQUIRED arguments and set RCSSNET3D_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RCSSNET3D DEFAULT_MSG RCSSNET3D_LIBRARY
         RCSSNET3D_INCLUDE_DIR)

IF(RCSSNET3D_FOUND)
  SET( RCSSNET3D_LIBRARIES ${RCSSNET3D_LIBRARY} )
ELSE(RCSSNET3D_FOUND)
  SET( RCSSNET3D_LIBRARIES )
ENDIF(RCSSNET3D_FOUND)

MARK_AS_ADVANCED( RCSSNET3D_LIBRARY RCSSNET3D_INCLUDE_DIR )

# Configure libLAS package
#
# It defines the following variables
#  libLAS_FOUND = LIBLAS_FOUND - TRUE
#  libLAS_INCLUDE_DIRS - include directories for libLAS
#  libLAS_LIBRARY_DIRS - library directory
#  libLAS_LIBRARIES    - the libraries (as targets)
#  libLAS_BINARY_DIRS  - the directory for dll and utilites
#  libLAS_VERSION      - libLAS library version

if (NOT libLAS_FIND_QUIETLY)
  message (STATUS "Reading ${CMAKE_CURRENT_LIST_FILE}")
  # libLAS_VERSION is set by version file
  message (STATUS "libLAS configuration, version " ${libLAS_VERSION})
endif ()

include (CMakeFindDependencyMacro)
find_dependency (Boost 1.71.0 EXACT COMPONENTS iostreams program_options serialization thread)

# Tell the user project where to find our headers and libraries
get_filename_component (_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
get_filename_component (PROJECT_ROOT_DIR "${_DIR}/../../.." ABSOLUTE)
set (libLAS_INCLUDE_DIRS "${PROJECT_ROOT_DIR}/include")
set (libLAS_LIBRARY_DIRS "${PROJECT_ROOT_DIR}/lib")
set (libLAS_BINARY_DIRS "${PROJECT_ROOT_DIR}/bin")

include ("${_DIR}/liblas-depends.cmake")
set (libLAS_LIBRARIES las las_c)

# For backwards compatibility
set (LIBLAS_FOUND TRUE)

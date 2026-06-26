# FindOptiX.cmake — locates NVIDIA OptiX SDK 7+ headers.
#
# Creates imported target OptiX::OptiX (interface/header-only — OptiX 7+ loads
# libnvoptix.so.1 at runtime via optixInit(); there is no link-time library).
#
# Search priority:
#   1. CMake variable OPTIX_ROOT (passed via -DOPTIX_ROOT=...)
#   2. Environment variable OPTIX_ROOT
#   3. /opt/optix  (default container install path)

find_path(OPTIX_INCLUDE_DIR
  NAMES optix.h
  PATHS
    "${OPTIX_ROOT}/include"
    "$ENV{OPTIX_ROOT}/include"
    /opt/optix/include
  NO_DEFAULT_PATH
)

# Extract the SDK version from optix.h for version checking / status messages.
if(OPTIX_INCLUDE_DIR AND EXISTS "${OPTIX_INCLUDE_DIR}/optix.h")
  file(STRINGS "${OPTIX_INCLUDE_DIR}/optix.h" _optix_ver_line
    REGEX "#define OPTIX_VERSION")
  if(_optix_ver_line MATCHES "#define OPTIX_VERSION ([0-9]+)")
    set(_ver ${CMAKE_MATCH_1})
    math(EXPR OPTIX_VERSION_MAJOR "${_ver} / 10000")
    math(EXPR OPTIX_VERSION_MINOR "(${_ver} % 10000) / 100")
    math(EXPR OPTIX_VERSION_PATCH "${_ver} % 100")
    set(OPTIX_VERSION "${OPTIX_VERSION_MAJOR}.${OPTIX_VERSION_MINOR}.${OPTIX_VERSION_PATCH}")
  endif()
  unset(_optix_ver_line)
  unset(_ver)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OptiX
  REQUIRED_VARS OPTIX_INCLUDE_DIR
  VERSION_VAR   OPTIX_VERSION
)
mark_as_advanced(OPTIX_INCLUDE_DIR)

if(OptiX_FOUND AND NOT TARGET OptiX::OptiX)
  add_library(OptiX::OptiX INTERFACE IMPORTED)
  target_include_directories(OptiX::OptiX INTERFACE "${OPTIX_INCLUDE_DIR}")
  message(STATUS "OptiX ${OPTIX_VERSION} found: ${OPTIX_INCLUDE_DIR}")
endif()

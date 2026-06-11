cmake_minimum_required(VERSION 3.25)

# CXX only at the top level. CUDA is enabled *conditionally* below so the minimal CPU build
# (dev-cpu preset) never requires nvcc. This is the spec §1.2 "minimal build must compile" rule.
project(quevedomp LANGUAGES CXX)

# ---- Global C++ settings -----------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Options -----------------------------------------------------------------------------
# WITH_CUDA defaults ON: we validate the GPU toolchain from Phase 0 (the dev-gpu preset).
# The dev-cpu preset turns it OFF to prove the no-GPU path still builds.
option(QUEVEDOMP_WITH_CUDA       "Enable CUDA toolchain + CUDA smoke test" ON)
option(QUEVEDOMP_WITH_OPTIX      "Enable OptiX collision backend (Phase 2b)" OFF)
option(QUEVEDOMP_WITH_RERUN      "Enable rerun.io visualization (Phase 1)"   OFF)
option(QUEVEDOMP_WITH_PYTHON     "Build nanobind Python bindings (Phase 4)"  OFF)
option(QUEVEDOMP_BUILD_TESTS     "Build tests"                               ON)
option(QUEVEDOMP_ENABLE_SANITIZERS "Enable ASan/UBSan in Debug builds"       OFF)

# ---- Sanitizers (Debug, CPU) -------------------------------------------------------------
if(QUEVEDOMP_ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---- The library (placeholder until Phase 1) ---------------------------------------------
# INTERFACE keeps Phase 0 source-free; becomes a real CPU static library in Phase 1.
add_library(quevedomp INTERFACE)
add_library(quevedomp::quevedomp ALIAS quevedomp)
target_include_directories(quevedomp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(quevedomp INTERFACE cxx_std_20)

# ---- Optional CUDA toolchain validation --------------------------------------------------
# The library stays CPU-only; this only builds a throwaway smoke test that proves nvcc + the
# GPU + --gpus passthrough all work, so a toolchain mistake surfaces now, not in Phase 2b.
if(QUEVEDOMP_WITH_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    message(FATAL_ERROR
      "QUEVEDOMP_WITH_CUDA=ON but no CUDA compiler was found (need nvcc in PATH). "
      "Use the dev-cpu preset for a no-GPU build, or fix the CUDA toolchain.")
  endif()
  enable_language(CUDA)
  message(STATUS "CUDA enabled: ${CMAKE_CUDA_COMPILER}")
  add_subdirectory(tools/cuda_smoke)
endif()

# ---- OptiX (Phase 2b) --------------------------------------------------------------------
if(QUEVEDOMP_WITH_OPTIX)
  message(FATAL_ERROR "QUEVEDOMP_WITH_OPTIX is not implemented until Phase 2b.")
endif()

# ---- Tests -------------------------------------------------------------------------------
if(QUEVEDOMP_BUILD_TESTS)
  enable_testing()
  # Module mode (CMake's bundled FindGTest) — robust against the apt-vs-vcpkg config-package
  # mismatch the earlier attempt hit.
  find_package(GTest REQUIRED)
  add_subdirectory(tests)
endif()

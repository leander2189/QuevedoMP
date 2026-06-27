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

# ---- Testing ------------------------------------------------------------------------------
# enable_testing() must run BEFORE any add_subdirectory() that calls add_test(), or those
# tests are silently dropped from the generated CTest files (cuda_smoke was lost this way:
# the WITH_CUDA block below ran first and its add_test never registered).
if(QUEVEDOMP_BUILD_TESTS)
  enable_testing()
endif()

# ---- Sanitizers (Debug, CPU) -------------------------------------------------------------
if(QUEVEDOMP_ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---- The library -------------------------------------------------------------------------
# Real CPU static library as of Phase 1 (Task 1.1). Stays CPU-only and Eigen-only at the core;
# the GPU backend is a separate target added in Phase 2b. Eigen is header-only but we link the
# imported target so consumers inherit its include path transitively.
find_package(Eigen3 CONFIG REQUIRED)

add_library(quevedomp
  src/core/types.cpp
  src/core/rng.cpp
)
add_library(quevedomp::quevedomp ALIAS quevedomp)
target_include_directories(quevedomp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(quevedomp PUBLIC cxx_std_20)
target_link_libraries(quevedomp PUBLIC Eigen3::Eigen)

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

# ---- OptiX (Task 0.9 smoke test + Phase 2b backend) ------------------------------------
if(QUEVEDOMP_WITH_OPTIX)
  if(NOT QUEVEDOMP_WITH_CUDA)
    message(FATAL_ERROR "QUEVEDOMP_WITH_OPTIX=ON requires QUEVEDOMP_WITH_CUDA=ON.")
  endif()
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  find_package(OptiX REQUIRED)
  add_subdirectory(tools/optix_smoke)
endif()

# ---- Tests -------------------------------------------------------------------------------
if(QUEVEDOMP_BUILD_TESTS)
  # Module mode (CMake's bundled FindGTest) — robust against the apt-vs-vcpkg config-package
  # mismatch the earlier attempt hit.
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)  # test_rng exercises spawn() across threads
  add_subdirectory(tests)
endif()

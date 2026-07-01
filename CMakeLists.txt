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
option(QUEVEDOMP_ENABLE_COVERAGE "Build with gcov coverage instrumentation"  OFF)
option(QUEVEDOMP_BUILD_FUZZERS   "Build libFuzzer fuzz targets (needs clang)" OFF)

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

# ---- Coverage (Task 1.9 gate) — gcov instrumentation across all targets defined below -----
if(QUEVEDOMP_ENABLE_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

# ---- Fuzzing (Task 1.9) — libFuzzer + ASan/UBSan; the fuzz exe adds -fsanitize=fuzzer ------
if(QUEVEDOMP_BUILD_FUZZERS)
  add_compile_options(-fsanitize=fuzzer-no-link,address,undefined -fno-omit-frame-pointer -O1 -g)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---- The library -------------------------------------------------------------------------
# Real CPU static library as of Phase 1. Stays CPU-only at the core; the GPU backend is a
# separate target added in Phase 2b. Eigen is a PUBLIC dep (appears in headers); urdfdom and
# yaml-cpp are PRIVATE implementation details of URDF parsing (Task 1.3) and are not leaked
# through the public headers.
find_package(Eigen3 CONFIG REQUIRED)
find_package(urdfdom REQUIRED)   # URDF parsing (Task 1.3)
find_package(yaml-cpp REQUIRED)  # optional accel/jerk limit extension
find_package(assimp REQUIRED)    # mesh loading STL/DAE/OBJ (Task 1.4b)
find_package(fcl CONFIG REQUIRED) # CPU collision backend (Task 2a.2)

add_library(quevedomp
  src/core/types.cpp
  src/core/rng.cpp
  src/core/mesh_io.cpp
  src/robot/robot_model.cpp
  src/robot/mesh_resolver.cpp
  src/kinematics/fk.cpp
  src/kinematics/jacobian.cpp
  src/kinematics/ik.cpp
  src/collision/collision_scene.cpp
  src/collision/fcl_scene.cpp
  src/collision/containment.cpp
  src/collision/edge_check.cpp
  src/capture/serialize.cpp
  src/viz/visualizer.cpp
)
add_library(quevedomp::quevedomp ALIAS quevedomp)
target_include_directories(quevedomp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(quevedomp PRIVATE ${urdfdom_INCLUDE_DIRS})
target_compile_features(quevedomp PUBLIC cxx_std_20)
target_link_libraries(quevedomp PUBLIC Eigen3::Eigen)
target_link_libraries(quevedomp PRIVATE ${urdfdom_LIBRARIES} yaml-cpp assimp::assimp fcl)

# ---- Optional rerun.io visualization (Task 1.8, deviation D3) -----------------------------
# OFF by default: viz/visualizer.cpp compiles to no-ops and the SDK is never fetched. When ON,
# pull the prebuilt rerun C++ SDK (matches the `rerun` viewer / `pip install rerun-sdk==0.33.1`).
if(QUEVEDOMP_WITH_RERUN)
  include(FetchContent)
  FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/download/0.33.1/rerun_cpp_sdk.zip)
  FetchContent_MakeAvailable(rerun_sdk)
  target_link_libraries(quevedomp PRIVATE rerun_sdk)
  target_compile_definitions(quevedomp PRIVATE QUEVEDOMP_WITH_RERUN)
  message(STATUS "rerun visualization: ON (C++ SDK 0.33.1)")
endif()

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
  find_package(CUDAToolkit REQUIRED)
  add_subdirectory(tools/optix_smoke)

  # ---- OptiX collision backend (Task 2b.1) --------------------------------------------------
  # Device programs -> PTX (OBJECT lib with CUDA_PTX_COMPILATION). The library reads the .ptx at
  # runtime via QUEVEDOMP_OPTIX_PTX_PATH (a generator expression to the compiled PTX); embedding
  # the PTX into the binary is a later hardening step. Turing+ (CC 7.5) is the host floor (M1).
  add_library(quevedomp_optix_ptx OBJECT src/collision/optix/optix_programs.cu)
  set_target_properties(quevedomp_optix_ptx PROPERTIES CUDA_PTX_COMPILATION ON
                                                       CUDA_ARCHITECTURES 75)
  target_include_directories(quevedomp_optix_ptx
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/optix)
  target_link_libraries(quevedomp_optix_ptx PRIVATE OptiX::OptiX)
  target_compile_options(quevedomp_optix_ptx
    PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)

  target_sources(quevedomp PRIVATE src/collision/optix/optix_pipeline.cpp
                                   src/collision/optix/optix_scene.cpp)
  add_dependencies(quevedomp quevedomp_optix_ptx)
  target_include_directories(quevedomp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/optix)
  target_link_libraries(quevedomp PRIVATE OptiX::OptiX CUDA::cudart)
  target_compile_definitions(quevedomp
    PRIVATE QUEVEDOMP_WITH_OPTIX
            QUEVEDOMP_OPTIX_PTX_PATH="$<TARGET_OBJECTS:quevedomp_optix_ptx>")
endif()

# ---- Tests -------------------------------------------------------------------------------
if(QUEVEDOMP_BUILD_TESTS)
  # Module mode (CMake's bundled FindGTest) — robust against the apt-vs-vcpkg config-package
  # mismatch the earlier attempt hit.
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)  # test_rng exercises spawn() across threads
  add_subdirectory(tests)
endif()

# ---- Examples ----------------------------------------------------------------------------
option(QUEVEDOMP_BUILD_EXAMPLES "Build example programs" ON)
if(QUEVEDOMP_BUILD_EXAMPLES)
  add_subdirectory(examples/cpp)
endif()

# ---- Fuzz targets ------------------------------------------------------------------------
if(QUEVEDOMP_BUILD_FUZZERS)
  add_subdirectory(tests/fuzz)
endif()

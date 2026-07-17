# Unit/integration tests. GTest is found by the root CMakeLists (module mode).
include(GoogleTest)

add_executable(test_bootstrap unit/test_bootstrap.cpp)
target_link_libraries(test_bootstrap PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_bootstrap)

add_executable(test_types unit/test_types.cpp)
target_link_libraries(test_types PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_types)

add_executable(test_rng unit/test_rng.cpp)
target_link_libraries(test_rng PRIVATE quevedomp::quevedomp GTest::gtest_main Threads::Threads)
gtest_discover_tests(test_rng)

add_executable(test_robot_model unit/test_robot_model.cpp)
target_link_libraries(test_robot_model PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_robot_model
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_robot_model)

add_executable(test_robot_fixtures unit/test_robot_fixtures.cpp)
target_link_libraries(test_robot_fixtures PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_robot_fixtures
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_robot_fixtures)

add_executable(test_robot_robustness unit/test_robot_robustness.cpp)
target_link_libraries(test_robot_robustness PRIVATE quevedomp::quevedomp GTest::gtest_main)
# These tests feed malformed URDF on purpose; urdfdom (third-party) leaks on such input, so
# suppress its leaks under ASan/LSan — the same suppression the fuzzer uses (Task 1.9).
gtest_discover_tests(test_robot_robustness PROPERTIES
  ENVIRONMENT "LSAN_OPTIONS=suppressions=${CMAKE_SOURCE_DIR}/tests/fuzz/lsan_suppressions.txt")

add_executable(test_clearance_field unit/test_clearance_field.cpp)
target_link_libraries(test_clearance_field PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_clearance_field)

add_executable(test_acm_env unit/test_acm_env.cpp)
target_link_libraries(test_acm_env PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_acm_env
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_acm_env)

add_executable(test_edge_discretization unit/test_edge_discretization.cpp)
target_link_libraries(test_edge_discretization PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_edge_discretization)

add_executable(test_tessellate unit/test_tessellate.cpp)
target_link_libraries(test_tessellate PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_tessellate)

add_executable(test_mesh_io unit/test_mesh_io.cpp)
target_link_libraries(test_mesh_io PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_mesh_io
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_mesh_io)

add_executable(test_mesh_resolver unit/test_mesh_resolver.cpp)
target_link_libraries(test_mesh_resolver PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_mesh_resolver)

add_executable(test_robot_meshes unit/test_robot_meshes.cpp)
target_link_libraries(test_robot_meshes PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_robot_meshes
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_robot_meshes)

add_executable(test_fk unit/test_fk.cpp)
target_link_libraries(test_fk PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_fk
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_fk)

add_executable(test_jacobian unit/test_jacobian.cpp)
target_link_libraries(test_jacobian PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_jacobian
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_jacobian)

add_executable(test_ik unit/test_ik.cpp)
target_link_libraries(test_ik PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_ik
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_ik)

add_executable(test_viz unit/test_viz.cpp)
target_link_libraries(test_viz PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_viz
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_viz)

add_executable(test_collision_contract unit/test_collision_contract.cpp)
target_link_libraries(test_collision_contract PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_collision_contract)

add_executable(test_collision_fcl unit/test_collision_fcl.cpp)
target_link_libraries(test_collision_fcl PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_collision_fcl
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_collision_fcl)

add_executable(test_dtc_scene unit/test_dtc_scene.cpp)
target_link_libraries(test_dtc_scene PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_include_directories(test_dtc_scene PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/support)
target_compile_definitions(test_dtc_scene
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
# The OptiX agreement case exercises the CUDA/OptiX driver, whose internal allocations trip LSan the
# same way the smoke test does — reuse the same suppressions (harmless for the CPU-only cases).
gtest_discover_tests(test_dtc_scene PROPERTIES
  ENVIRONMENT "ASAN_OPTIONS=protect_shadow_gap=0:detect_leaks=0")

add_executable(test_capture_serialize unit/test_capture_serialize.cpp)
target_link_libraries(test_capture_serialize PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_capture_serialize)

add_executable(test_planning_types unit/test_planning_types.cpp)
target_link_libraries(test_planning_types PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_planning_types
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_planning_types)

add_executable(test_planner unit/test_planner.cpp)
target_link_libraries(test_planner PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_planner)

add_executable(test_parameterization unit/test_parameterization.cpp)
target_link_libraries(test_parameterization PRIVATE quevedomp::quevedomp GTest::gtest_main)
target_compile_definitions(test_parameterization
  PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(test_parameterization)

add_executable(test_smoother unit/test_smoother.cpp)
target_link_libraries(test_smoother PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_smoother)

add_executable(test_refiner unit/test_refiner.cpp)
target_link_libraries(test_refiner PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_refiner)

add_executable(test_collision_viz unit/test_collision_viz.cpp)
target_link_libraries(test_collision_viz PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_collision_viz)

# OptiX backend tests — only when the OptiX build is on (dev-optix preset). CUDA/OptiX trip
# ASan/LSan the same way the smoke test does, so use the same suppressions.
if(QUEVEDOMP_WITH_OPTIX)
  add_executable(test_optix_backend unit/test_optix_backend.cpp)
  target_link_libraries(test_optix_backend PRIVATE quevedomp::quevedomp GTest::gtest_main)
  target_compile_definitions(test_optix_backend
    PRIVATE QUEVEDOMP_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
  gtest_discover_tests(test_optix_backend PROPERTIES
    ENVIRONMENT "ASAN_OPTIONS=protect_shadow_gap=0:detect_leaks=0")
endif()

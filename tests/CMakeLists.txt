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

add_executable(test_capture_serialize unit/test_capture_serialize.cpp)
target_link_libraries(test_capture_serialize PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_capture_serialize)

add_executable(test_collision_viz unit/test_collision_viz.cpp)
target_link_libraries(test_collision_viz PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_collision_viz)

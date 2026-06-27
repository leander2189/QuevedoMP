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

# Unit/integration tests. GTest is found by the root CMakeLists (module mode).
include(GoogleTest)

add_executable(test_bootstrap unit/test_bootstrap.cpp)
target_link_libraries(test_bootstrap PRIVATE quevedomp::quevedomp GTest::gtest_main)
gtest_discover_tests(test_bootstrap)

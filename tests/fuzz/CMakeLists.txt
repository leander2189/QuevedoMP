# Fuzz targets (Task 1.9). Only configured under the `fuzz` preset (QUEVEDOMP_BUILD_FUZZERS=ON,
# clang). The instrumentation flags are applied globally in the root CMakeLists; here we just add
# the libFuzzer driver/main at link time.
add_executable(fuzz_urdf fuzz_urdf.cpp)
target_link_libraries(fuzz_urdf PRIVATE quevedomp::quevedomp)
target_link_options(fuzz_urdf PRIVATE -fsanitize=fuzzer)

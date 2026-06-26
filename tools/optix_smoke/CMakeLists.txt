# OptiX and CUDA language are already enabled by the parent CMakeLists.
find_package(CUDAToolkit REQUIRED)

add_executable(optix_smoke main.cpp)
set_target_properties(optix_smoke PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(optix_smoke PRIVATE OptiX::OptiX CUDA::cudart)
add_test(NAME optix_smoke COMMAND optix_smoke)

# Same CUDA/ASan suppression as cuda_smoke (libcuda LSan false positives,
# address-space collision with CUDA's reserved ranges).
set_tests_properties(optix_smoke PROPERTIES
  ENVIRONMENT "ASAN_OPTIONS=protect_shadow_gap=0:detect_leaks=0")

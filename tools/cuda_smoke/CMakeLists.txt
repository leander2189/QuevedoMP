# Only added when QUEVEDOMP_WITH_CUDA=ON (see root CMakeLists). CUDA language is already
# enabled by the parent at that point.
add_executable(cuda_smoke main.cu)

# "native" targets the architecture(s) of the GPU(s) present at build time (CMake >= 3.24).
set_target_properties(cuda_smoke PROPERTIES CUDA_ARCHITECTURES native)

add_test(NAME cuda_smoke COMMAND cuda_smoke)

# Under ASan (dev-gpu preset): the CUDA driver's internal allocations trip LeakSanitizer
# (well-known libcuda false positives) and ASan's shadow gap can collide with CUDA's
# address-space reservations. Neither says anything about this probe's health.
set_tests_properties(cuda_smoke PROPERTIES
  ENVIRONMENT "ASAN_OPTIONS=protect_shadow_gap=0:detect_leaks=0")

# Only added when QUEVEDOMP_WITH_CUDA=ON (see root CMakeLists). CUDA language is already
# enabled by the parent at that point.
add_executable(cuda_smoke main.cu)

# "native" targets the architecture(s) of the GPU(s) present at build time (CMake >= 3.24).
set_target_properties(cuda_smoke PROPERTIES CUDA_ARCHITECTURES native)

add_test(NAME cuda_smoke COMMAND cuda_smoke)

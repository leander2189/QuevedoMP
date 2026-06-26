// Task 0.9 smoke test: proves libnvoptix.so.1 is mounted (NVIDIA_DRIVER_CAPABILITIES
// includes "graphics") and the device supports OptiX — before committing to Phase 2b.
//
// optix_function_table_definition.h defines the g_optixFunctionTable storage for the
// stub mechanism; must appear in exactly one translation unit per executable.
#include <cstdio>
#include <cuda_runtime.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>

#define CUDA_CHECK(call)                                                           \
    do {                                                                           \
        cudaError_t _rc = (call);                                                  \
        if (_rc != cudaSuccess) {                                                   \
            std::fprintf(stderr, "CUDA error %s: %s\n", #call,                    \
                         cudaGetErrorString(_rc));                                  \
            return 1;                                                               \
        }                                                                           \
    } while (0)

#define OPTIX_CHECK(call)                                                          \
    do {                                                                           \
        OptixResult _rc = (call);                                                  \
        if (_rc != OPTIX_SUCCESS) {                                                \
            std::fprintf(stderr, "OptiX error %s: %s (%d)\n", #call,             \
                         optixGetErrorName(_rc), static_cast<int>(_rc));           \
            return 1;                                                               \
        }                                                                           \
    } while (0)

int main() {
    // Initialize CUDA runtime — creates the primary context on device 0.
    CUDA_CHECK(cudaFree(nullptr));

    // Load libnvoptix.so.1 dynamically via the stub table.
    // Fails with OPTIX_ERROR_LIBRARY_NOT_FOUND if the container's
    // NVIDIA_DRIVER_CAPABILITIES omits "graphics".
    OPTIX_CHECK(optixInit());

    // Create a device context (nullptr = current CUDA primary context).
    // Proves the device has RT cores accessible to OptiX.
    OptixDeviceContext ctx = nullptr;
    OptixDeviceContextOptions opts{};
    OPTIX_CHECK(optixDeviceContextCreate(nullptr, &opts, &ctx));

    std::printf("optix_smoke OK: OptiX initialized, device context created\n");

    OPTIX_CHECK(optixDeviceContextDestroy(ctx));
    return 0;
}

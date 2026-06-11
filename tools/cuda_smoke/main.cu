// Throwaway CUDA toolchain validator (build-plan Task 0.6).
// Its only job: prove nvcc compiled a kernel, the driver is reachable, --gpus passthrough
// works, and a kernel actually runs on the GPU. Fails loudly (non-zero exit) otherwise.
// Removable once Phase 2b lands real GPU code.
#include <cstdio>

#include <cuda_runtime.h>

__global__ void add_one(int* x) { *x += 1; }

#define CK(e)                                                                  \
  do {                                                                         \
    cudaError_t r = (e);                                                       \
    if (r != cudaSuccess) {                                                    \
      std::fprintf(stderr, "CUDA error: %s\n", cudaGetErrorString(r));         \
      return 2;                                                                \
    }                                                                          \
  } while (0)

int main() {
  int device_count = 0;
  CK(cudaGetDeviceCount(&device_count));
  if (device_count == 0) {
    std::fprintf(stderr, "no CUDA device visible (is the container run with --gpus all?)\n");
    return 1;
  }

  int* d = nullptr;
  int h = 41;
  CK(cudaMalloc(&d, sizeof(int)));
  CK(cudaMemcpy(d, &h, sizeof(int), cudaMemcpyHostToDevice));
  add_one<<<1, 1>>>(d);
  CK(cudaGetLastError());
  CK(cudaDeviceSynchronize());
  CK(cudaMemcpy(&h, d, sizeof(int), cudaMemcpyDeviceToHost));
  cudaFree(d);

  std::printf("cuda_smoke OK: result=%d (device count=%d)\n", h, device_count);
  return h == 42 ? 0 : 3;
}

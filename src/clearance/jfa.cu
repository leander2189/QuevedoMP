// clearance/jfa — the CUDA jump-flooding passes of ClearanceField::build (roadmap R3).
// Numerically equivalent to the OpenMP fallback in clearance_field.cpp: same origin-relative
// float coordinates, same 26-neighbour propagation rule, same step schedule. Any CUDA error
// (including "no device", the common no---gpus container case) returns false and the caller
// falls back to the CPU path — the field is bit-comparable either way.
#include <cstdint>
#include <vector>

#include <cuda_runtime.h>

#include <Eigen/Core>

namespace quevedomp::clearance {
namespace {

__global__ void jfa_pass(const std::int32_t *in, std::int32_t *out, const float3 *seeds, int nx,
                         int ny, int nz, float res, int step) {
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  const int z = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);
  if (x >= nx || y >= ny || z >= nz) {
    return;
  }
  const std::size_t i = (static_cast<std::size_t>(z) * ny + y) * static_cast<std::size_t>(nx) + x;
  const float px = x * res, py = y * res, pz = z * res;

  std::int32_t bid = in[i];
  float bd = 3.4e38f;
  if (bid >= 0) {
    const float3 s = seeds[bid];
    const float dx = px - s.x, dy = py - s.y, dz = pz - s.z;
    bd = dx * dx + dy * dy + dz * dz;
  }
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const int qx = x + dx * step, qy = y + dy * step, qz = z + dz * step;
        if (qx < 0 || qy < 0 || qz < 0 || qx >= nx || qy >= ny || qz >= nz) {
          continue;
        }
        const std::int32_t cid =
            in[(static_cast<std::size_t>(qz) * ny + qy) * static_cast<std::size_t>(nx) + qx];
        if (cid < 0) {
          continue;
        }
        const float3 s = seeds[cid];
        const float ddx = px - s.x, ddy = py - s.y, ddz = pz - s.z;
        const float cd = ddx * ddx + ddy * ddy + ddz * ddz;
        if (cd < bd) {
          bd = cd;
          bid = cid;
        }
      }
    }
  }
  out[i] = bid;
}

#define JFA_CHECK(call)                                                                            \
  do {                                                                                             \
    if ((call) != cudaSuccess) {                                                                   \
      cudaFree(d_a);                                                                               \
      cudaFree(d_b);                                                                               \
      cudaFree(d_seeds);                                                                           \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

} // namespace

bool jfa_gpu(const std::vector<Eigen::Vector3f> &seed_points, std::vector<std::int32_t> &ids,
             int nx, int ny, int nz, float res) {
  const std::size_t voxels = static_cast<std::size_t>(nx) * ny * nz;
  std::int32_t *d_a = nullptr, *d_b = nullptr;
  float3 *d_seeds = nullptr;
  if (cudaFree(nullptr) != cudaSuccess) { // primes the context; fails fast without a device
    return false;
  }
  JFA_CHECK(cudaMalloc(&d_a, voxels * sizeof(std::int32_t)));
  JFA_CHECK(cudaMalloc(&d_b, voxels * sizeof(std::int32_t)));
  JFA_CHECK(cudaMalloc(&d_seeds, seed_points.size() * sizeof(float3)));
  JFA_CHECK(cudaMemcpy(d_a, ids.data(), voxels * sizeof(std::int32_t), cudaMemcpyHostToDevice));
  static_assert(sizeof(Eigen::Vector3f) == sizeof(float3));
  JFA_CHECK(cudaMemcpy(d_seeds, seed_points.data(), seed_points.size() * sizeof(float3),
                       cudaMemcpyHostToDevice));

  const dim3 block(8, 8, 4);
  const dim3 grid((nx + 7) / 8, (ny + 7) / 8, (nz + 3) / 4);
  int max_dim = nx > ny ? nx : ny;
  max_dim = max_dim > nz ? max_dim : nz;
  for (int step = max_dim / 2; step >= 1; step /= 2) {
    jfa_pass<<<grid, block>>>(d_a, d_b, d_seeds, nx, ny, nz, res, step);
    JFA_CHECK(cudaGetLastError());
    std::swap(d_a, d_b);
  }
  JFA_CHECK(cudaMemcpy(ids.data(), d_a, voxels * sizeof(std::int32_t), cudaMemcpyDeviceToHost));
  JFA_CHECK(cudaDeviceSynchronize());
  cudaFree(d_a);
  cudaFree(d_b);
  cudaFree(d_seeds);
  return true;
}

} // namespace quevedomp::clearance

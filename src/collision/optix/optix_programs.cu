// collision/optix/optix_programs — OptiX device programs for the collision backend (Task 2b.1).
//
// Scaffolding stage: a raygen that writes a sentinel (proves the module/pipeline/SBT/launch
// toolchain end-to-end on the GPU) plus a no-op miss. The environment-GAS trace (raygen shoots
// each robot test ray, any-hit + terminate-on-first-hit, atomicOr into the per-config result) is
// added on top of this same pipeline as the boolean query lands.
#include <optix.h>

#include "launch_params.hpp"

using quevedomp::collision::optix_backend::LaunchParams;

extern "C" __constant__ LaunchParams params;

extern "C" __global__ void __raygen__rg() {
  const unsigned i = optixGetLaunchIndex().x;
  if (i < params.width)
    params.out[i] = 1;
}

extern "C" __global__ void __miss__ms() {}

// collision/optix/optix_programs — OptiX device programs for the collision backend (Task 2b.1).
//
// Current stage: each raygen thread traces one world-space ray against the environment GAS with
// terminate-on-first-hit and writes a boolean. Miss -> 0, closest-hit -> 1 (the first hit is the
// closest under the terminate flag). The full backend keeps these programs and only changes how
// rays are produced (per-config, per-link transforms applied on the fly) + how results reduce.
#include <optix.h>

#include "launch_params.hpp"

using quevedomp::collision::optix_backend::LaunchParams;

extern "C" __constant__ LaunchParams params;

extern "C" __global__ void __raygen__rg() {
  const unsigned i = optixGetLaunchIndex().x;
  if (i >= params.width)
    return;

  const float3 origin = make_float3(params.ray_origin[3 * i], params.ray_origin[3 * i + 1],
                                    params.ray_origin[3 * i + 2]);
  const float3 dir =
      make_float3(params.ray_dir[3 * i], params.ray_dir[3 * i + 1], params.ray_dir[3 * i + 2]);

  unsigned int hit = 0;
  optixTrace(params.handle, origin, dir, 0.0f, params.tmax, 0.0f, OptixVisibilityMask(255),
             OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, 0, 1, 0, hit);
  params.out[i] = static_cast<unsigned char>(hit);
}

extern "C" __global__ void __miss__ms() { optixSetPayload_0(0); }
extern "C" __global__ void __closesthit__ch() { optixSetPayload_0(1); }

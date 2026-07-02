// collision/optix/optix_programs — OptiX device programs for the collision backend (Task 2b.1,
// ADR-014). Batched raygen: one thread per (ray, config). Each thread fetches its link-local ray,
// applies that config's rigid transform for the ray's link, and traces the environment GAS with
// terminate-on-first-hit; a hit atomicOr's into the config's result slot. Miss -> payload 0,
// closest-hit -> 1 (the first hit is the closest under the terminate flag).
#include <optix.h>

#include "launch_params.hpp"

using quevedomp::collision::optix_backend::LaunchParams;

extern "C" __constant__ LaunchParams params;

extern "C" __global__ void __raygen__rg() {
  const uint3 idx = optixGetLaunchIndex();
  const unsigned r = idx.x; // ray
  const unsigned c = idx.y; // config
  if (r >= params.num_rays || c >= params.num_configs)
    return;

  // Broadphase cull: if this ray's link is far from the environment for this config, no ray of it
  // can hit — skip before touching the transform or tracing (see LaunchParams::link_cull).
  const int link = params.ray_link[r];
  if (params.link_cull &&
      params.link_cull[static_cast<std::size_t>(c) * params.num_links + link])
    return;

  const float3 o = make_float3(params.ray_origin[3 * r], params.ray_origin[3 * r + 1],
                               params.ray_origin[3 * r + 2]);
  const float3 d =
      make_float3(params.ray_dir[3 * r], params.ray_dir[3 * r + 1], params.ray_dir[3 * r + 2]);

  // Apply this config's transform for the ray's link (row-major 3x4). Rotation to the direction,
  // full affine to the origin. A rigid transform preserves length, so tmax stays ray_len.
  const float *T =
      params.xform + (static_cast<std::size_t>(c) * params.num_links + link) * 12;
  const float3 op = make_float3(T[0] * o.x + T[1] * o.y + T[2] * o.z + T[3],
                                T[4] * o.x + T[5] * o.y + T[6] * o.z + T[7],
                                T[8] * o.x + T[9] * o.y + T[10] * o.z + T[11]);
  const float3 dp =
      make_float3(T[0] * d.x + T[1] * d.y + T[2] * d.z, T[4] * d.x + T[5] * d.y + T[6] * d.z,
                  T[8] * d.x + T[9] * d.y + T[10] * d.z);

  unsigned int hit = 0;
  optixTrace(params.handle, op, dp, 1e-4f, params.ray_len[r], 0.0f, OptixVisibilityMask(255),
             OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, 0, 1, 0, hit);
  if (hit)
    atomicOr(&params.out[c], 1u);
}

extern "C" __global__ void __miss__ms() { optixSetPayload_0(0); }
extern "C" __global__ void __closesthit__ch() { optixSetPayload_0(1); }

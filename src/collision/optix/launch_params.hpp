// collision/optix/launch_params — the LaunchParams struct shared between the OptiX device programs
// (optix_programs.cu) and the host launcher (optix_pipeline.cpp). Must stay a plain POD usable from
// both nvcc device code and host C++ (Task 2b.1, ADR-014 batched raygen).
//
// Model: robot test rays are stored ONCE in link-local frame (SoA, flattened across links; each
// ray tags the link/transform it belongs to). Per query_batch the host uploads one block of
// per-(config,link) rigid transforms. The launch is 2D — (ray, config) — and each thread transforms
// its ray by its link's transform for that config, traces the environment GAS
// (terminate-on-first-hit), and atomicOr's a hit into that config's result slot.
#pragma once

#include <cstdint>

#include <optix_types.h> // OptixTraversableHandle

namespace quevedomp::collision::optix_backend {

struct LaunchParams {
  OptixTraversableHandle handle; // environment GAS

  // Robot test rays in link-local frame (flattened across all ray-bearing links).
  const float *ray_origin; // [3*num_rays]
  const float *ray_dir;    // [3*num_rays] (unit)
  const float *ray_len;    // [num_rays]  segment length = tmax
  const int *ray_link;     // [num_rays]  transform-block index for this ray's link
  unsigned num_rays;
  unsigned num_links; // transforms per config

  // Per-(config,link) rigid transforms, row-major 3x4 (12 floats): index (c*num_links + link)*12.
  const float *xform;
  unsigned num_configs;

  unsigned *out; // [num_configs]; atomicOr(1) on any hit

  // Broadphase cull mask [num_configs * num_links], row-major (config, link): 1 => this link's
  // world AABB does not overlap the environment for this config, so skip its rays entirely (they
  // cannot hit). Null => no culling. A conservative cull that never changes the boolean result.
  const unsigned char *link_cull;

  // ACM-allowed robot-link × environment-object pairs (Task 3.3d P4). `tri_object` maps each
  // environment GAS primitive to its object index (static, uploaded at build); `env_allowed` is a
  // [num_links * num_objects] mask, row-major (link slot, object): 1 => hits of this pair are
  // ignored (anyhit calls optixIgnoreIntersection). Null env_allowed => no filtering, and raygen
  // traces with anyhit DISABLED (the pre-P4 fast path).
  const unsigned *tri_object;
  const unsigned char *env_allowed;
  unsigned num_objects;
};

} // namespace quevedomp::collision::optix_backend

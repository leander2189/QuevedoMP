// collision/optix/launch_params — the LaunchParams struct shared between the OptiX device programs
// (optix_programs.cu) and the host launcher (optix_pipeline.cpp). Must stay a plain POD usable from
// both nvcc device code and host C++ (Task 2b.1). Current stage: trace a batch of world-space rays
// against the environment GAS, one boolean hit per ray. Robot-FK transforms + (config,link,ray)
// launch indexing + atomicOr reduction are layered on top as the full query lands.
#pragma once

#include <cstdint>

#include <optix_types.h> // OptixTraversableHandle

namespace quevedomp::collision::optix_backend {

struct LaunchParams {
  OptixTraversableHandle handle; // environment GAS
  const float *ray_origin;       // [3*width] world-space ray origins (x,y,z)
  const float *ray_dir;          // [3*width] world-space ray directions
  float tmax;                    // ray length
  std::uint8_t *out;             // [width] 1 if the ray hit the environment, else 0
  unsigned width;                // number of rays
};

} // namespace quevedomp::collision::optix_backend

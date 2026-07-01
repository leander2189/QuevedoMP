// collision/optix/launch_params — the LaunchParams struct shared between the OptiX device programs
// (optix_programs.cu) and the host launcher (optix_pipeline.cpp). Must stay a plain POD usable from
// both nvcc device code and host C++ (Task 2b.1). Scaffolding stage: the boolean-collision fields
// (GAS handle, per-config transforms, per-link ray SoA, result buffer) are added as tracing lands.
#pragma once

#include <cstdint>

namespace quevedomp::collision::optix_backend {

struct LaunchParams {
  std::uint8_t *out; // device buffer of length `width`; raygen writes 1 to each slot
  unsigned width;
};

} // namespace quevedomp::collision::optix_backend

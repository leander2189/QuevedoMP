// collision/optix/optix_pipeline — host-side OptiX context/module/pipeline/SBT setup + launch
// (Task 2b.1 scaffolding). This establishes the toolchain the batched-raygen boolean backend
// (ADR-014) is built on: load the PTX (compiled from optix_programs.cu), create the module and a
// raygen+miss pipeline, build the SBT, and launch. `optix_selftest` runs the trivial sentinel
// launch end-to-end on the GPU so the whole integration is verifiable before any tracing exists.
//
// optix_function_table_definition.h defines g_optixFunctionTable storage; it must appear in exactly
// one translation unit per binary. The library owns it here (optix_smoke is a separate executable).
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>

#include "launch_params.hpp"

namespace quevedomp::collision {
namespace {

using optix_backend::LaunchParams;

// Minimal error-propagating helpers: on failure, set `err` and return false from the caller.
#define OPTIX_TRY(call)                                                                            \
  do {                                                                                             \
    OptixResult _rc = (call);                                                                      \
    if (_rc != OPTIX_SUCCESS) {                                                                    \
      err = std::string(#call) + " failed: " + optixGetErrorName(_rc);                             \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

#define CUDA_TRY(call)                                                                             \
  do {                                                                                             \
    cudaError_t _rc = (call);                                                                      \
    if (_rc != cudaSuccess) {                                                                      \
      err = std::string(#call) + " failed: " + cudaGetErrorString(_rc);                            \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

template <typename T> struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) SbtRecord {
  char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  T data;
};
struct EmptyData {};
using RayRecord = SbtRecord<EmptyData>;

bool read_ptx(std::string &ptx, std::string &err) {
  std::ifstream f(QUEVEDOMP_OPTIX_PTX_PATH, std::ios::binary);
  if (!f) {
    err = "cannot open PTX at " QUEVEDOMP_OPTIX_PTX_PATH;
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  ptx = ss.str();
  return !ptx.empty();
}

} // namespace

// Build the OptiX pipeline and run the sentinel launch; return true iff every launch slot came back
// written. Kept self-contained (creates + tears down its own context) — it is a toolchain probe,
// not the query hot path.
bool optix_selftest(std::string &err) {
  CUDA_TRY(cudaFree(nullptr)); // create the primary context
  OPTIX_TRY(optixInit());

  OptixDeviceContext ctx = nullptr;
  OptixDeviceContextOptions ctx_opts{};
  OPTIX_TRY(optixDeviceContextCreate(nullptr, &ctx_opts, &ctx));

  std::string ptx;
  if (!read_ptx(ptx, err))
    return false;

  OptixModuleCompileOptions module_co{};
  module_co.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
  module_co.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  module_co.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  OptixPipelineCompileOptions pipeline_co{};
  pipeline_co.usesMotionBlur = 0;
  pipeline_co.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
  pipeline_co.numPayloadValues = 1;
  pipeline_co.numAttributeValues = 0;
  pipeline_co.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  pipeline_co.pipelineLaunchParamsVariableName = "params";
  pipeline_co.usesPrimitiveTypeFlags = static_cast<unsigned>(OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE);

  char log[2048];
  size_t log_size = sizeof(log);
  OptixModule module = nullptr;
  OPTIX_TRY(optixModuleCreate(ctx, &module_co, &pipeline_co, ptx.c_str(), ptx.size(), log,
                              &log_size, &module));

  OptixProgramGroupOptions pg_opts{};
  OptixProgramGroup raygen_pg = nullptr;
  OptixProgramGroup miss_pg = nullptr;

  OptixProgramGroupDesc raygen_desc{};
  raygen_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  raygen_desc.raygen.module = module;
  raygen_desc.raygen.entryFunctionName = "__raygen__rg";
  log_size = sizeof(log);
  OPTIX_TRY(optixProgramGroupCreate(ctx, &raygen_desc, 1, &pg_opts, log, &log_size, &raygen_pg));

  OptixProgramGroupDesc miss_desc{};
  miss_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
  miss_desc.miss.module = module;
  miss_desc.miss.entryFunctionName = "__miss__ms";
  log_size = sizeof(log);
  OPTIX_TRY(optixProgramGroupCreate(ctx, &miss_desc, 1, &pg_opts, log, &log_size, &miss_pg));

  OptixProgramGroup groups[] = {raygen_pg, miss_pg};
  OptixPipelineLinkOptions link_opts{};
  link_opts.maxTraceDepth = 1;
  OptixPipeline pipeline = nullptr;
  log_size = sizeof(log);
  OPTIX_TRY(
      optixPipelineCreate(ctx, &pipeline_co, &link_opts, groups, 2, log, &log_size, &pipeline));

  // Shader binding table: one raygen record + one miss record.
  RayRecord rg_rec{};
  RayRecord ms_rec{};
  OPTIX_TRY(optixSbtRecordPackHeader(raygen_pg, &rg_rec));
  OPTIX_TRY(optixSbtRecordPackHeader(miss_pg, &ms_rec));

  RayRecord *d_rg = nullptr;
  RayRecord *d_ms = nullptr;
  CUDA_TRY(cudaMalloc(reinterpret_cast<void **>(&d_rg), sizeof(RayRecord)));
  CUDA_TRY(cudaMalloc(reinterpret_cast<void **>(&d_ms), sizeof(RayRecord)));
  CUDA_TRY(cudaMemcpy(d_rg, &rg_rec, sizeof(RayRecord), cudaMemcpyHostToDevice));
  CUDA_TRY(cudaMemcpy(d_ms, &ms_rec, sizeof(RayRecord), cudaMemcpyHostToDevice));

  OptixShaderBindingTable sbt{};
  sbt.raygenRecord = reinterpret_cast<CUdeviceptr>(d_rg);
  sbt.missRecordBase = reinterpret_cast<CUdeviceptr>(d_ms);
  sbt.missRecordStrideInBytes = sizeof(RayRecord);
  sbt.missRecordCount = 1;

  // Launch: write a sentinel into every slot, read it back.
  const unsigned width = 256;
  std::uint8_t *d_out = nullptr;
  CUDA_TRY(cudaMalloc(reinterpret_cast<void **>(&d_out), width));
  CUDA_TRY(cudaMemset(d_out, 0, width));

  LaunchParams params{};
  params.out = d_out;
  params.width = width;
  LaunchParams *d_params = nullptr;
  CUDA_TRY(cudaMalloc(reinterpret_cast<void **>(&d_params), sizeof(LaunchParams)));
  CUDA_TRY(cudaMemcpy(d_params, &params, sizeof(LaunchParams), cudaMemcpyHostToDevice));

  OPTIX_TRY(optixLaunch(pipeline, nullptr, reinterpret_cast<CUdeviceptr>(d_params),
                        sizeof(LaunchParams), &sbt, width, 1, 1));
  CUDA_TRY(cudaDeviceSynchronize());

  std::vector<std::uint8_t> host(width, 0);
  CUDA_TRY(cudaMemcpy(host.data(), d_out, width, cudaMemcpyDeviceToHost));

  bool all_written = true;
  for (unsigned i = 0; i < width; ++i)
    all_written = all_written && host[i] == 1;

  cudaFree(d_out);
  cudaFree(d_params);
  cudaFree(d_rg);
  cudaFree(d_ms);
  optixPipelineDestroy(pipeline);
  optixProgramGroupDestroy(raygen_pg);
  optixProgramGroupDestroy(miss_pg);
  optixModuleDestroy(module);
  optixDeviceContextDestroy(ctx);

  if (!all_written)
    err = "sentinel launch: not all slots written";
  return all_written;
}

} // namespace quevedomp::collision

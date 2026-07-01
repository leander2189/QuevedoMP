// collision/optix/optix_pipeline — host-side OptiX context/module/pipeline/SBT + GAS + launch
// (Task 2b.1). Establishes the batched-raygen toolchain (ADR-014): build a GAS over environment
// triangles, create a raygen+miss+hitgroup pipeline, and trace a batch of world-space rays,
// terminate-on-first-hit -> one boolean per ray. `optix_selftest` runs this end-to-end against a
// known box so the whole path (GAS build + trace + SBT) is verifiable before robot FK/transforms
// and the CollisionScene wiring land on top.
//
// optix_function_table_definition.h defines g_optixFunctionTable storage; it must appear in exactly
// one translation unit per binary. The library owns it here (optix_smoke is a separate executable).
#include <cstdint>
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
using Record = SbtRecord<EmptyData>;

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

// Unit box (half-extents 0.5), centered at the origin: 8 vertices, 12 triangles.
void unit_box(std::vector<float3> &verts, std::vector<uint3> &tris) {
  for (int i = 0; i < 8; ++i)
    verts.push_back(make_float3(i & 1 ? 0.5f : -0.5f, i & 2 ? 0.5f : -0.5f, i & 4 ? 0.5f : -0.5f));
  const unsigned f[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                             {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (const auto &t : f)
    tris.push_back(make_uint3(t[0], t[1], t[2]));
}

} // namespace

// Build the OptiX pipeline + a GAS over a known box, trace four rays with known hit/miss answers,
// and check the returned booleans. Self-contained (owns its context) — a toolchain/GAS probe.
bool optix_selftest(std::string &err) {
  CUDA_TRY(cudaFree(nullptr)); // create the primary context
  OPTIX_TRY(optixInit());

  OptixDeviceContext ctx = nullptr;
  OptixDeviceContextOptions ctx_opts{};
  OPTIX_TRY(optixDeviceContextCreate(nullptr, &ctx_opts, &ctx));

  std::string ptx;
  if (!read_ptx(ptx, err))
    return false;

  // ---- module + pipeline ------------------------------------------------------------------
  OptixModuleCompileOptions module_co{};
  module_co.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
  module_co.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
  module_co.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

  OptixPipelineCompileOptions pipeline_co{};
  pipeline_co.usesMotionBlur = 0;
  pipeline_co.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
  pipeline_co.numPayloadValues = 1;
  pipeline_co.numAttributeValues = 2;
  pipeline_co.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
  pipeline_co.pipelineLaunchParamsVariableName = "params";
  pipeline_co.usesPrimitiveTypeFlags = static_cast<unsigned>(OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE);

  char log[2048];
  size_t log_size = sizeof(log);
  OptixModule module = nullptr;
  OPTIX_TRY(optixModuleCreate(ctx, &module_co, &pipeline_co, ptx.c_str(), ptx.size(), log,
                              &log_size, &module));

  OptixProgramGroupOptions pg_opts{};
  OptixProgramGroup raygen_pg = nullptr, miss_pg = nullptr, hit_pg = nullptr;

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

  OptixProgramGroupDesc hit_desc{};
  hit_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
  hit_desc.hitgroup.moduleCH = module;
  hit_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
  log_size = sizeof(log);
  OPTIX_TRY(optixProgramGroupCreate(ctx, &hit_desc, 1, &pg_opts, log, &log_size, &hit_pg));

  OptixProgramGroup groups[] = {raygen_pg, miss_pg, hit_pg};
  OptixPipelineLinkOptions link_opts{};
  link_opts.maxTraceDepth = 1;
  OptixPipeline pipeline = nullptr;
  log_size = sizeof(log);
  OPTIX_TRY(
      optixPipelineCreate(ctx, &pipeline_co, &link_opts, groups, 3, log, &log_size, &pipeline));

  // ---- environment GAS over the unit box --------------------------------------------------
  std::vector<float3> verts;
  std::vector<uint3> tris;
  unit_box(verts, tris);

  void *d_verts = nullptr, *d_tris = nullptr;
  CUDA_TRY(cudaMalloc(&d_verts, verts.size() * sizeof(float3)));
  CUDA_TRY(cudaMalloc(&d_tris, tris.size() * sizeof(uint3)));
  CUDA_TRY(
      cudaMemcpy(d_verts, verts.data(), verts.size() * sizeof(float3), cudaMemcpyHostToDevice));
  CUDA_TRY(cudaMemcpy(d_tris, tris.data(), tris.size() * sizeof(uint3), cudaMemcpyHostToDevice));

  CUdeviceptr dv = reinterpret_cast<CUdeviceptr>(d_verts);
  unsigned tri_flags[1] = {OPTIX_GEOMETRY_FLAG_NONE};
  OptixBuildInput bi{};
  bi.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
  bi.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
  bi.triangleArray.numVertices = static_cast<unsigned>(verts.size());
  bi.triangleArray.vertexBuffers = &dv;
  bi.triangleArray.vertexStrideInBytes = sizeof(float3);
  bi.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
  bi.triangleArray.numIndexTriplets = static_cast<unsigned>(tris.size());
  bi.triangleArray.indexBuffer = reinterpret_cast<CUdeviceptr>(d_tris);
  bi.triangleArray.indexStrideInBytes = sizeof(uint3);
  bi.triangleArray.flags = tri_flags;
  bi.triangleArray.numSbtRecords = 1;

  OptixAccelBuildOptions accel_opts{};
  accel_opts.buildFlags = OPTIX_BUILD_FLAG_NONE;
  accel_opts.operation = OPTIX_BUILD_OPERATION_BUILD;

  OptixAccelBufferSizes gas_sizes;
  OPTIX_TRY(optixAccelComputeMemoryUsage(ctx, &accel_opts, &bi, 1, &gas_sizes));
  void *d_temp = nullptr, *d_gas = nullptr;
  CUDA_TRY(cudaMalloc(&d_temp, gas_sizes.tempSizeInBytes));
  CUDA_TRY(cudaMalloc(&d_gas, gas_sizes.outputSizeInBytes));

  OptixTraversableHandle gas = 0;
  OPTIX_TRY(optixAccelBuild(ctx, nullptr, &accel_opts, &bi, 1,
                            reinterpret_cast<CUdeviceptr>(d_temp), gas_sizes.tempSizeInBytes,
                            reinterpret_cast<CUdeviceptr>(d_gas), gas_sizes.outputSizeInBytes, &gas,
                            nullptr, 0));
  CUDA_TRY(cudaDeviceSynchronize());
  cudaFree(d_temp);

  // ---- SBT --------------------------------------------------------------------------------
  Record rg{}, ms{}, hg{};
  OPTIX_TRY(optixSbtRecordPackHeader(raygen_pg, &rg));
  OPTIX_TRY(optixSbtRecordPackHeader(miss_pg, &ms));
  OPTIX_TRY(optixSbtRecordPackHeader(hit_pg, &hg));
  void *d_rg = nullptr, *d_ms = nullptr, *d_hg = nullptr;
  CUDA_TRY(cudaMalloc(&d_rg, sizeof(Record)));
  CUDA_TRY(cudaMalloc(&d_ms, sizeof(Record)));
  CUDA_TRY(cudaMalloc(&d_hg, sizeof(Record)));
  CUDA_TRY(cudaMemcpy(d_rg, &rg, sizeof(Record), cudaMemcpyHostToDevice));
  CUDA_TRY(cudaMemcpy(d_ms, &ms, sizeof(Record), cudaMemcpyHostToDevice));
  CUDA_TRY(cudaMemcpy(d_hg, &hg, sizeof(Record), cudaMemcpyHostToDevice));

  OptixShaderBindingTable sbt{};
  sbt.raygenRecord = reinterpret_cast<CUdeviceptr>(d_rg);
  sbt.missRecordBase = reinterpret_cast<CUdeviceptr>(d_ms);
  sbt.missRecordStrideInBytes = sizeof(Record);
  sbt.missRecordCount = 1;
  sbt.hitgroupRecordBase = reinterpret_cast<CUdeviceptr>(d_hg);
  sbt.hitgroupRecordStrideInBytes = sizeof(Record);
  sbt.hitgroupRecordCount = 1;

  // ---- rays with known answers ------------------------------------------------------------
  //   0: from -z straight through the box   -> hit
  //   1: parallel to +z but offset in x,y   -> miss
  //   2: starts past the box pointing away  -> miss
  //   3: from -x straight through the box    -> hit
  const std::vector<float> origins = {0, 0, -5, 5, 5, -5, 0, 0, 5, -5, 0, 0};
  const std::vector<float> dirs = {0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0};
  const std::vector<std::uint8_t> expected = {1, 0, 0, 1};
  const unsigned width = 4;

  void *d_o = nullptr, *d_d = nullptr, *d_out = nullptr;
  CUDA_TRY(cudaMalloc(&d_o, origins.size() * sizeof(float)));
  CUDA_TRY(cudaMalloc(&d_d, dirs.size() * sizeof(float)));
  CUDA_TRY(cudaMalloc(&d_out, width));
  CUDA_TRY(cudaMemcpy(d_o, origins.data(), origins.size() * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_TRY(cudaMemcpy(d_d, dirs.data(), dirs.size() * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_TRY(cudaMemset(d_out, 0, width));

  LaunchParams params{};
  params.handle = gas;
  params.ray_origin = static_cast<const float *>(d_o);
  params.ray_dir = static_cast<const float *>(d_d);
  params.tmax = 100.0f;
  params.out = static_cast<std::uint8_t *>(d_out);
  params.width = width;
  void *d_params = nullptr;
  CUDA_TRY(cudaMalloc(&d_params, sizeof(LaunchParams)));
  CUDA_TRY(cudaMemcpy(d_params, &params, sizeof(LaunchParams), cudaMemcpyHostToDevice));

  OPTIX_TRY(optixLaunch(pipeline, nullptr, reinterpret_cast<CUdeviceptr>(d_params),
                        sizeof(LaunchParams), &sbt, width, 1, 1));
  CUDA_TRY(cudaDeviceSynchronize());

  std::vector<std::uint8_t> host(width, 0xFF);
  CUDA_TRY(cudaMemcpy(host.data(), d_out, width, cudaMemcpyDeviceToHost));

  bool ok = host == expected;
  if (!ok) {
    std::ostringstream ss;
    ss << "trace mismatch: got [";
    for (unsigned i = 0; i < width; ++i)
      ss << int(host[i]) << (i + 1 < width ? "," : "");
    ss << "] expected [1,0,0,1]";
    err = ss.str();
  }

  cudaFree(d_o);
  cudaFree(d_d);
  cudaFree(d_out);
  cudaFree(d_params);
  cudaFree(d_rg);
  cudaFree(d_ms);
  cudaFree(d_hg);
  cudaFree(d_gas);
  cudaFree(d_verts);
  cudaFree(d_tris);
  optixPipelineDestroy(pipeline);
  optixProgramGroupDestroy(raygen_pg);
  optixProgramGroupDestroy(miss_pg);
  optixProgramGroupDestroy(hit_pg);
  optixModuleDestroy(module);
  optixDeviceContextDestroy(ctx);
  return ok;
}

} // namespace quevedomp::collision

// collision/optix/optix_scene — the OptiX CollisionScene backend (Task 2b.1, ADR-014).
//
// Robot-vs-environment runs on the GPU: the environment is one static triangle GAS; each robot
// collision-mesh triangle EDGE becomes a link-local test ray. Per query_batch the host FKs every
// config, uploads one block of per-(config,link) transforms, and issues ONE batched launch that
// transforms each ray on the fly and traces the GAS (terminate-on-first-hit), atomicOr'ing a hit
// into the config's result. Robot-vs-self runs on the CPU via an internal FCL scene (ADR-014
// item 4), OR'd into the same booleans.
//
// Containment (ADR-012): surface rays miss a link fully inside an obstacle, so a per-link parity-ray
// check (shared EnvContainment, CPU) runs alongside — analytic inside-tests for primitive solids,
// parity rays for watertight meshes.
//
// v0 scope (minimal boolean, per the build-plan decision): boolean only (opts.distance unsupported
// -> throws; exact distance stays with FCL per spec §4.5); environment GAS geometry is Box or Mesh
// (sphere/cylinder GAS tessellation is a follow-up; note containment still handles them analytically
// on the CPU); robot rays come from MESH collision links. The scene is static (build via
// make_static_scene; no post-build add/move/remove yet).
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <cuda_runtime.h>
#include <optix.h>
#include <optix_stubs.h>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"

#include "../containment.hpp"
#include "launch_params.hpp"

namespace quevedomp::collision {
namespace {

using optix_backend::LaunchParams;

void optix_check(OptixResult rc, const char *what) {
  if (rc != OPTIX_SUCCESS)
    throw std::runtime_error(std::string("OptiX: ") + what + " failed: " + optixGetErrorName(rc));
}
void cuda_check(cudaError_t rc, const char *what) {
  if (rc != cudaSuccess)
    throw std::runtime_error(std::string("CUDA: ") + what + " failed: " + cudaGetErrorString(rc));
}

template <typename T> struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) SbtRecord {
  char header[OPTIX_SBT_RECORD_HEADER_SIZE];
  T data;
};
struct EmptyData {};
using Record = SbtRecord<EmptyData>;

std::string read_ptx() {
  std::ifstream f(QUEVEDOMP_OPTIX_PTX_PATH, std::ios::binary);
  if (!f)
    throw std::runtime_error("OptiX: cannot open PTX at " QUEVEDOMP_OPTIX_PTX_PATH);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// A generic RAII device buffer. Grows geometrically and never shrinks, so a workspace reused
// across query_batch calls stops reallocating once it has seen its largest batch (the "persistent,
// geometrically-grown" buffers of the ADR-014 workspace design).
struct DeviceBuffer {
  void *ptr = nullptr;
  std::size_t bytes = 0; // capacity
  DeviceBuffer() = default;
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer &operator=(const DeviceBuffer &) = delete;
  ~DeviceBuffer() { cudaFree(ptr); }
  void alloc(std::size_t n) {
    if (n <= bytes)
      return;
    cudaFree(ptr);
    const std::size_t grown = n > bytes * 2 ? n : bytes * 2;
    cuda_check(cudaMalloc(&ptr, grown), "cudaMalloc");
    bytes = grown;
  }
  void upload(const void *src, std::size_t n) {
    alloc(n);
    cuda_check(cudaMemcpy(ptr, src, n, cudaMemcpyHostToDevice), "cudaMemcpy H2D");
  }
  template <typename T> T *as() const { return static_cast<T *>(ptr); }
};

// A generic RAII pinned (page-locked) host buffer, mirroring DeviceBuffer's grow-and-keep policy.
// Pinned staging makes the per-query H2D transforms / D2H results copies async-capable and faster
// than pageable memory (ADR-014 workspace: "pinned host staging").
struct PinnedBuffer {
  void *ptr = nullptr;
  std::size_t bytes = 0; // capacity
  PinnedBuffer() = default;
  PinnedBuffer(const PinnedBuffer &) = delete;
  PinnedBuffer &operator=(const PinnedBuffer &) = delete;
  ~PinnedBuffer() { cudaFreeHost(ptr); }
  void reserve(std::size_t n) {
    if (n <= bytes)
      return;
    cudaFreeHost(ptr);
    const std::size_t grown = n > bytes * 2 ? n : bytes * 2;
    cuda_check(cudaMallocHost(&ptr, grown), "cudaMallocHost");
    bytes = grown;
  }
  template <typename T> T *as() const { return static_cast<T *>(ptr); }
};

// ---- geometry preparation (host) ---------------------------------------------------------------

Eigen::Vector3f to_world(const Transform &pose, const Eigen::Vector3d &v) {
  return (pose * v).cast<float>();
}

// Append a posed box (half-extents) as 12 world-space triangles.
void append_box(std::vector<float3> &verts, std::vector<uint3> &idx, const Eigen::Vector3d &he,
                const Transform &pose) {
  const unsigned base = static_cast<unsigned>(verts.size());
  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d c(i & 1 ? he.x() : -he.x(), i & 2 ? he.y() : -he.y(),
                            i & 4 ? he.z() : -he.z());
    const Eigen::Vector3f w = to_world(pose, c);
    verts.push_back(make_float3(w.x(), w.y(), w.z()));
  }
  const unsigned f[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                             {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (const auto &t : f)
    idx.push_back(make_uint3(base + t[0], base + t[1], base + t[2]));
}

// Append a posed triangle mesh as world-space triangles.
void append_mesh(std::vector<float3> &verts, std::vector<uint3> &idx, const Mesh &m,
                 const Transform &pose) {
  const unsigned base = static_cast<unsigned>(verts.size());
  for (const Eigen::Vector3d &v : m.vertices) {
    const Eigen::Vector3f w = to_world(pose, v);
    verts.push_back(make_float3(w.x(), w.y(), w.z()));
  }
  for (const Eigen::Vector3i &t : m.triangles)
    idx.push_back(make_uint3(base + t.x(), base + t.y(), base + t.z()));
}

// ---- the scene ---------------------------------------------------------------------------------

class OptixWorkspace final : public Workspace {
public:
  explicit OptixWorkspace(std::unique_ptr<Workspace> fcl_ws) : fcl_ws_(std::move(fcl_ws)) {
    cuda_check(cudaStreamCreate(&stream), "cudaStreamCreate");
  }
  ~OptixWorkspace() override {
    if (stream)
      cudaStreamDestroy(stream); // every query_batch syncs, so nothing is in flight here
  }
  std::unique_ptr<Workspace> fcl_ws_;
  cudaStream_t stream = nullptr; // all this workspace's GPU work is ordered on one explicit stream
  DeviceBuffer d_xform, d_out, d_params, d_cull; // persistent, grown geometrically across calls
  PinnedBuffer h_xform, h_hits, h_cull;          // pinned staging for H2D transforms / cull / D2H
};

class OptixScene final : public CollisionScene {
public:
  OptixScene(std::shared_ptr<const RobotModel> model, const SceneDescription &env,
             const MeshSources &meshes)
      : model_(std::move(model)) {
    build_context_and_pipeline();
    build_environment_gas(env);
    build_robot_rays(meshes);
    containment_ = EnvContainment(env); // ADR-012 parity-ray containment (surface rays miss it)
    // Internal FCL scene (empty environment) for robot-vs-self collision on the CPU.
    fcl_self_ = make_static_scene(model_, SceneDescription{}, BackendHint::ForceCpuFcl, meshes);
  }

  ~OptixScene() override {
    if (pipeline_)
      optixPipelineDestroy(pipeline_);
    for (OptixProgramGroup pg : {raygen_pg_, miss_pg_, hit_pg_})
      if (pg)
        optixProgramGroupDestroy(pg);
    if (module_)
      optixModuleDestroy(module_);
    if (ctx_)
      optixDeviceContextDestroy(ctx_);
  }

  // The OptiX scene is static in v0 (built from the SceneDescription up front).
  SceneHandle add_object(std::string, const Geometry &, const Transform &) override {
    throw std::runtime_error("OptixScene: dynamic object editing not supported in v0");
  }
  void remove_object(SceneHandle) override {
    throw std::runtime_error("OptixScene: dynamic object editing not supported in v0");
  }
  void move_object(SceneHandle, const Transform &) override {
    throw std::runtime_error("OptixScene: dynamic object editing not supported in v0");
  }

  std::unique_ptr<Workspace> make_workspace() const override {
    return std::make_unique<OptixWorkspace>(fcl_self_->make_workspace());
  }

  BatchResult query_batch(const RobotInstance &robot, std::span<const JointPosition> qs,
                          const QueryOptions &opts, Workspace &ws) const override {
    if (opts.distance)
      throw std::runtime_error(
          "OptixScene: distance/witness not supported (v0 boolean-only); use the FCL backend");
    auto &ows = dynamic_cast<OptixWorkspace &>(ws);
    const RobotModel &model = robot.model();
    const std::size_t n = qs.size();

    std::vector<std::uint8_t> result(n, 0);

    // ---- GPU robot-vs-environment ----
    // Everything below is enqueued on the workspace's own stream from pinned staging and joined by a
    // single cudaStreamSynchronize — no per-op device sync, no per-call cudaMalloc/Free.
    if (num_rays_ > 0 && gas_ != 0 && n > 0) {
      // Broadphase robot-link cull (ADR-014 prototype). OPT-IN via QUEVEDOMP_OPTIX_CULL: it wins only
      // when a localized obstacle lets many links cull; on a workspace-spanning obstacle its host +
      // per-ray overhead exceeds the traces it skips, so it is off (and never computed) by default.
      static const bool cull_enabled = std::getenv("QUEVEDOMP_OPTIX_CULL") != nullptr;

      const std::size_t slots = ray_link_slots_.size();
      const std::size_t xform_floats = n * slots * 12;
      ows.h_xform.reserve(xform_floats * sizeof(float));
      float *xform = ows.h_xform.as<float>();
      unsigned char *cull = nullptr;
      if (cull_enabled) {
        ows.h_cull.reserve(n * slots); // one byte per (config, slot): 1 => cull this link's rays
        cull = ows.h_cull.as<unsigned char>();
      }
      for (std::size_t c = 0; c < n; ++c) {
        const std::vector<Transform> poses = fk_all(model, qs[c]);
        for (std::size_t s = 0; s < slots; ++s) {
          const Transform &P = poses[ray_link_slots_[s]];
          const Eigen::Matrix4d m = P.matrix();
          float *T = xform + (c * slots + s) * 12;
          for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 4; ++col)
              T[r * 4 + col] = static_cast<float>(m(r, col));

          if (!cull_enabled)
            continue;
          // Pose the link-local ray AABB (its 8 corners) into the world and test it against the
          // environment AABB. No overlap => none of this link's rays can hit => cull.
          Eigen::Vector3f wlo = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
          Eigen::Vector3f whi = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());
          for (int k = 0; k < 8; ++k) {
            const Eigen::Vector3d corner((k & 1) ? slot_hi_[s].x() : slot_lo_[s].x(),
                                         (k & 2) ? slot_hi_[s].y() : slot_lo_[s].y(),
                                         (k & 4) ? slot_hi_[s].z() : slot_lo_[s].z());
            const Eigen::Vector3f w = (P * corner).cast<float>();
            wlo = wlo.cwiseMin(w);
            whi = whi.cwiseMax(w);
          }
          const bool overlap = (wlo.array() <= env_hi_.array()).all() &&
                               (env_lo_.array() <= whi.array()).all();
          cull[c * slots + s] = overlap ? 0 : 1;
        }
      }
      ows.d_xform.alloc(xform_floats * sizeof(float));
      cuda_check(cudaMemcpyAsync(ows.d_xform.ptr, xform, xform_floats * sizeof(float),
                                 cudaMemcpyHostToDevice, ows.stream),
                 "cudaMemcpyAsync H2D xform");
      if (cull_enabled) {
        ows.d_cull.alloc(n * slots);
        cuda_check(
            cudaMemcpyAsync(ows.d_cull.ptr, cull, n * slots, cudaMemcpyHostToDevice, ows.stream),
            "cudaMemcpyAsync H2D cull");
      }
      ows.d_out.alloc(n * sizeof(unsigned));
      cuda_check(cudaMemsetAsync(ows.d_out.ptr, 0, n * sizeof(unsigned), ows.stream),
                 "cudaMemsetAsync out");

      LaunchParams p{};
      p.handle = gas_;
      p.ray_origin = d_ray_origin_.as<float>();
      p.ray_dir = d_ray_dir_.as<float>();
      p.ray_len = d_ray_len_.as<float>();
      p.ray_link = d_ray_link_.as<int>();
      p.num_rays = num_rays_;
      p.num_links = static_cast<unsigned>(slots);
      p.xform = ows.d_xform.as<float>();
      p.num_configs = static_cast<unsigned>(n);
      p.out = ows.d_out.as<unsigned>();
      p.link_cull = cull_enabled ? ows.d_cull.as<unsigned char>() : nullptr;

      // p is stack-local but stays alive until the stream sync below, so the async copy is safe.
      ows.d_params.alloc(sizeof(LaunchParams));
      cuda_check(cudaMemcpyAsync(ows.d_params.ptr, &p, sizeof(p), cudaMemcpyHostToDevice, ows.stream),
                 "cudaMemcpyAsync params");
      optix_check(optixLaunch(pipeline_, ows.stream, reinterpret_cast<CUdeviceptr>(ows.d_params.ptr),
                              sizeof(LaunchParams), &sbt_, num_rays_, static_cast<unsigned>(n), 1),
                  "optixLaunch");
      ows.h_hits.reserve(n * sizeof(unsigned));
      cuda_check(cudaMemcpyAsync(ows.h_hits.ptr, ows.d_out.ptr, n * sizeof(unsigned),
                                 cudaMemcpyDeviceToHost, ows.stream),
                 "cudaMemcpyAsync D2H");
      cuda_check(cudaStreamSynchronize(ows.stream), "cudaStreamSynchronize");

      const unsigned *hits = ows.h_hits.as<unsigned>();
      for (std::size_t c = 0; c < n; ++c)
        result[c] = hits[c] ? 1 : 0;
    }

    // ---- CPU robot-vs-self (FCL, honoring the ACM) ----
    if (opts.check_self_collision) {
      QueryOptions self_opts;
      self_opts.check_self_collision = true;
      self_opts.distance = false;
      const BatchResult self = fcl_self_->query_batch(robot, qs, self_opts, *ows.fcl_ws_);
      for (std::size_t c = 0; c < n && c < self.in_collision.size(); ++c)
        result[c] = (result[c] || self.in_collision[c]) ? 1 : 0;
    }

    // ---- CPU containment (ADR-012): a link fully inside an obstacle casts no surface ray ----
    if (containment_.any() && !link_interior_.empty()) {
      for (std::size_t c = 0; c < n; ++c) {
        if (result[c])
          continue;
        const std::vector<Transform> poses = fk_all(model, qs[c]);
        for (const auto &[li, p] : link_interior_) {
          if (containment_.inside(poses[li] * p)) {
            result[c] = 1;
            break;
          }
        }
      }
    }

    BatchResult out;
    out.in_collision = std::move(result);
    return out;
  }

private:
  void build_context_and_pipeline() {
    cuda_check(cudaFree(nullptr), "cudaFree(primary ctx)");
    optix_check(optixInit(), "optixInit");
    OptixDeviceContextOptions co{};
    optix_check(optixDeviceContextCreate(nullptr, &co, &ctx_), "optixDeviceContextCreate");

    const std::string ptx = read_ptx();
    OptixModuleCompileOptions mco{};
    mco.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
    mco.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    mco.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

    pco_.usesMotionBlur = 0;
    pco_.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
    pco_.numPayloadValues = 1;
    pco_.numAttributeValues = 2;
    pco_.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
    pco_.pipelineLaunchParamsVariableName = "params";
    pco_.usesPrimitiveTypeFlags = static_cast<unsigned>(OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE);

    char log[2048];
    size_t log_size = sizeof(log);
    optix_check(
        optixModuleCreate(ctx_, &mco, &pco_, ptx.c_str(), ptx.size(), log, &log_size, &module_),
        "optixModuleCreate");

    OptixProgramGroupOptions pgo{};
    OptixProgramGroupDesc rg{};
    rg.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    rg.raygen.module = module_;
    rg.raygen.entryFunctionName = "__raygen__rg";
    log_size = sizeof(log);
    optix_check(optixProgramGroupCreate(ctx_, &rg, 1, &pgo, log, &log_size, &raygen_pg_), "rg pg");

    OptixProgramGroupDesc ms{};
    ms.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    ms.miss.module = module_;
    ms.miss.entryFunctionName = "__miss__ms";
    log_size = sizeof(log);
    optix_check(optixProgramGroupCreate(ctx_, &ms, 1, &pgo, log, &log_size, &miss_pg_), "ms pg");

    OptixProgramGroupDesc hg{};
    hg.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    hg.hitgroup.moduleCH = module_;
    hg.hitgroup.entryFunctionNameCH = "__closesthit__ch";
    log_size = sizeof(log);
    optix_check(optixProgramGroupCreate(ctx_, &hg, 1, &pgo, log, &log_size, &hit_pg_), "hg pg");

    OptixProgramGroup groups[] = {raygen_pg_, miss_pg_, hit_pg_};
    OptixPipelineLinkOptions lo{};
    lo.maxTraceDepth = 1;
    log_size = sizeof(log);
    optix_check(optixPipelineCreate(ctx_, &pco_, &lo, groups, 3, log, &log_size, &pipeline_),
                "optixPipelineCreate");

    // SBT (one record each). Records live for the scene's lifetime.
    Record rg_rec{}, ms_rec{}, hg_rec{};
    optix_check(optixSbtRecordPackHeader(raygen_pg_, &rg_rec), "pack rg");
    optix_check(optixSbtRecordPackHeader(miss_pg_, &ms_rec), "pack ms");
    optix_check(optixSbtRecordPackHeader(hit_pg_, &hg_rec), "pack hg");
    d_rg_.upload(&rg_rec, sizeof(Record));
    d_ms_.upload(&ms_rec, sizeof(Record));
    d_hg_.upload(&hg_rec, sizeof(Record));
    sbt_.raygenRecord = reinterpret_cast<CUdeviceptr>(d_rg_.ptr);
    sbt_.missRecordBase = reinterpret_cast<CUdeviceptr>(d_ms_.ptr);
    sbt_.missRecordStrideInBytes = sizeof(Record);
    sbt_.missRecordCount = 1;
    sbt_.hitgroupRecordBase = reinterpret_cast<CUdeviceptr>(d_hg_.ptr);
    sbt_.hitgroupRecordStrideInBytes = sizeof(Record);
    sbt_.hitgroupRecordCount = 1;
  }

  void build_environment_gas(const SceneDescription &env) {
    std::vector<float3> verts;
    std::vector<uint3> tris;
    for (const SceneObject &o : env.objects) {
      if (const auto *b = std::get_if<BoxShape>(&o.geometry))
        append_box(verts, tris, b->half_extents, o.pose);
      else if (const auto *m = std::get_if<Mesh>(&o.geometry))
        append_mesh(verts, tris, *m, o.pose);
      else
        throw std::runtime_error(
            "OptixScene: environment sphere/cylinder not tessellated in v0 (use a box or mesh)");
    }
    if (tris.empty())
      return; // empty environment: gas_ stays 0, robot-vs-env skipped

    // World-space AABB of the whole environment — the broadphase-cull test target (per config, a
    // robot link whose world AABB misses this box casts no ray that can hit).
    env_lo_ = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    env_hi_ = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());
    for (const float3 &v : verts) {
      env_lo_ = env_lo_.cwiseMin(Eigen::Vector3f(v.x, v.y, v.z));
      env_hi_ = env_hi_.cwiseMax(Eigen::Vector3f(v.x, v.y, v.z));
    }

    d_env_verts_.upload(verts.data(), verts.size() * sizeof(float3));
    d_env_tris_.upload(tris.data(), tris.size() * sizeof(uint3));
    CUdeviceptr dv = reinterpret_cast<CUdeviceptr>(d_env_verts_.ptr);
    unsigned flags[1] = {OPTIX_GEOMETRY_FLAG_NONE};
    OptixBuildInput bi{};
    bi.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    bi.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    bi.triangleArray.numVertices = static_cast<unsigned>(verts.size());
    bi.triangleArray.vertexBuffers = &dv;
    bi.triangleArray.vertexStrideInBytes = sizeof(float3);
    bi.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    bi.triangleArray.numIndexTriplets = static_cast<unsigned>(tris.size());
    bi.triangleArray.indexBuffer = reinterpret_cast<CUdeviceptr>(d_env_tris_.ptr);
    bi.triangleArray.indexStrideInBytes = sizeof(uint3);
    bi.triangleArray.flags = flags;
    bi.triangleArray.numSbtRecords = 1;

    // The environment is static and traced by millions of rays, so build for FAST TRACE (not the
    // default fast-build) and COMPACT the result: a higher-quality, smaller BVH — the single biggest
    // lever on big-mesh traversal speed. Build time is irrelevant here (one-time, at scene setup).
    OptixAccelBuildOptions ao{};
    ao.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE | OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
    ao.operation = OPTIX_BUILD_OPERATION_BUILD;
    OptixAccelBufferSizes sizes;
    optix_check(optixAccelComputeMemoryUsage(ctx_, &ao, &bi, 1, &sizes), "accel mem usage");
    DeviceBuffer temp, uncompacted;
    temp.alloc(sizes.tempSizeInBytes);
    uncompacted.alloc(sizes.outputSizeInBytes);

    // Emit the compacted size from the build, then compact the BVH into the persistent d_gas_.
    DeviceBuffer d_compacted_size;
    d_compacted_size.alloc(sizeof(std::uint64_t));
    OptixAccelEmitDesc emit{};
    emit.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emit.result = reinterpret_cast<CUdeviceptr>(d_compacted_size.ptr);

    OptixTraversableHandle built = 0;
    optix_check(optixAccelBuild(ctx_, nullptr, &ao, &bi, 1, reinterpret_cast<CUdeviceptr>(temp.ptr),
                                sizes.tempSizeInBytes,
                                reinterpret_cast<CUdeviceptr>(uncompacted.ptr),
                                sizes.outputSizeInBytes, &built, &emit, 1),
                "optixAccelBuild");
    cuda_check(cudaDeviceSynchronize(), "sync after accel build");

    std::uint64_t compacted_size = 0;
    cuda_check(cudaMemcpy(&compacted_size, d_compacted_size.ptr, sizeof(compacted_size),
                          cudaMemcpyDeviceToHost),
               "read compacted size");
    d_gas_.alloc(compacted_size);
    optix_check(optixAccelCompact(ctx_, nullptr, built, reinterpret_cast<CUdeviceptr>(d_gas_.ptr),
                                  compacted_size, &gas_),
                "optixAccelCompact");
    cuda_check(cudaDeviceSynchronize(), "sync after accel compact");
  }

  void build_robot_rays(const MeshSources &meshes) {
    std::vector<float> origin, dir, len;
    std::vector<int> link;
    const auto &links = model_->links();
    for (int li = 0; li < static_cast<int>(links.size()); ++li) {
      std::size_t before = len.size();
      // Link-local AABB over this link's ray endpoints (for the per-config broadphase cull).
      Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d hi = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
      for (const CollisionGeometry &cg : links[li].collisions) {
        if (cg.type != GeometryType::Mesh)
          continue; // OptiX robot geometry is mesh-based (documented)
        Mesh m =
            load_mesh(resolve_mesh_uri(cg.mesh_filename, meshes.package_dirs, meshes.base_dir));
        // Link-frame vertices: apply URDF <mesh scale> then the collision origin.
        std::vector<Eigen::Vector3d> lv;
        lv.reserve(m.vertices.size());
        for (const Eigen::Vector3d &v : m.vertices)
          lv.push_back(cg.origin * v.cwiseProduct(cg.mesh_scale));
        link_interior_.push_back({li, mesh_centroid(lv)}); // ADR-012 interior point (link frame)
        const int slot = static_cast<int>(ray_link_slots_.size());
        std::map<std::pair<int, int>, char> seen; // dedup shared edges within this mesh
        auto add_edge = [&](int a, int b) {
          auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
          if (!seen.emplace(key, 1).second)
            return;
          const Eigen::Vector3d d = lv[b] - lv[a];
          const double l = d.norm();
          if (l < 1e-9)
            return;
          const Eigen::Vector3d u = d / l;
          origin.insert(origin.end(), {float(lv[a].x()), float(lv[a].y()), float(lv[a].z())});
          dir.insert(dir.end(), {float(u.x()), float(u.y()), float(u.z())});
          len.push_back(static_cast<float>(l));
          link.push_back(slot);
          lo = lo.cwiseMin(lv[a]).cwiseMin(lv[b]);
          hi = hi.cwiseMax(lv[a]).cwiseMax(lv[b]);
        };
        for (const Eigen::Vector3i &t : m.triangles) {
          add_edge(t.x(), t.y());
          add_edge(t.y(), t.z());
          add_edge(t.z(), t.x());
        }
        // slot is only "used" once we know this link produced rays; guard below.
        (void)slot;
      }
      if (len.size() > before) {
        ray_link_slots_.push_back(li); // this link contributed rays -> it gets a transform slot
        slot_lo_.push_back(lo.cast<float>());
        slot_hi_.push_back(hi.cast<float>());
      }
    }
    num_rays_ = static_cast<unsigned>(len.size());
    if (num_rays_ == 0)
      return;
    d_ray_origin_.upload(origin.data(), origin.size() * sizeof(float));
    d_ray_dir_.upload(dir.data(), dir.size() * sizeof(float));
    d_ray_len_.upload(len.data(), len.size() * sizeof(float));
    d_ray_link_.upload(link.data(), link.size() * sizeof(int));
  }

  std::shared_ptr<const RobotModel> model_;
  std::unique_ptr<CollisionScene> fcl_self_;

  OptixDeviceContext ctx_ = nullptr;
  OptixModule module_ = nullptr;
  OptixPipelineCompileOptions pco_{};
  OptixProgramGroup raygen_pg_ = nullptr, miss_pg_ = nullptr, hit_pg_ = nullptr;
  OptixPipeline pipeline_ = nullptr;
  OptixShaderBindingTable sbt_{};
  DeviceBuffer d_rg_, d_ms_, d_hg_;

  OptixTraversableHandle gas_ = 0;
  DeviceBuffer d_gas_, d_env_verts_, d_env_tris_;

  DeviceBuffer d_ray_origin_, d_ray_dir_, d_ray_len_, d_ray_link_;
  unsigned num_rays_ = 0;
  std::vector<int> ray_link_slots_; // slot -> model link index

  // Broadphase cull: per-slot link-local ray AABB + the environment world AABB (ADR-014 robot-link
  // cull — skip a link's rays when its posed AABB misses the environment).
  std::vector<Eigen::Vector3f> slot_lo_, slot_hi_;
  Eigen::Vector3f env_lo_ = Eigen::Vector3f::Zero(), env_hi_ = Eigen::Vector3f::Zero();

  EnvContainment containment_;
  std::vector<std::pair<int, Eigen::Vector3d>>
      link_interior_; // (link index, interior pt, link frame)
};

} // namespace

std::unique_ptr<CollisionScene> make_optix_scene(std::shared_ptr<const RobotModel> robot,
                                                 const SceneDescription &environment,
                                                 const MeshSources &meshes) {
  return std::make_unique<OptixScene>(std::move(robot), environment, meshes);
}

} // namespace quevedomp::collision

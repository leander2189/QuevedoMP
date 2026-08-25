# QuevedoMP Vulkan Design — a deployable GPU collision backend via `VK_KHR_ray_query`

> **Goal in one line:** ship QuevedoMP as a self-contained binary by replacing the login-gated
> OptiX SDK with apt-installable Vulkan. **CUDA stays** — it was never the problem.

> **Audience: the implementing agent.** This document is self-contained: it records the design
> decisions, the verified code map (file:line as of commit `0ca2c48`; re-verify lines before
> editing — they drift), the finished architecture, and a commit-by-commit execution plan with
> gates. Follow it top to bottom. Where this document says THROW / FAIL LOUDLY, that is the
> project invariant "no silent fallbacks" — do not soften it.
>
> **Status: NOT RATIFIED.** This is a design on the shelf, written ahead of the decision to build
> it. Do not start Phase A without Leandro's go-ahead. Records as **ADR-023** when Phase B lands
> (outline in §10) — ADR-022 is claimed by R7 (attached objects).

---

## 0. Context and goal

The GPU collision backend is [`OptixScene`](../src/collision/optix/optix_scene.cpp). The goal of
this work is **deployability: shipping QuevedoMP as a self-contained binary.** Running on AMD and
Intel GPUs is a welcome side effect, not the driver.

### Why OptiX specifically, and why CUDA is fine

CUDA and OptiX have very different deployment stories, and only one of them is a problem:

- **CUDA deploys fine.** `cudart` links statically (`CUDA::cudart_static`), and
  `CUDA_ARCHITECTURES 75` bakes SASS into the binary. A CUDA build is one binary plus the host
  NVIDIA driver — nothing to acquire, nothing to install, nothing to find at runtime.
- **OptiX does not.** The SDK is a login-gated installer behind an NVIDIA developer account and
  EULA. It cannot be apt-installed, cannot be committed, cannot go into a public CI image, and its
  headers carry redistribution restrictions. The evidence is in this repo:
  [`.devcontainer/Dockerfile:49-78`](../.devcontainer/Dockerfile#L49-L78) is twenty lines of
  comment explaining CRLF-versus-gzip surgery on a self-extracting archive that must be
  hand-copied into the build context.
- **Vulkan's build-time deps are `libvulkan-dev` and `glslang-tools`** — two apt packages, no
  account, no EULA, no installer. At runtime it needs `libvulkan.so.1`, a stable vendor-neutral
  loader ABI that every GPU driver installs.

**Scope note that is easy to get wrong:** the OptiX win is at *build* time. `libnvoptix.so.1` ships
with the NVIDIA driver, so OptiX runs fine on bare metal at runtime. And inside a container,
Vulkan needs `NVIDIA_DRIVER_CAPABILITIES` to include `graphics` to get the ICD mounted — exactly
like OptiX does. "Vulkan enables a slimmer container" is **false**; do not put it in the ADR.

### The blocker this exposes, which applies to OptiX today

[`CMakeLists.txt:174`](../CMakeLists.txt#L174) sets
`QUEVEDOMP_OPTIX_PTX_PATH="$<TARGET_OBJECTS:quevedomp_optix_ptx>"` — an absolute **build-tree**
path, read at runtime by [`optix_scene.cpp:73-80`](../src/collision/optix/optix_scene.cpp#L73-L80).
A relocated binary looks for its PTX in a directory that does not exist on the target machine. The
OptiX backend is therefore *already* undeployable, independent of the SDK question. This is why D6
below embeds the SPIR-V rather than mirroring the PTX pattern, and why fixing the PTX side is
folded into Phase A.

`VK_KHR_ray_query` + `VK_KHR_acceleration_structure` (ratified 2020, Vulkan 1.2+) expose the same
hardware BVH traversal that OptiX does, through the Khronos API. Support: NVIDIA Turing+, AMD
RDNA2+, Intel Arc, recent mobile. The backend uses **inline ray tracing from a compute shader**
(`rayQueryEXT`), not the ray-tracing *pipeline* — see D2.

### Why this port is unusually cheap

`OptixScene` uses a tiny slice of OptiX. Read
[`optix_programs.cu`](../src/collision/optix/optix_programs.cu) — 72 lines: one GAS, one raygen,
terminate-on-first-hit, an anyhit that filters ACM pairs, one `atomicOr`. There is no shading, no
recursion, no meaningful SBT. Every one of those has an exact Vulkan equivalent, and ray query
*deletes* the module/program-group/SBT machinery entirely (~150 of the ~270 setup lines in
`optix_scene.cpp`).

The expensive parts of `OptixScene` — the host FK pass, the edge-ray extraction, the broadphase
cull, containment, the CPU/GPU overlap — are **backend-agnostic host code** and get reused
verbatim. That is the single most important fact in this document: see D3.

### CUDA stays — deliberately

`ClearanceField`'s jump-flooding passes are raw CUDA kernels
([`src/clearance/jfa.cu`](../src/clearance/jfa.cu)), wired at
[`CMakeLists.txt:133-139`](../CMakeLists.txt#L133-L139) under `QUEVEDOMP_WITH_CUDA`, independent
of `QUEVEDOMP_WITH_OPTIX`. **This work does not touch them, and porting them is not a goal.** CUDA
deploys cleanly (above); it is not the thing standing between this project and a shippable binary.

The consequence is that the target deployment configuration is
**`WITH_CUDA=ON, WITH_OPTIX=OFF, WITH_VULKAN=ON`**: a binary with statically-linked cudart, embedded
SASS, embedded SPIR-V, and no acquired SDK — needing only the host GPU driver. That build still
requires an NVIDIA GPU for `ClearanceField` (it degrades to the CPU path elsewhere), so the claim
this work earns is **"deployable as a binary,"** *not* "runs on any vendor." Write it that way in
the ADR and the README; the AMD/Intel collision path is real but partial while `jfa.cu` is CUDA.

### Ratified project invariants that bind this work

Batch-first collision (one `query_batch` per logical validation); determinism per seed
(bit-identical across thread counts); one `Workspace` per querying thread (ADR-005); apt-only
deps; ADR discipline; no silent fallbacks; clang-format LLVM/100col/2sp; each commit leaves the
tree green.

### Canonical build/test commands (from Windows, via WSL)

```bash
wsl -d Ubuntu-24.04
# Existing GPU suite (OptiX):
cmake --preset dev-optix && cmake --build --preset dev-optix && ctest --preset dev-optix
# New (this work):
cmake --preset dev-vulkan && cmake --build --preset dev-vulkan && ctest --preset dev-vulkan
# Three-way differential (FCL vs OptiX vs Vulkan) — needs an NVIDIA GPU:
cmake --preset dev-gpu-all && ctest --preset dev-gpu-all -R Differential
```

---

## 1. Design decisions (ratify before coding; do not relitigate after)

| # | Decision | Rationale |
|---|----------|-----------|
| **D1** | **Add a third backend; do not replace OptiX.** `VulkanScene` is a peer of `OptixScene` and `FclScene` behind `CollisionScene`, gated by `QUEVEDOMP_WITH_VULKAN` (default OFF). OptiX stays. | `CollisionScene` is already the seam ([collision_scene.hpp:30-51](../include/quevedomp/collision/collision_scene.hpp#L30-L51)). Keeping OptiX gives a *three-way* differential test — the strongest correctness evidence available — and a measured A/B on identical hardware. Deleting OptiX is a separate, later decision that needs Vulkan numbers first. |
| **D2** | **Ray query (inline RT) in a compute shader.** NOT `VK_KHR_ray_tracing_pipeline`. | The workload has no shading. A pipeline would recreate the SBT/program-group complexity for zero benefit and add SBT indirection per ray. Ray query is also supported on strictly more devices than the RT pipeline. |
| **D3** | **Extract the backend-agnostic host code into a shared `GpuRayScene` base before writing any Vulkan.** Ray extraction, FK/transform fill, the cull, chunking policy, containment, and the CPU-self-collision overlap live there; `OptixScene` and `VulkanScene` become thin device layers. | Roughly 300 of `optix_scene.cpp`'s 735 lines are pure CPU logic with zero CUDA in them. Copy-pasting them into a second backend guarantees the two drift and the differential test degrades into "two copies of the same bug." **This refactor is Phase A and lands green on its own, with OptiX still the only GPU backend.** |
| **D4** | **One process-wide refcounted `VulkanContext`** (instance, physical device, logical device, queues, function-pointer table, shader module). Scenes hold a `shared_ptr` to it. | Mirrors the OptiX device context. Vulkan device creation is expensive (~100 ms) and queue counts are a scarce per-device resource; per-scene devices would exhaust them. |
| **D5** | **Hand-rolled `vkGetDeviceProcAddr` table for the ~9 extension entry points.** No volk, no VMA. | Apt-only-deps invariant. `libvulkan-dev` + `glslang-tools` are apt packages; volk and VMA are not. The table is ~30 lines and the allocator we need is a bump allocator over ~8 long-lived buffers. Revisit VMA only if allocation ever shows up in a profile (it will not — buffers are grow-and-keep, exactly as today). |
| **D6** | **SPIR-V compiled at build time by `glslangValidator` and EMBEDDED in the binary** as a `constexpr std::uint32_t[]`. No runtime file, no path define. **Phase A additionally fixes the OptiX PTX to embed the same way.** | Deployability is the entire point of this work (§0). A runtime path define is what makes the OptiX backend undeployable today — `QUEVEDOMP_OPTIX_PTX_PATH` points into the build tree ([CMakeLists.txt:174](../CMakeLists.txt#L174)). Mirroring a broken pattern for consistency would ship the bug twice. Generate the header with a CMake `file(READ ... HEX)` custom command; SPIR-V is a `uint32` stream, so assert `size % 4 == 0` and mind endianness (SPIR-V is little-endian; every target we support is too — static_assert it rather than assume). |
| **D7** | **The ACM filter branches on a specialization constant**, not a runtime `if`. Two `VkPipeline` objects are created from the same module: `pipeline_opaque_` (geometry opaque, single `rayQueryProceedEXT`) and `pipeline_filtered_` (`gl_RayFlagsNoOpaqueEXT`, candidate loop). | Preserves the "hardware skips the shader stage entirely" fast path that [optix_programs.cu:49-52](../src/collision/optix/optix_programs.cu#L49-L52) documents. A runtime branch would force every ray through the non-opaque candidate path. Two pipelines from one module cost nothing at build and are selected per launch by the same `env_allowed == nullptr` test the OptiX raygen uses. |
| **D8** | **`BackendHint` gains `ForceVulkan`; `Auto` prefers OptiX when both are built.** But the *intended trajectory* is that OptiX is deleted once §9's numbers are in — see D8b. | `Auto`'s job is to be fast and never surprise. Until Vulkan has recorded numbers on the DTC fixture, OptiX keeps the default on NVIDIA. On a build without OptiX, `Auto` uses Vulkan under the same eligibility rules ([fcl_scene.cpp:741-748](../src/collision/fcl_scene.cpp#L741-L748)). |
| **D8b** | **OptiX is transitional, not permanent.** It is kept through this work for the three-way differential (D1) and the A/B, and the shipping preset (`dev-vulkan`, §C2) already excludes it. Deleting it is a follow-up ADR gated on §9. | Under the §0 goal, two GPU backends is not the end state — OptiX is precisely the dependency being removed. Keeping it *forever* would mean the SDK gate stays in the build instructions and CI, which is the thing this work exists to delete. Say this out loud so a later agent does not treat the two-backend state as settled architecture. |
| **D9** | **Boolean-only, static scene — identical v0 scope to OptiX.** `opts.distance` THROWS. `add_object`/`move_object`/`remove_object` THROW. | Matching scope is what makes the differential test meaningful. Widening scope and porting at the same time is how you get a backend that disagrees and nobody knows why. |
| **D10** | **Minimum target Vulkan 1.2.** Device selection REQUIRES `rayQuery` + `accelerationStructure` + `bufferDeviceAddress`; a device lacking any of them is skipped, and if no device qualifies, construction THROWS with the enumerated device names and which feature each lacked. | No silent fallbacks. A user who asks for `ForceVulkan` on a GTX 1060 must get a message naming the missing feature, not a CPU-speed surprise. |

---

## 2. Verified code map (spot-checked at `0ca2c48`; re-verify line numbers before editing)

| What | Where | Notes for the port |
|---|---|---|
| Backend-agnostic interface | [collision_scene.hpp:23-51](../include/quevedomp/collision/collision_scene.hpp#L23-L51) | `Workspace` is opaque; `query_batch` is the primitive. Unchanged by this work except `BackendHint` (D8). |
| CPU vocabulary invariant | [types.hpp:1-4](../include/quevedomp/collision/types.hpp#L1-L4) | *"NO CUDA type ever appears in any collision/ header."* Extend to Vulkan: **no `Vk*` type in any header under `include/`.** |
| Backend dispatch + hybrid | [fcl_scene.cpp:625-750](../src/collision/fcl_scene.cpp#L625-L750) | `make_optix_scene` fwd-decl, `HybridScene`, `optix_available()`, `make_static_scene`. Add the Vulkan twins here in the same shape. |
| `HybridScene` routing | [fcl_scene.cpp:646-694](../src/collision/fcl_scene.cpp#L646-L694) | Backend-agnostic already — it holds `unique_ptr<CollisionScene>`. Rename its `optix_` member to `gpu_` in Phase A; no logic change. |
| GPU scene, whole | [optix_scene.cpp](../src/collision/optix/optix_scene.cpp) | 735 lines. The split for D3 is tabulated in §3. |
| Device programs | [optix_programs.cu](../src/collision/optix/optix_programs.cu) | 72 lines; the GLSL twin is in §5. |
| Launch params POD | [launch_params.hpp](../src/collision/optix/launch_params.hpp) | Becomes push constants + buffer device addresses (§5.1). |
| Ray extraction (reusable) | [optix_scene.cpp:619-694](../src/collision/optix/optix_scene.cpp#L619-L694) | `build_robot_rays` — zero CUDA except the four `upload` calls at the tail. Split there. |
| Env triangle flattening (reusable) | [optix_scene.cpp:533-564](../src/collision/optix/optix_scene.cpp#L533-L564) | `build_environment_gas` head — uses `float3`/`uint3` (CUDA vector types) purely as PODs. Phase A retypes to plain structs. |
| Host FK + cull + containment fill | [optix_scene.cpp:275-339](../src/collision/optix/optix_scene.cpp#L275-L339) | The OpenMP loop. 100% reusable, verbatim. |
| CPU/GPU overlap | [optix_scene.cpp:410-428](../src/collision/optix/optix_scene.cpp#L410-L428) | Self-collision runs between launch and sync. Vulkan equivalent is fence-deferred (§6.4). |
| Containment merge | [optix_scene.cpp:432-451](../src/collision/optix/optix_scene.cpp#L432-L451) | 100% reusable, verbatim. |
| Chunking (device limit) | [optix_scene.cpp:387-405](../src/collision/optix/optix_scene.cpp#L387-L405) | OptiX caps at 2^30 threads; Vulkan caps on `maxComputeWorkGroupCount`. Same loop, different bound (§6.3). |
| Grow-and-keep buffers | [optix_scene.cpp:85-126](../src/collision/optix/optix_scene.cpp#L85-L126) | `DeviceBuffer`/`PinnedBuffer` — the policy is the design; the Vulkan twins copy it exactly (§6.1). |
| Find module pattern | [cmake/FindOptiX.cmake](../cmake/FindOptiX.cmake) | Model for nothing — Vulkan has an upstream `FindVulkan`. Use `find_package(Vulkan REQUIRED COMPONENTS glslangValidator)`. |
| Test pattern | [test_optix_backend.cpp](../tests/unit/test_optix_backend.cpp) | Self-test + FCL agreement. The Vulkan suite mirrors it 1:1 (§8). |
| Container GPU caps | [.devcontainer/Dockerfile:16](../.devcontainer/Dockerfile#L16) | Already `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility`. `graphics` is what mounts the Vulkan ICD — **no change needed.** |

---

## 3. Phase A — extract `GpuRayScene` (no Vulkan yet)

**Gate: this phase changes no behavior. `ctest --preset dev-optix` must pass identically before
and after, and `bench_dtc` must be within noise.**

New files:

- `src/collision/gpu/gpu_ray_scene.hpp` / `.cpp` — the shared host layer.
- `src/collision/gpu/device_rays.hpp` — the POD types crossing to the device layer.

### A1. POD types (replace CUDA vector types)

`float3`/`uint3` in `optix_scene.cpp` are used only as PODs. Define in `device_rays.hpp`:

```cpp
struct Vec3f { float x, y, z; };
struct Tri   { unsigned a, b, c; };
static_assert(sizeof(Vec3f) == 12 && sizeof(Tri) == 12);
```

Both are layout-compatible with `float3`/`uint3` **and** with
`VK_FORMAT_R32G32B32_SFLOAT` / `VK_INDEX_TYPE_UINT32`, so no repacking on either side.

### A2. `RobotRaySet` — the output of ray extraction

Move [optix_scene.cpp:619-689](../src/collision/optix/optix_scene.cpp#L619-L689) (everything
before the four `upload` calls) into a free function
`RobotRaySet extract_robot_rays(const RobotModel&, const MeshSources&)` returning:

```cpp
struct RobotRaySet {
  std::vector<float> origin, dir, len;          // SoA, [3N]/[3N]/[N]
  std::vector<int> link;                        // [N] transform-slot index
  std::vector<int> slot_to_link;                // slot -> model link index
  std::unordered_map<int,int> link_to_slot;
  std::vector<Eigen::Vector3f> slot_lo, slot_hi; // link-local ray AABB, per slot
  std::vector<std::pair<int,Eigen::Vector3d>> link_interior; // ADR-012 interior points
};
```

### A3. `EnvTriangles` — the output of environment flattening

Move [optix_scene.cpp:533-561](../src/collision/optix/optix_scene.cpp#L533-L561) into
`EnvTriangles flatten_environment(const SceneDescription&)`:

```cpp
struct EnvTriangles {
  std::vector<Vec3f> verts;
  std::vector<Tri> tris;
  std::vector<unsigned> tri_object;   // per primitive -> object index (ACM)
  std::vector<std::string> object_ids;
  Eigen::Vector3f lo, hi;             // world AABB, for the broadphase cull
};
```

### A4. `GpuRayScene` — the shared `CollisionScene` implementation

```cpp
// src/collision/gpu/gpu_ray_scene.hpp
class GpuRayScene : public CollisionScene {
public:
  // Everything the device layer must provide. One call per query_batch, plus setup.
  struct Device {
    virtual ~Device() = default;
    virtual std::unique_ptr<Workspace> make_workspace() const = 0;
    // Enqueue the trace ASYNCHRONOUSLY and return immediately: the caller runs CPU
    // self-collision before calling join(). MUST NOT block.
    virtual void launch(Workspace &ws, const LaunchDesc &d) const = 0;
    // Block until the launch enqueued above completes; fill `hits` (one byte per config).
    virtual void join(Workspace &ws, std::span<std::uint8_t> hits) const = 0;
    virtual const char *name() const = 0;
  };

  GpuRayScene(std::shared_ptr<const RobotModel>, const SceneDescription &,
              const MeshSources &, std::unique_ptr<Device>);
  // add/remove/move_object THROW (D9). query_batch is FINAL — see below.
  BatchResult query_batch(...) const final;
  ...
};
```

`LaunchDesc` carries the per-batch host-side inputs: `xform` pointer + config/slot counts, the
optional `cull` mask, the optional `env_allowed` mask. **`query_batch` is `final`** — it is the
whole point of the refactor that neither backend can diverge in it.

The body of `GpuRayScene::query_batch` is [optix_scene.cpp:220-456](../src/collision/optix/optix_scene.cpp#L220-L456)
with three edits:
1. the H2D/launch/D2H block ([341-411](../src/collision/optix/optix_scene.cpp#L341-L411)) becomes `device_->launch(ws, desc)`;
2. the sync + hit merge ([423-428](../src/collision/optix/optix_scene.cpp#L423-L428)) becomes `device_->join(ws, hits)`;
3. the workspace is `GpuWorkspace { std::unique_ptr<Workspace> fcl_ws, device_ws; }`.

Note that `h_xform` staging currently lives in `OptixWorkspace` because it must be *pinned*.
Under `GpuRayScene` the FK pass writes into a device-owned staging pointer: add
`virtual float *xform_staging(Workspace&, size_t floats) const` and
`virtual unsigned char *cull_staging(Workspace&, size_t bytes) const` to `Device`. OptiX returns
its pinned buffer; Vulkan returns its mapped host-visible buffer. **Do not** substitute a plain
`std::vector` — that silently drops OptiX's pinned-memory async copies and would show up as a
benchmark regression in this phase's gate.

### A5. `OptixDevice`

`optix_scene.cpp` collapses to: context/pipeline/SBT setup, GAS build from `EnvTriangles`, the
four ray-buffer uploads from `RobotRaySet`, `launch()`, `join()`, and the staging accessors.
Expect ~400 lines, down from 735.

### A6. Phase-A commits

1. `refactor(collision): POD ray/triangle types + extract_robot_rays + flatten_environment` — pure move, OptiX still calls them inline.
2. `refactor(collision): GpuRayScene host layer; OptixScene becomes GpuRayScene::Device` — the real move.
3. `refactor(collision): rename HybridScene::optix_ -> gpu_` — cosmetic, unblocks D8.
4. `fix(collision): embed OptiX PTX in the binary instead of reading a build-tree path` — see below.

**Commit 4 (the PTX fix).** Independent of Vulkan and worth landing whether or not Phase B is ever
approved. Replace the `QUEVEDOMP_OPTIX_PTX_PATH` define
([CMakeLists.txt:172-174](../CMakeLists.txt#L172-L174)) and `read_ptx()`
([optix_scene.cpp:73-80](../src/collision/optix/optix_scene.cpp#L73-L80)) with a generated header
holding the PTX as a `constexpr char[]`. PTX is ASCII text, so this is a `file(READ ... HEX)`
custom command plus a NUL terminator — simpler than the SPIR-V case, and it establishes the
CMake helper that D6 reuses. Write the helper as a reusable function
(`quevedomp_embed_binary(<target> <infile> <symbol>)`) in `cmake/`, since Phase C calls it again.

Gate after each: `ctest --preset dev-optix` green, `bench_dtc` within 3% of the recorded number
in [docs/benchmarks/PROTOCOL.md](benchmarks/PROTOCOL.md). Gate for commit 4 specifically: copy the
built test binary to a scratch directory, delete the build tree, and confirm it still runs — the
regression test for the bug being fixed.

---

## 4. Phase B — `VulkanContext`

New: `src/collision/vulkan/vulkan_context.hpp` / `.cpp`.

### B1. Instance

`VK_API_VERSION_1_2`. No surface, no swapchain, no window — this is headless compute.
Enable `VK_LAYER_KHRONOS_validation` **only** when `QUEVEDOMP_VULKAN_VALIDATION=1` is set in the
environment, and route its debug messenger to the same throw-on-error discipline used elsewhere:
`VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT` logs to `stderr` with the full message. Validation
must be **off by default** — it costs 5-10x on dispatch-heavy workloads and would poison any
benchmark run by an unaware agent.

### B2. Physical-device selection (D10)

Enumerate; for each device chain and query:

```cpp
VkPhysicalDeviceRayQueryFeaturesKHR              rq{ .sType = ...RAY_QUERY_FEATURES_KHR };
VkPhysicalDeviceAccelerationStructureFeaturesKHR as{ .sType = ...ACCELERATION_STRUCTURE_FEATURES_KHR, .pNext = &rq };
VkPhysicalDeviceVulkan12Features                 v12{ .sType = ...VULKAN_1_2_FEATURES, .pNext = &as };
VkPhysicalDeviceFeatures2                        f2{ .sType = ...FEATURES_2, .pNext = &v12 };
vkGetPhysicalDeviceFeatures2(dev, &f2);
```

Require **all** of: `rq.rayQuery`, `as.accelerationStructure`, `v12.bufferDeviceAddress`, and the
device extensions `VK_KHR_acceleration_structure`, `VK_KHR_ray_query`,
`VK_KHR_deferred_host_operations`. Prefer `VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU`; break ties by
device index. Honor `QUEVEDOMP_VULKAN_DEVICE=<index>` as an override (mirrors how CUDA users
expect `CUDA_VISIBLE_DEVICES` to work).

On zero qualifying devices, THROW listing every enumerated device name and the first requirement
each failed. This message is the entire UX of a failed `ForceVulkan`; write it carefully.

### B3. Logical device + queues

One compute-capable queue family (prefer a family with `COMPUTE` and without `GRAPHICS` — the
async-compute family, less contended). Request
`min(family.queueCount, QUEVEDOMP_VULKAN_QUEUES or 4)` queues.

**Correctness note that will bite you:** `VkQueue` submission is *externally synchronized*. Two
threads calling `vkQueueSubmit` on the same queue is undefined behavior. Workspaces round-robin
over the queue pool at creation time and each queue carries its own `std::mutex`, held only across
the `vkQueueSubmit` call itself (never across the fence wait — that would serialize the very
overlap this design exists for).

This is the one place Vulkan is genuinely stricter than CUDA: `cudaStreamCreate` is unbounded,
`VkQueue` count is fixed at device creation. A 64-thread planner gets 4 queues and 64 command
buffers; that is fine, because the GPU work per query is short and the mutex is held for
microseconds.

### B4. Function-pointer table (D5)

Load via `vkGetDeviceProcAddr` after device creation:

```
vkCreateAccelerationStructureKHR                 vkDestroyAccelerationStructureKHR
vkGetAccelerationStructureBuildSizesKHR          vkCmdBuildAccelerationStructuresKHR
vkGetAccelerationStructureDeviceAddressKHR       vkCmdCopyAccelerationStructureKHR
vkCmdWriteAccelerationStructuresPropertiesKHR    vkGetBufferDeviceAddress
```

Any null pointer after loading is a THROW naming the function. (Do not call these through the
loader's static exports — that is not spec-guaranteed for device-level extension commands.)

### B5. Shader module

From the embedded SPIR-V array (D6), one `VkShaderModule`, two `VkPipeline`s via
specialization constant `constant_id = 0` (`FILTER_ACM`, `bool`) per D7. One
`VkDescriptorSetLayout` — see §5.1. One `VkPipelineLayout` with an 88-byte push-constant range.

---

## 5. Phase B — the shader

New: `src/collision/vulkan/collision.comp`. This is the direct twin of
[optix_programs.cu](../src/collision/optix/optix_programs.cu); keep the comments in sync with it.

### 5.1 Binding model

All large arrays go through **buffer device addresses** (`GL_EXT_buffer_reference2`), not
descriptors — that makes the shader's parameter block a near-copy of `LaunchParams` and lets the
chunk loop offset pointers exactly as [optix_scene.cpp:393-396](../src/collision/optix/optix_scene.cpp#L393-L396)
does, with no descriptor rewrites per chunk. The only descriptor is the TLAS (acceleration
structures cannot be addressed by pointer).

- Descriptor set 0, binding 0: `accelerationStructureEXT tlas` (`VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR`). Bound once per workspace, never rewritten.
- Push constants (**88 bytes**: 9 × 8-byte `VkDeviceAddress` + 4 × 4-byte `uint`; every device
  guarantees ≥128, so this fits everywhere with room to spare).

### 5.2 The shader

```glsl
#version 460
#extension GL_EXT_ray_query              : require
#extension GL_EXT_buffer_reference2      : require
#extension GL_EXT_scalar_block_layout    : require
#extension GL_EXT_shader_explicit_arithmetic_types_int64 : require

// FILTER_ACM mirrors OptiX's `params.env_allowed != nullptr` branch: false selects the fast
// opaque path (the hardware never surfaces a candidate), true selects the candidate loop that
// re-implements __anyhit__ah. Two pipelines, one module (design D7).
layout(constant_id = 0) const bool FILTER_ACM = false;

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout(buffer_reference, scalar) readonly buffer Floats  { float v[]; };
layout(buffer_reference, scalar) readonly buffer Ints    { int   v[]; };
layout(buffer_reference, scalar) readonly buffer Uints   { uint  v[]; };
layout(buffer_reference, scalar) readonly buffer Bytes   { uint8_t v[]; };
layout(buffer_reference, scalar) buffer          OutBuf  { uint  v[]; };

layout(set = 0, binding = 0) uniform accelerationStructureEXT tlas;

layout(push_constant, scalar) uniform Params {
  Floats ray_origin;   // [3*num_rays]
  Floats ray_dir;      // [3*num_rays] unit
  Floats ray_len;      // [num_rays]
  Ints   ray_link;     // [num_rays] transform-slot index
  Floats xform;        // [num_configs*num_links*12] row-major 3x4, chunk-offset by the host
  OutBuf outb;         // [num_configs] chunk-offset by the host
  Bytes  link_cull;    // [num_configs*num_links] or 0
  Bytes  env_allowed;  // [num_links*num_objects] or 0
  Uints  tri_object;   // [num_env_tris] or 0
  uint   num_rays, num_links, num_configs, num_objects;
} p;

void main() {
  const uint r = gl_GlobalInvocationID.x;  // ray
  const uint c = gl_GlobalInvocationID.y;  // config
  if (r >= p.num_rays || c >= p.num_configs) return;

  // Early abort, identical to the OptiX raygen: out[c] only ever goes 0 -> 1, so this
  // non-atomic read races benignly with the atomicOr below — a stale 0 merely traces a ray
  // another invocation already decided.
  if (p.outb.v[c] != 0u) return;

  const int link = p.ray_link.v[r];
  if (uint64_t(p.link_cull) != 0 && p.link_cull.v[c * p.num_links + uint(link)] != uint8_t(0))
    return;

  const vec3 o = vec3(p.ray_origin.v[3*r], p.ray_origin.v[3*r+1], p.ray_origin.v[3*r+2]);
  const vec3 d = vec3(p.ray_dir.v[3*r],    p.ray_dir.v[3*r+1],    p.ray_dir.v[3*r+2]);

  // Row-major 3x4: rotation to the direction, full affine to the origin. Rigid transforms
  // preserve length, so tmax stays the stored ray length.
  const uint b = (c * p.num_links + uint(link)) * 12u;
  const vec4 T0 = vec4(p.xform.v[b+0],  p.xform.v[b+1],  p.xform.v[b+2],  p.xform.v[b+3]);
  const vec4 T1 = vec4(p.xform.v[b+4],  p.xform.v[b+5],  p.xform.v[b+6],  p.xform.v[b+7]);
  const vec4 T2 = vec4(p.xform.v[b+8],  p.xform.v[b+9],  p.xform.v[b+10], p.xform.v[b+11]);
  const vec3 op = vec3(dot(T0.xyz,o), dot(T1.xyz,o), dot(T2.xyz,o)) + vec3(T0.w,T1.w,T2.w);
  const vec3 dp = vec3(dot(T0.xyz,d), dot(T1.xyz,d), dot(T2.xyz,d));

  const uint flags = gl_RayFlagsTerminateOnFirstHitEXT |
                     (FILTER_ACM ? gl_RayFlagsNoOpaqueEXT : gl_RayFlagsOpaqueEXT);

  rayQueryEXT rq;
  rayQueryInitializeEXT(rq, tlas, flags, 0xFFu, op, 1e-4, dp, p.ray_len.v[r]);

  if (FILTER_ACM) {
    // The __anyhit__ah twin. NoOpaque surfaces every triangle as a candidate; confirm only
    // the pairs the ACM does NOT allow. Confirming under TerminateOnFirstHit ends the query.
    while (rayQueryProceedEXT(rq)) {
      const uint obj = p.tri_object.v[rayQueryGetIntersectionPrimitiveIndexEXT(rq, false)];
      if (p.env_allowed.v[uint(link) * p.num_objects + obj] == uint8_t(0))
        rayQueryConfirmIntersectionEXT(rq);
    }
  } else {
    rayQueryProceedEXT(rq);   // opaque + terminate-on-first-hit: commits or exhausts, once
  }

  if (rayQueryGetIntersectionTypeEXT(rq, true) != gl_RayQueryCommittedIntersectionNoneEXT)
    atomicOr(p.outb.v[c], 1u);
}
```

**Three semantic notes the implementing agent must not get wrong:**

1. `rayQueryGetIntersectionPrimitiveIndexEXT(rq, false)` is the *candidate* (uncommitted) index —
   the `false` is load-bearing. `tri_object` is indexed by the flattened primitive index across
   the single BLAS geometry, matching how [optix_scene.cpp:548](../src/collision/optix/optix_scene.cpp#L548)
   fills it. With one geometry in one BLAS these indices coincide; **if the BLAS is ever split
   into multiple geometries, this index becomes geometry-local** and needs
   `rayQueryGetIntersectionGeometryIndexEXT` plus a per-geometry base offset. Keep it one geometry.
2. The final read uses `true` (committed). Reading the candidate there is a classic ray-query bug
   and would report hits for rays that only *approached* a triangle.
3. `atomicOr` on a buffer-reference member requires the buffer to be non-readonly; `OutBuf` above
   is deliberately not `readonly`.

### 5.3 Compilation

```
glslangValidator -V --target-env vulkan1.2 -o collision.comp.spv collision.comp
```

Then embed the `.spv` via the `quevedomp_embed_binary` helper from Phase A commit 4 (D6). Wire
both steps as CMake custom commands with `DEPENDS` on the `.comp` so edits rebuild.
`glslangValidator` comes from `find_package(Vulkan REQUIRED COMPONENTS glslangValidator)` →
`Vulkan::glslangValidator`.

Note `-Os` is available in recent glslang but do **not** use it here: SPIR-V size is irrelevant
(a few KB embedded) and the driver's compiler does the real optimization. `-g` likewise off, so the
embedded blob is byte-stable across rebuilds.

---

## 6. Phase B — `VulkanDevice`

New: `src/collision/vulkan/vulkan_device.hpp` / `.cpp`, implementing `GpuRayScene::Device`.

### 6.1 Buffers — the `DeviceBuffer`/`PinnedBuffer` twins

Copy the grow-geometrically-never-shrink policy from
[optix_scene.cpp:85-126](../src/collision/optix/optix_scene.cpp#L85-L126) exactly; it is the
reason a warmed workspace allocates nothing.

```cpp
struct VkBuf {                       // RAII: VkBuffer + VkDeviceMemory + optional mapped ptr
  VkBuffer buf = VK_NULL_HANDLE; VkDeviceMemory mem = VK_NULL_HANDLE;
  void *mapped = nullptr; VkDeviceAddress addr = 0; VkDeviceSize bytes = 0;
  void alloc(const VulkanContext&, VkDeviceSize n, VkBufferUsageFlags, VkMemoryPropertyFlags);
};
```

Every buffer that the shader dereferences needs
`VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT` and allocation with
`VkMemoryAllocateFlagsInfo{ .flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT }` — omitting either
is the single most common `VK_ERROR_DEVICE_LOST` in this design. Get `addr` from
`vkGetBufferDeviceAddress` once at allocation.

**Staging strategy (host→device transforms).** Two tiers, chosen once at context creation:

- **ReBAR/integrated tier:** if a memory type with
  `DEVICE_LOCAL | HOST_VISIBLE | HOST_COHERENT` and a heap ≥256 MB exists, allocate the transform
  buffer there. The FK pass writes straight into GPU-visible memory — **no copy at all**, which
  is strictly better than the current pinned-staging + `cudaMemcpyAsync`. Modern AMD exposes this
  for the whole VRAM heap; NVIDIA exposes a 256 MB BAR window on older drivers and the full heap
  on newer ones.
- **Fallback tier:** `HOST_VISIBLE|HOST_COHERENT` staging + `vkCmdCopyBuffer` into a
  `DEVICE_LOCAL` buffer. This is the exact analogue of today's pinned staging.

`xform_staging()` returns the mapped pointer of whichever tier is active, so the shared FK pass in
`GpuRayScene` is unaware of the difference.

**Readback:** `DEVICE_LOCAL` out buffer, `vkCmdCopyBuffer` to a `HOST_VISIBLE|HOST_CACHED`
readback buffer. If that memory type is not `HOST_COHERENT`, `vkInvalidateMappedMemoryRanges`
before reading — do not skip this because it happens to work on NVIDIA.

### 6.2 Acceleration structure build

Once, at scene construction, from `EnvTriangles` (§A3). Mirrors
[optix_scene.cpp:563-617](../src/collision/optix/optix_scene.cpp#L563-L617).

**BLAS**, one geometry:
- `VkAccelerationStructureGeometryTrianglesDataKHR`: `vertexFormat = VK_FORMAT_R32G32B32_SFLOAT`,
  `vertexStride = 12`, `indexType = VK_INDEX_TYPE_UINT32`, device addresses of the vertex/index
  buffers (usage must include
  `VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR`).
- `geometry.flags`: **`VK_GEOMETRY_OPAQUE_BIT_KHR`**. The filtered path overrides per-ray with
  `gl_RayFlagsNoOpaqueEXT` (D7), so one BLAS serves both pipelines.
- `buildFlags = PREFER_FAST_TRACE_BIT_KHR | ALLOW_COMPACTION_BIT_KHR` — the same reasoning as
  [optix_scene.cpp:580-583](../src/collision/optix/optix_scene.cpp#L580-L583): built once, traced
  by millions of rays.
- Scratch buffer alignment is `VkPhysicalDeviceAccelerationStructurePropertiesKHR::minAccelerationStructureScratchOffsetAlignment`
  — query it, do not assume 256.

**Compaction** (the analogue of `optixAccelCompact`):
1. `vkCmdBuildAccelerationStructuresKHR`
2. barrier `ACCELERATION_STRUCTURE_WRITE_BIT_KHR` → `ACCELERATION_STRUCTURE_READ_BIT_KHR`
3. `vkCmdWriteAccelerationStructuresPropertiesKHR` with a `VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR` pool
4. submit, wait, `vkGetQueryPoolResults`
5. create the compacted AS at that size, `vkCmdCopyAccelerationStructureKHR` with
   `VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR`, submit, wait, destroy the uncompacted one.

**TLAS**, one instance, identity transform, `instanceCustomIndex = 0`, `mask = 0xFF`,
`accelerationStructureReference` = the compacted BLAS device address. Ray query *requires* a
TLAS; there is no BLAS-direct trace. Note that `VkAccelerationStructureInstanceKHR::transform` is
a row-major 3x4 float matrix — the identical layout to the per-link transforms, should a future
version ever want robot-link instancing (the ADR-014 fallback).

Empty environment ⇒ no BLAS, no TLAS, `has_tlas_ = false`, and `GpuRayScene` skips the GPU path
exactly as it does today when `gas_ == 0`.

### 6.3 `launch()` — record and submit, do not wait

Per call, on the workspace's command buffer (reset, not reallocated):

1. If on the fallback staging tier: `vkCmdCopyBuffer` transforms (and cull) staging → device.
2. `vkCmdFillBuffer(out, 0, n*4, 0)` — the `cudaMemsetAsync` twin.
3. Barrier: `TRANSFER_WRITE` → `SHADER_READ|SHADER_WRITE`, stage `TRANSFER` → `COMPUTE_SHADER`.
4. Bind `pipeline_filtered_` if `env_allowed != nullptr` else `pipeline_opaque_` (D7); bind the
   TLAS descriptor set.
5. **Chunk loop** — the twin of [optix_scene.cpp:387-405](../src/collision/optix/optix_scene.cpp#L387-L405).
   Bound is `maxComputeWorkGroupCount[1]` (spec minimum **65535**, so assume it):
   `max_configs = min(num_configs, maxComputeWorkGroupCount[1])`. Per chunk, push constants with
   `xform`, `outb`, `link_cull` addresses offset by the chunk base — byte offsets on
   `VkDeviceAddress`, exactly as the OptiX version offsets its pointers — then
   `vkCmdDispatch(ceil(num_rays/64), chunk_configs, 1)`.
   Also assert `ceil(num_rays/64) <= maxComputeWorkGroupCount[0]`; at 64 rays/group that allows
   ~4.2M rays, well past any real robot, but THROW rather than dispatch a truncated grid.
   Chunks need no barrier between them: they write disjoint `out` slices.
6. Barrier `SHADER_WRITE` → `TRANSFER_READ`; `vkCmdCopyBuffer` out → readback.
7. `vkEndCommandBuffer`; `vkQueueSubmit` with the workspace's fence, under the queue mutex (§B3).

**Return immediately.** No fence wait here — that is what lets the caller's CPU self-collision
pass overlap the trace.

### 6.4 `join()` — the deferred sync

```cpp
vkWaitForFences(dev, 1, &ws.fence, VK_TRUE, UINT64_MAX);   // the cudaStreamSynchronize twin
vkResetFences(dev, 1, &ws.fence);
// invalidate if the readback memory is not HOST_COHERENT, then copy out
```

Called by `GpuRayScene::query_batch` *after* the FCL self-collision pass, preserving the overlap
documented at [optix_scene.cpp:410-428](../src/collision/optix/optix_scene.cpp#L410-L428).

Use a plain `VkFence`, not a timeline semaphore: the wait is a single join point per batch, and
every `query_batch` fully synchronizes before returning — which is exactly what makes the
grow-and-keep buffer reuse safe call-to-call.

### 6.5 `VulkanWorkspace`

Owns: queue index (round-robin), `VkCommandPool` (`RESET_COMMAND_BUFFER_BIT`), one
`VkCommandBuffer`, one `VkFence`, one `VkDescriptorPool` + set (TLAS), and the grow-and-keep
buffers: `xform` (+ staging), `cull` (+ staging), `out`, `readback`, `env_allowed`.

Destruction is safe without a device wait *only because* every `query_batch` joins its fence.
Keep that invariant; if a future async API breaks it, the destructor needs `vkQueueWaitIdle`.

---

## 7. Phase C — wiring

### C1. CMake

```cmake
option(QUEVEDOMP_WITH_VULKAN "Enable Vulkan ray-query collision backend" OFF)

if(QUEVEDOMP_WITH_VULKAN)
  find_package(Vulkan REQUIRED COMPONENTS glslangValidator)
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/collision.comp.spv
    COMMAND Vulkan::glslangValidator -V --target-env vulkan1.2
            -o ${CMAKE_CURRENT_BINARY_DIR}/collision.comp.spv
            ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/vulkan/collision.comp
    DEPENDS src/collision/vulkan/collision.comp VERBATIM)
  # D6: embed, never read at runtime. quevedomp_embed_binary() lands in Phase A commit 4.
  quevedomp_embed_binary(quevedomp
    ${CMAKE_CURRENT_BINARY_DIR}/collision.comp.spv quevedomp_collision_spv)
  target_sources(quevedomp PRIVATE src/collision/vulkan/vulkan_context.cpp
                                   src/collision/vulkan/vulkan_device.cpp)
  target_link_libraries(quevedomp PRIVATE Vulkan::Vulkan)
  target_compile_definitions(quevedomp PRIVATE QUEVEDOMP_WITH_VULKAN)
endif()
```

`QUEVEDOMP_WITH_VULKAN` must be **independent of** `QUEVEDOMP_WITH_CUDA` in both directions:
`WITH_CUDA=ON, WITH_VULKAN=ON` is the shipping configuration (§0), and `WITH_CUDA=OFF,
WITH_VULKAN=ON` must also configure, build, and pass — that combination is what proves no CUDA
symbol leaked into the Vulkan path. Both are presets below.

**Link `CUDA::cudart_static`, not `CUDA::cudart`** ([CMakeLists.txt:138](../CMakeLists.txt#L138)).
The shipping binary should carry the CUDA runtime, not hunt for `libcudart.so` on the target. This
is a one-word change with no downside here (single translation unit set, no plugin boundary) and it
is half of what makes §0's "one binary plus the host driver" true. Verify with `ldd`: the only
NVIDIA entry left should be the driver's `libcuda.so.1`.

Apt deps for the devcontainer: `libvulkan-dev glslang-tools` (plus
`vulkan-validationlayers` and `vulkan-tools` for `vulkaninfo`, dev-only). `NVIDIA_DRIVER_CAPABILITIES`
already includes `graphics` at [.devcontainer/Dockerfile:16](../.devcontainer/Dockerfile#L16), which
is what mounts the Vulkan ICD into the container — verify with `vulkaninfo --summary` in the
Phase-B smoke test.

### C2. Presets

- `dev-vulkan` — `WITH_CUDA=ON, WITH_OPTIX=OFF, WITH_VULKAN=ON`. **The shipping configuration**
  (§0): full functionality, no acquired SDK. This is the preset the README's build instructions
  should point at once Phase C lands, and the one CI must build without any login-gated artifact.
- `dev-vulkan-nocuda` — `WITH_CUDA=OFF, WITH_OPTIX=OFF, WITH_VULKAN=ON`. Proves no CUDA symbol
  leaked into the Vulkan path; also the AMD/Intel configuration (collision only, no
  `ClearanceField` GPU path).
- `dev-gpu-all` — `WITH_CUDA=ON, WITH_OPTIX=ON, WITH_VULKAN=ON`. Three-way differential; NVIDIA
  only; requires the OptiX SDK. This is the *only* preset that will still need the login-gated
  installer, and it disappears when D8b retires OptiX.
- `bench-vulkan` — Release, sanitizers OFF, mirroring `bench-optix`.

### C3. Dispatch (`fcl_scene.cpp`)

Add, in the same shape as the OptiX block at
[fcl_scene.cpp:625-750](../src/collision/fcl_scene.cpp#L625-L750):

```cpp
bool vulkan_available() noexcept;                       // header: collision_scene.hpp
enum class BackendHint { Auto, ForceCpuFcl, ForceOptix, ForceVulkan };   // D8
```

`ForceVulkan` on a build without it THROWS (mirroring the OptiX message). `Auto` tries OptiX
first, then Vulkan, then FCL-only — each in a `try` that falls through on failure, since `Auto`
must never throw where FCL alone would work.

### C4. Bindings + studio

`bindings/python/src/bind_collision.cpp`: add the `ForceVulkan` enumerator and
`vulkan_available()`. The studio's backend selector reads the same pair, so it picks this up with
no studio change beyond adding the option to whatever list enumerates backends — grep for
`ForceOptix` in `tools/quevedomp-studio` and mirror every hit.

---

## 8. Tests

Mirror [test_optix_backend.cpp](../tests/unit/test_optix_backend.cpp) case for case in
`tests/unit/test_vulkan_backend.cpp`, gated on `QUEVEDOMP_WITH_VULKAN`:

1. `VulkanBackend.DeviceToolchainSelfTest` — `vulkan_selftest(std::string&)`: enumerate, select,
   build a two-triangle BLAS/TLAS, dispatch one ray that hits and one that misses. Fails with the
   full device-selection diagnostic (§B2), which is what an agent debugging a container will read.
2. `VulkanBackend.BuildsPrimitiveRobotEmptyEnvironment` — degenerate: no rays, no TLAS, queries free.
3. `VulkanBackend.AgreesWithFclOnMeshRobot` — the UR5 + surface-crossing cube case, config for config.
4. `VulkanBackend.AgreesWithFclOnContainment` — a link fully inside a closed box (the ADR-012
   false-free trap). This exercises shared host code, but it must be asserted per backend.
5. `VulkanBackend.AgreesWithFclUnderAcm` — the D7 filtered pipeline. **The highest-risk test in
   this suite**: it is the only one exercising the candidate loop, and the `false`/`true`
   candidate/committed distinction (§5.2 note 1-2) fails silently-plausibly if written wrong.
   Assert both directions: an ACM-allowed pair reports free, and the *same* geometry with the ACM
   entry removed reports collision.
6. `VulkanBackend.ChunkingMatchesSingleDispatch` — force `max_configs` low via a test-only env
   var (`QUEVEDOMP_VULKAN_MAX_CONFIGS`) and assert an N=200k batch equals the unchunked answer.
   Without this, the chunk-offset arithmetic in §6.3 is untested until a user hits 65536 configs.
7. **`deploy_smoke.sh` — the acceptance test for §0's actual goal, and the one that must not be
   skipped.** Under `dev-vulkan`: build, copy *only* the binary (and the shared lib, if not a
   static build) to a scratch directory, **delete the entire build tree**, then run it there and
   assert a real query answers correctly. Also assert on `ldd` output: no `libnvoptix`, no
   `libcudart` (D6 + `cudart_static`), only the driver's `libcuda.so.1` and `libvulkan.so.1`.
   Every other test in this suite can pass while the binary remains undeployable — this is the
   only one that measures the thing the work is for. Wire it as a ctest fixture so CI runs it.
8. `GpuDifferential.OptixEqualsVulkan` (preset `dev-gpu-all` only) — the same randomized
   configuration sweep through both GPU backends and FCL; all three must agree bit-for-bit on the
   boolean array. This is the payoff of D1 and the reason OptiX is not deleted.

Also extend the sanitizer exclusions the way OptiX already is at
[tests/CMakeLists.txt:108](../tests/CMakeLists.txt#L108) and
[:149-156](../tests/CMakeLists.txt#L149-L156) — Vulkan drivers leak by design at process exit and
will trip LSan.

Python: extend `bindings/python/tests/test_collision.py` to parametrize its backend-agreement test
over whichever backends `vulkan_available()`/`optix_available()` report.

---

## 9. Numbers to record (no claim without a measurement)

Run under `bench-vulkan` / `bench-optix` on the same machine, DTC cell fixture, per
[docs/benchmarks/PROTOCOL.md](benchmarks/PROTOCOL.md). Record in the ADR:

| Metric | OptiX baseline | Vulkan |
|---|---|---|
| batch 10 000, cull on, self-collision off | ~23 ms ([optix-collision.md:182-184](optix-collision.md#L182-L184)) | ? |
| batch 1 / 10 (latency floor) | ~0.1-0.2 ms | ? |
| scene build (BLAS+TLAS+compaction) | ? | ? |
| compacted AS bytes | ? | ? |
| crossover vs FCL (configs) | ~256 | ? |

**Expected outcome, to be confirmed not assumed:** parity or slightly better on NVIDIA. Ray query
skips SBT indirection, and on a ReBAR device the transform upload copy disappears entirely (§6.1).
Note also that the full DTC query is currently bound by the *CPU* FCL self-collision pass with the
GPU fully hidden behind it — so a Vulkan trace even somewhat slower than OptiX may not move the
end-to-end number at all. Report both the trace-only and the end-to-end figures; reporting only
one of them is how a regression hides.

Also record, since these are the deliverable (§0) and not decoration:

| Deployment metric | Before (`dev-optix`) | After (`dev-vulkan`) |
|---|---|---|
| Login-gated artifacts needed to build | 1 (OptiX SDK installer) | **0** |
| Runtime files beside the binary | 1 (`.ptx`, at a build-tree path — broken) | **0** |
| Non-driver NVIDIA `.so` in `ldd` | `libcudart` | **none** |
| Binary runs after `rm -rf build/` | **no** | yes |

If Vulkan matches or beats OptiX on NVIDIA **and** the three-way differential is green, that is
the evidence needed to open the D8b "delete the OptiX backend" decision. Do not pre-empt it here —
but note that under §0's goal the bar for deleting OptiX is *not* "Vulkan is faster," it is
"Vulkan is not enough slower to matter," because the SDK gate has a standing cost that a few
percent of trace time does not.

---

## 10. ADR-023 outline (`docs/architecture/adr-023-vulkan-ray-query-backend.md`)

- **Context** — QuevedoMP cannot ship as a binary: the OptiX SDK is login-gated and EULA-bound at
  build time, and the PTX is loaded from a build-tree path at runtime. CUDA is *not* part of the
  problem (statically linkable, SASS embedded). `VK_KHR_ray_query` is the ratified equivalent of
  the small slice of OptiX this project uses, and its build-time deps are apt packages.
- **Decision** — D1-D10 above, with the measured numbers and the deployment table from §9.
- **Consequences** — three backends to keep green *transitionally* (D8b: OptiX is scheduled for
  removal, not adoption); a `GpuRayScene` layer both GPU devices share, so a semantic change lands
  in one place; the shipping preset becomes `dev-vulkan` (`WITH_CUDA=ON, WITH_OPTIX=OFF`), which
  still needs an NVIDIA GPU for `ClearanceField` — the claim earned is "deployable as a binary,"
  not "vendor-neutral"; Vulkan queue count is a fixed device resource where CUDA streams were
  unbounded; validation layers become a required part of GPU bring-up.
- **Alternatives considered** —
  - *Port to a Vulkan RT pipeline instead of ray query.* Rejected: rebuilds the SBT machinery for
    a shader that does no shading.
  - *hipRT* (AMD's OptiX-shaped library, runs on both AMD and NVIDIA). Rejected: swaps one vendor
    library for another, and adds a non-apt dependency.
  - *Hand-rolled BVH traversal in a plain compute shader, no RT extension.* Genuinely attractive:
    runs on **every** GPU (Vulkan 1.1, and portable to WebGPU), no extension gating, no device
    selection minefield. Costs an estimated 2-5x on the trace — which may be free in wall-clock
    terms given the GPU already hides behind the CPU self-collision pass. **Rejected only because
    hardware RT is strictly faster where present; revisit if Vulkan RT device-support gating turns
    out to be the real portability blocker rather than the vendor SDK.**
  - *Drop the GPU path, use Embree on CPU.* The ADR-014 hedge; still available, still not needed.

---

## 11. Not planned — `jfa.cu` → compute shader

Previously drafted as a "Phase D"; **cut, on the §0 reframing.** CUDA deploys cleanly, so
`jfa.cu` is not an obstacle to a shippable binary and porting it buys nothing this work is for.

It remains the one thing standing between QuevedoMP and running on non-NVIDIA hardware, so if that
ever becomes a goal in its own right, the port is easy: jump flooding is a grid stencil with no ray
tracing, needs only Vulkan 1.1 core compute and no extensions, and reuses `VulkanContext` from
Phase B as-is. It would get its own design note and ADR (its owner is ADR-018 `ClearanceField`,
not this work). Do not bundle it into Phase B under any circumstance — it has a different test
surface and would make the Phase-B differential un-bisectable.

---

## 12. Risks

| Risk | Mitigation |
|---|---|
| **The ACM candidate loop is subtly wrong** (candidate vs committed, §5.2). Fails as *plausible wrong answers*, not a crash. | Test 5 asserts both directions of the filter. Do not mark Phase B done on test 3 alone. |
| **Missing `SHADER_DEVICE_ADDRESS` usage or `DEVICE_ADDRESS` alloc flag** → `VK_ERROR_DEVICE_LOST` with no useful message. | Enable validation layers (`QUEVEDOMP_VULKAN_VALIDATION=1`) for the entire Phase-B bring-up; they name this exact mistake. |
| **Queue external-synchronization violation** (§B3) → nondeterministic corruption under a multi-threaded planner, invisible in single-threaded tests. | Per-queue mutex around `vkQueueSubmit` only. Add a test that runs 16 threads × 1000 queries and asserts determinism against the serial answer. |
| **ReBAR tier chosen but the heap is 256 MB**, and a large batch's transform buffer silently exceeds it → allocation failure mid-planner. | Cap the ReBAR tier at heap_size/4; fall back to staging when a requested buffer exceeds the cap, per allocation, not per context. |
| **Vulkan 1.2 features absent in the container** even though the host GPU supports them (ICD not mounted). | The §8 test-1 diagnostic plus `vulkaninfo --summary` in the smoke gate. `NVIDIA_DRIVER_CAPABILITIES` already has `graphics`. |
| **Phase A regresses OptiX performance** by dropping pinned staging. | The A4 staging-accessor design keeps pinned memory. Gate every Phase-A commit on `bench_dtc` within 3%. |
| **Scope creep into distance queries / dynamic scenes.** | D9. Both THROW. Widening scope is a separate ADR after the differential is green. |
| **The backend works, the binary still does not deploy** — some other build-tree path, a `dlopen`, or a fixture path creeps in and nobody notices because every unit test runs from the build tree. | Test 7 (`deploy_smoke.sh`) deletes the build tree before running. It is the only test that can catch this class, so it must be a CI gate, not a manual step. |
| **`ClearanceField` silently disabled** in a `dev-vulkan-nocuda` build, and a planner quietly loses its heuristic. | Not a new risk (it predates this work), but the new preset makes it reachable. Confirm the existing no-CUDA path FAILS LOUDLY or is explicitly opt-out; if it degrades silently today, fix that in Phase C. |

---

## 13. Out of scope (say no in review)

- Distance / witness queries on Vulkan (D9 — stays on FCL, spec §4.3/§4.5).
- Dynamic scene edits (`add_object`/`move_object`/`remove_object`) on any GPU backend.
- Deleting the OptiX backend (needs §9's numbers first; separate ADR — but see D8b, it is the
  intended destination, not a hypothetical).
- Porting `jfa.cu` — explicitly cut, see §11. CUDA is not the deployment problem.
- Any other CUDA removal. `WITH_CUDA=ON` is part of the shipping configuration.
- macOS / MoltenVK. Metal's ray-query support through MoltenVK is incomplete; do not claim it and
  do not add a preset for it.
- Robot-link instancing in a per-batch TLAS (the ADR-014 fallback). The transform layout is
  compatible (§6.2) so the door is open, but this work does not walk through it.
- Any change to the sampling/planning layers. This is a collision-backend port; `query_batch`
  semantics are unchanged by construction (`GpuRayScene::query_batch` is `final`).

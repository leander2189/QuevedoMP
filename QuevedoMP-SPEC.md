# QuevedoMP — Unified Project Specification (Build-Ready)

> Single source of truth for building QuevedoMP. This document merges the long-term
> architecture vision with the agreed **v0 scope cuts**, and resolves every conflict
> between them so an implementing agent never sees contradictory instructions.

**Project name:** QuevedoMP (Quevedo Motion Planner), pending trademark/availability
verification. Named after Leonardo Torres Quevedo (1852–1936), Spanish engineer who built
*El Ajedrecista* in 1912 — one of the earliest autonomous machines. `MP` = "Motion Planner".

**Naming conventions:**
- `quevedomp` (lowercase) — namespace, package, binary, file paths.
- `QuevedoMP` (mixed case) — display name, brand, docs.

---

## 0. Overview

### 0.1. Purpose

A lightweight, modular, GPU-accelerated C++20 library for robot-arm trajectory planning.
Goal: a ROS-free, modular core that integrates into non-ROS industrial stacks, with a clean
CPU/GPU backend abstraction and rigorous cross-validation.

### 0.2. How to use this document (read first)

This spec describes both a **v0 deliverable** and a **long-term vision**. The rules:

1. **Build only what `§6 Phase Plan` lists for the current phase.** Phases are strictly
   ordered; do not start a phase before its predecessor meets its exit criteria.
2. **`[DEFERRED]` means do not build it now.** Deferred items are documented so that what
   you build today stays compatible with them — never so you implement them early.
   Anything in `§7` is off-limits for v0.
3. **Where this document and any older note disagree, this document wins.**
4. **The interfaces in `§4` and the types in `§6` are contracts.** Match the signatures.
   If a signature seems wrong, raise it as a decision (`§12`) — do not silently diverge.
5. **CPU-first, GPU-second, on every axis.** Every capability ships and is tested on a CPU
   reference before its GPU implementation exists. The GPU version then proves itself by
   differential testing against the running CPU system.
6. **YAGNI.** Don't add an abstraction until a second real implementation exists. Real
   flexibility comes from clean modularity, not from many options.

### 0.3. v0 scope at a glance

**In v0:** URDF load · FK · numerical IK · `Rng` + seed recording · collision interface +
**FCL (CPU) and OptiX-static (GPU)** backends · discretized edge checking · **RRT-Connect**
planner · shortcut smoother · **TOPP-RA** time parameterization · best-effort capture &
replay · rerun visualization · Python (nanobind) bindings.

**Deferred / dropped (see `§7`):**

| Item | Status | Reason |
|---|---|---|
| Bit-exact GPU determinism | `[DEFERRED]` | Its cost lived in MPPI's atomic reductions (also deferred). Replaced by best-effort replay + tracing. |
| ESDF / dynamic scene / nvblox / depth ingestion | `[DEFERRED]` | Not needed for static-environment RRT planning. |
| MPPI & Hybrid planners | `[DEFERRED]` | RRT covers the v0 pipeline end to end. |
| Continuous (swept-volume) collision | `[DEFERRED]` | RRT edges validated by discretized batch checks. |
| CUDA-BVH fallback backend | `[DEFERRED]` | Host requirements mandate Turing+ (CC ≥ 7.5); pre-Turing hedge is YAGNI. Interface stays; impl skipped. |
| Ruckig time parameterization | **dropped** | Value (online jerk-limited reactive replanning) only matters with dynamic scenes. |
| B-spline smoother | `[DEFERRED]` | Shortcut smoothing makes RRT output usable. |
| ROS2 adapter | `[DEFERRED]` (optional stretch) | Heavy; not required for a demonstrable v0. |

> **Coupling note:** the deferred items above are not independent. ESDF, MPPI/Hybrid,
> continuous collision, Ruckig, and bit-exact determinism return **together** as one epic
> (`§7`), because OptiX yields ray *hits*, not the signed-distance + gradient that
> optimization planners and reactive replanning require.

### 0.4. Structural decisions

| Decision | Value | Rationale |
|---|---|---|
| Primary language | C++20 | Concepts, ranges, modernity, portability |
| GPU | CUDA + OptiX (RT cores) | Best available HW; abstracted for later porting |
| ROS2 | External adapter, not core `[DEFERRED]` | Enables non-ROS clients |
| Build | CMake ≥ 3.25 + Ninja | Industry standard |
| Python bindings | nanobind | Faster than pybind11, zero-copy with DLPack |
| Visualization | rerun.io | Debug/inspection from day 1 |
| Dev environment | WSL2 + Docker + VS Code Dev Containers | Reproducibility |
| Testing | GoogleTest + pytest + **differential testing** | Cross-validation against FCL |
| License | Dual (AGPL + commercial) | Tesseract/Qt model (decision pending, `§12`) |
| Reproducibility | **Best-effort capture + replay** | CPU/RNG determinism + capture bundle + trace; *not* bit-exact in v0 |
| Capture format | MCAP | Robotics standard, Foxglove-compatible |

### 0.5. Differential value proposition (v0-honest)

- **ROS-free core:** ROS2 is an adapter, not a dependency. No ROS type ever appears in `quevedomp/`.
- **GPU-first, not GPU-only:** abstracted backend, CPU fallback for testing and portability.
- **Backend-validated correctness:** GPU backends are proven against an FCL CPU oracle via
  exhaustive differential testing, not asserted.
- **Commercial-friendly license**, unlike cuRobo.
- **Debuggable failures:** every failure emits a self-contained capture bundle that re-runs
  in a debug build and doubles as a permanent regression fixture.

> Honest positioning: v0's defensible wins are **integrability, license, and reproducibility**,
> not raw speed superiority over cuRobo. Treat "beat cuRobo on speed" as aspirational and
> dependent on the deferred optimization epic.

---

## 1. Project Structure

### 1.1. Directory layout

```
quevedomp/
├── .devcontainer/              # VS Code Dev Container config
├── .github/workflows/          # CI: build, test, benchmark, sanitizers
├── cmake/                      # Custom modules (FindOptiX.cmake, etc.)
├── docs/
│   ├── architecture/           # ADRs, diagrams
│   ├── api/                    # Doxygen + Sphinx + Breathe
│   └── tutorials/
├── include/quevedomp/          # Public API (headers)
│   ├── core/  robot/  collision/  kinematics/  planning/
├── src/                        # Implementations (.cpp, .cu)
│   ├── core/  robot/  kinematics/  planning/
│   ├── collision/
│   │   └── backends/
│   │       ├── cpu_fcl/        # reference oracle (built FIRST)
│   │       ├── optix/          # GPU static (built SECOND)
│   │       ├── cuda_bvh/       # [DEFERRED] interface only
│   │       └── esdf/           # [DEFERRED]
│   └── capture/                # serializers + writers (MCAP)
├── tools/
│   ├── quevedomp-replay/       # standalone CLI replay tool
│   └── quevedomp-studio/       # pure-Python interactive IDE (viser + rerun, ADR-016)
├── bindings/
│   ├── python/                 # nanobind
│   └── ros2/                   # [DEFERRED] adapter (separate colcon package)
├── tests/
│   ├── unit/ integration/ differential/ benchmarks/ fixtures/
├── examples/{cpp,python}/
├── third_party/                # submodules / vcpkg
├── CMakeLists.txt  CMakePresets.json  vcpkg.json  README.md
```

### 1.2. Dependency policy

**Hard (always):** Eigen 3.4+ · spdlog · yaml-cpp · urdfdom or tinyxml2 · fmt · mcap · zstd.

**v0 reference/oracle:** **FCL** — not optional in practice; it is the differential-testing
oracle *and* the CPU collision backend that keeps the no-GPU build functional.

**Optional (build flags):**
- CUDA Toolkit 12.x — `QUEVEDOMP_WITH_CUDA=ON`
- OptiX SDK 8.x — `QUEVEDOMP_WITH_OPTIX=ON`
- rerun-cpp — `QUEVEDOMP_WITH_RERUN=ON`
- nanobind — `QUEVEDOMP_WITH_PYTHON=ON`
- `[DEFERRED]` nvblox (ESDF), rclcpp (ROS2)

**Rule:** the minimal build (`WITH_CUDA=OFF`, `WITH_OPTIX=OFF`) **must compile and pass all
CPU-only tests**, including the full RRT pipeline on the FCL backend. This is the proof that
the abstraction holds.

### 1.3. Code conventions

- **Style:** clang-format, LLVM base, line length 100.
- **Naming:** `snake_case` functions/variables, `PascalCase` types, `kCamelCase` constants.
- **Namespaces:** everything under `quevedomp::`; submodules `quevedomp::collision`, etc.
- **Headers:** `.hpp` for C++, `.cuh` for CUDA, `.h` only for C-API.
- **Include order:** own → third-party → standard, blank-line separated.
- **No `using namespace`** in headers.
- **`[[nodiscard]]`** on factories and resource-returning functions.
- **Docs:** Doxygen `///` in public headers; free comments in implementations.
- **Thread-safety documented** per class: `const-safe`, `not-thread-safe`, or `clone-required`.

---

## 2. Development Environment

### 2.1. Host requirements

- Windows 11 + WSL2 + Ubuntu 22.04/24.04, or native Linux.
- Docker Engine + nvidia-container-toolkit (GPU phases only).
- NVIDIA driver ≥ 535, GPU compute capability ≥ 7.5 (Turing+) for OptiX.
- VS Code with the Dev Containers extension.

> Phases 0–2a and the full CPU pipeline need **no GPU**. GPU is required only from Phase 2b.

### 2.2. Dev Container

`.devcontainer/Dockerfile` (structure):

```dockerfile
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04
RUN apt-get update && apt-get install -y \
    build-essential cmake ninja-build git \
    clang-15 clang-format-15 clang-tidy-15 clangd-15 gdb cuda-gdb \
    python3-pip python3-dev python3-venv \
    libeigen3-dev libspdlog-dev libfmt-dev libyaml-cpp-dev liburdfdom-dev \
    libfcl-dev \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/microsoft/vcpkg.git /opt/vcpkg && /opt/vcpkg/bootstrap-vcpkg.sh
# OptiX SDK — manual download from NVIDIA dev portal (needed from Phase 2b)
COPY optix-8.0.0-linux64.sh /tmp/
RUN sh /tmp/optix-8.0.0-linux64.sh --skip-license --prefix=/opt/optix
RUN pip3 install pytest numpy scipy rerun-sdk
ARG UID=1000
RUN useradd -m -u $UID dev && chown -R dev /opt/vcpkg
USER dev
ENV PATH="/opt/vcpkg:${PATH}"
```

`.devcontainer/devcontainer.json` enables clangd, cmake-tools, nsight, python extensions;
sets `clangd.arguments: ["--compile-commands-dir=build"]`; `runArgs: ["--gpus=all"]`.

### 2.3. Build presets (`CMakePresets.json`)

- `dev-cpu`: Debug, CPU-only (FCL), sanitizers ON. **Primary preset through Phase 2a.**
- `dev-gpu`: Debug, CUDA + OptiX, compute-sanitizer compatible.
- `release`: Release, all enabled backends, LTO ON.
- `ci-fast`: Release, no OptiX, for GPU-less runners. Runs the full CPU pipeline.
- `ci-gpu`: Full release, self-hosted GPU runner.

### 2.4. Profiling & debug tools

Nsight Compute (kernels) · Nsight Systems (timeline) · compute-sanitizer (GPU races/memory,
nightly CI) · ASan/UBSan (CPU builds) · TSan (dedicated CI) · OptiX validation mode (debug).

---

## 3. Modular Architecture

### 3.1. Layers (dependencies only go downward)

```
┌─────────────────────────────────────────────────────────────┐
│  bindings/    python (nanobind)   · ros2_adapter [DEFERRED]  │
│  facades/     SimplePlanner (convenience)                    │
├─────────────────────────────────────────────────────────────┤
│  planning/                                                   │
│  ├── PlanningProblem, TaskLimits, QueryOptions (types)       │
│  ├── Planner (interface)        ◄── RrtConnect   (Mppi/Hybrid [DEFERRED]) │
│  ├── Smoother (interface)       ◄── Shortcut     (BSpline    [DEFERRED]) │
│  └── TimeParameterization       ◄── ToppRa       (Ruckig     [DROPPED])  │
├─────────────────────────────────────────────────────────────┤
│  kinematics/  fk() · jacobian() · InverseKinematics ◄── Numerical        │
├─────────────────────────────────────────────────────────────┤
│  collision/   CollisionScene (interface)                     │
│               ◄── CpuFcl, StaticOptix   (Esdf/Hybrid/CudaBvh [DEFERRED]) │
├─────────────────────────────────────────────────────────────┤
│  robot/   RobotModel (shared_ptr<const>) · RobotInstance · load_urdf()   │
│           KinematicChain · RobotLimits · AllowedCollisionMatrix          │
├─────────────────────────────────────────────────────────────┤
│  core/types/ (JointState, SE3, …) · core/compute/ (ComputeBackend)       │
│  core/math/ (lie algebra) · core/rng/ (Rng, deterministic substreams)    │
├─────────────────────────────────────────────────────────────┤
│  capture/  (cross-cutting: serialization, replay artifacts)  │
└─────────────────────────────────────────────────────────────┘
```

`capture/` depends on the modules above it (to serialize their state) but nothing depends on
it. It is opt-in instrumentation, never on the runtime planning path.

### 3.2. Architectural rules

1. **Dependencies only go downward.** Any lateral/upward dependency is an architectural bug.
2. **Shared types live in `core/types/`** with no module dependencies.
3. **No god classes.** No `QuevedoMPManager`. Explicit composition.
4. **Abstract interfaces only where there are ≥ 2 real or imminently planned implementations.**
   (Collision and planning qualify; e.g. the `Planner` interface is justified because MPPI is
   a known future implementation even though v0 ships only RRT.)
5. **Thread-safety documented** for every class.
6. **Explicit ownership** (`§3.3`).
7. **The core does NOT know about ROS.** ROS lives only in `bindings/ros2/`.

### 3.3. Ownership and lifetime

| Type | Ownership | Reason |
|---|---|---|
| `RobotModel` | `shared_ptr<const>` | Immutable, shared across planner/IK/FK/scene |
| `RobotInstance` | `unique_ptr` | Mutable (attachments, ACM); single owner |
| `CollisionScene` | `unique_ptr` or `shared_ptr` | Expensive to build; shareable across planners |
| `Workspace` | `unique_ptr`, one per thread | Holds all mutable per-call scratch (see `§4`) |
| `Planner` | `unique_ptr` | Not shared; clone if N instances needed |
| `Trajectory`, `JointState`, `Pose` | values | Small or movable |

### 3.4. Configuration

Typed structs per module + an optional YAML loader. `std::variant` for algorithm-specific
options (type-safe, no downcasts).

```cpp
struct CollisionSceneConfig {
    BackendHint backend = BackendHint::Auto;
    float default_padding = 0.0f;
    std::optional<OptixConfig> optix;
};
CollisionSceneConfig load_collision_config(const YamlNode&);
```

---

## 4. Collision Module — Interface & Contract (technical heart)

The interface is designed so the FCL↔OptiX swap is invisible above `collision/`, the no-GPU
build compiles and passes, and GPU resource ownership is explicit.

### 4.1. Design decisions

1. **CPU vocabulary only across the boundary.** No CUDA type appears in any
   `include/quevedomp/collision/` header. Queries speak `JointPosition`, `Transform`,
   `CollisionResult`. This is what makes backends interchangeable and the minimal build honest.
2. **Batch-first.** The primary method is `query_batch`; `query` is a one-element convenience.
   GPUs want batches, and an RRT edge check *is* a batch (the discretized configs of an edge).
   FCL loops; OptiX launches once.
3. **Explicit `Workspace` for all mutable scratch.** A `const` GPU query still needs a stream
   and device buffers. The scene hands out a backend-specific `Workspace`; **one per thread ⇒
   concurrent const queries are lock-free**, and `const` stays honest (scene read-only;
   workspace owns mutable state).
4. **FCL defines the semantic contract.** The contract (`§4.3`) is prose; the differential
   test (`§4.6`) is that contract made executable. OptiX is implemented *to* the contract.

### 4.2. Public headers

```cpp
// include/quevedomp/collision/types.hpp
namespace quevedomp::collision {

struct QueryOptions {
    bool  distance             = false; // also compute signed min-distance + witness
    float safety_margin        = 0.0f;  // operational threshold: collision if dist < margin
    float robot_padding        = 0.0f;  // physical inflation of robot collision geometry
    float max_distance         = 0.10f; // distance clamps beyond this (perf)
    bool  check_self_collision = true;  // robot-vs-robot, honoring the ACM
    std::optional<PaddingMap> per_pair_padding;
};

struct CollisionPair {                  // witness, for debug/visualization
    std::string a, b;                   // link or object ids
    Eigen::Vector3d point_a, point_b;
};

struct CollisionResult {
    bool  in_collision = false;
    float min_distance = 0.0f;          // signed; valid only if distance requested
    std::optional<CollisionPair> witness;
};

struct BatchResult {
    std::vector<uint8_t> in_collision;  // one per config (NOT vector<bool>)
    std::vector<float>   min_distance;  // empty unless distance requested
    std::vector<CollisionPair> witnesses; // empty unless requested (expensive)
};

}
```

```cpp
// include/quevedomp/collision/collision_scene.hpp
namespace quevedomp::collision {

// Opaque, backend-specific scratch. FCL's is ~trivial; OptiX's owns a CUDA stream,
// device buffers, pinned host staging, SBT and params. Never crosses the API as a type.
class Workspace { public: virtual ~Workspace() = default; };

using SceneHandle = uint32_t;

class CollisionScene {
public:
    virtual ~CollisionScene() = default;

    // Static-environment editing (typically once at setup).
    virtual SceneHandle add_object(std::string id, const Geometry&, const Transform&) = 0;
    virtual void        remove_object(SceneHandle) = 0;
    virtual void        move_object(SceneHandle, const Transform&) = 0;

    // Each querying thread owns one workspace ⇒ lock-free concurrent queries.
    [[nodiscard]] virtual std::unique_ptr<Workspace> make_workspace() const = 0;

    // Primary query. The scene FKs the robot at each q, poses its collision geometry,
    // and tests robot-vs-environment and robot-vs-self (per ACM).
    [[nodiscard]] virtual BatchResult query_batch(
        const RobotInstance& robot,
        std::span<const JointPosition> qs,
        const QueryOptions& opts,
        Workspace& ws) const = 0;

    // Convenience over query_batch, defined once in the base class.
    [[nodiscard]] CollisionResult query(
        const RobotInstance& robot, const JointPosition& q,
        const QueryOptions& opts, Workspace& ws) const;
};

enum class BackendHint { Auto, ForceCpuFcl, ForceOptix };

[[nodiscard]] std::unique_ptr<CollisionScene> make_static_scene(
    std::shared_ptr<const RobotModel> robot,   // needed for FK + collision geometry
    const SceneDescription& environment,
    BackendHint = BackendHint::Auto);

}
```

```cpp
// include/quevedomp/collision/edge_check.hpp
namespace quevedomp::collision {

struct EdgeResult { bool valid; float first_contact_t; }; // t in [0,1]; 1.0 if valid

// The RRT primitive. q0->q1 is sub-sampled at `resolution` (rad, max joint step) and
// checked as ONE batch. Continuous/swept checking is a future swap-in behind this exact
// signature — callers never change.
[[nodiscard]] EdgeResult check_edge(
    const CollisionScene&, const RobotInstance&,
    const JointPosition& q0, const JointPosition& q1,
    float resolution, const QueryOptions&, Workspace&);

}
```

### 4.3. Semantic contract (FCL defines it)

**Boolean collision.** A config is `in_collision` iff, after inflating robot collision
geometry by `robot_padding`, any of these overlap: a robot link vs. any environment object,
**or** a robot link vs. another robot link not allowed to touch by the `AllowedCollisionMatrix`
(only when `check_self_collision`). If `safety_margin > 0`, "overlap" means signed distance
`< safety_margin`.

**Signed min-distance.** When requested: positive = separation (clamped at `max_distance`),
negative = penetration depth, zero = touching. Witness is the nearest pair when free, the
deepest pair when colliding.

**Robot posing.** The scene computes FK at each `q` to place robot collision geometry.
Environment geometry is fixed at `add_object` time. (v0 FKs on the host; batched GPU FK is a
later optimization behind the same API.)

**The boundary band.** Backends may legitimately disagree on the boolean within a thin band
around the surface, because FP noise flips the sign of a near-zero distance. The contract
guarantees agreement **only outside** a band `± ε_band` (default **1e-4 m**). This is a
*boolean* contract with an ambiguity band — not a blanket numeric tolerance — which is the
realistic relationship between FCL (exact mesh distance) and OptiX (ray-cast) near contact.

### 4.4. FCL backend (built FIRST — Phase 2a)

- Wraps FCL broad-phase managers for environment and posed robot links.
- `query_batch`: loop configs, FK each, update FCL transforms, run collide/distance. Slow,
  exact. **Defines truth** for boolean (outside the band) and is **authoritative for distance
  + witness**.
- `make_workspace()` returns near-empty scratch; concurrency is trivially safe.
- **Zero GPU dependencies** → runs the entire pipeline before OptiX exists.

### 4.5. OptiX backend (built SECOND — Phase 2b)

- **One-time:** GAS per distinct environment mesh and per robot-link collision mesh; IAS over
  environment (static) + robot-link instances (transforms updated per config).
- **Per config:** host FK → per-link transforms → write IAS instance transforms → refit/rebuild
  robot portion → launch ray-gen testing each robot link against environment + other links;
  **any-hit + early termination** gives the boolean; ACM filters self-pairs in the program.
- **Batch realization:** simplest first impl iterates configs (refit + launch each); faster
  version encodes config index in launch dimensions with per-config transforms in a device
  buffer. **Build the simple one, measure, then optimize.**
- **v0 distance policy:** OptiX returns **boolean only** (optionally coarse ray-cast distance);
  exact distance + witness stays with FCL. RRT needs only boolean, so this sidesteps OptiX's
  real weakness (no natural signed distance/gradient). A proper distance+gradient backend
  arrives with the deferred ESDF epic, not as a bolt-on here.
- **Workspace:** CUDA stream, device buffers (configs/transforms/results), pinned host staging,
  SBT, params. One per thread.

### 4.6. Differential testing = the contract, executed

```text
for trial in 1..N (N = 10_000):
    scene   = random_environment()
    q       = random_config(robot)            # mix free / colliding / grazing
    r_fcl   = fcl.query(q, {distance=true})    # truth
    r_optix = optix.query(q, {distance=false}) # boolean only

    if abs(r_fcl.min_distance) <= EPS_BAND:
        ambiguous += 1; continue               # in-band disagreement allowed
    assert r_optix.in_collision == r_fcl.in_collision   # MUST match outside band

report(success_rate, ambiguous_fraction, optix_throughput / fcl_throughput)
```

Any out-of-band disagreement is, by definition, an OptiX bug. Captured failures (`§5`) feed
straight back as permanent fixtures. Generate enough grazing configs that `ambiguous_fraction`
is meaningful, not accidental.

---

## 5. Reproducibility — Best-Effort Capture & Replay

A first-class concern, scoped honestly for v0: **deterministic-enough record-and-replay**.
We keep the cheap, high-value half (CPU/RNG determinism + a self-contained capture bundle +
a structured trace) and drop only the expensive bit-exact-GPU promise.

### 5.1. Goals

- Any planning failure (exception, timeout, no-solution, validation failure) is captured as a
  self-contained artifact.
- The artifact reloads and re-runs in a separate debug build (rerun, sanitizers, verbose
  tracing) without rebuilding production.
- Replay reproduces the failure **class**, and on the **same GPU arch + driver** usually the
  exact path; a `PlanningTrace` explains any divergence. **No bit-exact promise.**
- Captures double as permanent regression fixtures.

### 5.2. RNG and seeds (kept — cheap and foundational)

No global RNG. Every randomized component receives an `Rng` explicitly. The seed is **always
recorded** in the result whether passed or auto-generated. Substream spawning gives
deterministic CPU-side sampling regardless of thread count/order — which makes RRT essentially
reproducible for free.

```cpp
namespace quevedomp {
class Rng {
public:
    explicit Rng(uint64_t seed);
    Rng spawn(uint64_t stream_id) const;     // independent deterministic substream
    double uniform(double lo, double hi);
    Eigen::VectorXd sample_in_box(const Eigen::VectorXd& lo, const Eigen::VectorXd& hi);
private:
    std::mt19937_64 gen_;
};
}
struct PlanningProblem { /* ... */ std::optional<uint64_t> seed; };
struct PlanningResult  { /* ... */ uint64_t used_seed; };  // always populated
```

> `[DEFERRED]` GPU bit-exact determinism (`DeterminismOptions`, tree/Kahan reductions, sorted
> any-hit, fixed launch config). Its cost was concentrated in MPPI, which is deferred; revisit
> with the optimization epic.

### 5.3. Capture bundle (kept)

The capture is a versioned, language-agnostic record of everything needed to re-run a plan:
QuevedoMP + format versions, timestamp, optional sanitizable hostname; serialized robot
(URDF + tool YAML inlined), robot state (attachments + ACM), scene (objects + poses), the
`PlanningProblem`, module configs, the recorded `master_seed`; the outcome reason
(`Exception | Timeout | NoSolution | ManualDump | AssertionFailure | ValidationFailure`),
error message, optional stack trace; optional `partial_result` and `PlanningTrace`.

**Serialization format:** MCAP, zstd-compressed by default. The serializers for `RobotModel`,
`RobotInstance`, and `CollisionScene` are needed for differential testing too — build them in
Phase 2a and reuse them here.

### 5.4. Execution trace (optional, more important without bit-exactness)

A capture lets you re-run; a trace lets you understand without re-running. A `PlanningTrace`
is a timestamped event log (`IterationStart`, `SampleGenerated`, `CollisionQuery`,
`TreeNodeAdded`, `GoalChecked`, `UserMessage`, …). **Off by default in production** (recording
every collision query would dominate runtime); enabled via `CapturePolicy::include_trace`.

### 5.5. Policy and triggers

```cpp
struct CapturePolicy {
    bool dump_on_exception = true;
    bool dump_on_failure   = false;          // Timeout, NoSolution
    std::filesystem::path output_dir = "/var/log/quevedomp/captures";
    size_t max_captures = 100;               // rotate oldest
    bool include_trace = false;
    bool include_scene_meshes = true;
    std::function<void(PlanningCapture&)> sanitizer; // strip/anonymize before write
};
```

Three trigger paths: automatic on exception (capture, then rethrow), automatic on failure
status (if enabled), and manual ("this worked but looks wrong" → `make_capture` + `write_capture`).

### 5.6. Replay tool (best-effort)

Standalone `quevedomp-replay` and Python `quevedomp.replay`, both built against the same
library as production but linked with all debug dependencies. Replay reproduces the failure
class, prints a summary, and offers: open in rerun, drop into gdb, re-run verbose-trace,
re-run with modified parameters, export minimal reproducer. Python mirrors this
(`qr.load(...).replay()`, mutate config and re-run, `visualize_in_rerun()`).

### 5.7. Captures as regression tests

Failure in production → capture written → sent to team → bug fixed → capture added to
`tests/fixtures/captures/` → CI replays it on every build so the bug stays fixed. Over time
this builds a real-world scenario dataset no synthetic benchmark matches.

### 5.8. Privacy, IP, storage

Sanitizer hook runs before serialization; mesh-hash mode replaces meshes with content hashes
(reproducible locally, useless to third parties); zstd compression; rotation by count/size;
optional upload callback to a customer-controlled location.

---

## 6. Phase Plan (the build roadmap)

Each phase ends only when its **Definition of Done** is met.

### Phase 0 — Bootstrap (Weeks 1–2)

**Deliverables:** repo with the layout; functional Dev Container ("Reopen in Container"
works); root `CMakeLists.txt` + `CMakePresets`; `vcpkg.json` with Eigen, spdlog, fmt,
yaml-cpp, gtest, fcl; GitHub Actions `ci-fast` job (clone → build → `ctest`); a trivial
passing test; README quick-start; `.clang-format`, `.gitignore`, `LICENSE` placeholder.

**DoD:** a new developer goes clone → open in VS Code → reopen in container → build → test
pass in under 30 minutes; `ci-fast` is green.

### Phase 1 — Robot, URDF, FK, IK, Rng (Weeks 3–7)

**Core types** (`core/types/`, Eigen-only, no inter-dependencies):

```cpp
namespace quevedomp {
    using JointPosition = Eigen::VectorXd;
    using JointVelocity = Eigen::VectorXd;
    struct JointState { JointPosition pos; JointVelocity vel; };
    class  Transform { /* wraps Eigen::Isometry3d */ };
    struct Pose   { Transform tf; double pos_tol = 1e-3; double rot_tol = 1e-2; };
    struct Sphere { Eigen::Vector3d center; double radius; };
    struct Box    { Transform tf; Eigen::Vector3d half_extents; };
    struct Mesh   { /* vertices, indices */ };
    struct Waypoint { JointState state; double time = 0.0; };
    using  Trajectory = std::vector<Waypoint>;
}
```

**Robot model** (`robot/`): `Link`, `Joint` (Revolute/Prismatic/Fixed, with
pos/vel/acc/jerk limits), `KinematicChain` (`fk`, `jacobian`), `RobotModel::from_urdf(urdf,
optional yaml_extension) -> shared_ptr<const RobotModel>`.

**Kinematics** (`kinematics/`): pure `fk(model, q, link)` and `fk_all(model, q)`;
`InverseKinematics` interface with `NumericalIk` (damped least squares, multi-seed restart);
`make_numerical_ik(shared_ptr<const RobotModel>)`. (Analytic/TRAC-IK are future.)

**Visualization** (`viz/`): `log_robot`, `log_trajectory`, `log_pose`; no-ops when
`WITH_RERUN=OFF`.

**`Rng`:** implemented with substream spawning; determinism policy documented in an ADR.

**Testing:** URDF load for 5 robots (UR5, UR10, Franka Panda, KUKA iiwa, ABB IRB); FK vs
published/KDL reference; FK∘IK ≈ identity (1000 seeds); analytic vs finite-difference jacobian
< 1e-6; libFuzzer on malformed URDFs; rerun visual-sanity artifact stored in CI.

**DoD:** 5 robots loaded and visualized; FK position error < 1e-9 m vs reference; IK
convergent < 10 ms for reasonable targets; coverage > 80% in `core/types`, `robot`,
`kinematics`; `Rng` + ADR done.

> Industrial-vendor caveat: clean URDFs/limits for KUKA/ABB/Fanuc are a data-gathering slog
> unlike UR/Franka. Budget time or substitute community URDFs and record the source.

### Phase 2a — Collision interface + FCL backend (Weeks 8–10)

**Deliverables:** the `§4` interface and semantic contract; the FCL backend; `check_edge`;
the serializers for `RobotModel`/`RobotInstance`/`CollisionScene` (reused by captures).

**Testing:** closed-form sphere-sphere/box/triangle; property-based symmetry/non-negativity;
serializer round-trip equality; `check_edge` on a known free vs colliding edge.

**DoD:** full CPU pipeline buildable and testable on FCL; serializer round-trips; coverage
> 80% in `collision/`. **No GPU required.**

### Phase 2b — OptiX static backend (Weeks 11–13)

**Deliverables:** OptiX backend behind the same interface (`§4.5`); `FindOptiX.cmake`; PTX/
OptiX-IR build of the `.cu` kernels; backend auto-selection.

**Testing:** the differential harness (`§4.6`) with boundary band; pathological cases
(grazing, tunneling avoided via edge resolution, extreme self-collision); throughput / p50/
p95/p99 latency / memory / AS-build benchmarks tracked in CI (alert if regression > 10%).

**DoD:** boolean agreement with FCL on every config outside the ±1e-4 m band (in-band
disagreement reported, not failed); distance is FCL-authoritative; OptiX boolean throughput
≥ 5× FCL.

### Phase 3 — RRT pipeline + capture (Weeks 14–19)

**Planning types** (`planning/`): `TaskLimits` (TCP vel/acc + frame); `Goal` base with
`JointGoal`, `PoseGoal`, `MultiGoal`; `Constraints`; `PlanningProblem` (start, goal,
constraints, task limits, collision opts, timeout, optional seed); `PlanningResult`
(`Success | Timeout | NoSolution | InvalidProblem`, path, stats, `used_seed`).

**Planner interface + v0 impl:**

```cpp
class Planner { public: virtual ~Planner()=default;
    virtual PlanningResult plan(const PlanningProblem&) const = 0; };
class RrtConnectPlanner : public Planner { /* CPU; collision via CollisionScene batch */ };
std::unique_ptr<Planner> make_planner(PlannerConfig,
    std::shared_ptr<const RobotModel>, std::shared_ptr<CollisionScene>);
// MppiPlanner / HybridPlanner: [DEFERRED] — interface already accommodates them.
```

**Smoother:** `ShortcutSmoother` (iterative shortcut; required to make RRT output usable).
`BSplineSmoother` `[DEFERRED]`.

**Time parameterization:** `ToppRaParameterization` (time-optimal under joint + TCP vel/acc).
`RuckigParameterization` dropped.

**Capture system:** full `PlanningCapture`, automatic dump on exception, basic `quevedomp-replay`
CLI (best-effort).

**Testing:** RRT finds a known 2D solution in < N nodes and validates against OMPL; smoother
preserves collision-freeness; time-param respects vel/acc at every point; full-pipeline
integration on a MotionBenchMaker static subset; rerun trajectory/overlay visual validation;
benchmark vs MoveIt2 RRTConnect.

**DoD:**
- ≤ 50 ms mean plan for UR5 in a moderate static scene (RRT-Connect + shortcut, OptiX collision).
- ≥ 95% success in free space, ≥ 80% with obstacles.
- Time parameterization respects **velocity and acceleration** at every point. **Jerk is NOT
  guaranteed** by classic TOPP-RA (bang-bang switching → unbounded jerk at switch points);
  v0 accepts C¹ output and lets the controller absorb it. A jerk-limiting post-filter is a
  stretch goal. *(This corrects the original "vel/acc/jerk" criterion, which the chosen tool
  cannot meet alone.)*
- Best-effort reproducibility: every exception/timeout/no-solution emits a capture bundle that
  reloads and re-runs, with a `PlanningTrace`. No bit-exact promise.

### Phase 4 — Python bindings (Weeks 20–23)

> **Amended 2026-07-11 (ADR-016, build-plan M6): split into 4a/4b.** **4a** — a minimal
> binding slice over the Phase 3a API (types, robot, FK/IK/Jacobian, collision, planner +
> smoother) plus **`quevedomp-studio`** (`tools/quevedomp-studio/`): a pure-Python interactive
> "Motion Planning IDE" — viser for 3D editing (gizmos, joint sliders, obstacle placement,
> plan/scrub), rerun (Python SDK) for logging/replay. 4a may start at the Phase 3a exit,
> independent of Phase 3b. **4b** — the remainder below (capture/TOPP-RA bindings, full
> parity, notebook). Performance guardrails: bindings are verb-level only (no Python inside
> C++ loops), blocking calls release the GIL, and the core is never modified for Python —
> `QUEVEDOMP_WITH_PYTHON=OFF` builds are unaffected.

**Deliverables** (`bindings/python/`, nanobind): `module.cpp` + `bind_{types,robot,collision,
planning}.cpp`; Python package wrapper with `_native.pyi` stubs. Python API mirrors C++ 1:1;
numpy zero-copy for `JointPosition` etc.; DLPack interop reserved for the future GPU-tensor path.
Plus (4a) `quevedomp-studio` as above.

**Testing:** pytest replicating critical C++ tests; zero-copy numpy validation; an end-to-end
notebook (load robot → plan → smooth → parameterize → visualize in rerun); a headless scripted
studio smoke test.

**DoD:** the notebook runs end to end; pytest green in `ci-fast`; type stubs present; a studio
session covers IK + collision + plan + smooth interactively.

> `[DEFERRED]` ROS2 adapter (`bindings/ros2/`, separate colcon package, `MoveGroup`-compatible
> action server, `/planning_scene` subscriber, TF2, RViz publishers). Optional stretch; no ROS
> type may enter `quevedomp/` core.

---

## 7. Deferred — "Dynamic + Optimization" Epic (DO NOT BUILD IN v0)

These return **together**, because OptiX yields ray hits, not signed-distance + gradient:

- **ESDF dynamic scene** (incremental TSDF→ESDF on GPU; integrate nvblox first, own impl as a
  later goal) with `update_from_depth(...)`; provides the smooth distance + gradient field.
- **MPPI** (GPU-parallel rollouts) and **Hybrid** (RRT seed + optimization refinement) planners.
- **Continuous (swept-volume) collision** behind the existing `check_edge` signature.
- **Ruckig** online jerk-limited reparameterization for reactive replanning.
- **Bit-exact GPU determinism** (deterministic reductions, sorted any-hit, fixed launch config)
  — the determinism work that MPPI actually motivates.
- **CUDA-BVH** backend for pre-Turing hardware / OptiX-licensing hedge.

Building any one alone produces an interface with no backend to feed it. Scope as one milestone.

---

## 8. Cross-cutting Testing

**Pyramid:** many unit (fast, isolated) · fewer integration (module-to-module) · few E2E ·
**differential** as a dedicated layer validating GPU vs CPU reference.

**Golden datasets:** MotionBenchMaker / BARN subsets + custom industrial scenes (tables,
shelves, cabinets). Each PR runs a fast subset; nightly runs the full set.

**CI metrics (alert if regression > 10%):** mean plan time/scenario · trajectory RMS jerk
(tracked even though not bounded in v0) · path length · success rate · collision throughput ·
memory peak · build time.

**CI platforms:** Ubuntu 22.04 no-GPU (CPU fallback — must pass full pipeline on FCL) ·
Ubuntu 22.04 + CUDA 12.4 + OptiX 8 + self-hosted GPU · Ubuntu 24.04 + CUDA 12.6 + GPU ·
Windows 11 + WSL2 (container validation). ARM/Jetson `[DEFERRED]`.

**Mandatory CI tools:** clang-format check · clang-tidy on new code · ASan + UBSan (debug) ·
TSan (dedicated runner) · compute-sanitizer (nightly) · coverage (lcov + Codecov).

---

## 9. Documentation & ADRs

Every significant decision → `docs/architecture/adr-NNN.md` (Context · Decision · Consequences
· Alternatives). Expected initial ADRs:

- ADR-001 — OptiX over pure CUDA for static collision.
- ADR-002 — nanobind over pybind11.
- ADR-003 — `[DEFERRED]` ESDF as primary dynamic representation.
- ADR-004 — `[DEFERRED]` ROS2 as adapter, not core.
- ADR-005 — Threading policy and the `Workspace` ownership model.
- ADR-006 — RNG substream architecture; CPU determinism; **best-effort (not bit-exact) replay**.
- ADR-007 — MCAP as capture serialization format.
- ADR-008 — Capture-based regression-testing strategy.
- ADR-009 — Collision semantic contract + boundary-band differential testing.
- ADR-010 — FCL-first/OptiX-second sequencing and the boolean-authoritative distinction.
- ADR-011 — TOPP-RA only for v0; jerk not bounded; consequences accepted.
  **(Superseded 2026-07-14 by ADR-017:** TOPP with tip vel/acc limits + optional jerk phase;
  jerk bounded in Scp mode on C³ paths; ConvexOnly keeps ADR-011's contract.)

**API docs:** Doxygen (C++) + Sphinx/Breathe + `.pyi` stubs; tutorials for quick start, custom
robot, planning constraints, performance tuning.

---

## 10. Roadmap Summary

| Phase | Wk | Main output |
|---|---|---|
| 0. Bootstrap | 2 | Repo, Dev Container, `ci-fast`, trivial test |
| 1. Robot + FK/IK + Rng | 5 | URDF load, FK, numerical IK, rerun, `Rng` + seed |
| 2a. Collision + FCL | 3 | Interface + contract + FCL + `check_edge` + serializers (CPU pipeline) |
| 2b. OptiX static | 3 | OptiX behind same interface; boundary-band differential testing |
| 3. RRT pipeline | 6 | RRT-Connect + shortcut + TOPP-RA + capture + best-effort replay |
| 4a. Python slice + studio | 2 | minimal nanobind slice over the 3a API; `quevedomp-studio` interactive IDE (ADR-016) |
| 4b. Python parity | 2 | full nanobind API, numpy zero-copy, end-to-end notebook |

**v0 total ≈ 23 weeks (~5.5 months)** to a demonstrable end-to-end system. Hardening and the
`§7` Dynamic+Optimization epic follow.

---

## 11. Risks and Mitigation

| Risk | Prob | Impact | Mitigation |
|---|---|---|---|
| OptiX learning curve dominates schedule | High | Med | FCL-first keeps OptiX off the critical path; it swaps in validated. |
| cuRobo releases a more permissive license | Med | High | Differentiate on architecture, integrability, support. |
| OptiX terms become restrictive | Low | High | CUDA-BVH kept as a documented (deferred) fallback behind the same interface. |
| Subtle collision bugs (false free) | High | High | Exhaustive differential testing; capture-replay for every reported case; HIL later. |
| FCL/OptiX distance can't agree near contact | High | Med | Resolved by design: boolean-with-band contract; OptiX boolean-authoritative, FCL distance-authoritative. |
| TOPP-RA leaves unbounded jerk | Med | Med | Accept C¹ for v0; jerk-limiting post-filter as stretch; Ruckig in the deferred epic. |
| Industrial-vendor URDF data scarce | Med | Low | Use community URDFs, record provenance, validate FK against published frames. |
| Scarcity of CUDA/OptiX talent | Med | Med | Detailed internal docs, pair programming, FCL-first reduces early GPU surface. |

---

## 12. Pending Decisions

Decide before the relevant phase:

- [ ] **FK location (Phase 2a).** v0 commits to **scene-internal FK** (the scene holds the
      `RobotModel` and poses the robot per query — matches the `§4.2` headers). Alternative:
      FK outside, link transforms passed in (keeps the scene FK-agnostic, complicates batched
      GPU FK later). Ratify or change before building the FCL backend.
- [ ] Exact YAML tool-extension schema.
- [ ] Versioning / ABI-compatibility policy.
- [ ] Dual-license model: pricing, what "commercial" includes (note: any LGPL/GPL deps need
      an explicit ADR given the dual-license intent).
- [ ] Telemetry: opt-in, what's collected, where stored.
- [ ] Multi-arm / > 7-DOF / humanoid support (future).
- [ ] TAMP integration (future).

---

## Final Notes for the Implementing Agent

- **Strict YAGNI.** Don't abstract until a second implementation appears. The `Planner`,
  `Smoother`, and `CollisionScene` interfaces are the sanctioned exceptions (≥ 2 known impls).
- **Tests first on the critical path** — especially collision: a false-free is a real robot
  hitting something real.
- **Build the CPU path before the GPU path, always.** A working FCL pipeline is the spec the
  OptiX backend is measured against.
- **Document decisions as ADRs** — cheap to write, expensive to recover later.
- **Design GPU-friendly data structures from the start** (SoA, alignment) but don't prematurely
  optimize kernels — measure first.
- **Every PR is small, atomic, cleanly revertible, and lands with tests.**
- **When in doubt between simplicity and a "flexible future," choose simplicity.** Real
  flexibility comes from clean modularity, not from many options.
- **Respect `[DEFERRED]`.** If a task seems to need a deferred capability, stop and raise it as
  a decision — don't pull the future forward.

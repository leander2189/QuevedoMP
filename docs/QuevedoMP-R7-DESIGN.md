# QuevedoMP R7 Design — Attached Objects + `quevedomp_tasks` (MTC-lite) + Studio Tasks Mode

> **Audience: the implementing agent.** This document is self-contained: it records the ratified
> decisions, the verified code map (file:line as of commit `ddcd5d0`; re-verify lines before
> editing — they drift), the finished architecture, and a commit-by-commit execution plan with
> gates. Follow it top to bottom. Where this document says THROW / FAIL LOUDLY, that is the
> project invariant "no silent fallbacks" — do not soften it.
>
> Ratified with Leandro 2026-07-18/19. Records as **ADR-022** when Phase A lands (outline in §9).

---

## 0. Context and goal

Roadmap **R7** (`docs/QuevedoMP-ROADMAP-v1.md`, open row): attached objects in `RobotInstance`
(C++: a grasped part moves with the robot's collision geometry and participates in the ACM) →
`quevedomp_tasks`, an MTC-lite task layer in Python (Sequence/Alternatives, `PlanTo`,
`IkBranches` via `solve_all`, `CartesianMove`, trajectory stitching + final `parametrize`) →
the studio's Tasks mode becomes its inspector/runner (stage tree, per-stage preview, Run,
hand-off to Trajectory mode). R6 (ADR-021) already restructured the studio into modes and left
`modes/tasks.py` as the placeholder.

### Ratified product decisions (do not relitigate)

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | **Rebuild-on-attach.** Scenes snapshot the robot's attachments at construction; attach/detach = rebuild scene + workspaces + caches. No dynamic-geometry API in the backends. Stale use must FAIL LOUDLY. | Attach/detach are rare structural events; matches the quasi-static philosophy (PRM/ClearanceField already invalidate on env edits); avoids surgery in two backends. |
| D2 | **DFS backtracking** over choice points (IK branches, Alternatives children) with a time/attempt budget. | The combinatorial search is most of MTC's practical value. |
| D3 | **Full studio inspector**: stage tree + per-stage status/solutions + click-to-preview + Run + hand-off to Trajectory. Tasks are *defined in Python*, never built graphically. | ADR-021 record. |
| D4 | Old `.qmps` files MUST remain loadable (Leandro's benchmark sessions are versioned artifacts). | Standing invariant. |
| D5 | Attached-object geometry is the **inline `collision::Geometry` variant**, not URI-based `CollisionGeometry`. `MeshSources` is NOT part of the attachment path. | The studio hand-off starts from an env obstacle whose geometry is already this variant; the serializer's shape helpers write exactly it; both backends already consume it. A user with a mesh file attaches `load_mesh(...)`'s result. |
| D6 | "One final parametrize" is refined to **one parametrize per contiguous group**, with rest-to-rest stops at every Attach/Detach boundary. | The gripper physically actuates there (robot must stop anyway), and a single spline across an attach boundary is unvalidatable — the scene changes mid-spline. Record this refinement in the R7 roadmap record. |

### Standing invariants that bind this work

Batch-first collision (fatten batches; one `query_batch` per logical validation); determinism per
seed (bit-identical across thread counts); one `Workspace` per querying thread (ADR-005);
apt-only deps (the tasks package may depend on **numpy only**); ADR discipline (next free number:
**ADR-022**); recorded-numbers-not-vibes (every performance claim measured and written down);
clang-format LLVM/100col/2sp (`.clang-format` at repo root); each commit leaves the tree green.

### Canonical build/test commands (from Windows, via WSL)

```bash
# C++ suite (ASan/UBSan):
wsl -d Ubuntu-24.04 -- bash -lc "docker run --rm -v /mnt/d/Inventos/quevedoMP:/work -w /work \
  quevedomp-cuda bash -lc 'cmake --preset dev-cpu && cmake --build --preset dev-cpu && \
  ctest --preset dev-cpu --output-on-failure'"
# GPU differential tests: same with --gpus all and preset dev-gpu.
# Python bindings tests: ctest target python_pytest inside the preset, or directly:
#   PYTHONPATH=build/release-py/bindings/python python3 -m pytest bindings/python/tests -q
# Tasks package tests:
#   PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-tasks \
#     python3 -m pytest tools/quevedomp-tasks/tests -q
# Studio tests:
#   PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio:tools/quevedomp-tasks \
#     python3 -m pytest tools/quevedomp-studio/tests -q
```

The release build for Python work: `cmake --preset release-py && cmake --build --preset release-py`.

---

## 1. Verified code map (spot-checked; re-verify line numbers before editing)

**C++ core**
- `include/quevedomp/robot/robot_instance.hpp` — header-only; `AllowedCollisionMatrix` =
  `std::set<pair<string,string>>` of normalized name pairs, fully mutable
  (`allow/disallow/is_allowed/pairs`, :18-35); `RobotInstance` holds `model_` + `acm_` (:50-51);
  comments already reserve "attached objects" (:1-4, :37-38).
- `include/quevedomp/collision/geometry.hpp` — `collision::Geometry =
  std::variant<BoxShape, SphereShape, CylinderShape, Mesh>` (:28), pose-free; `SceneObject{id,
  geometry, pose}`; `SceneDescription`.
- `src/collision/fcl_scene.cpp` — robot `LinkShape{link_index, geom, origin}` built ONCE in the
  ctor from the model (:252-272); `FclWorkspace` ctor makes one `FclObject` per shape, **link
  index as userdata** (:421); internal OpenMP `pool_` of sub-workspaces (:433-436); FK flows as
  `link_poses[ls.link_index] * ls.origin` (:565) — the attach hook composes identically;
  self-collision skips same-link + ACM-by-name (:191-212); P4 env-ACM masks per batch
  (:334-358); env userdata negative-encoded (:279); `make_env_geom` consumes the inline variant
  incl. Mesh (:83-100); ADR-012 interior points (:260-265).
- `src/collision/optix/optix_scene.cpp` — robot mesh edges → rays ONCE in ctor
  `build_robot_rays` (:619-694); primitives tessellated (:628-647, `src/collision/tessellate.*`);
  env = static GAS with per-triangle object ids (:533-617); self-collision delegated to internal
  `fcl_self_` (:190, :414-420); `add/move/remove_object` THROW (:206-214); `ray_link_slots_`
  maps slot→link index and per-slot transforms come from `link_poses[ray_link_slots_[slot]]`, so
  **duplicate link indices across slots are already tolerated** — an attachment can own its own
  slot for per-object GPU ACM masks.
- `include/quevedomp/collision/collision_scene.hpp` — `make_static_scene(model, env, hint,
  meshes)` (:76-78) takes the MODEL; the robot (ACM) is passed per
  `query_batch(robot, qs, opts, ws)` (:44-46). `HybridScene` demotes on env edits.
- `include/quevedomp/collision/edge_discretization.hpp` — `cartesian_lever_weights(model,
  meshes)` (:26): P3 lever weights from the bare model. **An attached object extends its parent
  link's swept radius — bare-model weights break the max_link_sweep guarantee.**
- `src/capture/serialize.cpp` — ONE global `kVersion = 1` (:13) shared by "QMRM" (model),
  "QMRI" (robot instance, :155-167 = model blob + ACM pairs), "QMSC" (scene); `expect_version`
  hard-rejects mismatches (:123-126); `GeomTag{kBox,kSphere,kCylinder,kMesh}` append-only (:129);
  the scene blob's geometry write/read bodies live inline in `serialize_scene` (:184-219) /
  `deserialize_scene` (:221-262) — extract and reuse them.
- `include/quevedomp/kinematics/fk.hpp` — `fk_all(model, q)` → per-link `Transform` (:17);
  `include/quevedomp/kinematics/ik.hpp` — `InverseKinematics::solve_all(link, target,
  max_solutions, seed)` (:60-62): deterministic per `IkOptions::seed`, distinct per `branch_tol`
  (:26), nearest-first ordering when a dof-sized seed is given.
- Planner surface: `make_planner(params, robot, scene)` + `Planner::plan(problem)`
  (`planning/planner.hpp`); `make_shortcut_smoother` (`smoother.hpp:61-71`); `make_prm_planner`
  (`roadmap.hpp:84-87`); goals `JointGoal/PoseGoal/MultiGoal` (`planning/types.hpp:33-91`);
  `resolve_goal` is TU-local in three planners and stays untouched (the tasks layer resolves
  pose goals itself via `solve_all` — the parked P5 note anticipated exactly this).
- Parameterization: `PathSpline::fit` = C⁴ quintic through ALL waypoints (chord-length
  parameter); `fit_collision_free(waypoints, scene, robot, disc, opts, ws)` re-validates as one
  `query_batch` per round; `parametrize(model, spline, limits, options)` rest-to-rest over the
  whole spline (`parameterization/*.hpp`, bound in `bind_parameterization.cpp`). Reference flow:
  `StudioSession.parametrize` (`session.py:594-650`).
- **No SE(3) interpolation exists anywhere** (Transform has compose/inverse/`quaternion()` wxyz
  only; `bind_types.cpp:43-90`). `check_edge` is not bound to Python.

**Python / studio**
- Bindings: `bindings/python/src/module.cpp` registers 7 `bind_*.cpp` TUs (list in
  `bindings/python/CMakeLists.txt:26-36`); stubs regenerate automatically (:48-52); ctest
  `python_pytest` (:55-62). New core `.cpp` files append to the library list in root
  `CMakeLists.txt:62-88` (R5's `prm.cpp` precedent :82). C++ test = 3-line block in
  `tests/CMakeLists.txt` (template `test_prm` :141-143; fixture-define variant :69-73; OptiX
  section guard :151-158).
- Studio session (`tools/quevedomp-studio/quevedomp_studio/session.py`): ACM seeding :139-145;
  scene + workspace construction :147-151; caches `_clearance_field`, `_prm`, `_robot_spheres`
  (:433-437, cached as "model-immutable" — becomes WRONG under attachments), `_lever_weights`
  (:326-331) **plus the baked copy** `planner_params.lever_weights` (only recomputed when
  `size == 0`, :351 — must be explicitly cleared); `solve_ik_branches` :239-266 (`IkBranch`
  dataclass :76-83) is the model for the IkBranches stage; `_ik_track` (max_restarts=0) :177-181
  is the model for Cartesian IK tracking; `plan_async/refine_async/build_roadmap_async` share
  one `_plan_thread` guarded by `is_planning`.
- Studio modes (ADR-021): `Mode` contract = `build()/set_active()/shutdown()`
  (`modes/base.py`); `StudioContext` with `ui_lock`, `ee_link`, `ik_gizmo`, `last_attempt`,
  `AttemptView`, and listener lists `config/scene_changed/attempt` (`context.py`);
  `modes/tasks.py` is the placeholder to replace. `.qmps` `SAVE_FORMAT = "quevedomp-studio/2"`
  — unchanged by R7; attachments ride inside the QMRI blob.
- Packaging: `quevedomp_tasks` is pure Python → new `tools/quevedomp-tasks/` mirroring
  `tools/quevedomp-studio/` (setuptools `pyproject.toml`; `quevedomp` deliberately NOT declared
  as a dependency — it comes from the build tree via PYTHONPATH).

---

## 2. Phase A — C++ attached objects

### A1. `AttachedObject` + `RobotInstance` API

New header `include/quevedomp/robot/attached_object.hpp`:

```cpp
// robot/AttachedObject — a rigid body grasped by / bolted to a link (roadmap R7, ADR-022).
// Geometry is the inline collision variant (pose-free); world placement at config q is
// fk_all(model, q)[link] * local_pose.
struct AttachedObject {
  std::string id;                // unique among attachments; appears in ACM pairs + witnesses
  std::string link;              // parent link name (must exist in the model)
  collision::Geometry geometry;  // BoxShape | SphereShape | CylinderShape | Mesh (inline)
  Transform local_pose;          // parent-link frame -> object frame (the grasp offset)
};
```

`RobotInstance` additions (declarations in the header; implementation in a **new TU
`src/robot/robot_instance.cpp`** appended to the root CMake source list — keeps the hash code
out of the hot include):

```cpp
// Throws std::invalid_argument on: empty id, duplicate id, id equal to any link name
// (ACM ambiguity), unknown parent link.
void attach(AttachedObject obj);
// Throws std::invalid_argument on unknown id. Returns the detached object so the caller can
// put it back into the environment at its FK pose.
AttachedObject detach(const std::string &id);
const std::vector<AttachedObject> &attachments() const noexcept;
const AttachedObject *find_attachment(const std::string &id) const noexcept; // null if absent
// Monotone counter bumped on every attach/detach. A cheap "something changed" signal for
// Python-side cache invalidation. NOT the compatibility guard.
std::uint64_t attachment_revision() const noexcept;
// Content hash (FNV-1a 64) over the ordered attachment list — THE stale-scene guard token.
// Hashes per attachment: id bytes, link bytes, the 16 doubles of local_pose.matrix(), a
// geometry tag byte, then primitive params or (for Mesh) vertex/triangle counts + the raw
// vertex and index buffer bytes. 0 when there are no attachments. Cached; recomputed on
// mutation. Full-bytes hashing is deliberate: attachments are parts, not environments — the
// one-time cost per mutation buys a collision-proof guard.
std::uint64_t attachment_fingerprint() const noexcept;
```

`attach()` does **not** auto-edit the ACM. Attachment-vs-parent-link contact is suppressed
structurally in the backends (same link index ⇒ same-link skip); every other allowance (e.g.
`acm().allow("part", "socket")` during insertion) is the caller's explicit act.

### A2. Scene construction from a `RobotInstance` + guards

Add an overload in `include/quevedomp/collision/collision_scene.hpp` (the model overload stays
and forwards with an empty snapshot / fingerprint 0):

```cpp
// Build a scene over a static environment FOR THIS ROBOT: snapshots robot.attachments() and
// robot.attachment_fingerprint() at construction (rebuild-on-attach, ADR-022). query_batch
// throws if later called with a robot whose fingerprint differs from the snapshot.
[[nodiscard]] std::unique_ptr<CollisionScene>
make_static_scene(const RobotInstance &robot, const SceneDescription &environment,
                  BackendHint hint = BackendHint::Auto, const MeshSources &meshes = {});
```

**Two independent guards, both mandatory** (they catch different stale parties):

1. **Robot↔scene fingerprint.** Shared helper in `src/collision/collision_scene.cpp`:
   `void require_attachment_match(std::uint64_t scene_fp, const RobotInstance &robot)` —
   throws `std::runtime_error` naming both fingerprints: *"robot attachments changed after this
   scene was built — rebuild the scene (rebuild-on-attach, ADR-022)"*. Called at the top of
   `FclScene::query_batch` and `OptixScene::query_batch`. `HybridScene` forwards (both children
   check); `CollisionScene::query` routes through `query_batch` — covered.
2. **Workspace↔scene epoch.** `FclScene::query_batch` indexes `fws.objects_[s]` in lockstep
   with `link_shapes_` (:563-566): a workspace created by the pre-attach scene used against the
   post-attach scene has FEWER objects than shapes — **out-of-bounds write, not just a wrong
   answer**. Fix: every scene instance gets a unique id (monotonic `std::atomic<uint64_t>` at
   construction); every workspace records its creator's id; `query_batch` throws
   *"workspace was created by a different scene — call make_workspace() on the rebuilt scene"*
   on mismatch BEFORE any indexing. Applies to `FclWorkspace`, `OptixWorkspace`,
   `HybridWorkspace`. (This closes a pre-existing footgun that rebuild-on-attach turns from
   theoretical into routine.)

**FCL consumption** (`src/collision/fcl_scene.cpp`):
- Ctor gains the attachment snapshot + fingerprint. Extend `LinkShape` →
  `{int link_index; shared_ptr<FclGeom> geom; Transform origin; std::string name;
  bool is_attachment;}` — `name` = link name for model shapes, **object id** for attachments.
  Attachments append after the model loop: `link_index` = parent link, geom via the existing
  `make_env_geom` (handles the inline variant incl. Mesh), `origin` = `local_pose`. Inline-mesh
  attachments push an ADR-012 interior point (`local_pose * centroid`), mirroring :260-265.
- **Userdata semantics change**: robot `FclObject`s carry the **shape index** (≥0) instead of
  the link index (env objects stay negative-encoded). `self_callback` resolves
  `shape_index → (link_index, name)` through a scene pointer: the same-`link_index` skip now
  structurally covers attachment↔parent-link and sibling geometries on one link; the ACM check
  uses `name` uniformly, so `acm.allow("part", "forearm_link")` works by object id.
- `EnvAcm` keys become `(shape_index << 32) | env_handle`; `compute_env_acm` resolves each ACM
  name to *all shape indices of that link* or *the attachment's single shape index*.
  Containment skip masks and `link_interior_` entries move to shape-index keying alongside.
- Distance witnesses report `shape.name` — a colliding part reads `"part ↔ inlet"`, not the
  wrist link name.

**OptiX consumption** (`src/collision/optix/optix_scene.cpp`):
- `make_optix_scene` (internal, fwd-declared in fcl_scene.cpp:627-629) takes
  `const RobotInstance &` instead of the model; ctor stores snapshot + fingerprint.
- `build_robot_rays`: after the per-link loop, one loop over attachments — primitives through
  the existing `tessellate_*`, inline `Mesh` used directly; link-frame vertices =
  `local_pose * v`; **each attachment gets its own transform slot** mapped to the parent link's
  index (per-slot transforms already come from `link_poses[ray_link_slots_[slot]]`, so duplicate
  link indices are fine) — this keeps per-object granularity in the P4 `env_allowed` slot×object
  GPU mask. The slot-name resolution for mask building maps link name → its slots and
  attachment id → its slot. Attachment centroids append to the containment interior points.
- `fcl_self_` is built from the **same snapshot** via
  `make_static_scene(robot_snapshot, SceneDescription{}, ForceCpuFcl, meshes)` so the delegated
  self-collision call passes the fingerprint guard by construction.

### A3. Lever weights

`include/quevedomp/collision/edge_discretization.hpp` gains:

```cpp
// As above, but the robot's attachments EXTEND their parent link's geometry extent so the
// max_link_sweep guarantee still holds while carrying a part. Attachment meshes are inline —
// `meshes` only resolves the model's own URI-referenced links.
[[nodiscard]] JointPosition cartesian_lever_weights(const RobotInstance &robot,
                                                    const MeshSources &meshes = {});
```

Implementation: refactor the existing tip→base bounding-ball recursion to accept per-link extra
extents; each attachment contributes `max_v |local_pose * v|` (its farthest point from the link
origin) to its parent link's ball. Property test: a long box attached at the wrist strictly
increases the weights of every joint in its chain; detach → bitwise-equal to bare-model weights.

### A4. Serializer v2 (backward-compatible)

`src/capture/serialize.cpp`:
- Split the global `kVersion` into per-blob constants: `kModelVersion = 1`, `kSceneVersion = 1`,
  `kInstanceVersion = 2`. Rework `expect_version` to
  `uint32_t expect_version(Reader&, std::initializer_list<uint32_t> accepted)` (throws listing
  accepted versions). QMRM/QMSC continue to write and accept 1 — **do not** invalidate old
  model/scene blobs.
- Extract `write_geometry(Writer&, const collision::Geometry&)` and
  `collision::Geometry read_geometry(Reader&)` from the QMSC bodies (:184-262) and reuse in both
  blobs. `GeomTag` stays append-only.
- `serialize_robot_instance` writes version 2: after the ACM pairs, `u64 count`, then per
  attachment `str id, str link, transform local_pose, write_geometry(...)`.
- `deserialize_robot_instance`: version 1 → stop after ACM (zero attachments); version 2 → read
  the section and `robot.attach(...)` each; version ≥3 → throw.

### A5. Bindings

Extend existing TUs (no new TU):
- `bind_robot.cpp`: `nb::class_<AttachedObject>` (init + rw `id/link/geometry/local_pose`; the
  geometry field uses the same variant caster `add_object` relies on), and on `RobotInstance`:
  `attach`, `detach`, `attachments` (prop, returns copy or ro-ref), `attachment_revision`
  (prop). The fingerprint stays C++-internal.
- `bind_collision.cpp`: the `make_static_scene(RobotInstance, ...)` overload alongside the model
  overload (distinct nanobind types — no dispatch ambiguity; keep GIL-release parity), and the
  `cartesian_lever_weights(RobotInstance, ...)` overload.

### A6. Phase-A tests

- `tests/unit/test_attached_objects.cpp` (+ 3-line CMake block, fixture define): attach/detach
  semantics and all `invalid_argument` cases; revision monotonicity; fingerprint equality across
  two instances with identical lists, inequality on any field change, restoration after
  detach + identical re-attach. Collision behavior on UR5 (or the two-link fixture): a config
  whose links are free but whose attached box overlaps an env obstacle flips `in_collision`;
  detach + rebuild flips back; long attached box vs. base link self-collides,
  `acm.allow("part", base)` suppresses it, attachment-vs-parent-link never reports;
  `allow("part", "socket")` lets the part penetrate the socket while the wrist still collides
  (the insertion case); distance witness names the id. **Guard tests**: query after attach
  without rebuild throws; pre-attach workspace against a rebuilt scene throws; a ≥8-config batch
  exercises the OpenMP pool path with attachments under ASan.
- OptiX section (`dev-gpu`): FCL-vs-OptiX differential agreement over a batch with an attached
  box AND an attached inline mesh; guard throws on the GPU path too.
- `tests/unit/test_capture_serialize.cpp` (extend): v2 round-trip with 0/1/2 attachments incl.
  a mesh (exact geometry + pose); **v1 back-compat** — build v1 bytes with a small local writer
  replicating the old layout and assert it loads with zero attachments; version 3 throws;
  QMRM/QMSC still read as version 1.
- `bindings/python/tests/test_attached.py`: attach flips a collision verdict; stale-guard
  `RuntimeError` on both guard paths; serializer round-trip; lever-weight growth; a plan with an
  attached part is deterministic per seed.

---

## 3. Phase B — `quevedomp_tasks` (pure Python, MTC-lite)

### B0. Package layout

```
tools/quevedomp-tasks/
  pyproject.toml                # name quevedomp-tasks, deps: numpy only (D2); quevedomp comes
                                # from the build tree via PYTHONPATH (studio precedent)
  quevedomp_tasks/
    __init__.py                 # public surface: TaskContext, TaskState, stages, run_task,
                                # RunBudget, TaskResult, parametrize_result, errors
    context.py                  # TaskContext + ensure()/rebuild() + seed derivation
    state.py                    # TaskState, AttachSpec, pose keys
    stages.py                   # Stage protocol, Sequence, Alternatives, PlanTo, IkBranches,
                                # Attach, Detach, targets (JointTarget/PoseTarget)
    cartesian.py                # slerp + CartesianMove
    runner.py                   # DFS runner, budget, trace
    result.py                   # Segment, StageAttempt, TaskResult, stitching, parametrize
    errors.py                   # TaskDefinitionError, TaskExecutionError, TaskParameterizationError
  tests/
    test_stages.py  test_runner.py  test_cartesian.py  test_pick_place.py  test_stitching.py
```

### B1. State model (`state.py`)

The DFS needs immutable, comparable states so backtracking can restore the physical world
exactly. Poses are compared by bytes (float64, row-major `Transform.matrix()`), which makes
equality exact and deterministic:

```python
PoseKey = bytes  # np.asarray(T.matrix(), float).tobytes()

@dataclass(frozen=True)
class AttachSpec:
    id: str
    link: str
    local_pose: PoseKey          # link -> object
    geometry_id: str             # key into TaskContext's geometry registry
    touch: tuple[str, ...] = ()  # extra ACM allows active while attached (e.g. ("socket",))

@dataclass(frozen=True)
class TaskState:
    q: tuple[float, ...]                       # current configuration (hashable)
    attachments: tuple[AttachSpec, ...]        # objects riding the robot (ordered by id)
    env_poses: tuple[tuple[str, PoseKey], ...] # objects in the environment (ordered by id) —
                                               # poses may differ from the base description
                                               # after a place (Detach) event
```

Geometry never rides the state: `TaskContext` keeps a registry `{id: collision.Geometry}` built
from the base environment (plus nothing else — attach/detach only moves ids between
`env_poses` and `attachments`). Two states are interchangeable iff their dataclass equality
holds; `ensure()` relies on exactly that.

### B2. `TaskContext` (`context.py`)

```python
class TaskContext:
    """Thin ownership of (model, robot, environment, scene, workspace) with rebuild-on-attach.
    Headless by construction — the studio adapts its session onto this in Phase C."""
    def __init__(self, robot: q.RobotInstance, environment: q.SceneDescription, *,
                 mesh_sources: dict | None = None, backend=q.BackendHint.Auto,
                 planner_params: q.PlannerParams | None = None,
                 query_options: q.QueryOptions | None = None,
                 ik_options: q.IkOptions | None = None,
                 timeout: float = 2.0, seed: int = 0): ...

    def initial_state(self, q_start: np.ndarray) -> TaskState:
        # attachments from robot.attachments (usually empty), env_poses from the description.

    def ensure(self, state: TaskState) -> None:
        # Reconcile the LIVE robot + scene with `state`. Diff-driven and minimal:
        #  - robot attachments: detach ids not in state, attach missing ones (geometry from the
        #    registry, local_pose from the spec); apply/remove the specs' `touch` ACM allows.
        #  - environment: SceneDescription from state.env_poses (registry geometry).
        #  - IF anything differed: self.scene = q.make_static_scene(self.robot, env, backend,
        #    sources); self.ws = self.scene.make_workspace(); refresh attachment-aware lever
        #    weights into planner_params.lever_weights; bump self.rebuilds counter.
        # Correct under DFS by construction: backtracking past an Attach restores the previous
        # physical state on the next ensure(). Any ordering bug hits the Phase-A guards LOUDLY.

    def seed_for(self, stage_path: tuple[int, ...], attempt: int) -> int:
        # splitmix64(base_seed  XOR  fnv1a64(stage_path bytes)  XOR  attempt)
        # -> deterministic, independent streams per choice point; recorded in the trace.

    # Cached per (scene identity): prm planner for PlanTo(planner="prm"); invalidated by rebuild.
```

### B3. Stage protocol and containers (`stages.py`)

```python
@dataclass
class Segment:
    path: list[np.ndarray]        # joint waypoints; path[0] == state.q
    kind: str                     # "transit" | "cartesian" | "structural"
    cartesian: CartesianInfo | None = None   # commanded line, for the straightness verifier

@dataclass
class StageSolution:
    segment: Segment | None       # None for structural stages (Attach/Detach/IkBranches rebase)
    end_state: TaskState
    info: dict                    # planner stats / branch index / ik errors — inspector fodder

class Stage:
    name: str
    def solutions(self, ctx: TaskContext, state: TaskState) -> Iterator[StageSolution]:
        """Lazy generator; each yielded solution is one choice point. The runner guarantees
        ctx.ensure(state) was called before this. Raise TaskDefinitionError for structural
        misuse; yield nothing for 'tried and failed'."""

class Sequence(Stage):      # children run in order; a child failure backtracks into earlier
    def __init__(self, *stages: Stage, name: str = "seq"): ...
class Alternatives(Stage):  # first child whose subtree succeeds wins; order = preference
    def __init__(self, *stages: Stage, name: str = "alt"): ...
```

**`PlanTo`** — transit planning to a joint or pose target:

```python
@dataclass
class JointTarget:  q: np.ndarray
@dataclass
class PoseTarget:   link: str; pose: q.Transform; branches: int = 8
                    cost: Callable | None = None; pos_tol: float = 1e-3; rot_tol: float = 1e-2

class PlanTo(Stage):
    def __init__(self, target, *, planner="rrt_connect", retries=2, smooth=True, name=...): ...
```

- `JointTarget`: build `PlanningProblem` (start = state.q, `q.JointGoal`), plan via
  `q.make_planner(ctx.planner_params, ctx.robot, ctx.scene)` + optional shortcut smoothing
  (mirror `session.py:354-367`), `problem.seed = ctx.seed_for(path, attempt)`. Yields up to
  `retries` solutions (fresh derived seed each).
- `PoseTarget`: resolve branches exactly like `session.solve_ik_branches` —
  `q.make_numerical_ik` with `max_restarts = max(60, 12*branches)`, seeded options,
  `solve_all(link, pose, branches, seed=state.q)`; **collision-filter all branches in ONE
  `scene.query_batch`** (batch-first); order nearest-first or by `cost`. Then per surviving
  branch × per retry: plan to `JointGoal(branch_q)`. Each (branch, retry) is one yielded
  solution — the DFS choice points. This is the ratified "resolve pose goals via `solve_all`"
  path; the C++ TU-local `resolve_goal` stays untouched.
- `planner="prm"` uses the context's cached PRM for the current scene identity (built on first
  use; invalidated by any rebuild — document the cost: a PRM per attachment state).

**`IkBranches`** — standalone zero-motion rebase (config jumps to a branch). Legal ONLY as the
task's first stage (teleporting mid-sequence is meaningless); the runner raises
`TaskDefinitionError` otherwise. The common mid-task use is `PlanTo(PoseTarget(...))`, which
subsumes it.

**`Attach` / `Detach`** — structural stages, empty segment, single solution:

```python
class Attach(Stage):
    def __init__(self, object_id: str, link: str, *, touch: tuple[str, ...] = ()): ...
    # end_state: object_id moves env_poses -> attachments with
    #   local_pose = fk(model, state.q, link)^-1 * env_pose(object_id)
    # touch pairs become ACM allows while attached (applied by ctx.ensure()).
class Detach(Stage):
    def __init__(self, object_id: str): ...
    # end_state: object returns to env_poses at fk(model, state.q, link) * local_pose.
```

### B4. `CartesianMove` (`cartesian.py`)

```python
class CartesianMove(Stage):
    def __init__(self, link: str, *, to: q.Transform | None = None,
                 offset: np.ndarray | None = None, frame: str = "world",   # "world" | "tool"
                 max_step_pos: float = 0.005, max_step_rot: float = 0.02,
                 jump_threshold: float = 0.5, name: str = "cartesian"): ...
```

1. Start pose = `fk(model, state.q, link)`; goal = `to`, or start shifted by `offset` in
   `frame`. Waypoint count `n = ceil(max(d_pos/max_step_pos, d_rot/max_step_rot))`, poses by
   translation lerp + quaternion **slerp** (implement `_slerp(q0_wxyz, q1_wxyz, t)` in numpy
   with the shortest-arc sign fix — nothing exists in core).
2. Per-waypoint tracked IK: dedicated solver with `max_restarts=0` (the `_ik_track` pattern),
   seed = previous solution; failure ⇒ this solution fails with
   `"IK tracking lost at waypoint k/n (pos_err=…)"` in the trace.
3. Joint-jump guard: `max|Δq|` between consecutive solutions > `jump_threshold` ⇒ fail (branch
   flip detected) — loudly, with the offending waypoint index.
4. Collision: the tracked waypoints (≤ ~5 mm apart in Cartesian space; plus joint-space
   midpoints where consecutive solutions exceed the planner edge resolution) validated in **ONE
   `scene.query_batch`**. (`check_edge` is not bound — not needed.)
5. Single yielded solution (choices come from upstream branches); `segment.kind="cartesian"`
   with the commanded line recorded in `CartesianInfo` for the §B6 straightness verifier.

### B5. DFS runner (`runner.py`)

```python
@dataclass
class RunBudget:  max_seconds: float = 30.0; max_attempts: int = 200

def run_task(task: Stage, ctx: TaskContext, start_q, *, budget=RunBudget()) -> TaskResult
```

- Classic lazy DFS over the stage tree: `Sequence` = nested generators (a failed tail advances
  the nearest previous stage's iterator); `Alternatives` = chained children. Before expanding
  any stage the runner calls `ctx.ensure(state)` — the **only** place the live world mutates.
- Budget checked between expansions; every expansion appends
  `StageAttempt(stage_path, stage_name, attempt, seed, status, message, wall_time,
  waypoints)` to the trace — the studio inspector's data source. Statuses:
  `ok | planner_failed | ik_failed | collision | jump | pruned | budget`.
- First full success wins (preference order = declaration order). Exhaustion returns
  `TaskResult(status="failed" | "budget", trace=...)` — a loud, inspectable outcome, never a
  silent partial. Determinism: identical inputs + seeds ⇒ identical trace and result.

### B6. Stitching + parameterization (`result.py`)

- `TaskResult.segments` (ordered, motion segments only) are split into **groups** at every
  structural boundary (Attach/Detach). Per group, under `ctx.ensure(group.start_state)`:
  `q.fit_collision_free(waypoints, ctx.scene, ctx.robot, disc, opts, ws)` (disc mirrors
  `session.py:614-618` with the attachment-aware lever weights) →
  `q.parametrize(model, fit.spline, limits, options)` — **rest-to-rest per group**, which is
  exactly what a gripper event physically requires (D6).
- **Straightness verifier** (the corner-rounding answer): the C⁴ spline through a group rounds
  transit→cartesian junctions. Cartesian segments are densely sampled (≤ `max_step_pos`), so
  bowing concentrates near junctions. For each cartesian segment: map its waypoints to spline
  parameters via the fit's chord-length fractions, FK-sample the fitted spline over that
  subrange, measure max deviation from the commanded line, and raise
  `TaskParameterizationError` above `straightness_tol` (default `2 * max_step_pos`) **with the
  measured number in the message** (recorded numbers, not vibes). Collision safety is
  independently guaranteed by `fit_collision_free`.
- `parametrize_result(ctx, result, *, default_acceleration=8.0, tip_linear_velocity=0.0,
  tip_linear_acceleration=0.0, max_jerk=0.0, tip_link=None) -> StitchedTrajectory` —
  concatenates per-group `ParameterizationResult`s with cumulative time offsets;
  `StitchedTrajectory{times, positions, velocities, accelerations, duration, group_offsets,
  sample(t)}`. Public — the studio reuses it (Phase C).
- Rejected alternatives (record in ADR-022): one spline across attach boundaries (scene changes
  mid-spline — unvalidatable); per-stage parametrize (a full stop at every stage defeats
  stitching).

### B7. Phase-B acceptance test (`test_pick_place.py`)

UR5 fixture (mesh wiring as in `test_smoke.py:20-27`). Environment: table box, part box,
obstacle wall placed to block the nearest grasp branch. Task:

```python
Sequence(
  PlanTo(PoseTarget(pregrasp, branches=8)),
  CartesianMove("wrist_3_link", offset=[0, 0, -0.10], frame="tool"),
  Attach("part", "wrist_3_link"),
  CartesianMove("wrist_3_link", offset=[0, 0, +0.10], frame="tool"),
  PlanTo(PoseTarget(preplace, branches=8)),
  CartesianMove("wrist_3_link", offset=[0, 0, -0.10], frame="tool"),
  Detach("part"),
  CartesianMove("wrist_3_link", offset=[0, 0, +0.10], frame="tool"),
)
```

Assert: success; q-continuity across all segments; a mid-transit config re-queried with the
attached scene shows the part clearing the wall; the trace shows ≥1 rejected branch (the wall
forces backtracking); byte-identical `TaskResult` across two runs with the same seed;
stitching produces 3 groups with monotone concatenated times; the straightness verifier passes
at default tolerance and **fails loudly** when artificially tightened.

---

## 4. Phase C — Studio integration

### C1. Session attach/detach + rebuild + invalidation + persistence

`tools/quevedomp-studio/quevedomp_studio/session.py`:

```python
def attach_obstacle(self, oid: str, link: str) -> None:
    # RuntimeError while is_planning. local = link_pose(q)^-1 * obstacle.pose; remove the
    # obstacle from self.obstacles; self.robot.attach(AttachedObject(...)); _rebuild_scene().
def detach_object(self, oid: str) -> None:
    # obj = robot.detach(oid); re-add as obstacle at fk(link, q) * local_pose; _rebuild_scene().
@property
def attachments(self): ...
def _rebuild_scene(self) -> None:
    # self.scene = q.make_static_scene(self.robot, self.environment(), hint, self.mesh_sources)
    # self._ws = self.scene.make_workspace()
    # invalidate: _clearance_field, _prm, _robot_spheres, _lever_weights, AND explicitly
    # planner_params.lever_weights = np.empty(0)   # session.py:351 only recomputes when empty
```

`save()` needs no schema change — attachments ride the QMRI v2 blob; `SAVE_FORMAT` stays
`"quevedomp-studio/2"`. `load()` replays `robot.attachments` onto the fresh session (attach
each, one `_rebuild_scene()`); a saved attachment never re-appears as an obstacle. Old `.qmps`
(v1 QMRI blobs) load unchanged (D4).

Refactor the three `*_async` methods onto one private
`_submit_async(fn, on_done, thread_name)` (same `_plan_thread` + `is_planning` semantics) and
reuse it for the task runner.

### C2. Views

`robot_view.py`: `RobotView.set_attachments(attachments)` draws `/robot/attached/<id>` via the
existing `geometry_mesh` helper (already renders the inline variant), distinct tint;
`update_config` poses them at `link_poses[parent] * local_pose` and includes them in collision
tinting (witnesses now carry the object id, from Phase A). Scene mode: per-obstacle
**Attach to \<link dropdown\>** button; per-attachment **Detach** button; both fire
`ctx.scene_changed()` so the existing STALE markers trip.

### C3. Tasks mode (replace the `modes/tasks.py` placeholder)

- **Task source**: (a) a task-script path (`importlib` load of `build_task(session) -> Stage`);
  (b) a built-in "Pick & Place" template parameterized by object dropdown, grasp link, and the
  IK gizmo pose for pregrasp/preplace. No graphical builder (D3/ADR-021).
- **Adapter**: `StudioTaskContext(TaskContext)` over the session — shares robot/environment/
  params; routes `rebuild()` through `session._rebuild_scene()` **under `ctx.ui_lock`**; mirrors
  attachment state to the views via callback so previews show the part in hand.
- **Inspector**: viser has no tree widget — render the stage tree as markdown with status glyphs
  from `TaskResult.trace` (pending/✓/✗/branch counts); a stage-path dropdown + attempt selector;
  **Preview** = `ctx.ensure(segment start state)` under `ui_lock`, then scrub the segment via a
  slider (the `TrajectoryMode._on_scrub` pattern) with the path drawn through
  `ctx.attempt_view`.
- **Run**: via `session._submit_async` (one-at-a-time with plan/refine/roadmap); disabled while
  `is_planning`; `on_done` refreshes the tree. Headless shim `StudioApp.run_task_now()` for the
  smoke test (the `*_now` convention).
- **Hand-off**: on success call `quevedomp_tasks.parametrize_result(...)` with Trajectory
  mode's current caps → set `session.trajectory` (timed playback works immediately) and publish
  a synthetic ok-`Attempt` via `ctx.show_attempt` so the EE curve draws. Trajectory mode's
  Parameterize button must delegate to `parametrize_result` while the last attempt is a task
  result (a single-session-spline refit across attach boundaries would be wrong); otherwise
  unchanged.

### C4. Studio tests (`test_smoke.py`)

Attach flips a collision verdict and clears `has_roadmap`/`has_clearance_field` + the baked
lever weights; a manual `robot.attach` without rebuild makes the next `collision_state` raise
(loudness asserted at the session layer); save→load round-trips an attachment; headless
`run_task_now()` pick-place succeeds and sets `session.trajectory`; detach restores the
obstacle at its placed pose.

---

## 5. Commit sequence (each commit leaves the tree green)

| # | Content | Gate |
|---|---------|------|
| 1 | A1: `attached_object.hpp`, `RobotInstance` API + `src/robot/robot_instance.cpp`, `test_attached_objects.cpp` | ctest dev-cpu |
| 2 | A2 (FCL): instance overload of `make_static_scene`, LinkShape/name/userdata rework, fingerprint + workspace-epoch guards, collision/ACM/witness tests | ctest dev-cpu |
| 3 | A2 (OptiX/Hybrid): snapshot ctor, attachment rays/slots, fcl_self_ snapshot, differential + guard tests | ctest dev-gpu |
| 4 | A3: lever-weights overload + tests | ctest dev-cpu |
| 5 | A4: serializer v2 + v1 back-compat tests | ctest dev-cpu |
| 6 | A5: bindings + `test_attached.py` | python_pytest |
| 7 | B1-B2: package skeleton, state model, TaskContext.ensure/rebuild/seeds, Sequence + PlanTo(JointTarget), linear runner, determinism tests | tasks pytest |
| 8 | B3+B5: PoseTarget branches, IkBranches, Alternatives, full DFS + budget + trace; backtracking tests | tasks pytest |
| 9 | B4: CartesianMove (slerp, tracking, jump guard, one-batch validation) + tests | tasks pytest |
| 10 | Attach/Detach stages + ensure-driven world mutation + the pick-place acceptance test | tasks pytest |
| 11 | B6: grouping, fit+parametrize per group, straightness verifier, `parametrize_result` + tests | tasks pytest |
| 12 | C1: session attach/detach/_rebuild_scene/_submit_async + persistence + session-layer smoke tests | studio pytest |
| 13 | C2: robot_view attachments + Scene-mode attach/detach controls | studio pytest |
| 14 | C3: Tasks mode inspector/runner + `run_task_now` + smoke tests | studio pytest |
| 15 | C4: Trajectory hand-off + smoke test | studio pytest |
| 16 | ADR-022, roadmap R7 record (incl. D6 refinement + measured numbers), READMEs (tasks pkg + studio modes section) | docs |

Commit style: follow the repo log (`feat(...)`/`refactor(studio)`/`docs(...)`, e.g.
`feat(robot): roadmap R7 — attached objects in RobotInstance (ADR-022)`).

## 6. Measured numbers to record (ADR-022 / roadmap record)

- Scene rebuild cost on attach: FCL and Hybrid, on UR5 **and** the hires inlet fixture (the
  OptiX env-GAS rebuild is the expensive part — measure it, don't guess).
- Pick-place acceptance: wall time, attempts, backtracks, rebuild count.
- Straightness: max measured deviation on the acceptance task at default tolerances.

## 7. Risks and mitigations

1. **Stale workspace = OOB UB**, not just wrong answers → workspace-epoch guard in every
   backend, tested on both (commit 2/3).
2. **OptiX rebuild cost on hires scenes** (full ctor re-runs the env GAS for an unchanged
   environment) → accept + measure (§6); record "factor env-GAS for reuse across rebuilds" as
   the follow-up optimization.
3. **Spline corner-rounding / Cartesian bowing** → per-group rest-to-rest + dense Cartesian
   sampling + loud straightness verifier (§B6).
4. **Session rebuild racing a plan** → `is_planning` refusal on attach/detach; rebuild + view
   mirroring under `ui_lock`; residual ordering slips become loud guard throws.
5. **Stale lever weights silently break the sweep guarantee** → `_rebuild_scene()` is the single
   choke point clearing both the cache and the baked `planner_params.lever_weights`; smoke test
   asserts it.
6. **`decompose_robot` sphere cover excludes attachments** → the R4 refiner's gradients ignore
   the carried part. SAFE (the exact-backend certificate still re-validates every edge) but
   record as a known limitation with a `decompose_robot(RobotInstance)` follow-up.
7. **ACM name collisions** → `attach()` throws on id == link name; id == env id is the same
   merge semantics as duplicate env ids today (document).
8. **Backtracking thrash** (rebuilds across Attach boundaries) → `ensure()` rebuilds only on
   actual state diff; the budget caps total work; rebuild count lands in `TaskResult.stats`.

## 8. Out of scope (say no in review)

Graphical task builder; dynamic backend geometry; PRM/env-GAS persistence across rebuilds
(recorded follow-ups); Cartesian path *constraints* in the planner (`Constraints` stays
joint-bounds-only); multi-robot; `.qmps` v3.

## 9. ADR-022 outline (`docs/architecture/adr-022-attached-objects-and-tasks.md`)

House style per adr-020/021. **Status**: Accepted (decisions ratified 2026-07-18/19).
**Context**: R7; both backends bake robot geometry at scene construction; no-silent-fallback
invariant. **Decision**: inline-Geometry `AttachedObject` on `RobotInstance`; rebuild-on-attach
with the **fingerprint** robot↔scene guard AND the **epoch** workspace↔scene guard (state why a
bare revision counter is insufficient: two instances can share a revision with different
attachments); name-carrying shapes (per-id ACM incl. attachment×env insertion, id-named
witnesses); attachments own their OptiX slots (per-object GPU masks); pure-Python
`quevedomp_tasks` (DFS/budget/derived seeds; solve_all-based pose resolution — P5 stays
parked); **per-group parameterization with rest-to-rest at gripper events** + straightness
verifier, alternatives recorded. **Verify**: test inventory + §6 numbers. **Consequences**:
attach cost = scene build (rare; env-GAS reuse recorded as follow-up); sphere-cover limitation
(risk 6); robot userdata now shape-index (internal). **Alternatives rejected**: dynamic
geometry, revision-only guard, one-spline stitching, C++ task layer, graphical builder.

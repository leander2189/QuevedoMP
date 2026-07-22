# QuevedoMP — Roadmap v1 (post-v0 feature plan)

> Ratified 2026-07-15 with Leandro. **Supersedes the remaining open items of
> `QuevedoMP-BUILD-PLAN.md`** — that document stays as the v0 record (everything in it is either
> ✅-with-evidence or explicitly parked below). The v0 goal ("beat MoveIt on high-poly scenes")
> is *parked, not abandoned*: Leandro benchmarks with the studio for now; the MoveIt container
> comparison can be revived when a claim needs it.
>
> **Standing invariants (carry over from v0 unchanged):** the planner performance contract
> (batch-first collision, determinism per seed, one Workspace per thread, no silent fallbacks),
> deviation D2 (apt-only deps; FetchContent only by ratified decision), ADR discipline for
> significant decisions, and recorded-numbers-not-vibes for every performance claim
> (versioned `.qmps` sessions + `session_profile.py`).

| # | Feature | Size | Status |
|---|---------|------|--------|
| R1 | High-poly inlet fixture + internal studio benchmark protocol (low-poly vs high-poly comparison; the GPU-crossover measuring stick) | S | ✅ 2026-07-16 (Leandro; see dtc_test_inlet/PROVENANCE.md — 7.3M tris, NOT watertight, baseline table recorded; the saved hires problem TIMES OUT and needs a look in the studio) |
| R2 | Studio: trajectory playback (real-time via `parametrize`), joint vel/acc + tip-speed plots, RRT exploration-tree snapshot view | S–M | ✅ 2026-07-15 |
| R3 | `ClearanceField` — GPU voxel SDF of the static environment (exact-seed + CUDA jump-flooding with equivalent OpenMP fallback, column-parity sign for watertight solids), conservative robot sphere cover, batched distance+gradient queries; analytic-SDF validation; studio clearance heatmap slice. **Separate type, NOT a `CollisionScene` extension.** | L | ✅ 2026-07-16 (ADR-018; hires inlet: 16.8M vox @10 mm in 1.34 s GPU JFA) |
| R4 | Optimization-based refiner (CHOMP/TrajOpt-flavored) over R3, registered behind the `Planner` interface (refiner + standalone modes, per build-plan Task 3.3c); exact-backend re-validation certificate | L | ✅ 2026-07-17 (ADR-019; `planning/refiner.{hpp,cpp}` `TrajectoryRefiner` + `make_refiner`, A⁻¹-preconditioned CHOMP, batched field gradients, exact certificate; C++ 206/206 + Python 64, studio `session.refine()`) |
| R5 | Roadmap/multi-query planner (PRM-flavored) for quasi-static cells: construction = unbounded fat batches (where the GPU finally wins outright), queries = graph search + P6 smoother, target single-digit ms/plan | M–L | ✅ 2026-07-17 (ADR-020; `planning/roadmap.{hpp,cpp}` `PrmPlanner` + `make_prm_planner` → (planner, PrmBuildStats), fat-batch node/edge build + A* query + P6 smooth; C++ 212/212 + Python 68, studio build_roadmap()/plan_roadmap() + "Roadmap (PRM)" panel) |
| R6 | Studio mode restructure — modal UI (Scene / IK / Plan / Trajectory / Tasks) grouped by role over a shared `StudioContext`; prerequisite for the R7 tasks inspector | S–M | ✅ 2026-07-18 (ADR-021; ratified with Leandro 2026-07-18 — app.py 961→~190 lines + `modes/` package, planners-as-peers Plan mode, `build_roadmap_async` + staleness props, studio smoke 34 passed) |
| R7 | Attached objects in `RobotInstance` (C++: grasped part moves collision geometry + ACM) → `quevedomp_tasks` MTC-lite layer in Python (Sequence/Alternatives, `PlanTo`, `IkBranches` via `solve_all`, `CartesianMove`, trajectory stitching + one final `parametrize`); studio Tasks mode becomes its inspector/runner (stage tree, per-stage preview) | M | — |
| R8 | Adaptive-step RRT-Connect — per-node extension step sized from the local width of free space (success-rate 1/5-rule and/or R3-clearance), so a tree rooted in a narrow pocket (typically the goal) can crawl out where a single global step cannot. Default off; explicit mode. | S–M | — (spec below, 2026-07-22) |

**Parked** (each with its recorded rationale): MoveIt baseline container (B.3) + goal-gate run
(Task 3.5) — revive per above; capture/replay v3 (design settled 2026-07-15: `.qmps` v3 with
recorded attempts, deterministic replay by seed, no core changes — build when wanted); P5
goal-sampling budget (wire `resolve_goal` to `solve_all` if ever needed); P7b GPU self-collision
(the "GPU frees the CPU" lever if deployments get core-starved); MCAP; OMPL cross-check; wheels +
notebook (Phase 4b polish).

## R8 spec (proposed 2026-07-22) — Adaptive-step RRT-Connect

> Not yet built. Groundwork landed this session (uncommitted): PRM connectivity diagnostics +
> roadmap-by-component view, studio RRT `extension step` / `goal bias` knobs, and the
> goal-escapability probe (`StudioSession.escapability`). Those turned the diagnosis concrete —
> the hard motions are an **isolated wedged goal**, not disjoint roadmap regions — which is what
> this feature targets.

**Motivation.** v0 RRT-Connect uses one global `max_extension` (0.5 rad) and grows all-or-nothing:
a candidate node is kept only if its *entire* steered edge is collision-free
(`src/planning/rrt_connect.cpp`, growth loop). One step size can't serve both regimes — large steps
explore open space fast but are rejected wholesale in tight regions, so a tree rooted in a narrow
pocket (empirically the **goal**) never takes a single step (its tree stays a lone root; the
escapability probe confirms the goal is free but can't move). Sizing the step **per node to the
local width of free space** — crawl where tight, stride where open — directly unblocks that case.

Prior art (for the record): Dynamic-Domain RRT (Yershova/Jaillet/Siméon/LaValle 2005) adapts a
per-node sampling radius from extension success/failure; Rechenberg's **1/5 success rule**
(evolution strategies) is the canonical success-rate step update; clearance-proportional step is the
model-based analogue. This spec takes both signals.

**Two adaptation signals** (per node *i*, keeping `step_i ∈ [min_extension, max_extension]`):

1. **Success-rate (model-free, registry-compatible).** Multiplicative 1/5-rule update after each
   batch validates extensions originating at node *i*:
   - extension from *i* succeeds → `step_i ← min(step_i · grow, max_extension)`
   - extension from *i* fails → `step_i ← max(step_i · shrink, min_extension)`
   with `grow ≈ 1.3`, `shrink ≈ 0.7`. Needs no extra inputs — a pure `rrt_connect` change.
2. **Clearance (model-based, needs the R3 field).**
   `step_i = clamp(clearance_gain · d(q_i), min_extension, max_extension)`, where `d(q_i)` is the
   robot's clearance at `q_i` (min over the sphere cover — exactly what `ClearanceField::query`
   returns). No cold start: at a wedged goal `d ≈ 0` ⇒ `step_i` floors immediately.
   **`ClearanceThenSuccess`** blend: seed `step_i` from clearance at insertion, then let the
   success-rate rule correct it during growth.

**New-node warm start.** A node inherits its parent's adapted step (not a reset to
`max_extension`), so a subtree threading a passage stays fine-grained.

**Params (PlannerParams additions).** `adaptive_extension: {Off, SuccessRate, Clearance,
ClearanceThenSuccess}` — default **Off** (existing behavior + benchmarks unchanged; no silent
fallback); `min_extension` (floor, default a small multiple of `edge_resolution`);
`extension_grow` / `extension_shrink` (success-rate); `clearance_gain` (m → rad). `max_extension`
stays the ceiling.

**Contract compliance (standing invariants).**
- *Batch-first collision*: unchanged — the batch of candidate extensions is still ONE
  `query_batch`; each candidate merely steers to its origin node's `step_i`. Clearance is one
  `ClearanceField::query` per growth step (or cached per node, since it's constant per config).
- *Determinism per seed*: preserved — step updates are deterministic functions of the batch
  results; the single-threaded RNG stream is untouched, so a seed reproduces the trees.
- *No silent fallback*: the mode is explicitly selected; `Off` is the default.

**Plumbing.** `SuccessRate` needs no new input → stays in `make_planner("rrt_connect", …)`.
`Clearance` / `ClearanceThenSuccess` need the field, which doesn't fit the registry's
`(params, robot, scene)` signature (same constraint as R4 refiner / R5 PRM) → a dedicated
`make_adaptive_rrt_connect(params, robot, scene, field)` factory for the field-backed modes; keep
the registry planner for the model-free mode.

**Known limitation (documented, not solved here).** Magnitude-only adaptation is partial: in a
narrow passage the free directions form a thin cone, so shrinking the step raises
success-per-attempt but still wastes samples on colliding directions. The full fix also biases
*direction* toward the clearance gradient `∇d` (available from R3) — left as a follow-on
("gradient-biased extension"); this spec is magnitude-only.

**Interplay.** Composes with the deferred **partial-extension** growth fix (keep the collision-free
prefix of a rejected step, then shrink `step_i`) and with `goal_bias`. The escapability probe is the
a-priori check on whether adaptivity can help at all: a **WEDGED** goal (no free step in any
direction) is a goal-pose/calibration problem, not one adaptivity can fix.

**Validation.** On the wedged-goal + narrow-passage studio scenes, compare `Off` vs each mode:
solve rate, time-to-first-solution, `collision_configs` (the contract metric), and tree sizes.
Determinism test (same seed ⇒ identical trees). `benchmark.qmps` numbers must be unchanged with the
default `Off`.

**Related deferred narrow-passage items** (siblings, not part of R8): partial-extension growth (keep
the free prefix); bridge-test PRM sampling (place roadmap nodes *inside* narrow gaps, the canonical
fix for the isolated-goal roadmap split the diagnostics surface).

## R6 record (2026-07-18)

- Ratified in discussion 2026-07-18: fully modal UI (only Session + the mode switcher stay
  global), features grouped **by role, not algorithm** (CHOMP standalone = a Plan-mode planner,
  CHOMP refine = a Trajectory-mode polisher; PRM = a planner choice; clearance heatmap = a Plan
  debug view), Tasks mode = future *inspector/runner* (not a graphical builder), restructure
  lands BEFORE the attached-objects work (old R6 → R7).
- `quevedomp_studio/context.py` (`StudioContext` + `AttemptView`) + `modes/{base,scene,ik,plan,
  trajectory,tasks,chomp_params}.py`; `app.py` keeps the server, Session panel, switcher, and the
  headless `*_now` entry points. Cross-panel widget reads eliminated; obstacle edits fire
  `scene_changed` → STALE markers via new `session.has_roadmap`/`has_clearance_field`;
  `build_roadmap_async` closes the unguarded inline-thread race. Full detail in ADR-021.

## R5 record (2026-07-17)

- Core: `planning/roadmap.{hpp,cpp}` — `PrmPlanner : Planner` + `make_prm_planner(params, robot,
  scene, out_stats)`. Build once (fat batches): sample `num_nodes` free configs; k-nearest
  (+radius) candidate edges deduped and validated in `edge_batch_configs`-capped fat batches; keep
  the free ones as a weighted adjacency. Query: resolve goal, connect start/goal to k-nearest nodes
  (+ direct start↔goal) in one batch, A* with a joint-space straight-line heuristic, P6
  shortcut-smooth. The path is collision-free by construction at the roadmap's `edge_resolution`
  (no separate certificate — contrast R4's approximate SDF).
- Collision semantics fixed by `PrmParams::collision` at build (query's own options ignored — they
  would invalidate prevalidated edges). Deterministic: roadmap per build seed, query per
  problem.seed; A* ties broken by node index.
- Plumbing: dedicated factory (R4 precedent) — build config doesn't fit flat `PlannerParams`;
  `"prm"` registered but `make_planner` throws a directive → `make_prm_planner`. Returns
  `(planner, PrmBuildStats)` (nodes/edges/configs/build_seconds).
- Studio: `StudioSession.build_roadmap()` / `plan_roadmap()` (cached roadmap, invalidated on env
  edits) + a threaded "Roadmap (PRM)" panel (Build + Query).

## R4 record (2026-07-17)

- Core: `planning/refiner.{hpp,cpp}` — `TrajectoryRefiner : Planner` + `make_refiner(params,
  robot, scene, field, spheres)`. CHOMP-flavored: smoothness (finite-difference acceleration) +
  CHOMP obstacle hinge over the R3 sphere cover against the ClearanceField, mapped to joint space
  by each sphere center's position Jacobian; update Q ← Q − step·A⁻¹·∇U with A = KᵀK factored once
  (the preconditioner that keeps obstacle pushes from kinking). Refiner mode (seeded) + standalone
  mode (straight line to a resolved goal); mode in `PlanningStats::refiner_mode`.
- Certificate: the field only supplies gradients — every output edge is re-validated as ONE exact
  `CollisionScene::query_batch`. Refiner mode never returns worse than its (re-certified) seed;
  standalone returns `NoSolution` on a local minimum (ADR-018/019).
- Contract: per-iteration clearance/gradient = ONE fat `ClearanceField::query` over all (waypoint ×
  sphere) points; deterministic per seed, bit-identical across thread counts.
- Registry: `"chomp"` is listed by `registered_planners()` but `make_planner` throws a directive
  error pointing at `make_refiner` (it needs the field + spheres the `(params, robot, scene)`
  signature can't carry — same reasoning as `make_shortcut_smoother`).
- Studio: `StudioSession.refine()` (cached ClearanceField + sphere cover, invalidated on
  environment edits).

## R2 record (2026-07-15)

- Core: `PlannerParams::record_tree` → `PlanningResult::trees` ([start, goal] `TreeSnapshot`s,
  one copy at plan exit, zero growth-loop cost — deliberately NOT the deferred live
  `PlanningTrace`).
- Session: `StudioSession.parametrize()` — C⁴ spline fit + collision re-validation at the
  planner's edge fidelity + Task 3.4 parameterization (JerkLimited when a jerk cap is set);
  `TimedTrajectory` with `sample(t)`; invalidated by the next plan.
- Studio: *Trajectory* panel (default accel / tip speed / tip accel / jerk caps → Parameterize),
  ▶ Play (timed) at × time-scale (the true velocity profile, unlike the constant-rate scrub),
  uplot panels for joint velocity/acceleration + tip speed (no plotly dependency), and a
  *record exploration tree* toggle drawing both trees as EE line clouds.

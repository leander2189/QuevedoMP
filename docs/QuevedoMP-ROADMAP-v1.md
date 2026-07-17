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
| R5 | Roadmap/multi-query planner (PRM-flavored) for quasi-static cells: construction = unbounded fat batches (where the GPU finally wins outright), queries = graph search + P6 smoother, target single-digit ms/plan | M–L | — |
| R6 | Attached objects in `RobotInstance` (C++: grasped part moves collision geometry + ACM) → `quevedomp_tasks` MTC-lite layer in Python (Sequence/Alternatives, `PlanTo`, `IkBranches` via `solve_all`, `CartesianMove`, trajectory stitching + one final `parametrize`) | M | — |

**Parked** (each with its recorded rationale): MoveIt baseline container (B.3) + goal-gate run
(Task 3.5) — revive per above; capture/replay v3 (design settled 2026-07-15: `.qmps` v3 with
recorded attempts, deterministic replay by seed, no core changes — build when wanted); P5
goal-sampling budget (wire `resolve_goal` to `solve_all` if ever needed); P7b GPU self-collision
(the "GPU frees the CPU" lever if deployments get core-starved); MCAP; OMPL cross-check; wheels +
notebook (Phase 4b polish).

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

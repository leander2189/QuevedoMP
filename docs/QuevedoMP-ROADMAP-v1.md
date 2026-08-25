# QuevedoMP Roadmap v1 — completion records (superseded)

> **Superseded by [`../ROADMAP.md`](../ROADMAP.md) on 2026-08-25.** That file is the live plan.
> This one is retained only for the R1–R6 completion records below, which are the evidence behind
> the "what works today" table there.
>
> The R8 specification that used to live here moved to
> [`QuevedoMP-R8-DESIGN.md`](QuevedoMP-R8-DESIGN.md).
>
> Original ratification: 2026-07-15 with Leandro, as the post-v0 feature plan superseding the open
> items of [`QuevedoMP-BUILD-PLAN.md`](QuevedoMP-BUILD-PLAN.md).

---

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

# ADR-020 — PrmPlanner: a PRM-flavored multi-query roadmap planner (roadmap R5)

**Status:** Accepted 2026-07-17 (roadmap R5; build-plan Task 3.2's "quasi-static cells make a
per-cell roadmap a natural candidate"). Builds on the planner performance contract, the P3 edge
discretization, and the P6 shortcut smoother.

## Context

The quasi-static-environment assumption that the OptiX GAS design and the ClearanceField (ADR-018)
exploit pays off a third way for planning: a **roadmap built once** over the static scene amortizes
across many queries. Construction — sampling free configs and validating candidate edges — is the
shape where fat batches finally dominate: thousands of edges × their samples make one giant
`query_batch`, exactly what the GPU backend wants (build-plan M4). Queries then reduce to graph
search + smoothing, targeting single-digit ms.

## Decision

**A `PrmPlanner` (`planning/roadmap.hpp`) built by `make_prm_planner(params, robot, scene)`,
returning a first-class `Planner`.**

### Build (once, at construction — the fat-batch phase)

1. **Nodes:** sample `num_nodes` configs in the bounded free space (URDF limits ∩
   `Constraints::joint_bounds`), validate them in fat batches, keep the collision-free ones.
   `num_nodes` is a *budget*, not a guaranteed free count (clutter thins the roadmap).
2. **Edges:** for each node, gather its k nearest neighbours (+ any within `connection_radius`) as
   candidate edges; dedup; validate them all in fat batches capped at `edge_batch_configs` configs
   each (so a huge roadmap validates as a few fat calls, not one oversized allocation); keep the
   free ones with weight = joint-space L2. k-NN is a linear scan — fine for v0 sizes; a kd-tree is
   the scale follow-up.

### Query (`plan()`, cheap + reentrant)

Resolve the goal to configs (JointGoal / PoseGoal-IK / MultiGoal, the shared resolver), validate
start + goals free, connect start and each goal to their k nearest roadmap nodes (+ a direct
start↔goal edge) in one batch, run **A\*** over (roadmap ∪ connections) with a joint-space
straight-line heuristic (admissible — edge weights are exact distances), then **P6 shortcut-smooth**
the extracted path. Every edge on the path was validated collision-free (roadmap at build,
connections at query), so the output is collision-free by construction at the roadmap's
`edge_resolution` — no separate certificate needed (contrast the R4 refiner, whose SDF is
approximate).

### Collision semantics fixed at build

The roadmap is validated under `PrmParams::collision`; those same options govern the query-time
connections. A query's own `PlanningProblem::collision` is deliberately **ignored** — using
different padding/margins would invalidate the prevalidated edges. Documented on the field.

### Plumbing — `make_prm_planner()`, not the `make_planner()` string registry

Like the R4 refiner (ADR-019), PRM gets a dedicated factory: its construction config
(`num_nodes`, connectivity, build seed) does not fit the flat `PlannerParams`, and building the
roadmap is heavy work that belongs explicitly at construction, not hidden in a registry lookup.
`make_prm_planner` returns `(planner, PrmBuildStats)` so callers see what the build produced.
`"prm"` **is** registered in `make_planner` so it is discoverable via `registered_planners()`, but
the stub throws a directive error pointing at `make_prm_planner` — discoverable, never a silent
fallback.

### Determinism

The roadmap is a pure function of (params, robot, scene, seed); a query is deterministic per
`PlanningProblem::seed` (goal IK + smoother seed). A\* breaks ties by node index.

## Verify (2026-07-17)

`tests/unit/test_prm.cpp` (6) + `bindings/python/tests/test_prm.py` (4): build stats populated;
solves a start→goal that must route around a wall (path independently collision-free at the
roadmap's own resolution — a finer check would flag grazes the planner never promised, the
discretization gap); ONE roadmap answers MANY queries; determinism per (build seed, query seed);
registry stub + null args fail loudly. Full C++ 212/212 (dev-cpu ASan/UBSan), Python 68 passed,
studio smoke 29 passed. clang-format clean. Studio: `StudioSession.build_roadmap()` /
`plan_roadmap()` (cached roadmap, invalidated on environment edits) + a "Roadmap (PRM)" UI panel.

## Consequences

- New `planning/roadmap.{hpp,cpp}`; no change to the `Planner` interface or existing planners.
- The build-time fat batches are where the ≥1M-tri Phase B fixtures should show the GPU winning
  outright (build-plan M4) — re-measure there. Query-time collision is thin (start/goal
  connections); the contract's fat-batch story for PRM lives in `PrmBuildStats`, not the per-query
  `PlanningStats` histogram.
- Scale follow-ups (noted in-source, not needed for R5): kd-tree k-NN; lazy edge evaluation (build
  optimistically, validate only edges on a candidate path — the build-plan alternative, better in
  heavy clutter); roadmap persistence across sessions.

## Alternatives rejected

- **Lazy PRM** (defer edge validation to query time): better in extreme clutter, but it moves the
  fat batches out of the amortized build and into every query — the opposite of R5's "construction =
  fat batches, queries = cheap" thesis. Kept as a documented scale option.
- **Eager edges in the flat `PlannerParams` via `make_planner`** — PRM's build config doesn't fit,
  and hiding a multi-second roadmap build inside a registry lookup surprises callers. The dedicated
  factory (R4 precedent) is clearer.

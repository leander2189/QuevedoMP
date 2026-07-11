# ADR-016 — Early minimal Python slice + `quevedomp-studio` (pure-Python IDE)

**Status:** Accepted 2026-07-11 (build-plan amendment M6; splits spec §6 Phase 4 into 4a/4b).

## Context

Debugging the pipeline features (IK, collision, planning, smoothing) currently requires writing
a C++ example per scenario (`examples/cpp/*_visualize.cpp`), rebuilding, and inspecting the
result in rerun. What's missing is an *interactive* surface: add robots and parts, drag them and
their goal poses, and re-run IK / collision / planning live — a "Motion Planning IDE".

The spec already commits to Python bindings (Phase 4, nanobind, ADR-002), but scheduled them
after capture/replay (Phase 3b) and as a full 1:1 mirror of the C++ API. An interactive IDE
built on those bindings raises two questions this ADR answers:

1. **Does a Python layer threaten the performance contract?** The project's headline goal is a
   ≤ 50 ms mean plan; the planner performance contract (build plan, 2026-07-08) is binding.
2. **Where does the IDE's UI come from?** rerun (already integrated) is a *viewer* — excellent
   for replay/inspection, but it has no transform gizmos or editing widgets.

## Decision

1. **Split Phase 4.** Phase 4a is a *minimal binding slice* over the API that exists at the
   Phase 3a exit — types, robot/scene construction, FK/IK/Jacobian, collision queries,
   planner + smoother. Phase 4b is the rest of the original Phase 4 (capture bindings, TOPP-RA
   once Task 3.4 lands, full pytest parity, the end-to-end notebook). Phase 4a may start as soon
   as Phase 3a exits; it does not gate — and is not gated by — Phase 3b.
2. **Bind at the "verb" level only.** Python calls whole operations (`plan`, `smooth`,
   `ik.solve`, `query_batch`, `check_edge`, `make_static_scene`); no Python callback is ever
   invoked from inside a C++ loop. Debug introspection of a run comes from `PlanningStats` and
   (later) the Phase 3b capture system — not from live hooks.
3. **Release the GIL on every bound call that can block** (`plan`, `smooth`, `solve`,
   `query_batch`, `check_edge`, `make_static_scene`, `load_mesh`, `from_urdf`). Threading
   follows ADR-005: one `Workspace` per querying thread; `Planner::plan` is const + reentrant.
4. **The IDE is `quevedomp-studio`** — a pure-Python app in `tools/quevedomp-studio/`, *outside*
   the C++ build. UI: **viser** (web-based 3D viewer with transform gizmos, sliders, buttons,
   scene tree) for interactive editing; **rerun** (via its own Python SDK, `rerun-sdk`) for
   logging every attempt for replay/inspection. The C++ `Visualizer` is *not* bound in 4a —
   studio logs to rerun directly from Python.

## Rationale

- **Zero cost to the core.** The bindings are an optional CMake target
  (`QUEVEDOMP_WITH_PYTHON=ON`) linking the same `libquevedomp` everyone else uses. No `#ifdef`,
  no virtual hook, no indirection is added to the core for Python's benefit. `OFF` builds are
  bit-identical to today.
- **Boundary crossings stay out of the hot path.** Per-call nanobind overhead (~100 ns–1 µs)
  only matters inside loops; verb-level binding keeps every IK iteration, collision batch, and
  RRT expansion inside one C++ call. Converting a `(N, dof)` numpy batch to
  `vector<JointPosition>` at the boundary is an O(batch) copy of a few tens of KB — noise next
  to the query itself. Genuinely large arrays (mesh vertices/triangles, `BatchResult` vectors,
  a result path) cross zero-copy or with a single copy.
- **GIL release is what makes the IDE feel interactive**: studio runs `plan()` on a worker
  thread while the UI thread keeps rendering; concurrent planners on one scene are already
  legal per the planner contract (item 4).
- **The IDE becomes the binding test harness.** Every core feature gains an interactive debug
  surface, and awkward-to-bind API shapes are discovered while the C++ API is still cheap to
  change — instead of at week 20.
- **viser + rerun, not one tool stretched.** viser gives editing (gizmos/widgets) in ~zero code;
  rerun keeps the deep timeline inspection it already provides for the C++ examples. Both are
  pip installs; neither touches the C++ build.

## Consequences

- `bindings/python/` lands early with four TUs (`bind_{types,robot,collision,planning}.cpp`)
  covering the Phase 3a surface; Phase 4b extends the same TUs rather than rewriting them.
- `tools/quevedomp-studio/` is a Python package with runtime deps `quevedomp`, `viser`,
  `rerun-sdk`, `numpy`. It is a dev tool: no C++ code, no CMake target, droppable from any
  performance milestone.
- Small per-call copies at the boundary (a 6-DOF `q`, a `Transform`) are accepted and
  documented; the zero-copy DoD applies to the large arrays listed above.
- Studio state (robot + ACM, scene objects) should serialize via the Task 2a.5 serializers so
  that, once Phase 3b lands, a capture bundle opens directly in studio ("bug in production run"
  → "open capture, tweak, re-plan" is one workflow).
- Risk: API churn between 3a and 4b means re-touching bindings. Accepted — the slice is thin by
  design, and churn discovered early is the point.

## Alternatives considered

- **Wait for full Phase 4 as scheduled.** No IDE until after capture/replay; binding-hostile API
  shapes discovered late. Rejected: the IDE is wanted *for* Phase 3 debugging.
- **Build the IDE in C++** (Qt/ImGui + custom 3D). Heaviest option by far; duplicates what
  viser/rerun give for free; UI iteration speed is the whole point of the tool. Rejected.
- **rerun alone as the UI.** No gizmos/editing widgets; would reduce the IDE to sliders in a
  sidebar driving re-logs. Kept for replay/inspection only.
- **Python callbacks for live planner introspection.** Puts Python inside the hot loop, breaks
  the performance contract, and duplicates what `PlanningStats` + capture already provide.
  Rejected; revisit only as an explicitly opt-in debug mode if capture proves insufficient.

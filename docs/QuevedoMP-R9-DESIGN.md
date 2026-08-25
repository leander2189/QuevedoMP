# QuevedoMP R9 Design — Dresskit Analysis (post-processing trajectory tool)

> **Audience: the implementing agent.** This document is self-contained: it records the ratified
> decisions, the verified code map (file:line as of commit `0ca2c48`; re-verify lines before
> editing — they drift), the finished architecture, and a commit-by-commit execution plan with
> gates. Follow it top to bottom. Where this document says THROW / FAIL LOUDLY, that is the
> project invariant "no silent fallbacks" — do not soften it.
>
> Designed with Leandro 2026-08-01. Records as the **next free ADR** when Phase B lands
> (outline in §9). At time of writing the highest ADR on disk is 021, with 022 reserved for R7
> and 023 expected for R8 — **re-check `docs/architecture/` and take the next free number.**

---

## 0. Context and goal

A **dresskit** (supply chain / umbilical) is the bundle of hoses and cables strapped along a
robot arm: a bracket on an upper link, a clamp at the wrist, slack in between. It is a leading
cause of real-world cell failures — the hose sweeps into a fixture, gets crushed below its rated
bend radius, or fatigues and cracks — and it is invisible to every planner in the pipeline
today, because it is not part of the URDF.

**R9 adds a dresskit *analysis* tool: post-processing over an already-planned trajectory.** It
takes a route spec, solves the hose's static equilibrium at each waypoint, reports clearance /
bend-radius / slack / fatigue metrics, and draws the draped hose in the studio.

### What this is deliberately NOT

It is **not** a planning constraint. Not a collision-scene participant. Not a certificate. That
was considered and **rejected** (§0, D1) — see the reasoning below, and §8.

### Ratified product decisions (do not relitigate)

| # | Decision | Rationale |
|---|----------|-----------|
| **D1** | **Post-processing only.** The dresskit never enters `CollisionScene`, `Planner`, or edge validation. It consumes a finished `Path` / `TimedTrajectory`. | Three independent reasons. (a) **Hysteresis**: a clamped-clamped elastic rod under gravity has *multiple stable equilibria* (buckling modes — a slack hose can loop left or right). Its shape is therefore not a function of `q` alone; it depends on the path taken. Admitting it into the planner would violate the axiom every C-space planner rests on, that `valid(q)` is single-valued. (b) **Batch-first**: warm-starting is inherently sequential along a path; the whole collision design is fat batches of *independent* configs. (c) The model cannot be validated well enough to be trusted as a gate — false positives/negatives would poison planning. As an *advisory* tool the same model is genuinely useful. |
| **D2** | **Pure Python + numpy.** New package `tools/quevedomp-dresskit/`. **Zero C++ changes.** | At N=30 the KKT system is ~109×109; a dense `np.linalg.solve` is ~50–150 µs. Live scrub needs one warm step (~0.3 ms → 60 fps); a 2000-waypoint sweep is ~0.6 s. Nothing here needs the core, and staying out of it means R9 cannot regress the planner. Matches the `quevedomp_tasks` dependency discipline (deviation D2: apt-only, numpy only). |
| **D3** | **Contact is a *violation*, not a *force*.** The solver computes the free-space equilibrium and ignores obstacles; rod-vs-world collision is then reported as a metric. | Putting contact in the solve makes it a complementarity problem: non-smooth, no clean derivatives, ~10× cost. And it is unnecessary — "the dresskit must not touch anything" is the actual industrial requirement, so the moment it touches, the answer is already "fail". |
| **D4** | **Clearance comes from `ClearanceField`**, not from `CollisionScene`. | ADR-018 already carved out this type's job as "approximate, gradient-bearing, serves optimization *and visualization*". An analysis tool is exactly that. It is voxel-approximate; the report and the UI must SAY SO (§3.4). The exact backend remains the only certificate — R9 never claims to be one. |
| **D5** | **Node positions with inextensibility constraints**, not hinge angles. | The hose is clamped at *both* ends. In reduced (hinge-angle) coordinates that is a shooting BVP: dense Jacobian, tip pose wildly nonlinear in the base angles, conditioning degrading with N. In node coordinates both Dirichlet BCs are free (freeze two nodes at each end), the Hessian and constraint Jacobian are banded, and — see §2.2 — the bending energy is *quadratic*, so the only nonlinearity in the whole problem is the segment-length constraint. |
| **D6** | **`length` (total hose length) is the headline parameter.** Stiffness and mass-per-length are presented as a sensitivity band, never as truth. | `EI` and `ρ` are effectively unmeasurable for a real bundled dresskit. `length` is the one thing the integrator actually chooses, and "what is the shortest hose that survives this cycle" (§3.3) is the question the tool exists to answer. Do not build a UI that implies the stiffness number is meaningful to three digits. |
| **D7** | **No twist.** The rod model has bending + gravity only. | Matches the agreed scope. It is a real omission — a hose clamped to a rolling wrist stores twist energy and corkscrews — so it is a **documented limitation** (§7.1), not a silent one. Adding it later is one scalar per edge (material frame + parallel transport, standard DER) and does not disturb this architecture. |
| **D8** | Old `.qmps` files MUST remain loadable, and `.qmps` written by an R9 build MUST load in a pre-R9 build. | Standing invariant (R7 D4). Routes go in under a new optional top-level key read with `.get(...)` — see §4.4. |

### Standing invariants that bind this work

Determinism (no RNG anywhere in this feature — assert *bit-identical* repeat runs); apt-only deps
(this package may depend on **numpy only**); no silent fallbacks; ADR discipline; recorded-numbers-
not-vibes (every performance claim in §6 measured and written down); the studio's one-workspace-per-
thread rule (ADR-005) and `ui_lock` discipline; `.clang-format` is irrelevant here (no C++), but
Python matches the existing studio/tasks style.

### Canonical build/test commands (from Windows, via WSL)

No C++ build is required for R9, but the bindings must exist on `PYTHONPATH`:

```bash
# Build the bindings once (if not already built):
wsl -d Ubuntu-24.04 -- bash -lc "docker run --rm -v /mnt/d/Inventos/quevedoMP:/work -w /work \
  quevedomp-cuda bash -lc 'cmake --preset release-py && cmake --build --preset release-py'"

# Dresskit package tests (the only gate for Phases A and B):
#   PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-dresskit \
#     python3 -m pytest tools/quevedomp-dresskit/tests -q
# Studio tests (Phase C):
#   PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio:tools/quevedomp-tasks:tools/quevedomp-dresskit \
#     python3 -m pytest tools/quevedomp-studio/tests -q
```

---

## 1. Verified code map (spot-checked; re-verify line numbers before editing)

**Python bindings (consumed, never modified)**
- `bindings/python/src/bind_clearance.cpp` — `ClearanceField.build(env, opts)` (:34-36);
  `distance(p)` (:37), `gradient(p)` (:39), **batched `query(points) -> (distances, gradients)`
  (:42)** — releases the GIL, this is the one R9 uses; `origin`/`resolution`/`dims`/`data`
  (:72-75); `decompose_robot(model, meshes, target_radius=0.05, max_spheres_per_geometry=8)`
  → `RobotSpheres` with `.spheres` (:94-97); `clearance_batch(field, model, spheres, qs)` (:103-114)
  — **per-config min over the ROBOT's spheres; not what R9 needs** (R9 needs the raw sphere
  set posed by FK, to test against rod nodes). Use `decompose_robot` + `fk_all` directly.
- `include/quevedomp/kinematics/fk.hpp` — `fk_all(model, q)` → per-link `Transform` (:17);
  `fk(model, q, link)` (:21). Both bound in Python (`q.fk_all`, `q.fk`).
- `include/quevedomp/core/types.hpp` — `Transform` wraps `Eigen::Isometry3d`; `.matrix()`,
  `.translation()`, `.rotation()`, `operator*` (:27-52).

**Studio (extended)**
- `tools/quevedomp-studio/quevedomp_studio/session.py` — `StudioSession` owns `model`, `robot`,
  `mesh_sources`, `q`; `trajectory: Optional[TimedTrajectory]` = last `parametrize()` output
  (:194); **`clearance_field(resolution=0.02, force=False)`** lazily builds + caches, invalidated
  on every env edit (:498-510, :368/:376/:381); `has_clearance_field` (:627); `environment()`
  (:384); `link_poses(q_at)` = `fk_all` (:245); `refine_async` (:477) is the **async precedent to
  copy** for the sweep; `save`/`load` (:738/:783) — JSON, the persistence hook for §4.4.
- `.../session.py` `TimedTrajectory` (:99-119) — `times`, `positions (n,dof)`, `velocities`,
  `accelerations`, `duration`, `sample(t)`. `max_node_speed` needs `times`; a bare `Path` has none.
- `.../context.py` — `StudioContext`; **`config_listeners` (:170)** fires from scrub, play and IK
  via `set_config` (:178-182) — this is how the hose follows the scrub slider from any mode;
  `scene_changed_listeners` (:171) → invalidate the field/report; `attempt_listeners` (:172) →
  invalidate the report when a new plan lands. `ui_lock` for scene mutation.
- `.../modes/base.py` — `Mode` with `name`, `title`, `build()`, `set_active(active)`,
  `shutdown()`. Modes are built once per `_mount` and discarded wholesale on `load_session`.
- `.../modes/__init__.py` — flat re-export list; add `DresskitMode`.
- `.../app.py` — `MODE_LABELS` + `_mode_switcher` button group (:46-48), mode construction
  (:50-58), `self._modes` list (:60), `_set_mode` (:74-79), `shutdown` loop (:90-91). Five call
  sites, all trivial.
- `.../primitives.py` — **`cylinder_mesh(radius, length, n=24)` (:61)**, "*axis along +Z, centered
  at the origin*" — exactly the convention a rod segment wants. Everything renders through viser's
  `add_mesh_simple`.
- `.../modes/trajectory.py` — the `uplot` plotting pattern (`_draw_plots`, :109-151) and the
  scrub/playback widgets (:51-58). R9's plots copy this shape.

**Nothing in `include/`, `src/`, or `bindings/` is modified by R9.**

---

## 2. Phase A — the solver package (`quevedomp_dresskit`)

### A0. Package layout

```
tools/quevedomp-dresskit/
  pyproject.toml                # name quevedomp-dresskit, deps: numpy only (D2); quevedomp
                                # comes from the build tree via PYTHONPATH (studio precedent)
  quevedomp_dresskit/
    __init__.py                 # public surface: DresskitRoute, solve_rod, RodShape,
                                #   sweep_trajectory, DresskitReport, min_feasible_length,
                                #   swept_envelope, errors
    route.py                    # DresskitRoute + clamp-frame resolution from FK
    solver.py                   # solve_rod(): the SQP. The heart of the package.
    metrics.py                  # per-waypoint metrics (clearance, bend radius, slack, speed)
    sweep.py                    # sweep_trajectory(), DresskitReport, Violation, verdict
    study.py                    # min_feasible_length() bisection, swept_envelope()
    errors.py                   # DresskitRouteError, DresskitSolveError
  tests/
    test_solver.py  test_analytic.py  test_metrics.py  test_sweep.py  test_study.py
```

### A1. The route spec (`route.py`)

```python
@dataclass(frozen=True)
class DresskitRoute:
    name: str
    # --- attachment ------------------------------------------------------------------
    base_link: str
    base_origin: "q.Transform"    # bracket pose in base_link frame; +Z = hose exit direction
    tip_link: str
    tip_origin: "q.Transform"     # clamp pose in tip_link frame;  +Z = hose ENTRY direction,
                                  #   i.e. pointing back along the hose toward the base
    # --- geometry --------------------------------------------------------------------
    length: float                 # total hose length (m) — THE knob (D6)
    segments: int = 30            # N
    radius: float = 0.025         # m, hose outer radius, used for clearance
    # --- material (sensitivity band, not truth — D6) ----------------------------------
    bend_stiffness: float = 1.0   # EI (N·m²)
    mass_per_length: float = 0.5  # kg/m
    min_bend_radius: float = 0.15 # m — manufacturer spec; violating it destroys the hose
    gravity: tuple = (0.0, 0.0, -9.81)
```

The two `Transform`s carry **position and tangent**. That is the whole point: a clamped rod needs
both, and freezing two nodes at each end delivers both with no constraint rows.

`resolve_clamps(route, model, q_at) -> (x0, x1, xN_1, xN)`:

```
L      = route.length / route.segments
T_base = fk(model, q_at, route.base_link) * route.base_origin
T_tip  = fk(model, q_at, route.tip_link)  * route.tip_origin
x0     = T_base.translation()
x1     = x0     + L * T_base.rotation()[:, 2]      # +Z column = exit tangent
xN     = T_tip.translation()
xN_1   = xN     + L * T_tip.rotation()[:, 2]       # +Z column = entry tangent
```

> **Do not skip this:** because `x1` is *constructed* at distance `L` from `x0` (and likewise
> `x_{N-1}` from `x_N`), the segment constraints `c_0` and `c_{N-1}` are satisfied **by
> construction and are not functions of any free variable**. They MUST be dropped from the
> constraint set or the KKT matrix is structurally singular. Constraints run over segments
> `1 .. N-2` only (count `N-2`).

**Validation (throw `DresskitRouteError`):** `segments >= 6`; `length > 0`; `radius > 0`;
`base_link` and `tip_link` exist in the model and are distinct; `min_bend_radius > 0`.

### A2. The solver (`solver.py`)

**Variables.** Nodes `x_0 … x_N ∈ ℝ³`. Frozen: `x_0, x_1, x_{N-1}, x_N`. Free: `x_2 … x_{N-2}`
→ `3(N-3)` variables. For N=30: 81 free variables, 28 constraints, KKT 109×109.

**Energy.** With discrete curvature `κ_i ≈ ‖x_{i-1} - 2x_i + x_{i+1}‖ / L²` at interior nodes:

```
E_bend(x) = (EI / 2L³) · Σ_{i=1..N-1} ‖x_{i-1} - 2x_i + x_{i+1}‖²      # QUADRATIC in x
E_grav(x) = Σ_i m_i · (-g · x_i),   m_i = ρL (half at the two end nodes) # LINEAR in x
E(x)      = ½ xᵀ H_b x + gᵀx
```

> `E_bend` is quadratic, so `H_b` is **constant** — build it once per route, not per solve.
> `E_grav` is linear, so its Hessian is zero. **The only nonlinearity in the entire problem is
> the segment-length constraint.** Exploit this.

**Constraints.** `c_j(x) = ‖x_{j+1} - x_j‖² - L² = 0` for `j = 1 … N-2`. (Squared, not the norm:
smooth everywhere including at zero, and `∇²c_j` is a constant ±2I block pattern on nodes
`j, j+1`.)

**SQP step.** Each iteration solves the equality-constrained Newton (KKT) system

```
⎡ W   Aᵀ ⎤ ⎡ Δx ⎤   ⎡ -∇E(x) - Aᵀλ ⎤
⎢        ⎥ ⎢    ⎥ = ⎢               ⎥ ,   W = H_b + Σ_j λ_j ∇²c_j ,  A = ∇c(x)
⎣ A   0  ⎦ ⎣ Δλ ⎦   ⎣ -c(x)         ⎦
```

- `W` is banded (bandwidth ~9); `A` is banded. Assemble dense (`np.zeros((n+m, n+m))`) — at this
  size dense `np.linalg.solve` beats any pure-numpy banded scheme, and scipy is not available.
  Revisit only if §6 says otherwise.
- **Inertia correction.** If the factorization fails or `W` is not positive definite on the null
  space of `A`, add `δI` to `W` with `δ` doubling from `1e-8·tr(H_b)/n`. **Record that `δ > 0`
  was needed** — see the next bullet.
- **`multimodal_flag`.** A required inertia correction means the reduced Hessian lost positive
  definiteness — i.e. the rod is at or near a **buckling point, where the equilibrium is not
  locally unique**. This is precisely the regime where D1's hysteresis objection bites and the
  answer is untrustworthy. Surface it per waypoint, all the way to the UI. Do not hide it.
- **Line search.** Armijo on the ℓ1 merit function `φ(x) = E(x) + μ‖c(x)‖₁`, with
  `μ = 10·max|λ|` (floored at 1.0), backtracking `α ← 0.5α`, max 20 backtracks.
- **Convergence.** `‖∇ₓℒ‖_∞ < 1e-8` **and** `‖c‖_∞ < 1e-10·L²`. Max 100 iterations; exceeding it
  throws `DresskitSolveError` (no silent "best effort" return).

**Cold-start initialisation** (deterministic, no RNG):

1. If `‖x_N - x_0‖ >= length` → the hose is physically too short. THROW `DresskitSolveError`
   naming the shortfall. Do not return a stretched rod.
2. Otherwise lay a **planar circular arc** in the plane spanned by `(x_N - x_0)` and the gravity
   direction, chord `‖x_N - x_0‖`, arc length `length`. Solve the half-angle `θ` from
   `sin θ / θ = chord / length` by bisection on `θ ∈ (0, π)` (monotone; 60 iterations is exact to
   machine precision). Sample `N+1` points along it, then overwrite the four frozen nodes.

**Warm start.** `solve_rod(route, model, q_at, warm_start: Optional[RodShape])` — when given, use
its free nodes as `x⁰` (after overwriting the four clamps from the new FK). This is the sequential
path along a trajectory and is where the 2–3-iteration cost lives.

**Output:**

```python
@dataclass(frozen=True)
class RodShape:
    nodes: np.ndarray        # (N+1, 3) world positions
    curvature: np.ndarray    # (N-1,) κ at interior nodes, 1/m
    iterations: int
    multimodal: bool         # inertia correction was required
    residual: float          # final ‖∇ₓℒ‖_∞
```

### A3. Phase-A tests (`test_solver.py`, `test_analytic.py`)

Analytic oracles — this is how the model earns trust, since there is no ground truth otherwise:

1. **Catenary.** `bend_stiffness → 0` (use 1e-9), both clamps at equal height, tangents vertical,
   `N = 200`. Compare against the analytic catenary `z = a·cosh((s - s₀)/a) + c` fitted on the
   span. Assert max relative deviation `< 1e-3`.
2. **Taut limit.** `length == ‖x_N - x_0‖ · (1 + 1e-9)`, zero gravity → straight line; assert max
   deviation from the chord `< 1e-6` and `E_bend ≈ 0`.
3. **Constant-curvature arc.** Zero gravity, slack, tangents symmetric about the chord bisector →
   the minimiser is a circular arc. Assert `std(κ) / mean(κ) < 1e-4`.
4. **Symmetry.** Clamps mirror-symmetric about a plane containing gravity → assert the solution is
   symmetric to `1e-9`.
5. **Inextensibility.** Every segment length equals `L` to `1e-10` in all of the above.
6. **Warm-start cost.** Perturb `q` by 0.05 rad (the default edge resolution) and re-solve warm;
   assert `iterations <= 4`. This assertion is the performance contract — if it ever fails, the
   §6 numbers are void.
7. **Determinism.** Solve the same input twice; assert `nodes.tobytes()` equality. There is no RNG
   in this package; keep it that way.
8. **Too-short hose throws** `DresskitSolveError` with the shortfall in the message.

---

## 3. Phase B — sweep, metrics and studies

### B1. Metrics (`metrics.py`)

Per waypoint `i`, given `RodShape` and the robot's FK at that waypoint:

| Metric | Definition | Notes |
|---|---|---|
| `env_clearance[i]` | `min(field.query(P)[0]) - radius` | `P` = rod nodes **plus `k=3` interpolated points per segment** — with `L ≈ 3–5 cm` and a 1–2 cm voxel grid, node-only sampling walks straight through thin obstacles. One batched `field.query` call for the whole waypoint. |
| `robot_clearance[i]` | `min over (node, sphere) of ‖p_n - c_s‖ - radius - r_s` | Spheres from `decompose_robot(model, mesh_sources)` posed by `fk_all`. **Exclude spheres on `base_link` and `tip_link`** — the hose is *supposed* to touch its own brackets. numpy broadcast, ~31×50 pairs ≈ 20 µs. |
| `min_bend_radius[i]` | `1 / max(κ)` | Violation when `< route.min_bend_radius`. Falls out of `RodShape.curvature` for free. |
| `slack_util[i]` | `‖x_N - x_0‖ / length` | `≥ 1.0` is physically impossible (the solver already threw); `> 0.98` is the practical warning band. |
| `max_node_speed[i]` | `max_n ‖x_n(t_{i+1}) - x_n(t_i)‖ / Δt` | Requires `TimedTrajectory.times`. **`None` for a bare `Path`** — do not fabricate a time base. |
| `curvature_cycle[j]` | `Σ_i \|κ_j(i+1) - κ_j(i)\|` | Per-*segment*, not per-waypoint: a fatigue proxy telling you **which section of the hose cracks first**. |

### B2. `sweep_trajectory()` (`sweep.py`)

```python
def sweep_trajectory(route, model, traj, field, spheres, *, mesh_sources=None,
                     progress=None) -> DresskitReport
```

`traj` is a `TimedTrajectory` **or** an `(n, dof)` array of configurations. Walk waypoints in
order; waypoint 0 cold-starts, every subsequent one warm-starts from the previous `RodShape`.
That sequential chain is the entire performance story (§6) — do not "optimise" it into a parallel
cold-start loop.

```python
@dataclass
class Violation:
    metric: str          # "env_clearance" | "robot_clearance" | "min_bend_radius" | "slack_util"
    first: int; last: int # inclusive waypoint interval
    worst_index: int; worst_value: float

@dataclass
class DresskitReport:
    route: DresskitRoute
    times: Optional[np.ndarray]        # (n,) or None
    shapes: np.ndarray                 # (n, N+1, 3)
    env_clearance: np.ndarray          # (n,)
    robot_clearance: np.ndarray        # (n,)
    min_bend_radius: np.ndarray        # (n,)
    slack_util: np.ndarray             # (n,)
    max_node_speed: Optional[np.ndarray]
    curvature_cycle: np.ndarray        # (N,)
    multimodal: np.ndarray             # (n,) bool
    iterations: np.ndarray             # (n,) int
    violations: list[Violation]
    verdict: str                       # "clear" | "marginal" | "violated" | "untrustworthy"
    field_resolution: float            # STAMPED — the report is only as good as this
```

**Verdict rules.** `violated` if any `Violation` exists; else `marginal` if
`min(env_clearance) < 2·field_resolution` (inside the field's own error bar) or
`max(slack_util) > 0.98`; else `untrustworthy` if `multimodal.any()`; else `clear`.
`untrustworthy` ranks *below* `clear` and *above* `marginal` deliberately — a buckling flag does
not make a violation go away, but it does void a clean bill of health.

### B3. Studies (`study.py`)

**`min_feasible_length(route, model, traj, field, spheres, *, tol=0.005) -> (float, DresskitReport)`**
— the payoff of D6. Bisect `length` on `[lo, hi]`:

- `lo = max_i ‖x_N(i) - x_0(i)‖ · 1.001` — the hard geometric floor (shorter cannot reach), taken
  from FK alone, no solves needed.
- `hi = route.length` if it already passes, else `2·lo` (and THROW if that fails too — the route
  is unsalvageable and the user needs to hear it, not get a made-up number).
- Predicate: `sweep_trajectory(...).verdict in {"clear", "untrustworthy"}` — note `marginal`
  counts as **pass** here, since bisecting to the edge of the field's error bar is the whole point;
  the returned report carries the caveat.
- ~10 bisections × 0.6 s ≈ **6 s**. Async in the studio (§4.3).

**`swept_envelope(report, *, voxel=None) -> np.ndarray`** — `(m, 3)` point cloud, the union of all
rod nodes over the trajectory, voxel-downsampled (default `voxel = field_resolution`) by
`np.unique` on the integer grid indices. **A point cloud, not a convex hull** — scipy is not
available and a hull would be wrong anyway (the envelope is non-convex). This is the keep-out
volume the cell designer needs and the single most legible artifact the feature produces.

### B4. Phase-B tests

- `test_metrics.py`: a hand-placed box obstacle at a known distance → assert `env_clearance`
  matches within the voxel resolution; a rod bent to a known radius → assert `min_bend_radius`;
  `max_node_speed is None` for an array input; brackets excluded from `robot_clearance`.
- `test_sweep.py`: monotone warm-start iteration counts (assert `iterations[1:].max() <= 4`);
  verdict transitions across a deliberately-colliding trajectory; violation intervals are
  contiguous and correctly bounded; **determinism** (two sweeps → identical `shapes.tobytes()`).
- `test_study.py`: `min_feasible_length` on a synthetic case with a known answer; asserts the
  result is `>= lo`; asserts the unsalvageable case throws.

---

## 4. Phase C — studio integration

### C1. `DresskitMode` (`modes/dresskit.py`)

`name = "dresskit"`, `title = "Dresskit"`. Registered in `modes/__init__.py` and in `app.py`
(`MODE_LABELS`, the constructor block, `self._modes`) — five one-line edits (§1).

Widgets, in one folder:

- **Route** — `add_dropdown` over `session.dresskits` names + `Add` / `Remove` buttons.
- **Attachment** — `base_link` / `tip_link` dropdowns over `session.model.links`; six numbers each
  for the origin (xyz + rpy). A transform gizmo is a nice-to-have, **not** v1.
- **Geometry** — `length`, `segments`, `radius`, `min_bend_radius`.
- **Material** (collapsed sub-folder, labelled *"estimates — see sensitivity"*, D6) —
  `bend_stiffness`, `mass_per_length`.
- **Actions** — `Solve here` (current config), `Sweep trajectory`, `Min feasible length`,
  `Show swept envelope` (toggle).
- **`follow config`** checkbox — when on, registers a `ctx.config_listeners` callback that
  re-solves warm and redraws. This is the live drape as you scrub, and because the listener is
  cross-mode it keeps working while the user is in Trajectory mode driving the scrub slider.
- **Status** `add_text` (disabled) + a `Violations` text block.

### C2. Rendering (`robot_view.py` or mode-local)

One scene node per segment: `/dresskit/<name>/seg_<i>`, created **once** in `build()` via
`add_mesh_simple(cylinder_mesh(radius, L))` and thereafter updated **in place** through the
handle's `.position` / `.wxyz`. Do **not** remove-and-re-add 30 nodes per scrub tick — that is the
difference between 60 fps and a slideshow.

Segment `i` pose: position = midpoint `(x_i + x_{i+1})/2`; orientation = the quaternion rotating
`+Z` onto `(x_{i+1} - x_i)` normalised (`cylinder_mesh` is +Z-aligned and origin-centred, so this
is exact). Colour by clearance: green above `4·field_resolution`, ramping to red at 0, and a
distinct colour for a `min_bend_radius` violation.

Swept envelope: one `add_point_cloud` at `/dresskit/<name>/envelope`, visibility on the toggle.

Plots: copy `TrajectoryMode._draw_plots`' `uplot` pattern — clearance (env + robot on one axis,
with the `field_resolution` error band drawn as a horizontal marker), `min_bend_radius` against
its spec line, `slack_util`, and the per-segment `curvature_cycle` bar.

### C3. Threading

A sweep is ~0.6 s and `min_feasible_length` ~6 s — both must run off the viser callback thread.
Copy `session.refine_async` (`session.py:477`) exactly: worker thread, `ui_lock` around scene
mutation, button relabelled while running, result delivered through the existing done-callback
shape. The live `follow config` solve is ~0.3 ms and stays inline.

Invalidation: register `ctx.scene_changed_listeners` (env edit → the `ClearanceField` is already
dropped by the session, so drop the report too) and `ctx.attempt_listeners` (new plan → report is
stale). Show staleness in the status text; never serve a stale report silently.

### C4. Persistence (`session.py`)

`StudioSession.dresskits: list[DresskitRoute]`, serialised under a **new optional top-level key**:

```python
# save(): blob["dresskits"] = [route_to_json(r) for r in self.dresskits]
# load(): [route_from_json(b) for b in blob.get("dresskits", [])]      # D8 — .get, always
```

Reuse the existing `_tf_to_json` / `_tf_from_json` helpers (`session.py:27/31`). A pre-R9 build
loading an R9 file ignores the unknown key; an R9 build loading an old file gets `[]`. Both
directions tested.

### C5. Studio tests (`test_smoke.py`)

Headless: add a route → `Solve here` → assert 31 scene nodes exist; `Sweep trajectory` on a short
planned trajectory → assert a report lands and the status text is populated; save → load →
assert routes round-trip; load a **pre-R9 fixture** → assert no throw and `dresskits == []`;
assert `shutdown()` stops the sweep thread.

---

## 5. Commit sequence (each commit leaves the tree green)

| # | Content | Gate |
|---|---------|------|
| 1 | A0-A1: package skeleton, `pyproject.toml`, `DresskitRoute`, `resolve_clamps`, route validation + tests | dresskit pytest |
| 2 | A2: `solve_rod` — energy assembly, KKT step, inertia correction, merit line search, arc init, warm start | dresskit pytest |
| 3 | A3: the analytic oracle suite (catenary, taut, arc, symmetry, inextensibility, warm-start cost, determinism) | dresskit pytest |
| 4 | B1: `metrics.py` + tests | dresskit pytest |
| 5 | B2: `sweep_trajectory`, `DresskitReport`, violations, verdict + tests | dresskit pytest |
| 6 | B3: `min_feasible_length`, `swept_envelope` + tests | dresskit pytest |
| 7 | C4: session `dresskits` + save/load + back-compat fixture test | studio pytest |
| 8 | C1+C2: `DresskitMode`, app.py registration, segment rendering, `follow config` live solve | studio pytest |
| 9 | C3: async sweep + study, invalidation listeners, plots, envelope toggle | studio pytest |
| 10 | ADR (next free number), roadmap R9 record with §6 numbers, package README, studio README modes section | docs |

Commit style follows the log (`feat(dresskit): ...`, `feat(studio): ...`, `docs: ...`), e.g.
`feat(dresskit): roadmap R9 — static rod solver for supply-chain analysis (ADR-0NN)`.

## 6. Measured numbers to record (ADR / roadmap record)

Estimates in this document are **arithmetic, not measurements** — the invariant is
recorded-numbers-not-vibes, so commit 3 and commit 5 must produce the real ones:

- Cold-start and warm-start iteration counts and wall time, at `N ∈ {15, 30, 60}`.
- Full-sweep wall time on a real planned trajectory (state the waypoint count), split into
  solve / clearance-query / metric time. **Predicted: ~0.6 s at n=2000, N=30 — verify.**
- `min_feasible_length` wall time and bisection count.
- Live `follow config` latency (the 60 fps claim).
- Fraction of waypoints flagged `multimodal` on a real trajectory — this is the honest measure of
  how much D1's hysteresis objection bites in practice, and it is worth knowing.

## 7. Risks, mitigations and stated limitations

### 7.1 Documented limitations (state these in the README and the UI, not just here)

1. **No twist (D7).** A hose clamped to a rolling wrist corkscrews; this model does not see it.
2. **Contact is not modelled (D3).** The hose does not drape over obstacles — it passes through
   them and the tool reports the intersection. Post-contact shapes are meaningless.
3. **Multiple equilibria (D1).** The reported shape is "the equilibrium in the basin of the
   canonical warm start". Single-valued by construction, but path-dependent in reality. The
   `multimodal` flag marks where that fiction is thinnest.
4. **`ClearanceField` is voxel-approximate (D4).** Clearances carry `field_resolution` as their
   error bar. **This is an analysis tool, never a certificate** — the exact `CollisionScene`
   backend remains the only one of those.
5. **Material parameters are estimates (D6).** Report a sensitivity band, not a number.

### 7.2 Risks

| # | Risk | Mitigation |
|---|---|---|
| 1 | **Singular KKT** from including the two construction-satisfied constraints | §A1's explicit note; the constraint set is `1..N-2`. A rank test on `A` in the tests. |
| 2 | Solver diverges / hits the iteration cap on a hard waypoint | THROW `DresskitSolveError` naming the waypoint index; the sweep surfaces it as a hard failure, never a silent skip or a "best effort" shape. |
| 3 | Node-only clearance sampling walks through thin obstacles | `k=3` interpolated points per segment (§B1); assert it in a test with a plate thinner than `L`. |
| 4 | 30 viser nodes per scrub tick tanks the frame rate | Create once, update `.position`/`.wxyz` in place (§C2). Measure (§6). |
| 5 | Sweep blocks the viser callback thread | `refine_async` pattern, `ui_lock` for scene mutation (§C3). |
| 6 | Stale report served after an env edit or a re-plan | `scene_changed_listeners` + `attempt_listeners` invalidate; staleness shown in the status text. |
| 7 | `.qmps` round-trip breaks old sessions | `.get("dresskits", [])` on load; both-direction back-compat tests (§C4/C5). |
| 8 | Users read the tool as a certificate | Verdict vocabulary is advisory (`clear`/`marginal`/`violated`/`untrustworthy`), `field_resolution` is stamped on every report, and the UI says "analysis, not certificate". |
| 9 | Field build cost surprises the user | `session.clearance_field()` is cached and lazily built; if `has_clearance_field()` is false, the status must say a build is about to happen and at what resolution. |

## 8. Out of scope (say no in review)

Planning integration of any kind — no `CollisionScene` participation, no edge validation, no
refiner cost term, no `Planner`-visible constraint (D1). Contact/friction forces. Twist and
torsional buckling. Dynamics (the model is quasi-static; "whipping" is a *proxy* metric only).
Multiple interacting dresskits (routes are independent; no rod-rod contact). Self-collision of a
rod with itself. Automatic route inference from a URDF. C++ implementation (D2) — revisit only if
§6 contradicts the estimates by an order of magnitude. Graphical route editing beyond the numeric
fields in §C1.

## 9. ADR outline (`docs/architecture/adr-0NN-dresskit-analysis.md`)

House style per adr-019/020/021. **Status**: Accepted (decisions ratified 2026-08-01).
**Context**: R9; dresskits cause real cell failures and are invisible to the pipeline; a clamped
elastic rod is cheap to solve but its equilibrium is multi-valued and its parameters are
unmeasurable. **Decision**: a *post-processing analysis tool*, deliberately outside the planner
(D1 — state the hysteresis argument in full, it is the load-bearing one); pure Python/numpy with
zero core changes (D2); contact as violation, not force (D3); `ClearanceField` as the clearance
source with its approximation stamped on every report (D4); node coordinates with inextensibility
constraints, exploiting the quadratic bending energy so the only nonlinearity is the length
constraint (D5); `length` as the headline parameter and `min_feasible_length` as the flagship
output (D6); no twist, documented (D7). **Verify**: the analytic oracle suite (§A3) + §6 numbers.
**Consequences**: a genuinely useful advisory tool that cannot regress planning, at the cost of
never being authoritative; the `multimodal` flag is the honest seam. **Alternatives rejected**:
planner integration (hysteresis + batch-first + unvalidatable), hinge-angle coordinates (shooting
BVP), contact forces in the solve (complementarity), C++ implementation (unnecessary at this size),
convex-hull envelope (non-convex, and no scipy).

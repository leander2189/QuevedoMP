# ADR-019 — TrajectoryRefiner: CHOMP/TrajOpt optimizer over the ClearanceField (roadmap R4)

**Status:** Accepted 2026-07-17 (roadmap R4; build-plan Task 3.3c). Builds on ADR-018
(ClearanceField) and the planner performance contract.

## Context

Sampling planners give *feasible*; "smooth and logical with clearance margins" is manufactured by
an optimization pass that consumes clearance **gradients**. ADR-018 built the `ClearanceField` (a
voxel SDF of the static environment) precisely to serve this. R4 is the consumer: a gradient-based
trajectory optimizer, exposed as a first-class `Planner`, selected explicitly — never an automatic
fallback (the standing anti-fallback contract).

Two forces shaped the design: (1) the optimizer must obey the performance contract (batched
clearance lookups, deterministic per seed); (2) the field is *approximate* (voxel-bounded), so it
can never be the collision certificate — the exact backend is (ADR-018's standing rule).

## Decision

**A `TrajectoryRefiner` (`planning/refiner.hpp`), CHOMP/TrajOpt-flavored, built by a dedicated
`make_refiner()` factory, certified by the exact `CollisionScene`.**

### Algorithm

Optimize a fixed-endpoint trajectory ξ = [q₀ … q_{M-1}] (endpoints clamped) to minimize
U = w_s·F_smooth + w_obs·F_obs:

- **Smoothness** F_smooth = ½ Σ‖q_{i-1} − 2q_i + q_{i+1}‖² (summed squared finite-difference
  acceleration). Gradient is the discrete biharmonic gₛ = A·Q + b with A = KᵀK the pentadiagonal
  smoothness metric (K = tridiag(−2; +1)).
- **Obstacle** F_obs sums a CHOMP hinge over the robot's conservative sphere cover (ADR-018's
  `decompose_robot`) against the field: cost 0 beyond ε clearance, quadratic in [0, ε], linear when
  penetrating. The workspace gradient (cost′·∇dist) is mapped to joint space by each sphere
  center's **position Jacobian** Jₚ = J_v − [x − o]×·J_w (geometric Jacobian of the link, shifted
  to the sphere center).
- **Update** (the CHOMP move): Q ← Q − step·A⁻¹·∇U. The A⁻¹ (smoothness-metric) preconditioning is
  the point — it spreads each local obstacle push smoothly along the trajectory instead of kinking
  it. A depends only on the interior count, so it is Cholesky-factored **once**. Joint limits are
  enforced by projection (clamp) each iteration; a soft joint-limit cost was rejected as
  unnecessary when projection is always feasible.

**Modes** (reported in `PlanningResult::message` + `PlanningStats::refiner_mode`):
- **Refiner** — `RefinerParams::seed` is a feasible trajectory (the pipeline's polish stage).
- **Standalone** — seed empty ⇒ a straight line from the problem's start to a resolved goal
  config; honest `NoSolution` on a local minimum.

### The certificate (ADR-018's rule made concrete)

The optimizer trusts the field; the **exact backend certifies**. After optimization, every
consecutive edge of the output is re-validated as ONE `CollisionScene::query_batch` (P3
discretization). Only a fully-free trajectory is returned `Success`. If the refined path fails
certification: refiner mode returns the (re-certified) seed — never worse than the caller's feasible
input, the same "never worse than input" guarantee the shortcut smoother makes; standalone mode
returns `NoSolution` (the honest local-minimum outcome).

### Plumbing — why `make_refiner()`, not the `make_planner()` string registry

The refiner needs two dependencies the registry's fixed `(params, robot, scene)` signature cannot
carry: a built `ClearanceField` and the robot sphere cover. So it gets its own factory —
**exactly** as `make_shortcut_smoother()` does for the same class of reason (a post-planning stage
with extra deps). To keep the "first-class, discoverable, no silent fallback" spirit, `"chomp"` **is**
registered in `make_planner`, but its stub throws a directive error pointing at `make_refiner()`.
So `registered_planners()` lists it, and selecting it the wrong way fails loudly.

### Contract compliance

- **Batch-first:** the per-iteration clearance/gradient lookup is ONE `ClearanceField::query` over
  all (interior waypoint × sphere) points — a fat, GPU/OpenMP-friendly batch by construction. FK is
  computed once per waypoint per iteration and reused for both point-gather and Jacobian origins.
- **Deterministic per seed:** CHOMP is RNG-free (only standalone goal IK draws, seeded);
  per-waypoint gradients are independent with a fixed intra-waypoint summation order, so the output
  is bit-identical across thread counts (verified).
- **Polished budget:** anytime via `problem.timeout` + convergence tolerance; the exact-backend
  certificate is the only collision query issued through `CollisionScene`.

## Simplifications (documented, deliberate)

- The obstacle gradient omits CHOMP's workspace-velocity (arc-length) weighting and the curvature
  term — the pointwise cost-field variant is adequate here and the certificate guards correctness.
- Joint limits by projection, not a soft cost.
- No TrajOpt-style hard-constraint SQP; this is gradient descent in the smoothness metric. If a
  future cell needs certified constraint satisfaction inside the optimizer (not just at the
  certificate), that is a separate decision.

## Verify (2026-07-17)

`tests/unit/test_refiner.cpp` (5) + `bindings/python/tests/test_refiner.py` (4): certificate
soundness (a `Success` path is independently collision-free); refiner mode raises min-clearance
measurably over a wall-hugging seed at equal (free) success; standalone solves the obstacle-free
subset from a straight line; determinism per seed (bit-identical); the registry stub and null/empty
args fail loudly. Full C++ suite 206/206 (dev-cpu, ASan/UBSan), Python 64 passed, studio smoke
green. Studio: `StudioSession.refine()` (cached ClearanceField + sphere cover, invalidated on
environment edits).

## Consequences

- New `planning/refiner.{hpp,cpp}`; `PlanningStats` gains `refiner_mode` (empty for sampling
  planners). No change to the `Planner` interface or existing planners.
- Next (R5): the PRM smoother path can reuse the same field; the Task 3.5 goal gate records the
  trajectory-quality metrics (smoothness, min-clearance) the refiner is built to improve.

## Alternatives rejected

- **Forcing the refiner into the `make_planner` string registry** — would require either changing
  every planner's factory signature to carry a field, or rebuilding the field per-construction from
  a scene that (correctly) does not expose its geometry. The smoother precedent settles it.
- **Trusting the field as the certificate** — voxel-approximate; a thin collision can slip through.
  ADR-018 already ruled the exact backend the only certificate.
- **OSQP/SCP TrajOpt** — the same D2 (no non-apt deps) and convexification concerns that reverted
  the Task 3.4 jerk SCP prototype (ADR-017) apply; the preconditioned-descent CHOMP is
  dependency-free and adequate for the "smooth + clearance" goal.

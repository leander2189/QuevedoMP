# ADR-017 — Time parameterization: TOPP with tip limits + jerk (supersedes ADR-011's jerk clause)

**Status:** Accepted 2026-07-14 (build-plan Task 3.4; spec: docs/topp_jerk_tip_spec.md, adopted
from Leandro's design after review). **Amended 2026-07-15:** Phase B is a velocity-reduction
kernel, not SCP-over-OSQP — see "Phase B amendment" below. OSQP was prototyped and reverted;
**no external solver dependency was introduced** (deviation D2 remains pristine).

## Context

ADR-011 scoped v0 parameterization to classic TOPP-RA: joint velocity + acceleration only, jerk
"not bounded; consequences accepted". Three things changed:

1. The Task 3.5 quality gate measures summed squared joint jerk — a metric with no producer that
   guarantees it is a soft spot in the headline gate.
2. `TaskLimits` and the build plan always promised TCP velocity **and acceleration** limits;
   stock TOPP-RA delivers neither without custom constraint classes.
3. Industrial use cases (paint/dispense passes) need capped, near-constant tool speed — a tip
   velocity bound that the time-optimal profile rides.

The dependency question (deviation D2: apt-only, vcpkg deferred) was always going to be forced
by Task 3.4; the adopted spec makes it explicit.

## Decision

Adopt the two-phase formulation of docs/topp_jerk_tip_spec.md with these ratifications:

- **(a) Solver strategy — the "OSQP hybrid".** Phase A (joint vel/acc + tip vel/acc, jerk-free)
  is a **dependency-free TOPP-RA-style backward/forward recursion**: all constraints are linear
  in (β, u), controllable sets come from exact 2-variable LPs (vertex enumeration), and the
  greedy maximal profile is provably time-optimal for these constraint types. This alone fulfils
  the original Task 3.4. Phase B (jerk, Stage 2) is **SCP over OSQP** (Apache-2, C, FetchContent
  — the recorded §12/D2 escape-hatch use): the subproblem objective is the PSD second-order
  Taylor model of the convex time objective, so every subproblem is a QP. Monolithic warm-started
  IPOPT (apt: coinor) is the documented fallback if SCP stalls.
- **(b) Tip acceleration is IN scope**, per-axis (box) form: `ẍ_tip = (J·q')·u + (J·q''+J'·q')·β`
  is linear in the decision variables; a scalar limit is applied per axis as `limit/√3`
  (box inscribed in the sphere — conservative, never permissive). The exact Euclidean-norm form
  is SOCP-shaped and deferred until someone needs the ≤42% conservatism back.
- **(c) The C³ path stage lives inside Task 3.4**: `PathSpline` (degree-5 B-spline, C⁴,
  chord-length parameterized; one fixed-degree 1-D Eigen spline per joint — Eigen 3.4's
  Dynamic-dimension spline asserts in `operator()`) plus `fit_collision_free()`, which re-validates
  the curve at P3 edge fidelity in ONE query_batch per round and densifies-and-refits toward the
  validated polyline on collision. No silent fallback: failure is reported and the caller decides.
- **(d) ADR-011 is superseded** as follows: jerk **is** bounded when requested (Scp mode) in the
  smooth interior of a C³ path, in the finite-difference sense (piecewise-constant u ⇒ node-rate
  bound, resampled smooth at controller rate). ConvexOnly mode keeps ADR-011's exact contract
  (vel/acc guaranteed, jerk measured only) and is the default inside the fast-to-feasible budget;
  jerk lives in the polished budget (performance-contract item 2).

## Phase B amendment (2026-07-15): velocity-reduction kernel instead of SCP

The SCP-over-OSQP phase was implemented and driven to a working-but-unsatisfying state before
being replaced at Leandro's suggestion. Post-mortem of the prototype (preserved in this ADR so
the door stays open knowingly):

- Every classic stabilization device was needed in sequence: slack-softened jerk rows (hard rows
  made subproblems infeasible from the bang-bang warm start), loosened ADMM termination (tight
  eps sent OSQP to 20k+ iterations on the LP-like exact-penalty structure), an epigraph max-slack
  (per-row penalty sums reject everything or lock in slow profiles), a feasible-side warm start
  (β → α²β scales node jerk by exactly α³ — one closed-form scaling lands inside the bounds), and
  finally relative trust regions + interior β floors (an absolute radius lets near-rest nodes
  swing many times their magnitude; the quadratic time model cannot see the 1/√β blow-up, so the
  QP proposed interior-zero profiles with 10⁷-second true cost).
- Root cause: everything hard lives at β → 0 — the time objective's barrier and the jerk row's
  √β curvature — exactly where a rest-to-rest profile must operate. Each robot/path/limit combo
  gets a chance to find the next corner; converged results were locally optimal with no gap
  bound, at 3–4× the true optimum on the 1-DOF S-curve testbed, in 10–100 ms.

**Replacement (Leandro's design): a velocity-reduction kernel over the exact node-jerk
evaluation.** The α³ law is used locally: where the exactly-evaluated jerk ratio exceeds 1, β is
dipped by a smooth envelope (per-node caps from the α³ law, per-pass depth clamp 0.25, min-filter
widening + box smoothing so plateau centers keep their target), the result re-evaluated EXACTLY
(no linearization anywhere), and a candidate accepted only if the Phase A acceleration rows still
hold — otherwise the ramps widen, degenerating at the width limit into the uniform scaling, which
shrinks every constraint at once. Terminal fallback: one uniform α³ scaling certifies the profile
at (1 + tolerance/2) of the limit, unconditionally. Properties: **always terminates certified**,
deterministic, O(N·dof) per pass, microseconds total (~2 ms for the whole test incl. Phase A).

Measured trade (test-recorded, not gated): UR5 spline with j_max = 40 rad/s³ — **+4.7% duration**
vs Phase A, 1 pass. Pathological 1-DOF S-curve (Phase A violates jerk 90×) — T = 7.0 s vs the
2.77 s reshaping optimum and the 10.3 s uniform-scaling ceiling: the kernel cannot reshape (it
slows the Phase A profile's shape), so heavy-violation cases pay a real price; mild realistic
violations pay a few percent. If a future application needs the optimality back, the SCP (or a
monolithic IPOPT solve) slots behind the same `Mode` enum; this post-mortem is the map.

## Consequences

- New module `parameterization/` (`PathSpline`, `parametrize`, `limits_from_model`), bound to
  Python; `JointState` gains `acc`.
- Phase A validated three ways: analytic 1-DOF trapezoid/triangle closed forms, per-node limit
  compliance on UR5 splines (joint + tip), and **differentially against pip-installed toppra**
  (velocity+acceleration on the same dense path, durations within 2%) — toppra is a TEST
  dependency only, never a library dependency.
- The `JointLimits.jerk` field (yaml extension) finally has a consumer; `TaskLimits` maps 1:1
  onto tip limits via `limits_from_model`.
- Phase A cost is microseconds–milliseconds (N ≈ 200–400 nodes, closed-form LPs), safely inside
  the ≤50 ms Phase 3a gate; the Stage 2 kernel adds microseconds (2026-07-15 amendment), so the
  WHOLE parameterization fits the fast budget — jerk no longer needs the polished-budget carve-out.
- ~~OSQP joins the build via FetchContent~~ **(amended 2026-07-15: no solver dependency — the
  kernel is dependency-free; the D2 escape hatch was exercised in a prototype and reverted).**

## Alternatives rejected

- **hungpham2511/toppra as a dependency:** solves only Phase A's joint-limit subset, custom
  constraint classes needed for tip limits, no jerk; we'd still write Phase B. Kept as the
  differential-test oracle instead.
- **Scene-independent SOCP for Phase A (ECOS/SCS/Clarabel):** exact convex objective but a
  heavier dependency for no optimality gain over the recursion (which is exact for these
  constraint types); ECOS is GPLv3, Clarabel needs a Rust toolchain in the container.
- **Monolithic IPOPT as the primary:** apt-clean but heavyweight and slower per solve; retained
  as fallback only.
- **GPU smoothing-style parameterization:** nothing to offload — the workload is tiny banded
  algebra, not batched collision.

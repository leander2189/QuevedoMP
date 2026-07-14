# ADR-017 — Time parameterization: TOPP with tip limits + jerk (supersedes ADR-011's jerk clause)

**Status:** Accepted 2026-07-14 (build-plan Task 3.4; spec: docs/topp_jerk_tip_spec.md, adopted
from Leandro's design after review).

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
  the ≤50 ms Phase 3a gate; Stage 2's SCP budget must be measured and, if needed, confined to
  the polished budget before the gate run.
- OSQP joins the build via FetchContent when Stage 2 lands (first non-apt C++ dependency;
  pinned tag, recorded here as the §12 decision D2 anticipated).

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

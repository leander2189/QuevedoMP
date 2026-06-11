# ADR-014 — OptiX backend: batched raygen over a static environment GAS

**Status:** Draft — ratify before Phase 2b coding starts (build-plan amendment M4).
Supersedes the per-config IAS-update flow sketched in spec §4.5.

## Context

The spec's original OptiX flow per configuration was: host FK → write IAS instance
transforms for robot links → refit/rebuild → launch any-hit. An RRT validates edges
*serially*, and one edge is a batch of only ~10–100 configs; putting an acceleration-
structure update + kernel launch + PCIe round-trip inside that serial loop makes the GPU
lose to a decent CPU at exactly the batch sizes planners produce — the classic failure
mode of GPU sampling-based planners. Meanwhile the project's target regime is
**quasi-static scenes with high-poly meshes**: the environment changes rarely; the robot
pose changes every query.

## Decision

Robot geometry never enters an acceleration structure. The environment is the only thing
traced *against*; the robot is the thing that *shoots rays*.

1. **Environment:** one GAS over all environment meshes, built at `add_object` time,
   compacted; refit (or rebuild, whichever measures faster) only on `move_object` /
   `remove_object`. Never touched by `query_batch`.
2. **Robot:** at scene build, precompute per-link **test-ray sets** in link-local frame
   (segment rays along collision-mesh triangle edges, deduplicated; plus the ADR-012
   parity ray origin), stored SoA in device memory once.
3. **Per `query_batch(qs)`:** host FK for all configs (v0) → upload **one** buffer of
   per-config × per-link transforms → **one launch** with dimensions indexing
   `(config, ray)`; each thread transforms its ray by its link's transform on the fly and
   traces against the environment GAS with `OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT`.
   Per-config result reduced via one `atomicOr` into the result buffer → one D2H copy of
   the boolean array. Zero AS updates, zero per-config launches.
4. **Self-collision runs on the CPU** inside the same `query_batch` call: FCL on the few
   link pairs the ACM allows (convex hulls of links where exact meshes are slow). A 6-DOF
   arm has a handful of pairs; this is µs-range, overlaps the GPU launch, and deletes the
   OptiX-side ACM/visibility-mask machinery (8-bit instance masks can't encode link pairs
   anyway).
5. **Workspace** (per thread): CUDA stream, pinned staging, persistent device buffers for
   transforms/results grown geometrically. Buffers are reused across calls — no per-call
   allocation.

## Consequences

- Per-batch cost ≈ one H2D (transforms) + one launch + one D2H (booleans) — small-batch
  latency is bounded by launch + PCIe (~tens of µs), not AS builds (~ms).
- Quasi-static scene edits cost one env-GAS refit, off the query path (budget: ≤ 100 ms on
  the largest fixture; tracked by the benchmark protocol).
- Edge-vs-surface coverage equals what triangle-edge rays see — the same class of test FCL
  triangle-triangle performs; containment is handled by ADR-012's parity ray, margins by
  ADR-013.
- Tunneling between ray samples along an edge is bounded by edge-check resolution exactly
  as on CPU (spec §4.3); no new failure mode.
- If profiling later shows raygen-side transform math dominating, the fallback *within the
  same API* is robot-link instancing in a per-batch IAS — the spec's original design —
  which we then adopt knowingly, with measurements.

## Alternatives considered

- **Per-config IAS transform update + launch (spec §4.5).** Rejected as the *default*:
  serializes AS updates with launches inside RRT's edge loop.
- **N robot instances in one big IAS per batch.** One launch, but requires building an
  IAS per batch (still an AS build on the hot path) and instance-mask gymnastics.
- **Sphere-approximated robot (cuRobo-style) + sphere queries.** Fastest known approach,
  but abandons exact link meshes; kept as a future option (see ADR-013 alternatives).
- **Embree CPU backend with this same design.** Not an alternative but a hedge (build-plan
  M1 fallback): identical ray semantics, no GPU; would also make a cleaner differential
  partner than FCL. Revisit if OptiX-in-WSL fails or small-batch latency disappoints.

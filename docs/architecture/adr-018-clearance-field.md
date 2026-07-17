# ADR-018 — ClearanceField: GPU voxel SDF as a separate type (roadmap R3)

**Status:** Accepted 2026-07-16 (roadmap R3; the §12 decision Task 3.3b anticipated).

## Context

Sampling planners produce *feasible*; "smooth and logical with clearance margins" needs an
optimization pass that consumes clearance **gradients**, which brute-force `distance()` cannot
serve inside a loop (Task 3.3b's motivation). The environment is quasi-static — the same
assumption the OptiX GAS design exploits — so a precomputed field amortizes.

## Decision

**A separate `ClearanceField` type — NOT a `CollisionScene` extension.** The scene's contract is
exact and boolean-authoritative; the field is approximate (voxel resolution), gradient-bearing,
and serves optimization/visualization. Mixing those semantics in one interface is how contracts
rot; the exact backend remains the only collision certificate (R4's refiner re-validates through
it).

Build pipeline (one-time per scene):
1. Environment triangles in world space; primitives ride the SAME closed tessellations as the
   OptiX backend (P2) — one geometry story everywhere.
2. **Seed**: near-surface voxels get exact point-to-triangle distances. Parallel over triangles
   with a 64-bit atomic min-encode (float distance bits << 32 | triangle id — the monotone
   float→uint map makes "min" a single atomic); the winning triangle's exact nearest point is
   recomputed per voxel afterwards, so no racing 3-float payloads.
3. **JFA**: jump flooding propagates nearest-seed ids (origin-relative float coordinates).
   Plain CUDA kernels when built with `QUEVEDOMP_WITH_CUDA` *and* a device answers at runtime;
   an equivalent OpenMP fallback otherwise (`built_on_gpu()` reports which ran; the
   GPU-vs-CPU agreement test holds them to < 1 mm).
4. **Sign**: even-odd column parity per watertight object (ADR-012 stance: non-watertight
   meshes contribute UNSIGNED distance only, warned once). Column samples are shifted by an
   irrational-ish sub-voxel offset — a column crossing exactly through a shared triangle edge
   (tessellated-box face diagonals hit grid columns dead-on) would otherwise be double-counted
   and flip the whole column's parity.

Queries: trilinear distance + central-difference gradients, batched (`query`) for R4's
optimizer shape. Outside the grid: clamped-border value plus the Euclidean gap (approximate,
documented — size the margin so the robot works inside the grid).

Robot side: `decompose_robot` builds a **conservative sphere cover** (per collision geometry:
centers along the vertex cloud's principal axis, count grown until the cover radius meets the
target) — every collision vertex lies inside some sphere, so `clearance_batch` (min over
spheres of field(center) − radius) never overestimates clearance.

## Measured (2026-07-16, RTX 4060 Ti, 16 CPU threads)

Hires inlet (7.3M triangles): 16.8M voxels @ 10 mm in **1.34 s GPU JFA** (2.02 s CPU);
2.1M voxels @ 20 mm in ~1.2 s either way (seeding + sign are CPU-bound and dominate at small
grids). The hires inlet is not watertight ⇒ unsigned field on that fixture (see the dtc_test_inlet
PROVENANCE note).

## Consequences

- New module `clearance/` (+ `jfa.cu` under `QUEVEDOMP_WITH_CUDA`; the library links `cudart`
  in CUDA builds — first CUDA code inside libquevedomp, still zero non-apt dependencies).
- Consumers now: studio heatmap slices, clearance metrics. Next: R4's CHOMP/TrajOpt-style
  refiner (batched gradient lookups over trajectory waypoints), optionally a gradient-informed
  shortcut.
- Field accuracy is voxel-bounded (validated ≤ 2.5·resolution vs analytic SDFs); anything
  needing exactness keeps using the collision backend.

## Alternatives rejected

- Extending `CollisionScene` with SDF queries: contract mixing (approximate vs authoritative).
- Exact per-voxel BVH distance build: O(V) exact queries at build time — minutes, not seconds,
  and JFA's ~voxel accuracy is all the optimizer needs.
- OptiX-based closest-hit sampling: ray probes give surface hits, not nearest-point fields, and
  would couple the field to the OptiX build.

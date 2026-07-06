# How the OptiX collision backend works

This is an explainer of the algorithm behind `OptixScene`
([`src/collision/optix/optix_scene.cpp`](../src/collision/optix/optix_scene.cpp)), the GPU
implementation of the `CollisionScene` interface. It describes *how* the pieces fit; the
*why* of the design decisions is recorded in
[ADR-014](architecture/adr-014-optix-batched-raygen.md) (batched raygen) and
[ADR-012](architecture/adr-012-raycast-containment.md) (containment), which this document
does not replace.

## The core idea: the robot shoots rays, the environment gets traced

A ray tracer answers one question extremely fast: *does this segment cross a triangle?*
The backend reduces "is the robot in collision with the environment?" to millions of that
question.

The naive GPU mapping — put robot and environment into acceleration structures and
intersect them — fails for motion planning, because the robot moves **every query**: you
would rebuild or refit an acceleration structure inside the planner's hot loop. So the
roles are split asymmetrically:

- The **environment** (quasi-static: changes rarely) is the only thing built into an
  acceleration structure — one triangle GAS, built once at scene construction.
- The **robot** (changes every config) never enters an acceleration structure. Each robot
  collision mesh is converted once into a set of **test rays** — one segment per unique
  triangle edge — stored in link-local frame. Posing the robot is then just a rigid
  transform applied to each ray at trace time, which costs a few multiplies per thread
  instead of an AS update.

Two triangle meshes intersect if and only if an edge of one crosses a face of the other
(or one is fully inside the other — handled separately, see
[containment](#the-containment-gap) below). Robot-edge rays traced against environment
faces cover the first half of that condition; the second half (an *environment* edge
crossing a *robot* face without any robot edge crossing an environment face) is the same
class of coverage FCL's triangle-triangle test provides on meshes of comparable
tessellation, and the two backends are differential-tested against each other for exactly
this reason.

## Scene build (once, off the query path)

`OptixScene`'s constructor does all the expensive work:

1. **Context + pipeline** — OptiX device context, module from the PTX compiled out of
   [`optix_programs.cu`](../src/collision/optix/optix_programs.cu), the three program
   groups (raygen / miss / closest-hit), and a minimal shader binding table.

2. **Environment GAS** — every `SceneDescription` object is flattened to world-space
   triangles (boxes are tessellated to 12 triangles; meshes are copied through their
   pose; spheres/cylinders are not supported in v0). One GAS is built over all of it with
   `PREFER_FAST_TRACE` and then **compacted**: build time is irrelevant (one-time), but
   the result is traced by millions of rays, so it is worth the highest-quality, smallest
   BVH. The world AABB of the environment is also recorded here for the broadphase cull.

3. **Robot ray sets** — for each robot link with a mesh collision geometry, the mesh is
   loaded, scaled (URDF `<mesh scale>`), and moved into link frame (collision `origin`).
   Every unique triangle edge becomes one segment ray `{origin, unit direction, length}`
   in link-local frame; shared edges are deduplicated so each is traced once. Rays are
   stored SoA (separate origin/direction/length/link arrays) in device memory, uploaded
   once. Each ray carries the index of its link's **transform slot**, and per slot the
   link-local AABB of its rays is kept for the cull.

4. **Containment interior points** — per mesh link, the centroid of its vertices (link
   frame), plus an `EnvContainment` tester built from the environment (analytic
   inside-tests for primitive solids, parity rays for watertight meshes).

5. **An internal FCL scene with an empty environment** — robot-vs-**self** collision
   stays on the CPU (ADR-014 item 4): a 6–7 DOF arm has only a handful of non-ACM link
   pairs, and 8-bit OptiX visibility masks cannot encode pairwise link filters anyway.

## Query: one batch, one launch

`query_batch(robot, qs, opts, ws)` takes N joint configurations and returns N booleans.
The whole batch costs one host FK pass, one H2D copy, one kernel launch, and one D2H
copy — nothing per-config touches the GPU API. The phases are pipelined so the CPU and
GPU work at the same time:

```
CPU  | FK all configs -> | H2D, launch,   | FCL self-collision   | sync, merge,
     | fill transforms   | D2H (enqueued, | (runs while GPU      | containment
     | (OpenMP-parallel) |  async)        |  traces)             | inside() tests
-----+-------------------+----------------+----------------------+--------------
GPU  |                   |                | trace N*num_rays rays|
```

### 1. Host FK pass (parallel)

For each config, `fk_all` poses every link once. That single FK pass produces both
consumers' inputs:

- the **transform block** — a row-major 3×4 float matrix per (config, ray-bearing link),
  written into pinned staging memory, and
- the **containment interior points** — each link centroid pushed through its link pose
  into world frame, saved for the final phase (so containment never re-runs FK).

Configs are independent and write disjoint slices, so the loop is an OpenMP
`parallel for` — but only when `n >= 64`: below that, fork/join overhead exceeds the FK
being parallelized, and a single-config probe (the RRT edge-check pattern) must stay at
its serial latency.

When the opt-in broadphase cull is enabled (`QUEVEDOMP_OPTIX_CULL=1`), this pass also
poses each slot's link-local ray AABB into the world (8 corners) and tests it against
the environment AABB; a miss writes a 1-byte cull flag meaning "none of this link's rays
can hit for this config". The cull is conservative — it can only skip rays that provably
cannot hit — so it never changes the result. It is opt-in because on a
workspace-spanning obstacle nothing culls and the host-side AABB math is pure overhead;
on a localized obstacle (the DTC work object) it roughly halves the batch time.

### 2. GPU trace

The transform block (and cull mask, if any) is copied H2D on the workspace's stream, the
per-config result array is zeroed, and **one** `optixLaunch` runs with a 2D grid indexed
`(ray, config)`. Each thread ([`optix_programs.cu`](../src/collision/optix/optix_programs.cu)):

1. reads its link-local ray and its config's transform for the ray's link,
2. checks the cull flag (skip if the link was culled),
3. applies the rigid transform on the fly — rotation to the direction, full affine to
   the origin; rigid transforms preserve length, so `tmax` stays the stored ray length,
4. traces the environment GAS with `OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT` — any hit at
   all means collision, so no closest-hit search is needed,
5. on hit, `atomicOr`s a 1 into its config's slot in the result array.

The boolean array is then copied D2H, also asynchronously. Nothing has synchronized yet.

### 3. CPU self-collision, overlapped

While the GPU traces, the host runs robot-vs-self through the internal FCL scene
(honoring the `AllowedCollisionMatrix`). Only after that does the stream synchronize —
so on a full query the CPU self-collision pass is effectively free as long as it is
shorter than the GPU trace (on the DTC benchmark the full query is now bound by this CPU
pass, with the GPU completely hidden behind it).

### 4. Merge + containment

GPU hits and CPU self results are OR-ed per config. Finally, for every config still
reported free, the precomputed world-frame interior points are run through
`EnvContainment::inside()` — cheap analytic/parity tests, no FK, no narrowphase.

## The containment gap

Both boolean strategies — FCL's BVH triangle-triangle and these surface rays — are
*surface intersection* tests. A robot link **entirely inside** a closed obstacle crosses
no surface: every ray starts and ends inside, hits nothing, and the query would report
free — the worst possible failure (false-free). ADR-012 closes this on the CPU: one
precomputed interior point per mesh link is tested against every environment solid.
Boxes/spheres/cylinders get exact analytic inside-tests; watertight meshes get a parity
ray (odd crossing count ⇒ inside); non-watertight meshes are excluded with a one-time
warning, since "inside" is undefined for them.

## Memory model

Each `OptixWorkspace` (one per thread, from `make_workspace()`) owns:

- its own **CUDA stream** — all of a workspace's GPU work is ordered on it, so
  concurrent const queries from different threads never share GPU state;
- **persistent device buffers** (transforms, cull, results, launch params) and
  **pinned host staging** for the H2D/D2H copies. Both grow geometrically and never
  shrink: after a workspace has seen its largest batch, a query allocates nothing.

Every `query_batch` fully synchronizes its stream before returning, which is what makes
the grow-only reuse safe call-to-call.

## v0 limitations

- **Boolean only.** `opts.distance` throws — signed distance + witnesses stay on the FCL
  backend (spec §4.3/§4.5).
- **Static scene.** `add_object` / `move_object` / `remove_object` throw; the
  environment is fixed at `make_static_scene` time. (The design supports a GAS refit off
  the query path; it is just not implemented yet.)
- **Robot links must be meshes** — primitive robot collision geometry casts no rays.
  Environment geometry is boxes and meshes; sphere/cylinder tessellation is a follow-up
  (containment still handles them analytically).
- **Sampling-level guarantees.** Tunneling between edge-check samples is bounded by the
  edge resolution exactly as on the CPU backend; the ray test itself adds no new failure
  mode relative to FCL triangle-triangle (both are exact surface tests plus the shared
  containment net).

## Where the time goes

On the DTC cell benchmark (`benchmarks/bench_dtc.cpp`, RTX 4060 Ti, batch 10 000,
cull on): ~23 ms per batch ≈ 2× the FCL backend, of which the dominant costs are the
parallel host FK/transform fill and the trace itself. With self-collision enabled the
query is bound by the CPU FCL self pass — the entire GPU portion hides behind it. Small
batches (1–10 configs) remain latency-bound at ~0.1–0.2 ms (launch + PCIe round-trip),
where the CPU backend wins; the crossover is around a few hundred configs. Methodology
and current numbers: [benchmarks/PROTOCOL.md](benchmarks/PROTOCOL.md).

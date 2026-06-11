# ADR-012 — Containment blind spot of surface-based boolean collision

**Status:** Draft — ratify before Phase 2b coding starts (build-plan amendment M4).

## Context

Both of our boolean collision strategies are *surface-intersection* tests:

- FCL mesh-mesh collision runs triangle-triangle intersection over BVHs.
- The OptiX backend traces rays along robot surface geometry against the environment.

Neither detects **containment**: a robot link entirely *inside* a thick closed obstacle
produces no surface intersection and reports *free*. Because the differential oracle (FCL)
shares the blind spot, differential testing will never surface it. For edge checking at
sane resolution a link must graze a surface before entering, so the practical exposure is
**states sampled or supplied already inside an obstacle** (e.g., an invalid start/goal, or
a sampler box overlapping geometry).

A false-free is the worst failure class this library can have (spec: "a false-free is a
real robot hitting something real").

## Decision

1. **v0 ships a parity-ray containment check** as part of `query_batch` in *both* backends:
   for each robot link, cast one ray from a precomputed interior point of the link (e.g.,
   centroid pushed inside via the surface normal) in a fixed direction against the
   environment; an **odd hit count ⇒ the point is inside an environment solid ⇒
   `in_collision`**. Cost: one ray per link per config — negligible next to the surface
   rays in OptiX; implemented with FCL ray casts (or the same Embree/utility code) on CPU.
2. **Parity is only sound for watertight meshes.** At scene build, each environment mesh is
   classified (closed-fan edge check / assimp diagnostics). For non-watertight meshes the
   parity check is **disabled per mesh** and the limitation is logged once, loudly:
   *"mesh X is not watertight: containment inside it is undetectable."* This is a
   documented contract limitation, not silent behavior.
3. The differential harness (§4.6) adds **explicit containment cases** (link fully inside a
   box / inside a watertight fixture mesh) as fixtures — they cannot arise from random
   sampling against a shared-blind-spot oracle.

## Consequences

- Start/goal states inside watertight obstacles are correctly rejected in both backends.
- Non-watertight scans degrade gracefully and visibly instead of invisibly.
- One extra ray per link per config; no API change (`QueryOptions` untouched).

## Alternatives considered

- **Do nothing (status quo of both FCL-mesh and raycast approaches).** Rejected: silent
  false-free on invalid starts is unacceptable for an industrial library.
- **Signed distance fields for containment.** Correct but drags the deferred ESDF epic
  (spec §7) into v0. Rejected for scope.
- **Convex decomposition + GJK containment.** Robust but adds a heavy preprocessing
  dependency (VHACD) and changes geometry semantics; revisit if parity proves brittle.

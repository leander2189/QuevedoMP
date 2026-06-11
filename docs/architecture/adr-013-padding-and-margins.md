# ADR-013 — `robot_padding` and `safety_margin` semantics under ray-cast collision

**Status:** Draft — ratify before Phase 2b coding starts (build-plan amendment M4).

## Context

`QueryOptions` (spec §4.2) exposes two distance-flavored knobs:

- `robot_padding` — physical inflation of robot collision geometry.
- `safety_margin` — "collision if signed distance < margin".

FCL can honor both (it computes distances). A ray-cast backend cannot: rays report *hit or
miss*, not clearance, and "inflate a triangle mesh by δ" (a Minkowski sum with a sphere) is
not something you can do exactly or cheaply at query time.

## Decision

1. **`robot_padding` is honored by both backends, applied at scene-build time, on the
   robot only.** The robot's per-link collision meshes are inflated once when the
   `CollisionScene` is constructed: each vertex offset along its angle-weighted
   pseudo-normal by `robot_padding` (+ optional per-pair padding later). This is an
   *approximate* Minkowski offset — exact for convex regions, slightly conservative at
   sharp concavities — and is applied identically in FCL and OptiX so the backends test
   the *same* geometry and stay differentially comparable. Changing padding ⇒ rebuilding
   the robot-side geometry (documented; cheap — robot meshes are small).
2. **`safety_margin` is FCL-only in v0.** `query_batch(distance=true | safety_margin>0)`
   on the OptiX backend returns `BackendUnsupported` (a typed error, not a silent wrong
   answer); `BackendHint::Auto` routes such queries to FCL. Rationale: RRT needs only the
   boolean; margins are used at validation/supervision time where FCL's exactness is
   wanted anyway. (Approximating margins by *also* inflating by `safety_margin` at build
   time was rejected: margin is per-query, inflation is per-scene — semantics drift.)
3. **Environment meshes are never inflated** (they can be huge; offsetting a 1M-triangle
   scan is expensive and quality-fragile). All inflation lives on the robot side, which is
   equivalent for robot-vs-environment pairs.

## Consequences

- The differential contract stays clean: both backends see identical (pre-inflated)
  geometry; the ±1e-4 m band continues to cover only FP/raycast noise, not modeling drift.
- v0 GPU path = boolean at fixed padding. Anything distance-shaped is CPU. This matches
  the spec's "OptiX boolean-authoritative, FCL distance-authoritative" split.
- A future ESDF epic (spec §7) supersedes this with true distance/gradient on GPU.

## Alternatives considered

- **Inflate per-query on GPU.** Rejected: offsetting meshes per query is orders of
  magnitude more expensive than the query itself.
- **Sphere-wrap the robot (cuRobo-style) and grow sphere radii.** Elegant for padding and
  margins, but replaces exact link geometry with an approximation everywhere and changes
  the FCL↔OptiX comparison into an apples-vs-oranges one. Deferred — worth revisiting if
  exact-mesh raycast proves too slow.
- **Reject `robot_padding` on GPU too.** Unnecessary: build-time inflation is cheap,
  exactness loss is small and conservative (errs toward *more* collision, never false-free).

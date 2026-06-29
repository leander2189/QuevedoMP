# ADR-015 — Forward kinematics lives inside the collision scene (v0)

**Status:** Accepted — ratified before the FCL backend (build-plan Task 2a.1; spec §12 "FK location").

## Context

The collision interface (spec §4.2) must place the robot's collision geometry at each queried
configuration `q`. There are two places the FK can live:

1. **Scene-internal FK** — the `CollisionScene` holds the `RobotModel` and computes FK per query
   (`query_batch` takes a `RobotInstance` + a batch of `JointPosition`s).
2. **FK outside** — the caller computes link transforms and passes them in; the scene stays
   FK-agnostic.

Spec §12 lists this as a decision to ratify before building the FCL backend.

## Decision

**v0 uses scene-internal FK**, exactly as the §4.2 headers are written:
`make_static_scene(shared_ptr<const RobotModel>, …)` and
`query_batch(const RobotInstance&, span<const JointPosition>, …)`. The scene FKs the robot at
each `q` (host-side, via `kinematics/fk`) and poses its collision geometry; environment geometry
is fixed at `add_object` time.

## Rationale

- **It matches the batched-GPU end state (M4).** The OptiX backend's whole design is per-config
  FK transforms uploaded in one buffer and applied in raygen. Owning FK lets the scene do batched
  FK on-device later *without any API change* — passing transforms in from outside would force
  the caller to do GPU FK or ship host transforms across PCIe every query (the failure mode M4
  exists to avoid).
- **Simplest correct caller.** RRT calls `check_edge`/`query_batch` with raw configurations; it
  never touches FK or link frames. Fewer, simpler call sites (a stated goal of M4/§4).
- **One FK implementation.** FK is validated once (Task 1.5, <1e-9 m). Both backends consume the
  same `fk_all`; "FK outside" risks two FK paths disagreeing.

## Consequences

- The scene depends on `kinematics/fk` and `robot/` — already in the CPU library; no new deps.
- v0 FK is host-side for both backends; **batched GPU FK is a later optimization behind the same
  API** (spec §4.3). The interface does not change when it lands.
- Callers cannot supply pre-computed link transforms in v0. If a future need appears (e.g. an
  external state estimator), add an overload — do not retrofit the existing signature.

## Alternatives considered

- **FK outside / transforms passed in.** Keeps the scene FK-agnostic, but complicates batched GPU
  FK and pushes FK duplication onto callers. Rejected for v0.

# QuevedoMP R8 Design — Adaptive-step RRT-Connect

> **Audience: the implementing agent.** Spec proposed 2026-07-22 and extracted from
> `QuevedoMP-ROADMAP-v1.md` on 2026-08-25 so that every open roadmap item has its own design
> document. Content is unchanged from the ratified spec apart from this header.
>
> **Status: NOT BUILT.** Scheduled as item 5 in [`../ROADMAP.md`](../ROADMAP.md) — the first
> engineering item after the product-readiness bundle. It is the fix for the project's
> headline limitation, the wedged goal.

---

## The spec, as proposed 2026-07-22

> Not yet built. Groundwork landed this session (uncommitted): PRM connectivity diagnostics +
> roadmap-by-component view, studio RRT `extension step` / `goal bias` knobs, and the
> goal-escapability probe (`StudioSession.escapability`). Those turned the diagnosis concrete —
> the hard motions are an **isolated wedged goal**, not disjoint roadmap regions — which is what
> this feature targets.

**Motivation.** v0 RRT-Connect uses one global `max_extension` (0.5 rad) and grows all-or-nothing:
a candidate node is kept only if its *entire* steered edge is collision-free
(`src/planning/rrt_connect.cpp`, growth loop). One step size can't serve both regimes — large steps
explore open space fast but are rejected wholesale in tight regions, so a tree rooted in a narrow
pocket (empirically the **goal**) never takes a single step (its tree stays a lone root; the
escapability probe confirms the goal is free but can't move). Sizing the step **per node to the
local width of free space** — crawl where tight, stride where open — directly unblocks that case.

Prior art (for the record): Dynamic-Domain RRT (Yershova/Jaillet/Siméon/LaValle 2005) adapts a
per-node sampling radius from extension success/failure; Rechenberg's **1/5 success rule**
(evolution strategies) is the canonical success-rate step update; clearance-proportional step is the
model-based analogue. This spec takes both signals.

**Two adaptation signals** (per node *i*, keeping `step_i ∈ [min_extension, max_extension]`):

1. **Success-rate (model-free, registry-compatible).** Multiplicative 1/5-rule update after each
   batch validates extensions originating at node *i*:
   - extension from *i* succeeds → `step_i ← min(step_i · grow, max_extension)`
   - extension from *i* fails → `step_i ← max(step_i · shrink, min_extension)`
   with `grow ≈ 1.3`, `shrink ≈ 0.7`. Needs no extra inputs — a pure `rrt_connect` change.
2. **Clearance (model-based, needs the R3 field).**
   `step_i = clamp(clearance_gain · d(q_i), min_extension, max_extension)`, where `d(q_i)` is the
   robot's clearance at `q_i` (min over the sphere cover — exactly what `ClearanceField::query`
   returns). No cold start: at a wedged goal `d ≈ 0` ⇒ `step_i` floors immediately.
   **`ClearanceThenSuccess`** blend: seed `step_i` from clearance at insertion, then let the
   success-rate rule correct it during growth.

**New-node warm start.** A node inherits its parent's adapted step (not a reset to
`max_extension`), so a subtree threading a passage stays fine-grained.

**Params (PlannerParams additions).** `adaptive_extension: {Off, SuccessRate, Clearance,
ClearanceThenSuccess}` — default **Off** (existing behavior + benchmarks unchanged; no silent
fallback); `min_extension` (floor, default a small multiple of `edge_resolution`);
`extension_grow` / `extension_shrink` (success-rate); `clearance_gain` (m → rad). `max_extension`
stays the ceiling.

**Contract compliance (standing invariants).**
- *Batch-first collision*: unchanged — the batch of candidate extensions is still ONE
  `query_batch`; each candidate merely steers to its origin node's `step_i`. Clearance is one
  `ClearanceField::query` per growth step (or cached per node, since it's constant per config).
- *Determinism per seed*: preserved — step updates are deterministic functions of the batch
  results; the single-threaded RNG stream is untouched, so a seed reproduces the trees.
- *No silent fallback*: the mode is explicitly selected; `Off` is the default.

**Plumbing.** `SuccessRate` needs no new input → stays in `make_planner("rrt_connect", …)`.
`Clearance` / `ClearanceThenSuccess` need the field, which doesn't fit the registry's
`(params, robot, scene)` signature (same constraint as R4 refiner / R5 PRM) → a dedicated
`make_adaptive_rrt_connect(params, robot, scene, field)` factory for the field-backed modes; keep
the registry planner for the model-free mode.

**Known limitation (documented, not solved here).** Magnitude-only adaptation is partial: in a
narrow passage the free directions form a thin cone, so shrinking the step raises
success-per-attempt but still wastes samples on colliding directions. The full fix also biases
*direction* toward the clearance gradient `∇d` (available from R3) — left as a follow-on
("gradient-biased extension"); this spec is magnitude-only.

**Interplay.** Composes with the deferred **partial-extension** growth fix (keep the collision-free
prefix of a rejected step, then shrink `step_i`) and with `goal_bias`. The escapability probe is the
a-priori check on whether adaptivity can help at all: a **WEDGED** goal (no free step in any
direction) is a goal-pose/calibration problem, not one adaptivity can fix.

**Validation.** On the wedged-goal + narrow-passage studio scenes, compare `Off` vs each mode:
solve rate, time-to-first-solution, `collision_configs` (the contract metric), and tree sizes.
Determinism test (same seed ⇒ identical trees). `benchmark.qmps` numbers must be unchanged with the
default `Off`.

**Related deferred narrow-passage items** (siblings, not part of R8): partial-extension growth (keep
the free prefix); bridge-test PRM sampling (place roadmap nodes *inside* narrow gaps, the canonical
fix for the isolated-goal roadmap split the diagnostics surface).

# QuevedoMP Benchmark Protocol

> Single source of truth for every performance claim this project makes (build-plan
> amendment M2). Any number quoted in a README, paper, or exit gate must be reproducible
> by following this document. Changes to this protocol go through PR review like code.

## 1. The claim under test

**QuevedoMP plans faster than MoveIt 2 in complex environments with high-polygon meshes,
in quasi-static scenes**, at equal or better success rate.

Honest-positioning notes (spec §0.5): we do not claim superiority over cuRobo, and we do
not claim wins on low-poly/primitive scenes — on those, well-tuned CPU checking may win
and `BackendHint::Auto` may select the FCL backend.

## 2. Benchmark axes

### 2.1 Scenes (fixtures — build-plan Task B.1)

| Fixture | Content | Triangles | Watertight |
|---|---|---|---|
| `industrial-cell` | scanned/CAD robot cell: fences, conveyor, fixtures | ≥ 500k | mostly |
| `dense-shelving` | shelving unit with high-res scanned objects | ≥ 300k | mixed |
| `cluttered-table` | tabletop, scanned clutter (YCB-scan class) | ≥ 200k | **at least one non-watertight mesh** |

Provenance + licenses in `tests/fixtures/scenes/PROVENANCE.md`. Fixtures are versioned;
results must record the fixture version (file content hash).

### 2.2 Robot and problems

- Robot: **UR5** (Phase 1 fixture URDF). Optional secondary: Franka Panda (7-DOF).
- Per scene: **30 planning problems** (start/goal joint configs), generated once with a
  recorded seed, stored as a fixture (`tests/benchmarks/problems/*.yaml`). Mix:
  ~1/3 easy (large clearance), ~1/3 cluttered, ~1/3 narrow-passage.
- Identical problems are fed to both planners. No per-problem tuning on either side.

### 2.3 Systems compared

| System | Configuration |
|---|---|
| MoveIt 2 baseline | RRTConnect (OMPL default), default simplification, scene loaded from the same meshes, single-threaded planning. Container: `benchmarks/moveit-baseline/` (Task B.3). |
| QuevedoMP / FCL | `RrtConnectPlanner` + `ShortcutSmoother`, FCL backend |
| QuevedoMP / OptiX | same, OptiX backend (Phase 2b+) |

Termination criteria must match: same planning timeout (default **5 s**), success =
collision-free path validated at the protocol resolution (below).

## 3. Metrics

### 3.1 End-to-end planning (Phase 3a gate)

Per scene, over the 30 problems × **10 seeds** each (seeds recorded):

- **p50 / p95 plan wall time** (plan + smooth; time parameterization reported separately).
- **Success rate** within the timeout.
- **Path length** (joint-space L2) — guards against "fast but garbage" paths.
- Scene **load/build time** (one-time) and **update latency**: `move_object` on one object
  + first subsequent query, p50/p95 — this is the quasi-static story. Budget: ≤ 100 ms on
  the largest fixture.

**Phase 3a exit gate:** QuevedoMP p50 ≥ **5×** faster than the MoveIt baseline on each
fixture, success rate ≥ baseline's.

### 3.2 Collision microbenchmarks (Phase 2a/2b gates)

Boolean `query_batch` on each fixture, configs drawn from the recorded problem seeds:

- **Small-batch latency** (what an RRT actually sees): wall time per `query_batch` at
  batch sizes **10 / 50 / 100**, p50/p95 over 1000 calls, single thread, one warm
  `Workspace`.
- **Bulk throughput**: configs/sec at batch 10k (the spec's ≥ 5× FCL number).
- **Crossover point**: smallest batch size where OptiX beats FCL on each fixture —
  this calibrates `BackendHint::Auto`.

### 3.3 Differential quality (Phase 2b, §4.6)

Reported alongside, from the differential harness: out-of-band disagreements (must be 0),
ambiguous (in-band) fraction.

## 4. Methodology rules

1. **Hardware is recorded** with every result: CPU model, GPU model, driver, RAM, power
   profile (laptops: plugged in, performance mode; this dev box: RTX 2000 Ada laptop GPU).
2. **Warmup:** discard the first 3 runs of any timed loop (JIT, AS builds, allocator,
   GPU clocks). Report steady-state. AS/scene build time is its own metric, never hidden
   in query time — and never amortized away silently.
3. **Releases builds only** (`release` preset; baseline container built `-O3`).
   Sanitizer builds are never benchmarked.
4. **Same machine, same session** for any A-vs-B comparison. No cross-machine comparisons.
5. **Seeds recorded** for every stochastic component (problems, planner); results files
   include them.
6. **Results land in-repo** as JSON under `tests/benchmarks/results/` with date, git SHA,
   fixture hashes. Regression alert threshold: > 10% on any §3.1/§3.2 metric (spec §8).
7. **Edge-validation resolution** is part of the protocol, not a tuning knob: both systems
   validate at the same joint-space resolution (default 0.05 rad max joint step). Changing
   it is a protocol change.

## 5. Reporting template

```json
{
  "date": "", "git_sha": "", "machine": {"cpu": "", "gpu": "", "driver": ""},
  "fixture": {"name": "", "hash": "", "triangles": 0},
  "system": "moveit-baseline | quevedomp-fcl | quevedomp-optix",
  "metrics": { "plan_ms_p50": 0, "plan_ms_p95": 0, "success_rate": 0,
               "path_len_mean": 0, "scene_update_ms_p50": 0,
               "batch_lat_us": {"10": 0, "50": 0, "100": 0},
               "bulk_cfg_per_s": 0 },
  "seeds": [], "notes": ""
}
```

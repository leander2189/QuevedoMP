# QuevedoMP R10 Design — Where planning time actually goes (self-collision first, GPU second)

> **Audience: the implementing agent.** This document is self-contained: it records the measured
> evidence, the decisions that follow from it, the verified code map (file:line as of commit
> `0ca2c48`; re-verify lines before editing — they drift), and a phase-by-phase execution plan
> with gates. Follow it top to bottom. Where this document says THROW / FAIL LOUDLY, that is the
> project invariant "no silent fallbacks" — do not soften it.
>
> **Status: NOT RATIFIED.** Measured and drafted with Leandro 2026-08-25. Records as the **next
> free ADR** when Phase A lands (outline in §11). At time of writing the highest ADR on disk is
> 021, with 022 reserved for R7, 023 claimed by both the R8 spec and the Vulkan design —
> **re-check `docs/architecture/` and take the next genuinely free number.**

---

## 0. Context and goal

### The end goal, as stated

> Complex, high-polygon environments where the RRT planner needs careful tuning because of
> bottlenecks. Tackle it with brute force: a 10× in collision checking on the GPU. Batch-oriented
> planners (batch RRT, PRM) and the studio then compound it, so that the setup produces
> sub-second trajectories.

Three of those four beliefs survive measurement. One does not, and it is the one the plan was
built on.

### What the measurements say

On the real cell (`bmt_9636` inlet, FCL, batch 10000 — the fat-batch regime the planner actually
issues), the full query decomposes as:

| pass | µs/config | |
|---|---|---|
| A floor (FK + link transforms + broadphase) | 0.430 | 7.7% of D |
| B environment only (self off) | 1.592 | |
| C self only (no env) | 5.523 | |
| **D full query** | **5.590** | |

Read D against B and C, **not** against the isolated deltas, because the environment pass
early-outs 18.6% of configs before the self pass ever runs:

- **Environment collision made entirely free: 5.590 → 5.523 = `1.01×`.** One percent.
- **Self-collision made entirely free: 5.590 → 1.592 = `3.51×`.**
- Everything but the floor made free: 5.590 → 0.430 = `13.0×` (the theoretical ceiling).

**The GPU backend accelerates the environment pass — the ~20% that early-out has already largely
paid for.** Self-collision is 91% of every query, it runs on the CPU in *both* backends, and its
cost is independent of environment polygon count.

### And brute force does not address the bottleneck problem at all

The saved hires problem burns **1.95M configs in 15 s and finds nothing** (R1 record,
`dtc_test_inlet/PROVENANCE.md`). At 6.7 µs/config that is ~13 s of pure collision checking, so a
genuine 10× makes it **fail in 1.5 s instead of 15 s**. The diagnosis is already on record from
the PRM connectivity work: an **isolated wedged goal**, and RRT-Connect's all-or-nothing
extension means a tree rooted in a narrow pocket cannot take a partial step out.

Throughput is time-to-solution for problems that are already solvable. This one is not.

### Ratified decisions (do not relitigate)

| # | Decision | Rationale |
|---|----------|-----------|
| **D1** | **Self-collision is the throughput lever; GPU environment work comes after it.** | Measured: 3.51× vs 1.01× on the real cell (§1.1). The ordering is not a preference, it is what the decomposition says. Doing them in the stated order also *creates* the conditions for D4 to pay: once self-collision stops dominating, the environment pass becomes the leading term and GPU work on it finally shows up in the total. |
| **D2** | **Solvability is a separate problem from throughput, and it comes first for Leandro's setup.** R8 (adaptive step + partial extension) is not superseded, deferred, or blocked by anything here. | No µs/config improves a plan that returns `NoSolution`. R10 must not be scheduled ahead of R8 on the grounds that it is "the performance work". |
| **D3** | **Within self-collision, cheapest lever first: integer ACM → reachability pruning → convex decomposition → GPU kernel.** Do NOT open with the GPU kernel (P7b). | Phase A is a hot-path data-structure fix with a verified precedent in the same file (§1.4) and no semantic change at all. P7b is a new kernel, a new differential-testing surface, and a new failure mode. If Phase A + B deliver most of the 3.51×, P7b may not be worth building — decide that with numbers, not in advance. |
| **D4** | **No single convex hulls on concave links.** Convex *decomposition* or nothing. | The inlet cell carries dress-kits and a jointA EE: wire-like and concave. A single hull around a dresskit sweeps up the free space between the hose and the arm, and would falsely block exactly the narrow motions R8 exists to find. A conservative-but-wrong self-collision result is a silent planning regression, which is worse than a slow one. |
| **D5** | **Reachability pruning must be conservative and verifiable.** A verification mode that re-enables every pruned pair and asserts agreement over a large sample is part of the deliverable, not a follow-up. | Over-pruning disables a real collision permanently and silently — the planner then happily returns paths that put the robot through itself. This is the single highest-severity risk in the document. |
| **D6** | **Re-measure after every phase; never propagate a modelled number.** | The 3.51× is an upper bound derived from one scene at one batch size. Early-out means the components are not additive (measured `D/sum = 0.84`), so partial improvements will not land where linear reasoning predicts. |
| **D7** | **The "10×" claim is honest only in two specific framings**, and any external statement must pick one: (a) against **single-core** FCL, or (b) against multi-core FCL with the **whole** query — self, environment and FK — on the GPU. | Against FCL on 16 idle cores at 4.4M triangles the measured gap today is 1.43× in FCL's favour with the cull on (§1.3). Claiming 10× against that configuration would be false. Note (a) is a legitimate deployment framing and is Leandro's own P8 argument — a real cell shares its cores. |
| **D8** | **Environment polygon count is not the problem and must stop being treated as the headline axis.** | Per-config cost is 7.3 µs on the low-poly cell and 7.6 µs on the 7.3M-triangle hires cell (R1 record) — a 4% penalty for 1800× the triangles. Today's independent sweep agrees: FCL grew 33% for 2.1× the triangles (§1.3). |

### Standing invariants that bind this work

- Batch-first collision; determinism per seed; one `Workspace` per thread; **no silent fallbacks**.
- Deviation D2: apt-only dependencies. FetchContent only by ratified decision. A convex-decomposition
  library (Phase B) is therefore a **ratification question**, not an implementation detail — see §5.
- Recorded-numbers-not-vibes: every performance claim reproducible from a versioned `.qmps` via
  `session_profile.py`, or from a benchmark binary with the command line recorded.
- The exact backend is the only collision certificate. Nothing in this document weakens that.

### Canonical build/test commands (from Windows, via WSL)

Note the **two different mount paths** — the CMake caches are not relocatable:
`bench-optix` was configured at `/workspace`, `dev-py` and `release-py` at `/work`.

```bash
# CPU suite (the gate for Phases A and B):
wsl -d Ubuntu-24.04 -- bash -lc "cd /mnt/d/Inventos/quevedoMP && docker run --rm \
  -v \$PWD:/work -w /work quevedomp-cuda bash -lc \
  'cmake --build build/dev-py -j 16 && ctest --test-dir build/dev-py -j 8 --output-on-failure'"

# The decomposition benchmark (the primary instrument for this work):
wsl -d Ubuntu-24.04 -- bash -lc "cd /mnt/d/Inventos/quevedoMP && docker run --rm --gpus all \
  -v \$PWD:/workspace -w /workspace -v \$PWD/.devcontainer/wsl-optix:/opt/wsl-optix:ro \
  quevedomp-cuda bash -lc 'export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\$LD_LIBRARY_PATH \
   && ./build/bench-optix/bench_dtc inlet'"

# End-to-end plan attribution (the number that actually matters):
wsl -d Ubuntu-24.04 -- bash -lc "cd /mnt/d/Inventos/quevedoMP && docker run --rm --gpus all \
  -v \$PWD:/work -w /work -v \$PWD/.devcontainer/wsl-optix:/opt/wsl-optix:ro quevedomp-cuda bash -lc \
  'export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\$LD_LIBRARY_PATH \
   && PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
      python3 examples/python/session_profile.py sessions/benchmark.qmps --seeds 8 --backend fcl'"
```

---

## 1. The evidence (measured 2026-08-25, RTX 4060 Ti, 16 cores, driver 595.97, commit `0ca2c48`)

### 1.1 Cost decomposition on the real cell

`bench_dtc inlet` — `bmt_9636` (UR10e + 500 mm lift + dress-kits + jointA EE, dof 7) against the
4,071-triangle work object. FCL, µs per config, `[]` = collision fraction:

| batch | A floor | B env | C self | D full | self/env | D/sum |
|---|---|---|---|---|---|---|
| 100 | 0.599 | 4.005 | 10.171 | 10.102 | 2.81× | 0.74 |
| 1000 | 0.449 | 1.695 | 5.993 | 6.675 | 4.45× | 0.92 |
| 10000 | 0.430 | 1.592 | 5.523 | 5.590 | 4.38× | 0.84 |

Structural anatomy behind those numbers, printed by the same run:

```
29 robot collision shapes (12 primitives, 10328 triangles total)
self pairs 406 total - 80 allowed/same-link = 326 CHECKED
environment: 1 object(s), 4071 triangles | ACM entries: 64
```

**326 link-shape pairs reach the narrowphase, per config.** That is the number Phase A and Phase B
attack.

### 1.2 It predicts the real planner

`session_profile.py sessions/benchmark.qmps --seeds 8 --backend fcl`:

```
seed 8: Success · 10 wp -> 7 smoothed
  total 676 ms · planner 5 ms · collision 671 ms · first solution 676 ms
  90253 configs · 18 queries (17 with batch>=256) · top sizes: 2270x2, 2x1, 386x1
```

671 ms / 90,253 configs = **7.4 µs/config**, against the decomposition's 6.675 µs/config at batch
1000. The model holds.

Two things to notice:

1. **Collision is 99.3% of the plan.** The planner itself costs 5 ms. There is no tree-management
   or nearest-neighbour work worth optimising.
2. **The planner already issues fat batches** — 18 queries for 90k configs, 17 of them ≥256, top
   size 2270. P3's Cartesian-bounded edge stepping turned each edge validation into one large
   batch. **There is no small-batch GPU win hiding here**; that problem was already solved.

Applying D's structure to the plan: self-collision free would take 671 ms → ~191 ms, i.e. the
whole plan 676 → ~196 ms, a **3.4×**.

### 1.3 Environment polygon count is nearly free

`bench_collision tests/fixtures/meshes/dummy_hires.stl` — UR5 vs one 4,380,564-triangle mesh,
self-collision off, batch 10000:

| threads | cull | FCL | OptiX | |
|---|---|---|---|---|
| 16 | off | **7.084 ms** | 13.846 ms | FCL 1.95× |
| 16 | on | **6.740 ms** | 9.620 ms | FCL 1.43× |
| 1 | off | 63.838 ms | **20.665 ms** | OptiX 3.09× |

Triangle-count sensitivity, 16 threads, cull off, batch 10000:

| triangles | FCL | OptiX |
|---|---|---|
| 2,079,292 | 5.306 ms | 14.215 ms |
| 4,380,564 | 7.084 ms | 13.846 ms |
| | **+33%** | **−3%** |

**OptiX is triangle-count-invariant; FCL is not.** That confirms OptiX's cost is pipeline
(upload → launch → download), not traversal. Solving Amdahl across the two thread counts, cull
off, batch 10000:

| | parallel host work | serial floor |
|---|---|---|
| FCL | 60.5 ms | 3.3 ms |
| OptiX | 7.3 ms | **13.4 ms** |

OptiX's parallel part matches the measured host FK floor (0.773 µs/config × 10000 = 7.73 ms)
almost exactly — **FK is the only host work OptiX parallelises**, and the other 13.4 ms is a fixed
pipeline cost no core count touches. The cull cuts that floor to ~9.2 ms (−31%).

Extrapolating FCL's growth linearly, OptiX-with-cull would overtake FCL at roughly **8M
triangles on 16 idle cores**. Treat that as order-of-magnitude only — FCL's BVH scaling is
sub-linear, so the true crossover sits higher. `inlet_mesh_hires.stl` is 7.3M triangles, i.e.
right at the estimate, so **measure it rather than trusting this paragraph**.

### 1.4 The hot-path defect Phase A fixes (verified by reading)

[`fcl_scene.cpp:191-213`](../src/collision/fcl_scene.cpp#L191-L213) — `self_callback`, invoked by
FCL's broadphase for every overlapping link-shape pair, for every config:

```cpp
const std::string &na = d->model->links()[li].name;
const std::string &nb = d->model->links()[lj].name;
if (d->acm->is_allowed(na, nb))
  return false;
```

`AllowedCollisionMatrix::is_allowed` ([`robot_instance.hpp:22-24`](../include/quevedomp/robot/robot_instance.hpp#L22-L24))
calls `key(a, b)`, which returns `std::pair<std::string, std::string>` **by value** — two string
copies — and then does an O(log n) lookup in a `std::set` of string pairs, comparing strings at
every node.

**The precedent for the fix is thirty lines above it in the same file.** Task 3.3d P4 already did
exactly this for the environment path: `compute_env_acm`
([`fcl_scene.cpp:334`](../src/collision/fcl_scene.cpp#L334)) resolves the ACM's string pairs to
integer keys **once per `query_batch`**, with the comment "resolved ONCE per query_batch from the
ACM's string pairs to integer keys the hot path can hash." The self path never got the same
treatment.

### 1.5 Prerequisite already landed: the `load_mesh` unit bug

Found while producing §1.3 and fixed in the same session — recorded here because every number
above depends on it. `AI_CONFIG_PP_FD_REMOVE` left assimp's area test enabled, and that test
compares against an **absolute** 1e-6 square units. On a metre-scale mesh it deleted every
triangle under 1 mm²: **2,301,272 of 4,380,564 triangles (53%) silently discarded**, purely
because the file was authored in metres rather than millimetres.

Fix: `AI_CONFIG_PP_FD_CHECKAREA = 0` plus a local exactly-zero-area filter that cannot open a hole
in a watertight mesh. Regression test `MeshIo.DegeneracyFilterIsScaleInvariant` asserts equal
triangle counts for the same geometry at metre and millimetre scale, and was confirmed to fail
without the fix (32 vs 64). This was a **collision-fidelity** bug, not just a benchmark artifact:
sub-millimetre features vanished from every metre-scale mesh the library loaded.

---

## 2. The plan

Four tracks. Only B, C and D are R10; A is R8 and is listed because the ordering matters.

| track | problem | lever | measured worth |
|---|---|---|---|
| **A** | Solvability — the wedged goal | R8 adaptive step + partial extension; bridge sampling | unbounded (no solution → solution) |
| **B** | Self-collision, 91% of every query | integer ACM → pair pruning → convex decomposition → P7b | up to 3.51× |
| **C** | Environment throughput | cull default, streams, joint-angle upload, GPU FK | 1.01× today; grows after B |
| **D** | Config count | PRM (built), batch RRT | compounds with all |

Track A is **not** in R10's scope and must not be scheduled behind it.

---

## 3. Phase A — integer self-ACM (no semantic change)

**Goal:** remove the string work from `self_callback`. Zero behaviour change; the phase is a pure
speed fix and its gate is bit-identical output.

**Design.** Mirror `compute_env_acm`. Resolve, once per `query_batch`, a `SelfAcm` that answers
"is link-index pair (i, j) allowed?" without touching a string:

- Build from `robot.acm().pairs()` plus the model's link-name → index map.
- Store as a symmetric bitset over link-index pairs — `n_links` is small (tens), so a
  `std::vector<std::uint64_t>` indexed `i * stride + j/64` is both compact and branch-free.
- `SelfData` carries a pointer to it instead of to the `AllowedCollisionMatrix` and `RobotModel`.
- The `li == lj` same-link early-out stays exactly where it is — it is already integer-only.

**Invariant to preserve:** the ACM is mutable on `RobotInstance`, so the resolved bitset is valid
only for the duration of one `query_batch`. Resolve it in the same place `compute_env_acm` is
resolved and pass it down. Do **not** cache it on the scene across calls — R7 will attach objects
and mutate the ACM, and a stale mask is a silent wrong answer.

**Gate:** the full CPU suite green (215/215), plus `session_profile.py sessions/benchmark.qmps
--seeds 8 --backend fcl` producing an **identical** path and config count, with a lower collision
time. Record the before/after.

**Expected:** unknown until measured — that is the point of D6. The work removed is two string
copies and a string-set descent per broadphase pair per config, on the order of 29M lookups for a
90k-config plan.

---

## 4. Phase B — self-pair reachability pruning

**Goal:** cut the 326 checked pairs by disabling those that can never collide inside the joint
limits. Standard practice (MoveIt does this in its SRDF generator); the win is that a pruned pair
never reaches the narrowphase at all.

**Design.** An offline, once-per-robot analysis producing additional ACM entries:

- Sample N configs uniformly in the joint limits (deterministic, seed recorded — the project's
  determinism invariant applies to tooling too).
- For each sampled config record, per link-shape pair, whether the pair collides *and* the minimum
  separation observed.
- A pair is a **prune candidate** only if it never collided across all N samples **and** its
  minimum observed separation exceeds a margin. Never-collided-in-N-samples alone is not
  sufficient evidence and must not be the criterion on its own.
- Emit as SRDF `<disable_collisions>` entries so the result is inspectable, diffable and
  reviewable by a human — not as an opaque binary blob.

**D5 obligations (non-negotiable).** Over-pruning is a silent correctness failure: the planner will
return paths that put the robot through itself, and no test will notice unless one is written.
Deliver, in the same phase:

- A `--verify` mode that loads the pruned SRDF, re-enables every pruned pair, and asserts identical
  collision results over a fresh, larger sample with a different seed. THROW on any disagreement.
- The margin and sample count recorded in the generated SRDF as a comment, so a later reader knows
  how much evidence stands behind each disabled pair.
- A note in the ADR that this is a *heuristic over sampling*, not a proof. It is not a certificate,
  and the exact backend remains the only one.

**Gate:** `--verify` clean at 10× the generation sample count; full CPU suite green; the inlet
decomposition re-run and the new `self` number recorded.

---

## 5. Phase C — convex decomposition of robot collision geometry (RATIFICATION REQUIRED)

**Goal:** make the narrowphase cheap. 10,328 triangles across 17 mesh shapes currently go through
FCL BVH-vs-BVH traversal; convex shapes go through GJK/EPA instead, which for boolean queries is
substantially faster.

**This phase is blocked on two decisions and must not start without them.**

1. **D4 — hulls are forbidden on concave links.** The dress-kits and the jointA EE are wire-like
   and concave. A single hull around a dresskit claims the free space between hose and arm, which
   would falsely block precisely the narrow motions R8 exists to find. Either every mesh link gets
   a genuine convex *decomposition*, or this phase does not happen.
2. **Deviation D2 — dependencies are apt-only.** A convex decomposition library (V-HACD or
   equivalent) is a new third-party dependency and therefore a ratification question for Leandro,
   not an implementation detail. The alternative is offline decomposition in Blender with the
   results committed as fixtures, which costs nothing at build time and keeps D2 pristine — that
   is the recommended route given the meshes are already authored in `DTC_Test.blend`.

**Gate, in addition to the suite:** a differential run asserting that the decomposed robot reports
the *same* collision verdicts as the triangle-mesh robot over a large config sample. Any pair where
the decomposition is more conservative is a finding to report, not a rounding error to accept.

---

## 6. Phase D — P7b, GPU self-collision (decide with numbers)

Currently parked in the roadmap as "the 'GPU frees the CPU' lever if deployments get core-starved".
§1.1 upgrades it from a core-starvation contingency to **the largest single throughput item in the
project** — but only if Phases A–C leave enough of the 3.51× unclaimed to justify a new kernel, a
new differential surface and a new failure mode.

**Do not open this phase before re-running the decomposition after Phase B/C.** If self drops from
5.523 to, say, 2 µs/config on the CPU, the remaining headroom may not warrant it.

Design notes for whoever does open it: this is 326 pairs of convex-or-BVH shapes per config, which
is a fundamentally different shape of problem from ray-casting against a static environment GAS —
it is not a small extension of `OptixScene`, and the Vulkan design's `GpuRayScene` extraction
(Phase A of that document) does not cover it.

---

## 7. Phase E — the GPU environment pipeline (the original plan, correctly placed)

Worth 1.01× today. Worth doing **after** B–D, when the environment pass is the leading term. In
priority order, all measured in §1.3:

1. **Make the broadphase cull the default.** `QUEVEDOMP_OPTIX_CULL=1` is already implemented and
   opt-in: 13.846 → 9.620 ms, −31%. Sweep it across the fixtures; if it never regresses, flip the
   default and delete the env var. Cheapest item in this document.
2. **Overlap transfer with compute.** The 13.4 ms serial floor is upload → launch → download,
   serialised, and the chunking added for the 2³⁰-thread launch cap runs each chunk's sequence in
   order. Pipeline chunks across 2–3 CUDA streams with pinned host memory.
3. **Upload joint angles, not link transforms.** ~10 links × 12 floats × 10000 configs ≈ 4.8 MB per
   batch, against 7 doubles × 10000 ≈ 0.56 MB — 8.5× less traffic across the exact bottleneck in
   (2). Requires (4).
4. **FK on the GPU.** Removes OptiX's entire 7.3 ms parallel part and is what makes (3) possible.
   After B–D this becomes the floor, so it is also the last item that can move the total.

**Also:** `BackendHint::Auto` currently routes on batch size (≥256). §1.3 says the axes that decide
the winner are **triangle count and available cores**; batch size is close to irrelevant above
~1000. Re-derive the routing rule from measurements once (1)–(4) land.

---

## 8. Commit sequence (each commit leaves the tree green)

| # | Commit | Gate |
|---|---|---|
| 1 | `perf(collision): integer self-ACM resolved once per query_batch` | suite green; identical plan output; collision time recorded |
| 2 | `feat(tools): self-pair reachability analysis → SRDF, with --verify` | `--verify` clean at 10× sample; suite green |
| 3 | `fix(fixtures): pruned SRDF for rbrobout_inlet + rbrobout` | decomposition re-run; new `self` number recorded |
| 4 | *(gate)* re-measure; decide whether Phases C/D are still worth it | numbers in the roadmap record |
| 5 | `perf(optix): broadphase cull on by default` | no regression on any fixture |
| 6+ | Phases C / D / E as ratified | per phase above |

Commits 1–3 are unconditional. Commit 4 is a **decision point with Leandro**, not a formality.

---

## 9. Numbers to record (ADR / roadmap record)

Re-run and record after each phase, on the same machine and session:

- `bench_dtc inlet` decomposition: A / B / C / D at batch 100 / 1000 / 10000, plus the anatomy line
  (shapes, checked pairs, triangles).
- `session_profile.py sessions/benchmark.qmps --seeds 8 --backend fcl`: total / planner / collision
  ms, config count, batch histogram.
- Checked-pair count before and after pruning, with the sample count and margin used.
- `bench_collision tests/fixtures/meshes/dummy_hires.stl` at 1 and 16 threads, cull on and off.
- **The hires problem specifically**: whether it now solves at all (that is R8's gate, but it is the
  number Leandro actually cares about, so record it here too).

Baseline, 2026-08-25, RTX 4060 Ti / 16 cores / driver 595.97 / `0ca2c48` / `bench-optix` preset, is
§1 of this document.

---

## 10. Risks and stated limitations

| risk | mitigation |
|---|---|
| **Over-pruning silently disables a real self-collision.** Highest-severity item here. | D5: mandatory `--verify` mode, separation margin not just non-collision, human-readable SRDF output. |
| Convex decomposition is more conservative than the mesh and blocks narrow motions. | D4 forbids hulls; Phase C gate is a differential run against the triangle-mesh robot. |
| The 3.51× is an upper bound from one scene at one batch size, and early-out makes components non-additive (`D/sum = 0.84`). | D6: re-measure every phase. Do not quote 3.51× as an achieved number anywhere. |
| The decomposition was measured on a **4,071-triangle** environment. On a high-poly cell the env share rises and the self share falls. | The ordering still holds — self-collision cost is invariant to environment complexity, so it is a floor under every scene — but re-run §1.1 against the hires inlet before committing to Phase E's placement. |
| `bench_dtc` requires OptiX and a GPU, so the primary instrument does not run in a CPU-only CI. | Phases A–C gate on the CPU suite + `session_profile.py`, both of which run without a GPU. The decomposition is a development instrument, not a gate. |

**Stated limitation:** nothing in R10 improves success rate, path quality, or solvability. It makes
solvable problems faster. If the hires problem still returns `NoSolution` after all of R10, that is
the expected outcome and R8 is the answer.

---

## 11. ADR outline (`docs/architecture/adr-0NN-self-collision-first.md`)

- **Context**: the end goal was framed as "10× GPU collision checking for high-poly cells". The
  decomposition says environment collision is ~1% of the full query on the real cell and
  self-collision is 91%.
- **Decision**: reorder the performance work — self-collision first (integer ACM → pruning →
  decomposition → P7b), GPU environment pipeline after. Keep R8 ahead of both for the wedged-goal
  problem.
- **Consequences**: P7b moves from "parked contingency" to a live candidate gated on measurement;
  `BackendHint::Auto`'s batch-size routing rule is invalidated and must be re-derived from triangle
  count and core budget; any external performance claim must adopt one of D7's two framings.
- **Evidence**: §1 of this document, with the commands to reproduce it.
- **Rejected**: opening with P7b (D3); single convex hulls (D4); treating environment polygon count
  as the headline axis (D8).

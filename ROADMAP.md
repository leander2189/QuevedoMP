# QuevedoMP — Roadmap

**QuevedoMP** (Quevedo Motion Planner) is a ROS-free, GPU-accelerated C++20 library for robot-arm
motion planning in complex, quasi-static industrial cells. Deterministic per seed, batch-first
collision checking, CPU and GPU backends behind one interface, and an exact collision certificate
on every path it returns.

This file and [`README.md`](README.md) are the two documents to follow the project. The README is
the quick start; this is the plan. Everything else is linked from one of the two.

---

## 1. What works today

Everything in this section is built, tested and in `main`. **215 tests** pass on the CPU suite;
the OptiX backend adds a GPU suite and a three-way differential check.

| Area | What you get | Record |
|---|---|---|
| **Robot model** | URDF parsing (urdfdom), SRDF allowed-collision matrices, mesh loading via assimp (STL/DAE/OBJ, normalized to metres, scale-invariant degeneracy filter), `package://` resolution | — |
| **Kinematics** | FK, analytic Jacobian, IK validated to <1e-9 / <1e-6, and multi-branch `solve_all` returning distinct IK branches deterministically | [ADR-015](docs/architecture/adr-015-fk-location.md) |
| **Collision** | One `CollisionScene` interface, **FCL** (CPU, OpenMP-parallel batches) and **OptiX** (GPU) backends, boolean + signed-distance queries, padding and safety margins, raycast containment, ACM covering robot×robot *and* robot×environment pairs | [ADR-012](docs/architecture/adr-012-raycast-containment.md), [ADR-013](docs/architecture/adr-013-padding-and-margins.md), [ADR-014](docs/architecture/adr-014-optix-batched-raygen.md), [optix-collision.md](docs/optix-collision.md) |
| **Edge validation** | Cartesian-bounded edge stepping — steps sized so no link sweeps more than `max_link_sweep` metres, instead of a joint-space resolution that means different things on different robots | — |
| **Clearance** | `ClearanceField`: GPU voxel SDF (exact seed + CUDA jump flooding, OpenMP fallback agreeing to <1 mm), conservative robot sphere cover, batched distance + gradient queries. 16.8M voxels @ 10 mm in 1.34 s | [ADR-018](docs/architecture/adr-018-clearance-field.md) |
| **Planning** | RRT-Connect, PRM roadmap planner (build once, answer many queries), CHOMP-flavored trajectory refiner over the clearance field, batched and time-budgeted shortcut smoothing | [ADR-019](docs/architecture/adr-019-trajectory-refiner.md), [ADR-020](docs/architecture/adr-020-prm-roadmap-planner.md) |
| **Time parameterization** | TOPP-RA-style joint/tip velocity and acceleration limits, validated against `toppra` to ≤2%, plus a certified jerk-limited mode | [ADR-017](docs/architecture/adr-017-time-parameterization.md), [spec](docs/topp_jerk_tip_spec.md) |
| **Python API** | nanobind bindings across types, robot, collision, planning, clearance, parameterization and capture | [ADR-016](docs/architecture/adr-016-python-slice-and-studio.md) |
| **Studio** | Browser-based planning IDE (viser + rerun): Scene / IK / Plan / Trajectory / Tasks modes, IK gizmo, obstacle editing, plan playback and scrubbing, PRM connectivity diagnostics, goal-escapability probe, clearance heatmap, `.qmps` session save/load | [ADR-021](docs/architecture/adr-021-studio-working-modes.md) |
| **Determinism** | Bit-identical results per seed across every stochastic component | [ADR-006](docs/architecture/adr-006-rng.md) |

**Measured, on an RTX 4060 Ti / 16 cores** (protocol: [`docs/benchmarks/PROTOCOL.md`](docs/benchmarks/PROTOCOL.md)):

- Reference plan (`sessions/benchmark.qmps`, 7-DOF industrial cell): **0.35 s**, down from 57.9 s
  over the course of v0 optimization.
- Collision throughput: **1.4M configs/s** on a 4.4M-triangle environment (FCL, batch 10000,
  16 threads).
- Environment polygon count is nearly free: **7.3 µs/config** low-poly vs **7.6 µs/config** at 7.3M
  triangles.

### Known limitations, stated on purpose

- **Narrow passages need tuning.** A goal wedged in a tight pocket can defeat RRT-Connect's
  all-or-nothing extension; the tree never takes a step. This is what **R8** fixes, and it is the
  most likely thing to bite a new user. See [`docs/tutorials/rrt-tuning.md`](docs/tutorials/rrt-tuning.md).
- **OptiX needs a login-gated SDK at build time**, which is why the binary is not yet
  self-contained. **Vulkan** fixes this.
- The GPU backend currently wins only when cores are scarce or triangle counts are very high — see
  [R10](docs/QuevedoMP-R10-DESIGN.md) for the measured reason and the plan.

---

## 2. What's next, in order

**Ordering decision (2026-08-25): sell first.** Items 1–4 are the product-readiness bundle and
come ahead of all engineering. They need no new library code, so they can proceed while
engineering questions stay open.

> **Recorded risk of this ordering.** Items 1–4 produce demos and evaluation material before **R8**
> lands, so any prospect who brings a narrow-passage problem will hit the wedged-goal failure in
> their own evaluation. Mitigations: keep the limitation stated above in the pitch itself, choose
> demo scenes deliberately, and treat R8 as the first engineering item after the bundle.

| # | Item | Size | Detail |
|---|---|---|---|
| **1** | [**Sales pitch**](docs/PITCH.md) — technical one-pager for engineers evaluating QuevedoMP against MoveIt, cuRobo or in-house code, with inline image/video suggestions and a 10-slide deck cut. | S | ✅ 2026-08-25 — **blocked on the license** before it leaves the building, see §3.1 |
| **2** | **C++ API documentation** — Doxygen over the public headers, `docs` CMake target, output in the build tree. | S | ✅ 2026-08-25 — §3.2 |
| **3** | **Python API documentation** — generated from the nanobind docstrings, same build step, one published surface with the C++ docs. | S | §3.3 |
| **4** | **Worked examples** — a small set of C++ and Python programs a newcomer can read end to end: load a robot, add obstacles, plan, parameterize, inspect. Distinct from the existing `examples/` visualizers, which are development tools. | S–M | §3.4 |
| **5** | [**R8 — adaptive-step RRT-Connect**](docs/QuevedoMP-R8-DESIGN.md) | S–M | Per-node extension step sized from the local width of free space, so a tree rooted in a narrow pocket can crawl out where a single global step cannot. **The fix for the headline limitation.** Spec written 2026-07-22. |
| **6** | [**R10 — planning-throughput reorder**](docs/QuevedoMP-R10-DESIGN.md), phases A–B | M | Self-collision is 91% of every query. Integer self-ACM, then self-pair reachability pruning. Measured worth: making self-collision free is 3.51×, against 1.01× for the environment pass everyone assumed was the bottleneck. |
| **7** | [**R7 — attached objects + task layer**](docs/QuevedoMP-R7-DESIGN.md) | M | Grasped parts move collision geometry and the ACM with them; an MTC-lite `quevedomp_tasks` layer above it; the studio Tasks mode becomes its inspector. Design ratified and written. |
| **8** | [**Vulkan backend**](docs/QuevedoMP-VULKAN-BACKEND-DESIGN.md) | M–L | Replace the login-gated OptiX SDK with apt-installable Vulkan `VK_KHR_ray_query`, so QuevedoMP ships as a self-contained binary. **Deployability, not performance.** Design written, not ratified. |
| **9** | [**R10 phases C–E**](docs/QuevedoMP-R10-DESIGN.md) | M–L | Convex decomposition, GPU self-collision, and the OptiX pipeline work — each gated on re-measuring after phase B rather than assumed. |
| **10** | [**R9 — dresskit analysis**](docs/QuevedoMP-R9-DESIGN.md) | M | Clamped elastic-rod solver over a planned trajectory reporting clearance, bend radius, slack and fatigue. Post-processing only, never a planning constraint. Pure Python, zero C++ changes. |

---

## 3. Detail for the unwritten items

Items 5–10 have their own design documents, linked above. Items 1–4 do not yet, so their scope is
recorded here.

### 3.1 Sales pitch — delivered, with one blocker

Written: [`docs/PITCH.md`](docs/PITCH.md). Audience: **technical buyer / integrator** — a robotics
engineer choosing a planning stack.

> **Blocker before it goes outside: [`LICENSE`](LICENSE) says "not yet finalized".** A technical
> buyer opens that file, and in procurement an unsettled license is a hard stop. It is also the one
> place the project would otherwise have an unambiguous advantage to state against cuRobo. Settle it
> before the deck leaves the building.

The constraints it was written under, kept here because they bind any revision:

- Lead with what is architecturally different: ROS-free core, one backend interface, determinism
  per seed, batch-first collision, an exact certificate on every returned path.
- Every number carries its conditions per [`PROTOCOL.md`](docs/benchmarks/PROTOCOL.md) §1. **Do not
  claim a GPU speedup without its core budget, and do not sell polygon count as the differentiator**
  — it is nearly free, and the honest claim is robustness, not speed.
- State the limitations in §1 above. A technical evaluator finds them in an afternoon; finding them
  in our own document instead is worth more than hiding them.
- Comparison against MoveIt 2 and cuRobo is **positioning, not benchmark data** — the MoveIt
  baseline was retired unbuilt. Say what is different, not what is faster, unless a number exists.
- Suggested visuals to capture: studio plan playback on the inlet cell; the PRM connectivity view
  colored by component; the clearance heatmap slice; a scrubbed trajectory with the velocity and
  jerk plots; a side-by-side of low-poly vs 4.4M-triangle scenes at the same per-config cost.

### 3.2 C++ API documentation — delivered

`cmake --build <dir> --target docs` → 395 pages over the nine public modules, with class graphs.
Config in [`docs/doxygen/`](docs/doxygen/); output goes to the build tree, so nothing generated is
committed. `doxygen` and `graphviz` were added to the dev container.

The one interesting problem: the headers are written to be read as source, in plain `//` comments,
which Doxygen does not recognise at all — a straight run produces signatures with no prose.
[`comment-filter.py`](docs/doxygen/comment-filter.py) rewrites them as it feeds Doxygen, so the
source keeps no markup. It is careful about three cases that would otherwise produce *quietly wrong*
documentation: the file-header block (would attach to the first struct in the file), trailing
comments (would attach to the next declaration — 146 of them in the public headers), and section
separators. Nine "unsupported xml/html tag" warnings remain and are deliberate: Doxygen renders
those placeholders correctly, and escaping them makes the output worse. That was checked against
generated HTML, not assumed.

### 3.3 Python API documentation

Generated from the nanobind signatures and docstrings in `bindings/python/src/bind_*.cpp`,
published alongside the C++ docs. Decide one generator for both surfaces rather than two toolchains.

### 3.4 Worked examples

`examples/` today holds five C++ visualizers and two Python profiling scripts — development tools,
not teaching material. Add a short, commented, runnable path through: load a robot → build a scene
→ plan → smooth → parameterize → inspect the result, in both languages. These double as the code
snippets the pitch and the docs quote.

---

## 4. On the shelf

Each parked with its reason. Not abandoned; not scheduled.

| Item | Why it is parked |
|---|---|
| **MoveIt 2 baseline comparison** | **Retired 2026-08-25**, not parked. Never built, and the project no longer positions on beating it. `PROTOCOL.md` was rewritten around the instruments that exist. `QuevedoMP-BUILD-PLAN.md` keeps the original gate language as the frozen v0 record. |
| **Capture/replay v3** | Design settled 2026-07-15: `.qmps` v3 with recorded attempts and deterministic replay by seed, no core changes. Build when wanted. |
| **P5 goal-sampling budget** | Dropped for v0 — `solve_all` plus the studio branch picker already cover the manual workflow. Wire `resolve_goal` to `solve_all` only if an application needs it. |
| **Scene-complexity backend routing (P8)** | Rejected with evidence: FCL wins by taking 16 idle cores, which a real cell does not have. [R10](docs/QuevedoMP-R10-DESIGN.md) supersedes the question — `BackendHint::Auto` should route on triangle count and core budget, not batch size. |
| **MCAP, OMPL cross-check, wheels, notebook** | Phase 4b polish. |

---

## 5. Standing invariants

These bind every item above. They are not negotiable per-feature.

- **Batch-first collision.** The planner issues fat batches of independent configs; anything that
  forces sequential single queries is a design error.
- **Determinism per seed.** Bit-identical results, every stochastic component, every backend.
- **One `Workspace` per thread.** Sharing one is out-of-bounds undefined behavior, not merely wrong.
- **No silent fallbacks.** If a backend cannot do what was asked, it throws. It never quietly
  degrades.
- **The exact backend is the only certificate.** Approximate structures — the clearance field
  above all — inform optimization and visualization. They never certify a path.
- **Recorded numbers, not vibes.** Every performance claim reproducible from a versioned `.qmps`
  or a benchmark command line, per [`PROTOCOL.md`](docs/benchmarks/PROTOCOL.md).
- **apt-only dependencies** (deviation D2). `FetchContent` only by ratified decision.
- **ADR discipline.** Significant decisions get a record in [`docs/architecture/`](docs/architecture/).

---

## 6. History

- [`docs/QuevedoMP-SPEC.md`](docs/QuevedoMP-SPEC.md) — the architecture specification.
- [`docs/QuevedoMP-BUILD-PLAN.md`](docs/QuevedoMP-BUILD-PLAN.md) — the v0 build plan. **Frozen**:
  every item is either done-with-evidence or listed in §4. Historical record, not a live plan.
- [`docs/QuevedoMP-ROADMAP-v1.md`](docs/QuevedoMP-ROADMAP-v1.md) — the post-v0 feature plan this
  file supersedes. Retained for the R2–R6 completion records, which are the evidence behind §1.

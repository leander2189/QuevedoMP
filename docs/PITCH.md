# QuevedoMP — a motion planning library you can actually ship

> **Audience: the engineer choosing a planning stack.** This document is the source for the deck;
> `> 🎬` and `> 🖼️` blocks are capture suggestions, not content. Every number carries the conditions
> it was measured under, per [`benchmarks/PROTOCOL.md`](benchmarks/PROTOCOL.md). Nothing here is
> aspirational — if it is in this document, it is in `main` and tested.

---

## The pitch in one paragraph

**QuevedoMP is a ROS-free C++20 motion planning library for industrial robot arms.** One collision
interface sits over a CPU (FCL) and a GPU (OptiX) backend, so the same planner code runs on a
workstation, an industrial PC, or a GPU box without changing. Every result is **bit-identical for a
given seed**, and every path it returns carries an **exact collision certificate** — not a
probabilistic one. It comes with a browser-based planning IDE that turns "the planner failed" from a
mystery into a diagnosis.

> 🖼️ **Title slide:** the studio with the UR10e-on-lift cell loaded, work object visible, a planned
> trajectory drawn as a swept EE curve.

---

## 1. Five things that are actually different

### 1.1 ROS is an adapter, not a dependency

No ROS type appears anywhere in the library. You link a C++20 library and call it — from a control
loop, a service, a test harness, or a Python notebook. ROS 2 integration is something you add on top
if you want it, not a runtime you inherit.

**Why an integrator cares:** the planner stops dictating your process architecture, your build
system, and your deployment image.

### 1.2 Deterministic to the bit, per seed

Every stochastic component — sampling, planner, smoother, roadmap construction — reproduces exactly
from its seed. Same seed, same tree, same path, same timing profile.

**Why an integrator cares:** a planning failure on the line is reproducible on your desk. This is
the difference between "we've seen it hang occasionally" and a regression test. For anyone who has
to argue about robot behaviour in a validation document, this is the feature that matters most, and
it is the one most planning stacks cannot offer.

### 1.3 An exact certificate on every path

Approximate structures exist in the library — a GPU voxel signed-distance field drives the
trajectory optimizer — but **they never certify a path**. The final answer always comes from an
exact collision backend. Approximation informs the search; it never signs off on it.

**Why an integrator cares:** "the optimizer thought it was clear" is not a defence when a
€200k end-effector hits a fixture.

### 1.4 The GPU backend is validated against the CPU one, not asserted

The two backends are held to the same semantic contract by differential testing: identical verdicts
across large random config sets, with out-of-band disagreements required to be zero. On the
4.4M-triangle scene, **0 disagreements in 2000 poses**.

**Why an integrator cares:** a GPU collision checker that is subtly wrong is worse than no GPU
collision checker. You get the speed with the CPU implementation as a live oracle.

### 1.5 Batch-first, so the GPU is usable at all

The planner issues **fat batches of independent configurations** — a real plan on the reference cell
runs 90,253 configurations in 18 queries. Architectures that check one configuration at a time
cannot use a GPU no matter how fast the kernel is, because per-launch overhead dominates. This one
choice, made at the interface level, is what makes GPU acceleration possible.

> 🖼️ **Slide:** a diagram of the layered architecture — planner → `CollisionScene` interface → {FCL,
> OptiX} — with the ROS adapter drawn *outside* the box.

---

## 2. What it does today

| | |
|---|---|
| **Robot model** | URDF + SRDF, `package://` mesh resolution, STL/DAE/OBJ, seven bundled robot fixtures |
| **Kinematics** | FK, analytic Jacobian, IK to <1e-9 position / <1e-6 orientation, and multi-branch IK returning every distinct elbow/wrist solution |
| **Collision** | Boolean and signed-distance queries, padding and safety margins, allowed-collision matrices over robot×robot *and* robot×environment pairs, containment detection |
| **Planners** | RRT-Connect; PRM roadmaps (build once, answer many queries in milliseconds); a CHOMP-flavoured trajectory optimizer; batched, time-budgeted shortcut smoothing |
| **Trajectories** | TOPP-RA-style time parameterization honouring joint *and* tool-tip velocity/acceleration limits, plus a certified jerk-limited mode |
| **Clearance** | GPU voxel SDF of the cell with distance and gradient queries — 16.8M voxels at 10 mm resolution in 1.34 s |
| **Python** | The whole surface, via nanobind |
| **Studio** | Browser-based IDE: scene editing, IK gizmo, planning, playback and scrubbing, velocity/jerk plots, and the diagnostics in §4 |

**215 tests**, plus fuzz targets and cross-backend differential suites.

---

## 3. Measured performance

All on one RTX 4060 Ti with 16 CPU cores. Conditions stated because a planning number without them
means nothing.

| | |
|---|---|
| Reference plan, 7-DOF industrial cell | **0.35 s** end to end (plan + smooth) |
| Collision throughput | **1.4M configurations/s** on a 4.4M-triangle environment |
| Roadmap queries after one PRM build | single-digit milliseconds |
| Clearance field, 16.8M voxels @ 10 mm | 1.34 s on GPU, 2.0 s on CPU, agreeing to <1 mm |
| Jerk-limited parameterization cost | +4.7% trajectory duration on a UR5 |

**The honest headline is robustness, not raw speed.** Per-configuration cost is **7.3 µs on a
low-poly cell and 7.6 µs on the same cell at 7.3M triangles** — a 4% penalty for 1800× the geometry.
We do not claim that high polygon counts make us fast. We claim they do not make us slow, which is
what actually bites teams who try to plan against scanned or CAD-accurate cells.

> 🖼️ **Slide:** two screenshots side by side — the low-poly work object and the 7.3M-triangle scan —
> captioned with 7.3 µs and 7.6 µs. The point lands visually in one second.

> 🎬 **Video (30 s):** load the 4.4M-triangle cell in the studio, hit Plan, watch it solve. No cuts.
> The absence of a loading spinner *is* the demo.

---

## 4. The studio is a differentiator, not a demo

Most planning stacks give you a visualizer. This is a diagnostic tool.

- **Goal-escapability probe** — answers "can the robot move *at all* from this goal?" before you
  waste an afternoon tuning a planner against a goal pose that is physically wedged.
- **Roadmap connectivity by component** — colours the PRM by connected component, so a start and
  goal in different components is visible instead of inferred from a timeout.
- **Clearance heatmap** — a slice through the cell showing how much room the robot actually has.
- **Multi-branch IK picker** — every distinct IK solution listed and annotated free or colliding,
  so you choose the branch instead of accepting whatever the solver returned first.
- **Session save/load** — a `.qmps` file captures the entire problem: robot, ACM, scene geometry,
  start, goal, planner parameters. Send it to us and we reproduce your failure exactly.

**Why an integrator cares:** commissioning time is dominated by *diagnosing* why a motion won't
plan, not by planning speed. That last point is also the support story — a bug report is a file, not
a conversation.

> 🎬 **Video (45 s):** the money demo. A goal that won't plan → open the escapability probe → it
> reports the goal is wedged → nudge the goal pose → it plans immediately. This is the single most
> persuasive thing in the product, because every robotics engineer in the room has lost a day to it.

> 🖼️ **Slide:** the PRM connectivity view, two components in different colours, start in one and
> goal in the other.

---

## 5. Where it fits

**Positioning, not benchmark data.** We have not run head-to-head comparisons; these are
architectural differences, and we say so rather than inventing numbers.

| | Their strength | Where QuevedoMP differs |
|---|---|---|
| **MoveIt 2** | Enormous ecosystem, many planners via OMPL, the ROS default | We are not a ROS application. Deterministic per seed. A GPU collision backend under the same interface. Batch-first collision. Far smaller surface to integrate and to audit. |
| **cuRobo** | GPU-native and genuinely fast | We run without a GPU at all, keep an exact certificate rather than an approximate one, and keep the CPU backend as a live correctness oracle. Licensing terms are ours to set. |
| **In-house planner** | Fits your cell exactly | 215 tests, cross-backend differential validation, a written benchmark protocol, and a decision record for every architectural choice. The parts nobody has time to build in-house. |

---

## 6. What it does not do yet

An evaluator finds these in an afternoon. Better they find them here.

- **Narrow passages need tuning.** A goal wedged in a tight pocket defeats the current
  RRT-Connect's all-or-nothing extension — the tree never takes a step. The studio *diagnoses* this
  today (§4); the planner fix is specified and is the next engineering item. If your application is
  dominated by tight-clearance insertion, talk to us about timing before committing.
- **Not yet a self-contained binary.** The GPU backend builds against NVIDIA's OptiX SDK, which is
  login-gated. A Vulkan port that removes this is designed and not yet built. The CPU library has no
  such constraint.
- **The GPU backend wins in specific regimes** — scarce CPU cores, or very high triangle counts. On
  a 16-core workstation with a normal cell, the CPU backend is faster, and the library will tell you
  so. We publish the measurement rather than the marketing.
- **No ROS 2 adapter shipped yet.** The core is designed for one; it is not written.
- **Attached objects** (a grasped part moving with the gripper) are designed, not built.

---

## 7. Integration

- **C++20 library**, CMake, links into your process. No middleware, no daemon, no message bus.
- **Python bindings** for prototyping, offline analysis, and CI.
- **Dependencies are all apt-installable** — Eigen, urdfdom, assimp, FCL, yaml-cpp. No vendored
  forks, no build-time downloads except one optional visualizer.
- **Dev container provided**; a build is one command.
- **Input is standard**: URDF and SRDF, the files you already have.

> 🖼️ **Closing slide:** the four-line quick start from the README. "This is the whole integration."

---

## 8. Deck assembly notes

Suggested 10-slide cut:

1. Title — studio screenshot (§ top)
2. The problem — commissioning time goes to diagnosing failures, not planning speed
3. What it is — one paragraph + architecture diagram (§1.5)
4. Determinism (§1.2) — the validation-document angle
5. Exact certificate (§1.3) — the €200k end-effector line
6. High-poly robustness (§3) — the two-screenshot slide
7. **The studio demo video** (§4) — the wedged-goal diagnosis. Lead with this in the room if you
   only get five minutes
8. Where it fits (§5)
9. Honest limitations (§6) — do not cut this slide; it is why the rest is believable
10. Integration + quick start (§7)

**Before this goes to anyone outside:** [`LICENSE`](../LICENSE) still says "not yet finalized". A
technical buyer will open it, and an unfinalized license is a hard stop in procurement — it has to
be settled before the deck leaves the building. It is also the one place where we would otherwise
have an unambiguous advantage to state.

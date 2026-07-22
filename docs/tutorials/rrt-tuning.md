# Tuning RRT-Connect for hard motions (narrow passages & wedged goals)

When RRT-Connect struggles — slow, or `NoSolution`/`Timeout` — it's almost never random bad luck.
The two studio debug views tell you *why*, and the fix follows from what you see. This guide maps
**what you observe** → **what it means** → **which knob to turn**.

All the controls below live in the **Plan** panel. The two diagnostics are the
**Debug: goal escapability** probe and the **record exploration tree** checkbox.

---

## The one mechanism you need to know

RRT-Connect grows two trees — one from the **start**, one from the **goal** — until they meet.
Growth is **all-or-nothing**: each step steers up to `extension step` radians toward a random
sample and is kept **only if the entire edge is collision-free**. A step that would cross an
obstacle is discarded whole — no partial progress.

The consequence: **a tree stuck in a tight region cannot take a big step out of it.** If the goal
sits in a narrow pocket, a default `0.5` rad step always clips an obstacle, so the goal tree never
grows past its root. That is the single most common failure, and it has a specific signature below.

> Reference: `src/planning/rrt_connect.cpp` — growth loop, node kept only when the edge check
> returns fully free.

---

## Diagnostic 1 — the exploration tree

Enable **record exploration tree**, plan, and look at the two line clouds:

- **blue** = the start tree
- **orange** = the goal tree

| What you see | What it means |
|---|---|
| Both trees bushy, reaching toward each other, but never touch | A genuine **narrow passage** *between* start and goal. Neither tree threads the gap. |
| **Blue fans out, orange is a single point (no orange lines)** | The **goal is wedged** — the goal tree can't take a single step. Most common. |
| Blue is also stunted near the start | The **start** is in a tight spot too (rare; probe it the same way). |
| Both bushy, nearly meeting, only just ran out of time | Right approach, **not enough budget** — raise `timeout`. |

An "orange = just a dot" result is the tell. Confirm it with the probe.

---

## Diagnostic 2 — the goal-escapability probe

**Debug: goal escapability → Probe goal escapability.** From the goal config it fires one batched
collision query over every ±joint direction plus random directions, across a ladder of step sizes
up to **probe span**, and reports the **largest collision-free step** in each direction.

The status line reads e.g. `max free 0.062 rad (+j3) · 4 dirs ≥ 0.05 · TIGHT — try RRT extension
step ≈ 0.06`, and the full per-joint table (`+j0/-j0 …`) prints to the **server console**.

| Verdict | Reading | Meaning & action |
|---|---|---|
| **escapable** | max free ≈ probe span, most directions open | The goal is fine — RRT should solve it. If it doesn't, the bottleneck is *elsewhere* on the path (see narrow passage, below). |
| **TIGHT — try ext ≈ X** | max free ≪ span but > a few · (span/16) | The goal is in a tight pocket but escapable. **Set `extension step` ≈ X** (the reported reach) and re-plan. |
| **WEDGED** | max free barely above the smallest ladder rung, few/no directions open | The goal is free by a hair — you **cannot move away from it** in any direction. This is **not a planner problem**: the goal pose / mesh calibration puts the robot essentially in contact. No amount of tuning helps; fix the goal pose (back off the approach, re-check mesh scale/placement) or accept it's infeasible. |

Tip: shrink **probe span** (e.g. 0.05) to zoom in and read tight reaches more precisely — the
ladder has a fixed number of rungs, so a smaller span gives finer resolution near the goal.

---

## The knobs

| Knob (Plan panel) | Units | Effect | Turn it when |
|---|---|---|---|
| **extension step** | rad (joint-space L2) | Max distance a tree reaches per step. Big = fast in open space, but overshoots tight regions and gets rejected. | Probe says **TIGHT**: set ≈ the reported reach (e.g. `0.05`–`0.1`). Raise back toward `0.5` for open motions. |
| **goal bias** | fraction 0–1 | How often the *start* tree aims straight at the goal (and vice-versa) instead of a random sample. Higher = more directed, less exploration. | Trees are bushy but wandering and nearly meet: try `0.1`–`0.2`. Too high (>0.3) can trap a tree against an obstacle between start and goal. |
| **timeout (s)** | s | Wall-clock budget. | Trees almost meet before stopping. Small extension steps need more budget — raise together. |
| **edge check step** | rad or m | Collision-check granularity along an edge. Finer = safer, slower; too coarse can *miss* a thin obstacle (false success). | Paths graze obstacles: make it finer. Planning is collision-bound and the scene is chunky: coarsen slightly. |
| **max link sweep** | mm (0 = off) | Workspace-bounded edge stepping — guarantees no robot point moves more than N mm between checks, replacing `edge check step`. | You want a stated "≤ 5 mm sweep" guarantee instead of a per-joint radian step. |
| **shortcut smoothing** | on/off | Post-processes the found path shorter/smoother. Doesn't affect *finding* a solution. | Leave on. Turn off only to see the raw tree path. |

---

## Decision recipe

1. **Probe the goal first.**
   - **WEDGED** → stop tuning the planner. It's a goal-pose/calibration issue. (Cross-check: does
     the goal make physical sense? Is the approach too deep?)
   - **TIGHT** → note the reach `X`, set **extension step ≈ X**, raise **timeout**, re-plan.
   - **escapable** → the goal is not the problem; go to step 2.
2. **Record the tree and re-plan.**
   - **Orange still a dot** despite an "escapable" probe → the pocket is directional; lower
     extension step further and/or nudge goal bias down so the goal tree explores instead of
     charging the wall.
   - **Both trees bushy, gap in the middle** → genuine narrow passage between start and goal.
     Lower **extension step** globally (both trees need small steps to thread it) and raise
     **timeout**. This is slow but works; a purpose-built sampler is the real fix (see below).
   - **Nearly meeting, timed out** → just raise **timeout**.
3. **Re-probe / re-record after each change** — you're looking for the goal tree to start growing
   (orange lines appear) and the two trees to close the gap.

---

## When RRT is the wrong tool

- **Wedged goal** → a planning fix cannot manufacture free space that isn't there. It's a goal
  definition / calibration problem.
- **Deep narrow passage, escapable but tiny** → RRT with a small extension step *can* solve it but
  wastes most of its samples. Two better fits, both on the roadmap:
  - **Partial-extension RRT** — keep the collision-free *prefix* of a rejected step so a tree can
    crawl out of a pocket regardless of `extension step`. (Planned.)
  - **Bridge-test PRM sampling** — deliberately places roadmap nodes *inside* narrow gaps. Build
    the **PRM roadmap** and enable **draw roadmap (by component)**: if you see two differently
    colored clusters with no edges between them, the roadmap is split by the passage — the query
    status will say *DISJOINT roadmap components* (or *isolated goal*). Bridge sampling is the fix.
    (Planned.)

---

## Quick reference: what each color/number is telling you

- **Orange tree = one dot** → goal tree can't grow → probe the goal.
- **Probe `max free` ≈ span** → goal fine; problem is mid-path.
- **Probe `max free` small, verdict TIGHT** → set extension step to that number.
- **Probe verdict WEDGED** → fix the goal pose, not the planner.
- **PRM: two colored clusters, no bridge** → narrow passage splits the roadmap.

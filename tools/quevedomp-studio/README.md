# quevedomp-studio

Interactive Motion Planning IDE for QuevedoMP (ADR-016, Task 4a.6). Pure Python, outside the
C++ build: **viser** serves the 3D editor (joint sliders, IK drag gizmo, obstacle gizmos,
Plan + scrub), **rerun** optionally records every planning attempt with its `PlanningStats`.

## Running (dev container)

The `quevedomp` package comes from the `dev-py` build tree until Phase 4b ships a wheel:

```bash
# 1. Build the bindings once (Release — the Debug dev-py build plans ~14x slower):
#    cmake --preset release-py && cmake --build --preset release-py
# 2. From the repo root, in the container (publish the UI port):
docker run --rm -p 8080:8080 -v "$PWD":/work -w /work quevedomp-cuda bash -lc '
  PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
  python3 -m quevedomp_studio --fixture ur5 --rerun-save /work/studio-session.rrd'

docker run --rm -p 8080:8080 -v /mnt/d/Inventos/quevedoMP:/work -w /work quevedomp-cuda bash -lc 'PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio python3 -m quevedomp_studio --load sessions/benchmark.qmps --rerun-save /work/studio-session.rrd'
# 3. Open http://localhost:8080
```

The robot is chosen at launch — one robot per session in v0. `--fixture {ur5,ur10,panda,iiwa,
irb2400,rbrobout,rbrobout_inlet}` loads a repo fixture with its mesh directories (and, for the
DTC cells, the SRDF-derived ACM) wired automatically; for any other robot pass `--urdf` plus
`--package-dir PKG=DIR` (repeatable) / `--base-dir` / `--srdf`, exactly like the C++
`MeshSources` + DTC harness.

## Workflow

1. **Pose** the robot with the joint sliders, or drag the **IK gizmo** (the selected leaf link
   follows; the status line shows convergence and the robot tints red on collision, witness
   pair named).
2. **Obstacles**: add box/sphere/cylinder, drag them with their gizmos; collision state updates
   live (`CollisionScene.add/move/remove_object` under the hood — no scene rebuild).
3. **Plan**: capture *Set start* / *Set goal* (joint-space, or check *goal = IK gizmo pose* for
   a `PoseGoal`), pick timeout/seed/smoothing, hit **Plan**. Planning runs on a worker thread —
   the bindings release the GIL, so the UI stays live. The result draws as the end-effector
   trace; **scrub** animates the robot along the path. *max link sweep* (mm, Task 3.3d P3)
   switches edge checking to a workspace-stated guarantee — no point of the robot moves more
   than that between collision samples — overriding the per-joint *edge check step* when > 0.
4. **Trajectory** (roadmap R2): after a plan, set the accel / tip-speed / tip-accel / jerk caps
   and hit **Parameterize** — the (smoothed) path is spline-fitted, re-validated against the
   scene, and time-parameterized (Task 3.4; jerk-certified when a jerk cap is set). **▶ Play
   (timed)** animates the robot in real time (× time scale) with the actual velocity profile,
   and the *Plots* folder charts joint velocity/acceleration and tip speed over time. In the
   *Planning* panel, *record exploration tree* snapshots the RRT trees at plan exit and draws
   them as end-effector line clouds (start tree blue, goal tree orange).
5. **Clearance** (roadmap R3): *Build clearance field* voxelizes the current obstacles into a
   signed-distance field (GPU jump-flooding when a device is present — the status line says
   which path ran); the *slice height* slider sweeps a heatmap layer through the scene
   (red = penetration/near, blue = far). Non-watertight meshes contribute unsigned distance
   only (ADR-012/ADR-018).
   The **Refine (CHOMP)** panel (roadmap R4, ADR-019) runs the CHOMP/TrajOpt refiner over the
   clearance field to polish the last plan (or a straight-line guess with *standalone*) toward
   smoother, higher-clearance motion — clearance/smoothness weights, ε, iterations and waypoints
   are knobs; the field resolution is shared with the *SDF resolution* control above. The output is
   certified collision-free by the exact backend and recorded as a new attempt, so the scrub,
   *Play*, plots and *Parameterize* apply to it unchanged (headless equivalent:
   `StudioSession.refine()`).
   The **Roadmap (PRM)** panel (roadmap R5, ADR-020) is the multi-query planner: *Build roadmap*
   samples + validates a per-cell roadmap once (nodes/edges/configs/time shown), then *Query
   roadmap* answers the current start→goal by graph search + smoothing — cheap and repeatable while
   the scene is unchanged (the roadmap is invalidated when an obstacle moves). Headless equivalents:
   `StudioSession.build_roadmap()` / `plan_roadmap()`.
6. **Sessions**: the *Session* panel saves/loads the whole problem setup — robot + ACM and
   obstacles (Task 2a.5 serializer blobs, the same format Phase 3b capture bundles will carry)
   plus start/goal, timeout, edge step, and planner settings — so a benchmark problem is one
   *Load* away. Paths resolve inside the container (e.g. `sessions/scene.qmps` lands in the
   repo, next to where `--load` picks it up at launch).

## Headless smoke test

```bash
docker run --rm -v "$PWD":/work -w /work quevedomp-cuda bash -lc '
  PYTHONPATH=build/dev-py/bindings/python:tools/quevedomp-studio \
  python3 -m pytest tools/quevedomp-studio/tests -q'
```

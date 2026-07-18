# quevedomp-studio

Interactive Motion Planning IDE for QuevedoMP (ADR-016, Task 4a.6; working modes per ADR-021).
Pure Python, outside the C++ build: **viser** serves the 3D editor, **rerun** optionally records
every planning attempt with its `PlanningStats`.

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

## Working modes (ADR-021)

The sidebar is modal: the **mode** button group switches between five working contexts, and only
the *Session* panel stays global. Each mode also controls what the 3D view emphasizes.

- **Scene** — pose the robot with the joint sliders (collision status + witness pair live) and
  edit obstacles: add box/sphere/cylinder or a mesh file, drag them with their gizmos
  (`CollisionScene.add/move/remove_object` under the hood — no scene rebuild). Robot and
  obstacles stay visible in every mode; only the panel is modal.
- **IK** — drag the gizmo (the selected link tracks; seeded, no branch-hopping), *Solve (global,
  multi-restart)* for an explicit branch switch, or *Solve branches* to enumerate distinct IK
  branches and pick one (free/COLLIDES marked). The *link* dropdown here selects the
  end-effector link every other mode uses for markers, traces, and tip plots.
- **Plan** — the planners are peers behind one **Plan** button: pick *RRT-Connect*, *PRM
  roadmap* (R5, ADR-020: *Build roadmap* once — nodes/edges/configs/time shown — then Plan
  queries it in ms while the scene is unchanged), or *CHOMP (standalone)* (R4, ADR-019:
  optimizes a straight line to the goal, exact-backend certified). Set start/goal (joint-space,
  or *goal = IK gizmo pose*), timeout/seed/smoothing; *max link sweep* (mm, Task 3.3d P3) gives
  the workspace-bounded edge guarantee. Debug views live here: *record exploration tree* draws
  the RRT trees as EE line clouds (R2), and *Debug: clearance heatmap* (R3) builds the voxel SDF
  (GPU JFA when present) and sweeps a slice heatmap. Obstacle edits mark built roadmap/field
  status lines **STALE**.
- **Trajectory** — after a plan: set accel / tip-speed / tip-accel / jerk caps and
  **Parameterize** (Task 3.4; jerk-certified when capped); *Refine (CHOMP polish)* runs the R4
  refiner over the last plan with its own knob set (never worse than its re-certified seed);
  playback = **scrub**, **▶ Play** (constant rate), **▶ Play (timed)** (real time × scale); the
  *Plots* folder charts joint velocity/acceleration and tip speed.
- **Tasks (MTC)** — placeholder; becomes the `quevedomp_tasks` inspector/runner with roadmap R7.

**Sessions** (global): saves/loads the whole problem setup — robot + ACM and obstacles (Task
2a.5 serializer blobs) plus start/goal, timeout, edge step, and planner settings — so a
benchmark problem is one *Load* away. Per-mode knob values (CHOMP weights, PRM sizes, caps) are
not persisted. Paths resolve inside the container (e.g. `sessions/scene.qmps` lands in the repo,
next to where `--load` picks it up at launch).

Headless equivalents for scripting/tests live on `StudioApp` (`plan_now`, `parametrize_now`,
`refine_now`, `build_clearance_now`, `build_roadmap_now`, `query_roadmap_now`, `play`,
`play_timed`) and `StudioSession` (`plan`, `refine`, `build_roadmap`/`plan_roadmap`,
`parametrize`).

## Headless smoke test

```bash
docker run --rm -v "$PWD":/work -w /work quevedomp-cuda bash -lc '
  PYTHONPATH=build/dev-py/bindings/python:tools/quevedomp-studio \
  python3 -m pytest tools/quevedomp-studio/tests -q'
```

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
   trace; **scrub** animates the robot along the path.
4. **Sessions**: the *Session* panel saves/loads the whole problem setup — robot + ACM and
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

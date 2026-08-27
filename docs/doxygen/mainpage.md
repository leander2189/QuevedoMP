# QuevedoMP API Reference

**QuevedoMP** is a ROS-free, GPU-accelerated C++20 library for robot-arm motion planning in
complex, quasi-static industrial cells. This reference covers the **public headers** under
`include/quevedomp/` — the surface you program against. Implementation under `src/` is deliberately
absent.

## Where to start

The layers depend downward only; read them in this order and each one uses only the ones above it.

| Module | Namespace | Start at | What it is |
|---|---|---|---|
| **core** | `quevedomp` | `core/types.hpp` | `Transform`, `Mesh`, `JointPosition`, `Trajectory`, and the deterministic `Rng` every seeded component draws from |
| **robot** | `quevedomp` | `robot/robot_model.hpp` | `RobotModel` (immutable, shared) parsed from URDF, and `RobotInstance` (one owner's mutable allowed-collision matrix) |
| **kinematics** | `quevedomp` | `kinematics/fk.hpp` | Forward kinematics, analytic Jacobian, and `InverseKinematics` including multi-branch `solve_all` |
| **collision** | `quevedomp::collision` | `collision/collision_scene.hpp` | The heart of the library: one `CollisionScene` interface over the FCL and OptiX backends. **Batch-first** — see below |
| **clearance** | `quevedomp::clearance` | `clearance/clearance_field.hpp` | GPU voxel signed-distance field of the environment. Approximate; drives optimization, never certifies |
| **planning** | `quevedomp::planning` | `planning/planner.hpp` | `Planner` interface + registry, RRT-Connect, PRM roadmaps, the CHOMP-flavoured refiner, and the shortcut smoother |
| **parameterization** | `quevedomp::parameterization` | `parameterization/parameterize.hpp` | Path → time-parameterized trajectory under joint and tool-tip limits, with a certified jerk-limited mode |
| **capture** | `quevedomp::capture` | `capture/serialize.hpp` | Serialize a robot, ACM or scene to bytes — what `.qmps` sessions are built from |
| **viz** | `quevedomp::viz` | `viz/visualizer.hpp` | Optional rerun logging. Compiles to no-ops unless `QUEVEDOMP_WITH_RERUN` |

## Three contracts worth knowing before you call anything

**Collision queries are batch-first.** `CollisionScene::query_batch` takes a span of configurations
and is the path everything else is built on. Issuing one configuration at a time works but forfeits
both the OpenMP parallelism on the CPU backend and any possibility of using the GPU one, where
per-launch overhead dominates a small batch.

**One `Workspace` per thread.** `CollisionScene::make_workspace()` returns per-thread scratch.
Sharing one across threads is out-of-bounds undefined behaviour, not merely a wrong answer.

**The exact backend is the only certificate.** `ClearanceField` is a voxel approximation. It exists
to give an optimizer a gradient and a human a heatmap. A path is collision-free when a
`CollisionScene` says so, and never on the strength of a clearance value.

## The Python API

The same library, through nanobind. Everything is re-exported from the `quevedomp` package, so
`quevedomp.RobotModel` is the class documented here — `quevedomp._native`, where it physically
lives, is an implementation detail you never import.

```python
import quevedomp as q
model = q.RobotModel.from_urdf(open("robot.urdf").read())
```

**Both languages share one page per class.** The C++ namespace and the Python module are both
called `quevedomp`, so `RobotModel` has a single entry listing **the C++ members first, then the
Python bindings** — `const std::string & name() const noexcept` above `str name(self)`. That is
deliberate: switching languages should not mean switching sites. The signatures tell you which is
which at a glance, and where a class carries documentation on both sides you will see both
paragraphs run together.

The Python half is generated from the binding's type stub, so **signatures and type annotations are
complete and authoritative** — including numpy array shapes and dtypes, which the C++ side cannot
express. Prose is thinner: roughly two in five classes and one in six functions carry a docstring.
Where a Python entity has none, **the C++ member above it is the semantic reference** — the
bindings are a thin re-export and add no behaviour of their own.

Three things behave differently from C++ and are worth knowing:

- Arrays cross the boundary **zero-copy** where the annotation says so (`BatchResult.in_collision`
  is a view over the C++ buffer, not a copy). It stays valid only as long as the owning object does.
- The GIL is **released** around long-running calls such as `query_batch` and `plan`, so a Python
  thread pool over one scene behaves the way you would hope — subject to the same one-`Workspace`
  -per-thread rule as C++.
- Factories that need more than `(params, robot, scene)` — the refiner and the PRM builder — are
  separate functions rather than registry strings, exactly as in C++.

## Beyond this reference

- **[README](https://github.com/leander2189/QuevedoMP#readme)** — building, presets, running the studio
- **ROADMAP.md** — what works today, what is next, and every design document
- **docs/QuevedoMP-SPEC.md** — the architecture specification, including the full collision contract
- **docs/architecture/** — an ADR for every significant decision

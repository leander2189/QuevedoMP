# Visualizing robots, FK & IK with rerun (Task 1.8)

The `viz/` module logs to [rerun.io](https://rerun.io). It is **off by default**: normal builds
(`dev-cpu`, `dev-gpu`) compile `viz/Visualizer` to no-ops and never fetch the rerun SDK. The
`dev-viz` preset turns it on (`WITH_RERUN=ON`), pulls the rerun **C++ SDK 0.33.1** via CMake
`FetchContent`, and builds the `quevedomp_visualize` example that writes `.rrd` recordings.

> Two halves, both version 0.33.1: the **C++ SDK** (logging side, fetched by CMake) and the
> **viewer** (`pip install rerun-sdk==0.33.1`). They must match.

## 1. Build with viz enabled and generate recordings

Run from WSL, in the project root, inside the GPU/CPU container (no GPU needed for viz):

```bash
docker run --rm -v "$(pwd):/workspace" -w /workspace quevedomp-cuda bash -lc "
  cmake --preset dev-viz &&
  cmake --build --preset dev-viz &&
  ./build/dev-viz/examples/cpp/quevedomp_visualize tests/fixtures build/dev-viz/viz_out
"
```

This writes one recording per robot to `build/dev-viz/viz_out/`:
`ur5.rrd`, `ur10.rrd`, `panda.rrd`, `iiwa.rrd`, `irb2400.rrd`.

Each recording contains, for that robot:
- the posed robot — per-link coordinate **frames**, a **skeleton** (segments between connected
  link origins), and the **collision meshes** placed by forward kinematics;
- an **IK demo** — the `ik_target` frame vs. the `ik_achieved` end-effector frame after solving;
- the **tip path** of a straight-line joint interpolation toward the IK solution.

> First `dev-viz` configure/build is slow and needs network: it downloads the rerun C++ SDK and
> builds Apache Arrow from source (a few minutes). Subsequent builds are cached. The preset sets
> `CMAKE_POLICY_VERSION_MINIMUM=3.5` so Arrow's bundled mimalloc configures under modern CMake.

## 2. Install the viewer (must be 0.33.1)

Pick whichever matches where you want the window to appear:

```bash
# On Windows (native window; opens files from the mounted drive):
py -m pip install "rerun-sdk==0.33.1"     # provides the `rerun` viewer

# …or inside WSL (needs WSLg for the GUI):
pip install "rerun-sdk==0.33.1"
```

## 3. Open a recording

```bash
# From Windows PowerShell (the repo is on D:):
rerun D:\Inventos\quevedoMP\build\dev-viz\viz_out\panda.rrd

# …or from WSL:
rerun build/dev-viz/viz_out/panda.rrd
```

Drag to orbit, scroll to zoom; the left panel lists the logged entities (`world/<robot>/...`).
You can open several `.rrd`s at once, or pass them all: `rerun build/dev-viz/viz_out/*.rrd`.

## Using the API from your own code

```cpp
#include "quevedomp/viz/visualizer.hpp"
quevedomp::Visualizer viz("my_app");
viz.save("out.rrd");                       // or viz.spawn() to stream to a live viewer
viz.log_robot("world/ur5", *model, q);     // frames + skeleton at config q
viz.log_pose("world/target", target_tf);   // a coordinate frame
viz.log_trajectory("world/path", *model, traj, tip_link);  // tip path of a trajectory
```

With `WITH_RERUN=OFF` every one of these is a no-op (and `viz.enabled()` returns `false`), so
the same code compiles and runs in builds that don't link rerun.

# QuevedoMP

**QuevedoMP** (Quevedo Motion Planner) is a lightweight, modular, GPU-accelerated C++20
library for robot-arm trajectory planning. ROS-free core, clean CPU/GPU backend abstraction,
rigorous differential testing. Named after Leonardo Torres Quevedo (1852–1936).

> Status: **Phase 1 complete; Phase 2a next.** Phase 0 proved the build/CUDA/OptiX/test
> environment; Phase 1 delivered the CPU robot core — types, deterministic RNG, URDF parsing,
> mesh loading, FK/Jacobian/IK (validated to <1e-9 / <1e-6), rerun visualization, and a
> fuzz + >80%-coverage gate. Phase 2a (collision interface + FCL backend) is next. See
> [`QuevedoMP-SPEC.md`](QuevedoMP-SPEC.md) for the architecture and
> [`docs/QuevedoMP-BUILD-PLAN.md`](docs/QuevedoMP-BUILD-PLAN.md) for the step-by-step plan.

## Requirements

- Windows 11 + WSL2 (Ubuntu) **or** native Linux, with Docker.
- NVIDIA GPU (compute capability ≥ 7.5, Turing+) and driver supporting CUDA ≥ 12.4.
- See the build plan **§H** for one-time host setup (Docker + NVIDIA Container Toolkit in WSL).

> **WSL note:** use the **Docker Engine running inside your WSL distro** (with
> `nvidia-container-toolkit`), not Docker Desktop's engine — Docker Desktop's GPU passthrough
> auto-detects `legacy` mode in WSL and fails to load the WSL GPU libraries. All `docker`
> commands below assume the WSL-native engine.

## Quick start

```bash
# 1. Build the CUDA dev container:
docker build -t quevedomp-cuda .devcontainer

# 2. Confirm the GPU is visible inside it:
docker run --rm --gpus all quevedomp-cuda bash -lc "nvcc --version && nvidia-smi"

# 3. Configure, build, and test — GPU preset (CPU library + CUDA smoke test):
docker run --rm --gpus all -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "cmake --preset dev-gpu && cmake --build --preset dev-gpu && ctest --preset dev-gpu --output-on-failure"
#   -> expect 2 tests passed, incl. "cuda_smoke OK: result=42"

# 4. Minimal CPU build (proves the no-GPU path; no nvcc needed):
docker run --rm -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "cmake --preset dev-cpu && cmake --build --preset dev-cpu && ctest --preset dev-cpu --output-on-failure"
#   -> expect 1 test passed
```

VS Code users: "Reopen in Container" uses `.devcontainer/` directly.

## Studio (GPU)

`quevedomp-studio` is the interactive planning IDE (viser + rerun; ADR-016, working modes in
ADR-021): scene editing, IK gizmo, planning, trajectory playback. Run it from the **`release-py`**
preset — Release, with CUDA + OptiX + the Python bindings on. A Debug build works but is ~9×
slower, which is not a useful thing to look at.

```bash
# 1. Build the bindings once:
docker run --rm --gpus all -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "cmake --preset release-py && cmake --build --preset release-py"

# 2. Launch (native Linux) — OptiX comes from the host driver automatically:
docker run --rm --gpus all -p 8080:8080 -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
   python3 -m quevedomp_studio --fixture rbrobout_inlet"
#   -> open http://localhost:8080

# 3. Launch on WSL2 — same, plus the extracted OptiX runtime (see note below):
docker run --rm --gpus all -p 8080:8080 -v "$PWD":/work -w /work \
  -v "$PWD/.devcontainer/wsl-optix:/opt/wsl-optix:ro" quevedomp-cuda bash -lc \
  "export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\$LD_LIBRARY_PATH && \
   PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
   python3 -m quevedomp_studio --fixture rbrobout_inlet"
```

The file
`d:\Inventos\quevedoMP\tests\fixtures\meshes\dummy_hires.stl` maps to `/work/tests/fixtures/meshes/dummy_hires.stl` in the studio.


`--fixture` takes `ur5`, `ur10`, `panda`, `iiwa`, `irb2400`, `rbrobout`, or `rbrobout_inlet` (the
two `rbrobout` cells load their SRDF ACM automatically); `--load sessions/benchmark.qmps` reopens
a saved session instead. `--host`/`--port` default to `0.0.0.0:8080`, which is what makes the
`-p 8080:8080` publish work.

> **WSL only:** the `wsl-optix` mount and the `LD_LIBRARY_PATH` prefix in step 3 exist because WSL
> does not expose `libnvoptix.so.1` to containers the way a native Linux driver does. Produce that
> directory once with `.devcontainer/setup-wsl-optix.sh`, and re-run it if the host driver's major
> version changes — see [`docs/tutorials/testing.md`](docs/tutorials/testing.md). Nothing else in
> the command differs between WSL and native Linux.

> **Mount the repo at `/work`.** A CMake cache is not relocatable: `build/release-py` records
> `/work` as its source directory, so mounting the repo anywhere else makes CMake reject the cache
> and reconfigure from scratch.

## Dependencies

CPU deps come from the container's system `apt` (build-plan deviation D2), found via CMake:
Eigen3 (core math, public), GoogleTest (tests). **URDF parsing uses `urdfdom`** (chosen over
hand-rolled tinyxml2 — it is purpose-built, already packaged, and yields the full joint/link
model); `yaml-cpp` carries the optional acceleration/jerk limit extension; **`assimp`** loads
STL/DAE/OBJ meshes. urdfdom, yaml-cpp and assimp are private to the library — they do not appear
in any public header.

**Optional visualization** (`WITH_RERUN=ON`, off by default): the `dev-viz` preset pulls the
[rerun](https://rerun.io) C++ SDK via CMake `FetchContent` to log robots/FK/IK to `.rrd` files.
With it off, `viz/Visualizer` compiles to no-ops and nothing is fetched. See
[`docs/tutorials/visualization.md`](docs/tutorials/visualization.md).

## Layout

See [`QuevedoMP-SPEC.md`](QuevedoMP-SPEC.md) §1.1. Library code lands from Phase 1 onward.

## License

Not yet finalized — see [`LICENSE`](LICENSE) and spec §12.

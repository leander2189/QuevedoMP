# QuevedoMP

**QuevedoMP** (Quevedo Motion Planner) is a ROS-free, GPU-accelerated C++20 library for robot-arm
motion planning in complex, quasi-static industrial cells. One collision interface over CPU (FCL)
and GPU (OptiX) backends, deterministic results per seed, batch-first collision checking, and an
exact collision certificate on every path it returns. Named after Leonardo Torres Quevedo
(1852–1936).

It ships with **quevedomp-studio**, a browser-based planning IDE for building scenes, solving IK,
planning, and inspecting trajectories.

📍 **[ROADMAP.md](ROADMAP.md)** — what works today, what's next, and links to every design document.
This README is the quick start; the roadmap is the plan. Those are the two files to follow.

## Requirements

- Windows 11 + WSL2 (Ubuntu) **or** native Linux, with Docker.
- NVIDIA GPU (compute capability ≥ 7.5, Turing+) and a driver supporting CUDA ≥ 12.4 — only for the
  GPU backends; the CPU library builds and tests without one.
- All dependencies live in the dev container. Nothing is installed on the host.

> **WSL note:** the tested path is the **Docker Engine running inside your WSL distro** with
> `nvidia-container-toolkit`. Docker Desktop's WSL integration also works on recent versions
> (verified with `--gpus all` and OptiX on driver 595.97), but older ones auto-detect `legacy` GPU
> mode and fail to load the WSL GPU libraries.

## Quick start

```bash
# 1. Build the dev container (once):
docker build -t quevedomp-cuda .devcontainer

# 2. Confirm the GPU is visible inside it:
docker run --rm --gpus all quevedomp-cuda bash -lc "nvcc --version && nvidia-smi"

# 3. Build and test the CPU library + Python bindings — the everyday preset:
docker run --rm -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "cmake --preset dev-py && cmake --build --preset dev-py && ctest --preset dev-py --output-on-failure"
#   -> expect the suite green (215 tests; 2 GPU tests skip without --gpus all)
```

VS Code users: "Reopen in Container" uses `.devcontainer/` directly.

> **Keep the mount path consistent.** A CMake cache records the source directory it was configured
> with and is not relocatable, so mounting the repo somewhere other than `/work` on a later run
> makes CMake reject the cache and reconfigure from scratch.

## Build presets

Pick by what you need. Everything is `cmake --preset X && cmake --build --preset X`, and the first
five also take `ctest --preset X`.

| Preset | Build | CUDA | OptiX | Python | Use it for |
|---|---|---|---|---|---|
| `dev-cpu` | Debug + sanitizers | — | — | — | the no-GPU path; fastest to build |
| `dev-gpu` | Debug + sanitizers | ✅ | — | — | CUDA work |
| `dev-optix` | Debug + sanitizers | ✅ | ✅ | — | GPU collision-backend work |
| `dev-py` | Debug | — | — | ✅ | **everyday development** — C++ and Python tests |
| `dev-viz` | Debug | — | — | — | rerun `.rrd` visualization |
| `release-py` | Release | ✅ | ✅ | ✅ | **running the studio**; anything user-facing |
| `bench-optix` | Release | ✅ | ✅ | — | benchmarks — never benchmark a Debug or sanitizer build |
| `release` | Release | ✅ | — | — | CPU + CUDA release library |
| `viz-optix` | Release | ✅ | ✅ | — | visualizing GPU scenes |
| `coverage` / `fuzz` | Debug | — | — | — | coverage gate; libFuzzer targets |

Debug is roughly **9× slower** than Release. Never quote a number from one.

## Running the studio

Needs the `release-py` preset. See [ROADMAP.md](ROADMAP.md) §1 for what the studio can do.

```bash
# Build the bindings once:
docker run --rm --gpus all -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "cmake --preset release-py && cmake --build --preset release-py"

# Launch (native Linux) — OptiX comes from the host driver automatically:
docker run --rm --gpus all -p 8080:8080 -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
   python3 -m quevedomp_studio --fixture rbrobout_inlet"
#   -> open http://localhost:8080

# Launch on WSL2 — same, plus the extracted OptiX runtime (see note below):
docker run --rm --gpus all -p 8080:8080 -v "$PWD":/work -w /work \
  -v "$PWD/.devcontainer/wsl-optix:/opt/wsl-optix:ro" quevedomp-cuda bash -lc \
  "export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\$LD_LIBRARY_PATH && \
   PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
   python3 -m quevedomp_studio --fixture rbrobout_inlet"
```

`--fixture` takes `ur5`, `ur10`, `panda`, `iiwa`, `irb2400`, `rbrobout` or `rbrobout_inlet` (the two
`rbrobout` cells load their SRDF ACM automatically). `--load sessions/benchmark.qmps` reopens a
saved session instead. `--host`/`--port` default to `0.0.0.0:8080`, which is what makes the
`-p 8080:8080` publish work.

**Mesh paths are container paths.** The repo is mounted at `/work`, so
`tests/fixtures/meshes/foo.stl` on the host is `/work/tests/fixtures/meshes/foo.stl` to the studio.
A mesh outside the repo needs its own `-v` mount.

> **WSL only:** the `wsl-optix` mount and `LD_LIBRARY_PATH` prefix exist because WSL does not expose
> `libnvoptix.so.1` to containers the way a native Linux driver does. Produce that directory once
> with `.devcontainer/setup-wsl-optix.sh`, and re-run it when the host driver's major version
> changes. Nothing else differs between WSL and native Linux.

## Benchmarking

Numbers only mean something under [`docs/benchmarks/PROTOCOL.md`](docs/benchmarks/PROTOCOL.md).
Build `bench-optix`, then:

```bash
# End-to-end plan attribution on a versioned session — the number that matters:
PYTHONPATH=build/release-py/bindings/python:tools/quevedomp-studio \
  python3 examples/python/session_profile.py sessions/benchmark.qmps --seeds 8 --backend fcl

./build/bench-optix/bench_collision                 # FCL vs OptiX, synthetic triangle sweep
./build/bench-optix/bench_collision <mesh> [scale]  # ...against a real mesh from disk
./build/bench-optix/bench_dtc inlet                 # where the time goes: self vs env vs floor
```

## Documentation

| | |
|---|---|
| [ROADMAP.md](ROADMAP.md) | Current features, ordered plan, links to all design documents |
| [docs/QuevedoMP-SPEC.md](docs/QuevedoMP-SPEC.md) | Architecture specification and module layout |
| [docs/architecture/](docs/architecture/) | ADRs — the record of every significant decision |
| [docs/tutorials/testing.md](docs/tutorials/testing.md) | Test suites, GPU tests, the WSL OptiX setup |
| [docs/tutorials/rrt-tuning.md](docs/tutorials/rrt-tuning.md) | Narrow passages and planner tuning |
| [docs/tutorials/visualization.md](docs/tutorials/visualization.md) | rerun logging and the `dev-viz` preset |
| [docs/optix-collision.md](docs/optix-collision.md) | How the GPU collision backend works |
| [docs/benchmarks/PROTOCOL.md](docs/benchmarks/PROTOCOL.md) | How to produce a number anyone should believe |

## Dependencies

From the container's system `apt` (deviation D2 — `FetchContent` only by ratified decision):
**Eigen3** (core math, public), **urdfdom** (URDF parsing), **yaml-cpp** (optional accel/jerk limit
extension), **assimp** (STL/DAE/OBJ meshes), **FCL** (CPU collision), **OpenMP** (parallel batch
queries), **GoogleTest**. urdfdom, yaml-cpp and assimp are private to the library — they appear in
no public header.

Optional: **rerun** C++ SDK (`WITH_RERUN=ON`, off by default, the one ratified `FetchContent`);
with it off, `viz/Visualizer` compiles to no-ops. **OptiX** for the GPU backend — currently a
login-gated SDK, which is what the [Vulkan backend](docs/QuevedoMP-VULKAN-BACKEND-DESIGN.md) work
would remove.

## License

Not yet finalized — see [LICENSE](LICENSE) and spec §12.

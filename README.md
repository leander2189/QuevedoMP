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

## API reference

Doxygen over the public headers, plus the Python bindings. The target is never part of a normal
build — ask for it:

```bash
# Build the reference (no GPU needed):
docker run --rm -v "$PWD":/work -w /work quevedomp-cuda bash -lc \
  "cmake --preset dev-py && cmake --build --preset dev-py --target docs"
```

Then open **`build/dev-py/docs/api/html/index.html`** in a browser. On Windows that is
`file:///D:/Inventos/quevedoMP/build/dev-py/docs/api/html/index.html` — the build tree is on the
host, so no server is needed.

> **The image needs `doxygen`.** It was added to `.devcontainer/Dockerfile` after the current image
> was built, so rebuild once to pick it up — and pass the OptiX installer, or the rebuilt image
> silently loses the GPU backend:
> ```bash
> docker build -t quevedomp-cuda \
>   --build-arg OPTIX_INSTALLER=NVIDIA-OptiX-SDK-8.1.0-linux64-x86_64-35015278.sh .devcontainer
> ```
> Until then the configure still succeeds — it just prints `Doxygen not found` and skips the
> target, so the build step fails with `unknown target 'docs'`.

**C++ and Python in one site.** On a preset with the bindings on (`dev-py`, `release-py`) the
nanobind type stub is documented alongside the headers — 520 pages, where each class page lists the
C++ members first and the Python bindings after. A CPU-only preset gets the C++ half (395 pages)
and says so at configure time.

Output lands in the build tree, so generated documentation is never committed. The headers use
plain `//` comments, which Doxygen ignores; [`docs/doxygen/comment-filter.py`](docs/doxygen/comment-filter.py)
rewrites them on the way in and leaves the source untouched. Run it on any header to see exactly
what Doxygen will parse. `QUEVEDOMP_BUILD_DOCS=OFF` drops the target entirely.

Nine "unsupported xml/html tag" warnings in `docs/api/doxygen-warnings.log` are expected and
harmless — Doxygen renders those angle-bracket placeholders correctly regardless.

## Documentation

| | |
|---|---|
| [ROADMAP.md](ROADMAP.md) | Current features, ordered plan, links to all design documents |
| [docs/QuevedoMP-SPEC.md](docs/QuevedoMP-SPEC.md) | Architecture specification and module layout |
| [docs/PITCH.md](docs/PITCH.md) | What QuevedoMP is and why, for someone evaluating it |
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

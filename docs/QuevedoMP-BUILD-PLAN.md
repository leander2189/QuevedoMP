# QuevedoMP — Granular Build Plan (agent-executable)

> Companion to `QuevedoMP-SPEC.md`. The **spec** is the *what/why* (architecture,
> contracts, scope). This **build plan** is the *how, step by step*: every phase is
> decomposed into small tasks, each with an explicit **Verify** command and a binary
> **Done-when** condition an agent can check before moving on.
>
> **Rules of engagement for the implementing agent**
> 1. Execute tasks **in order**. Do not start task N+1 until task N's *Done-when* is true.
> 2. After every task, run its **Verify** step and paste the actual output. If it fails,
>    fix it before proceeding — never mark a task done on assumption.
> 3. **Commit after each task** (small, atomic, revertible — spec §"Final Notes").
> 4. Where this plan and the spec disagree on *mechanics*, this plan wins; where they
>    disagree on *architecture/contracts*, the spec wins. Deviations are logged in §D below.

---

## D. Deviations from the spec (ratified decisions)

| # | Spec says | This plan does | Why |
|---|-----------|----------------|-----|
| D1 | §2.2 Dockerfile `COPY`s a login-gated OptiX installer that isn't in the repo | **Keep the CUDA base image from Phase 0** (`nvidia/cuda:…-devel`, `nvcc`, `--gpus all`) and validate the GPU toolchain on day one with a real kernel smoke test — but **drop the OptiX `COPY`**; OptiX is added later (Phase 2b) or via the optional early-validation task 0.10. | The OptiX `COPY` of an absent, login-gated file is a hard `docker build` failure — the *actual* original blocker. CUDA itself ships freely in the base image and is what the user wants proven early, so we keep it; only the un-automatable OptiX download is deferred. |
| D4 | §8 mandates CI (GitHub Actions, multiple runners) | **No CI for now.** Verification is **manual**: run each task's *Verify* step locally and the Phase-EXIT checklist by hand. No `.github/workflows/`. | User runs tests manually for now. CI can be added later by lifting the same preset commands into a workflow. |
| D2 | §1.2 / vcpkg.json drive deps via vcpkg | **CPU deps come from system `apt`** (`find_package(... CONFIG)`); vcpkg is deferred until a needed dep is missing from Ubuntu repos. | Avoids the dual-source (apt + vcpkg) conflict that caused the GTest shim friction; container builds in minutes instead of compiling everything from source. Reproducibility is pinned via the Ubuntu base image tag. |
| D3 | §2.2 `pip install rerun-sdk` in the base image | Deferred to Phase 1 (`viz/`), installed only when `WITH_RERUN` work begins. | Unused in Phase 0; keeps the bootstrap image lean and the build network-robust. |

> If a later phase genuinely needs a dependency not in Ubuntu's repos, **stop and
> introduce vcpkg then** (raise as a §12 decision) rather than guessing now.

---

## H. Host setup (Windows 11 + WSL2 + NVIDIA GPU) — do this once before Phase 0

You already have the NVIDIA driver and CUDA in Windows + WSL. To run **GPU-enabled
containers**, the host needs: a working driver inside WSL, a Docker engine, and the **NVIDIA
Container Toolkit** so `--gpus all` passes the GPU through. Run these **inside your WSL2
Ubuntu** distro.

> **Important:** do **not** install the Linux NVIDIA *driver* inside WSL — WSL uses the
> Windows driver via `/usr/lib/wsl/lib`. Installing a Linux driver breaks GPU passthrough.
> Only install the toolkit + Docker.

**Step 1 — Confirm the GPU is visible in WSL** (should already work):
```bash
nvidia-smi          # must list your GPU; note the "CUDA Version" (driver ceiling)
```

**Step 2 — Install Docker Engine in WSL** (skip if you use Docker Desktop with the WSL2
integration enabled — in that case just enable it in Docker Desktop → Settings → Resources →
WSL Integration, then go to Step 4):
```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# then close and reopen the WSL shell (or: wsl --shutdown from PowerShell) so the group applies
```

**Step 3 — Start Docker** (Docker Engine in WSL doesn't auto-start systemd on older setups):
```bash
sudo service docker start   # or: sudo systemctl start docker  (if systemd is enabled in /etc/wsl.conf)
docker run --rm hello-world  # sanity: Docker itself works
```

**Step 4 — Install the NVIDIA Container Toolkit** (this is the piece that enables `--gpus`):
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo service docker restart   # or: sudo systemctl restart docker
```

**Step 5 — Verify GPU passthrough into a container** (the gate for Phase 0):
```bash
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```
If this lists your GPU, the host is ready. If it errors, the problem is here on the host —
fix it before building the dev container (Task 0.2). Common fixes: reopen the shell after the
`usermod`, `sudo service docker restart`, ensure Docker Desktop WSL integration is on for this
distro, and confirm Step 1's `nvidia-smi` works.

---

## Phase 0 — Bootstrap (must go green before anything else)

**Goal:** a fresh clone opens in a **CUDA-enabled** dev container, configures, builds, and
passes the trivial CPU test **and** a CUDA kernel smoke test that runs on the real GPU —
all verified manually. **No library code yet** — `quevedomp` is an `INTERFACE` placeholder;
the CUDA smoke test is a throwaway target that exists only to prove the toolchain + GPU
passthrough work end to end before we commit to deeper phases.

> **Run host setup first** (§H below): driver check, Docker in WSL, NVIDIA Container Toolkit.

### Task 0.1 — Repo skeleton + housekeeping files
- **Create:**
  - Directory tree per spec §1.1 (empty dirs carry a `.gitkeep`):
    `include/quevedomp/{core,robot,collision,kinematics,planning}`, `src/{core,robot,kinematics,planning}`,
    `src/collision/backends/{cpu_fcl,optix}`, `src/capture`, `cmake/`, `docs/architecture`,
    `tests/{unit,integration,differential,benchmarks,fixtures}`, `examples/{cpp,python}`,
    `bindings/python`, `tools/quevedomp-replay`, `tools/cuda_smoke`, `.devcontainer`.
  - `.gitignore` (build dirs `build*/`, `.cache/`, `compile_commands.json`, editor cruft).
  - `LICENSE` (placeholder text + TODO referencing spec §12 dual-license decision).
  - `README.md` with the quick-start that the rest of Phase 0 must make true.
- **Verify:** `git status` shows the tree; `find . -type d -not -path './.git/*'` lists it.
- **Done-when:** the directory layout matches §1.1 and is committed.

### Task 0.2 — CUDA dev container (the fix for the original failure)
- **Create `.devcontainer/Dockerfile`:**
  ```dockerfile
  # CUDA from day one: -devel includes nvcc + headers. NO OptiX COPY (login-gated,
  # not in repo -> that was the original hard build failure). OptiX is added later.
  FROM nvidia/cuda:12.4.0-devel-ubuntu22.04
  ENV DEBIAN_FRONTEND=noninteractive
  RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git pkg-config gdb \
        clang clang-format clang-tidy clangd \
        libeigen3-dev libspdlog-dev libfmt-dev libyaml-cpp-dev \
        libgtest-dev libgmock-dev libfcl-dev liburdfdom-dev \
        ca-certificates \
      && rm -rf /var/lib/apt/lists/*
  ARG UID=1000
  ARG GID=1000
  RUN groupadd -g $GID dev && useradd -m -u $UID -g $GID dev
  USER dev
  ```
  > **Pin note:** `12.4.0-devel-ubuntu22.04` is a known-existing tag (matches spec's CUDA
  > 12.4). If you prefer Ubuntu 24.04 toolchains, bump to a CUDA ≥12.5 `…-ubuntu24.04` tag —
  > but **verify the tag exists on Docker Hub first** (a non-existent tag is another silent
  > build failure). Container CUDA version must be ≤ what your WSL/Windows driver supports
  > (`nvidia-smi` "CUDA Version" is the ceiling).
- **Create `.devcontainer/devcontainer.json`:**
  ```json
  {
    "name": "quevedomp-cuda",
    "build": { "dockerfile": "Dockerfile" },
    "runArgs": ["--gpus", "all"],
    "customizations": { "vscode": { "extensions": [
      "llvm-vs-code-extensions.vscode-clangd",
      "ms-vscode.cmake-tools",
      "nvidia.nsight-vscode-edition"
    ], "settings": { "clangd.arguments": ["--compile-commands-dir=build"] } } }
  }
  ```
  > `--gpus all` requires the host setup in §H. If "Reopen in Container" fails to start with
  > a GPU error, the fix is on the host (toolkit/driver), not in this file.
- **Verify (the step that previously failed):**
  ```bash
  docker build -t quevedomp-cuda .devcontainer
  ```
  Then confirm tools exist AND the GPU is visible inside the container:
  ```bash
  docker run --rm --gpus all quevedomp-cuda bash -lc \
    "cmake --version && ninja --version && clang++ --version && nvcc --version && nvidia-smi"
  ```
- **Done-when:** `docker build` exits 0, the tool versions print, and `nvidia-smi` lists your
  GPU **from inside the container**. (VS Code: "Reopen in Container" succeeds.)

### Task 0.3 — Root `CMakeLists.txt`
- **Create `CMakeLists.txt`:**
  - `cmake_minimum_required(VERSION 3.25)`, `project(quevedomp CXX)` (note: **CXX only** at
    top; CUDA is enabled conditionally below so the CPU preset never needs `nvcc`), C++20,
    `CMAKE_EXPORT_COMPILE_COMMANDS ON`.
  - Options: `QUEVEDOMP_WITH_CUDA` (**default ON** — validate the toolchain early; the CPU
    preset turns it OFF), `QUEVEDOMP_WITH_OPTIX` (OFF until Phase 2b),
    `QUEVEDOMP_WITH_RERUN`/`QUEVEDOMP_WITH_PYTHON` (OFF), `QUEVEDOMP_BUILD_TESTS` (ON),
    `QUEVEDOMP_ENABLE_SANITIZERS` (ON for Debug).
  - `add_library(quevedomp INTERFACE)` + `add_library(quevedomp::quevedomp ALIAS quevedomp)`
    and `target_include_directories(quevedomp INTERFACE include/)`. *(Becomes a real CPU
    library in Phase 1; INTERFACE keeps Phase 0 source-free. The library stays **CPU-only**;
    CUDA enablement here is purely the smoke test, not library code.)*
  - ```cmake
    if(QUEVEDOMP_WITH_CUDA)
      include(CheckLanguage)
      check_language(CUDA)
      if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "WITH_CUDA=ON but no CUDA compiler found (need nvcc in PATH).")
      endif()
      enable_language(CUDA)
      add_subdirectory(tools/cuda_smoke)   # throwaway toolchain validator (Task 0.6)
    endif()
    ```
  - `if(QUEVEDOMP_BUILD_TESTS)`: `enable_testing()`, `find_package(GTest REQUIRED)` (CMake's
    bundled `FindGTest` module mode — robust against the apt-vs-vcpkg config-package mismatch
    the old attempt hit), `add_subdirectory(tests)`.
  - Keep `QUEVEDOMP_WITH_OPTIX` fully guarded so the no-OptiX build never references it
    (spec §1.2 "minimal build must compile").
- **Verify:** configure happens in Task 0.7. For now, file-presence + `cmake --help` sanity.
- **Done-when:** file exists; GTest via module mode; CUDA gated behind `WITH_CUDA`; OptiX absent.

### Task 0.4 — `CMakePresets.json`
- **Create** presets:
  - `dev-gpu`: `Ninja`, Debug, `QUEVEDOMP_WITH_CUDA=ON`, `QUEVEDOMP_WITH_OPTIX=OFF`,
    `QUEVEDOMP_ENABLE_SANITIZERS=ON`, binaryDir `build/dev-gpu`. **Primary Phase 0 preset** —
    builds the CPU library + the CUDA smoke test. Requires the CUDA container (§H).
  - `dev-cpu`: `Ninja`, Debug, `QUEVEDOMP_WITH_CUDA=OFF` (all `WITH_*` OFF),
    `QUEVEDOMP_ENABLE_SANITIZERS=ON`, binaryDir `build/dev-cpu`. The "minimal build must
    compile" proof (spec §1.2) — must build with **no `nvcc` and no GPU**.
  - `release`: `Ninja`, Release, `WITH_CUDA=ON`, OptiX OFF (flips ON in Phase 2b).
  - `ci-gpu`: present as a placeholder for a future self-hosted GPU runner; not used now (D4).
  - Add matching `testPresets` for `dev-gpu` and `dev-cpu`.
- **Verify:** `cmake --list-presets` shows them.
- **Done-when:** `dev-gpu` and `dev-cpu` are present and selectable.

### Task 0.5 — Trivial test
- **Create `tests/CMakeLists.txt`** and **`tests/unit/test_bootstrap.cpp`**:
  ```cpp
  #include <gtest/gtest.h>
  TEST(Bootstrap, Sanity) { EXPECT_EQ(1 + 1, 2); }
  ```
  - In `tests/CMakeLists.txt`: `add_executable(test_bootstrap unit/test_bootstrap.cpp)`,
    link `GTest::gtest_main` and `quevedomp::quevedomp`, then
    `include(GoogleTest); gtest_discover_tests(test_bootstrap)`.
- **Done-when:** files exist; CMake target defined.

### Task 0.6 — CUDA smoke test (proves the toolchain + GPU early)
- **Create `tools/cuda_smoke/CMakeLists.txt`** and **`tools/cuda_smoke/main.cu`** — a minimal
  program that launches a trivial kernel and copies a result back, exiting non-zero on any
  CUDA error:
  ```cpp
  #include <cstdio>
  #include <cuda_runtime.h>
  __global__ void add_one(int* x) { *x += 1; }
  #define CK(e) do{ cudaError_t r=(e); if(r){ \
      std::fprintf(stderr,"CUDA error: %s\n",cudaGetErrorString(r)); return 2; } }while(0)
  int main() {
    int n=0; CK(cudaGetDeviceCount(&n));
    if(n==0){ std::fprintf(stderr,"no CUDA device visible\n"); return 1; }
    int *d=nullptr,h=41; CK(cudaMalloc(&d,sizeof(int)));
    CK(cudaMemcpy(d,&h,sizeof(int),cudaMemcpyHostToDevice));
    add_one<<<1,1>>>(d); CK(cudaGetLastError()); CK(cudaDeviceSynchronize());
    CK(cudaMemcpy(&h,d,sizeof(int),cudaMemcpyDeviceToHost)); cudaFree(d);
    std::printf("cuda_smoke OK: result=%d (device count=%d)\n",h,n);
    return h==42 ? 0 : 3;
  }
  ```
  - In its `CMakeLists.txt`: `add_executable(cuda_smoke main.cu)`,
    `set_target_properties(cuda_smoke PROPERTIES CUDA_ARCHITECTURES native)`, and register it
    as a test: `add_test(NAME cuda_smoke COMMAND cuda_smoke)`.
  > This target is **only** compiled under `WITH_CUDA=ON` (wired in Task 0.3). It's a
  > throwaway probe — once Phase 2b lands real GPU code it can be removed. Its whole job is to
  > fail loudly *now* if nvcc, the driver, or `--gpus` passthrough is wrong, instead of in Phase 2b.
- **Done-when:** target + test defined under `tools/cuda_smoke/`.

### Task 0.7 — Build + test locally, BOTH presets (the DoD core)
- **Verify GPU build (run inside the CUDA container, `--gpus all`):**
  ```bash
  cmake --preset dev-gpu
  cmake --build --preset dev-gpu
  ctest --preset dev-gpu --output-on-failure
  ```
  Expect `2 tests passed` — `Bootstrap.Sanity` **and** `cuda_smoke` (the latter prints
  `cuda_smoke OK: result=42`). A failing `cuda_smoke` means the GPU toolchain/passthrough is
  broken — fix it before any further phase, which is the entire point of doing this now.
- **Verify minimal CPU build (no nvcc needed — proves the abstraction, spec §1.2):**
  ```bash
  cmake --preset dev-cpu
  cmake --build --preset dev-cpu
  ctest --preset dev-cpu --output-on-failure
  ```
  Expect `1 test passed` (`Bootstrap.Sanity`); `cuda_smoke` is absent by design.
- **Done-when:** both presets configure, build, and pass their tests.

### Task 0.8 — `.clang-format`
- **Create `.clang-format`:** `BasedOnStyle: LLVM`, `ColumnLimit: 100`, `Standard: c++20`.
- **Verify:** `clang-format --dry-run --Werror tests/unit/test_bootstrap.cpp` (exit 0).
- **Done-when:** tracked sources are clean under the style.

### Task 0.9 — (OPTIONAL) OptiX early validation — de-risk the biggest unknown now
> Do this **only if** you want to prove OptiX works before Phase 2b. It needs a one-time
> **manual** download (NVIDIA dev login) — it cannot go in an unattended `docker build`.
- Download `NVIDIA-OptiX-SDK-8.x.x-linux64-*.sh` from the NVIDIA developer portal into the
  repo-ignored path `/.devcontainer/optix/` (add to `.gitignore` — do **not** commit it).
- Add an **optional** Dockerfile stage guarded by a build arg, e.g.
  `ARG OPTIX_INSTALLER=` + `RUN if [ -n "$OPTIX_INSTALLER" ]; then sh /tmp/$OPTIX_INSTALLER --skip-license --prefix=/opt/optix; fi`,
  built with `docker build --build-arg OPTIX_INSTALLER=NVIDIA-OptiX-SDK-8.x.x-linux64.sh ...`.
- **Verify:** build & run one stock OptiX SDK sample (e.g. `optixHello`) inside the container;
  it should render/initialize without error on your GPU.
- **Done-when:** an OptiX sample runs — confirming RT-core access works on your hardware.
  *(If you skip this, OptiX is first set up in Phase 2b task 2b.0 instead.)*

### Phase 0 EXIT (DoD — checked manually)
- [ ] Host setup §H done: `nvidia-smi` works in WSL; `docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi` lists the GPU.
- [ ] `docker build -t quevedomp-cuda .devcontainer` exits 0; `nvidia-smi` works **inside** it.
- [ ] `dev-gpu`: configure + build + `ctest` → `2 tests passed` (incl. `cuda_smoke OK: result=42`).
- [ ] `dev-cpu`: configure + build + `ctest` → `1 test passed`, with **no nvcc** required.
- [ ] No OptiX reference reachable unless Task 0.10 was deliberately enabled.
- **Then update memory `phase_status.md` to Phase-0-complete with the commit SHA.**

---

## Phase 1 — Robot, URDF, FK, IK, Rng

> Promote `quevedomp` from INTERFACE to a real static library in Task 1.1. Each task below
> is independently testable; do not batch them.

### Task 1.1 — `core/types/` (Eigen-only, zero inter-module deps)
- Implement `JointState`, `Transform` (wraps `Eigen::Isometry3d`), `Pose`, `Sphere`, `Box`,
  `Mesh`, `Waypoint`, `Trajectory`, plus aliases `JointPosition`/`JointVelocity` (spec §6 Phase 1 block).
- Convert `quevedomp` to `add_library(quevedomp ...)` (static); `find_package(Eigen3 CONFIG REQUIRED)`.
- **Verify:** `tests/unit/test_types.cpp` — `Transform` compose/inverse round-trips to identity (<1e-12); `Box`/`Sphere` field access.
- **Done-when:** types compile, link, tests pass; no dependency on `robot/` or above.

### Task 1.2 — `core/rng/` + ADR-006
- `Rng(seed)`, `spawn(stream_id)`, `uniform`, `sample_in_box` (spec §5.2).
- Write `docs/architecture/adr-006-rng.md` (substreams, CPU determinism, best-effort replay).
- **Verify:** same seed → identical sequence; `spawn(k)` independent of call order/thread count (run on 1 and 4 threads, assert equality); statistical sanity (mean of `uniform` ≈ midpoint over 1e6 draws).
- **Done-when:** determinism tests pass; ADR committed.

### Task 1.3 — URDF parsing → `RobotModel` (data only, no FK yet)
- `Link`, `Joint` (Revolute/Prismatic/Fixed + pos/vel/acc/jerk limits), `KinematicChain`,
  `RobotModel::from_urdf(urdf, optional yaml_ext) -> shared_ptr<const RobotModel>`.
  Parse with `urdfdom` (apt) or `tinyxml2`; pick one and note it in the ADR/README.
- **Verify:** parse a tiny hand-written 2-link URDF fixture; assert joint count, parent/child wiring, limit values.
- **Done-when:** the toy URDF loads with correct topology.

### Task 1.4 — Source the 5 robot URDFs (own task — schedule risk per spec §6 note)
- Acquire/clean UR5, UR10, Franka Panda, KUKA iiwa, ABB IRB URDFs into `tests/fixtures/robots/`.
- Record provenance (source URL, license, any edits) in `tests/fixtures/robots/PROVENANCE.md`.
- **Verify:** each loads via Task 1.3 without error; DOF count matches the datasheet.
- **Done-when:** all 5 parse; provenance recorded.

### Task 1.5 — `kinematics/` FK
- `fk(model, q, link)` and `fk_all(model, q)`.
- **Verify:** FK position error **< 1e-9 m** vs a reference (KDL or published frames) for the 5 robots at several configs (spec DoD).
- **Done-when:** all 5 pass the 1e-9 m bar.

### Task 1.6 — Jacobian
- `KinematicChain::jacobian` (geometric/analytic).
- **Verify:** analytic vs finite-difference Jacobian **< 1e-6** across random configs.
- **Done-when:** passes for all 5 robots.

### Task 1.7 — `NumericalIk`
- `InverseKinematics` interface + `NumericalIk` (damped least squares, multi-seed restart);
  `make_numerical_ik(shared_ptr<const RobotModel>)`.
- **Verify:** FK∘IK ≈ identity over **1000 seeds**; convergence **< 10 ms** for reachable targets (spec DoD).
- **Done-when:** success rate + latency meet DoD; record numbers in test output.

### Task 1.8 — `viz/` (rerun, no-op when OFF)
- `log_robot`, `log_trajectory`, `log_pose`; compile to no-ops under `WITH_RERUN=OFF`.
- Add `pip install rerun-sdk` to the container **here** (deviation D3), guarded by `WITH_RERUN`.
- **Verify:** minimal build (`WITH_RERUN=OFF`) still compiles and links; with ON, a CI artifact image is produced.
- **Done-when:** both build modes pass; visual-sanity artifact stored in CI.

### Task 1.9 — Fuzz + coverage gate
- libFuzzer harness on malformed URDFs; coverage measurement.
- **Verify:** fuzzer runs N iterations without crash/UB (ASan clean); coverage **> 80%** in `core/types`, `robot`, `kinematics`.
- **Phase 1 EXIT:** all of §6 Phase 1 DoD met; update memory.

---

## Phase 2a — Collision interface + FCL backend (CPU pipeline complete)

### Task 2a.1 — Headers/contract (§4.2) — compile-only
- Land `collision/types.hpp`, `collision_scene.hpp`, `edge_check.hpp` exactly per §4.2
  signatures. Base-class `query` defined in terms of `query_batch`.
- Ratify the **§12 FK-location decision** (v0 = scene-internal FK) in an ADR before coding the backend.
- **Verify:** headers compile against a stub; signatures match §4.2 (no CUDA types present).

### Task 2a.2 — FCL backend `query_batch` (boolean)
- Broad-phase managers for environment + posed robot links; per-config FK → update transforms → `collide`.
- **Verify:** closed-form sphere-sphere / sphere-box / box-box cases (known overlap/clear).

### Task 2a.3 — FCL distance + witness
- Signed min-distance, clamp at `max_distance`, witness pairs (§4.3 semantics).
- **Verify:** property-based symmetry, non-negativity of separation, sign correctness at known penetration.

### Task 2a.4 — `check_edge`
- Discretize q0→q1 at `resolution`, batch-check, return `first_contact_t`.
- **Verify:** known free edge → `valid`; known colliding edge → correct `first_contact_t∈[0,1]`.

### Task 2a.5 — Serializers (`RobotModel`/`RobotInstance`/`CollisionScene`)
- Build now; reused by captures in Phase 3 (spec §5.3).
- **Verify:** round-trip equality (serialize→deserialize→compare).
- **Phase 2a EXIT:** full CPU pipeline buildable/testable on FCL; round-trips pass; coverage >80% in `collision/`. **No GPU.** Update memory.

---

## Phase 2b — OptiX static backend (FIRST GPU work)

### Task 2b.0 — Add OptiX to the (already CUDA-enabled) container + `FindOptiX.cmake`
> The CUDA container, `--gpus all`, and `nvcc` already exist and are validated from Phase 0.
> This task only adds **OptiX** on top — unless you already did the optional Task 0.9.
- Add the OptiX SDK to the Dockerfile via the build-arg stage from Task 0.9 (login-gated
  installer, build-arg, never committed), and flip `QUEVEDOMP_WITH_OPTIX=ON` in `dev-gpu`/`release`.
- Implement real `cmake/FindOptiX.cmake` (locates the SDK headers under `/opt/optix`).
- **Verify:** `find_package(OptiX)` succeeds under `dev-gpu`; an OptiX sample initializes on the GPU.
- **Done-when:** `cmake --preset dev-gpu` configures with OptiX found and the library builds.

### Task 2b.1 — OptiX backend (simple per-config launch first)
- GAS/IAS build, per-config transform update + any-hit boolean (§4.5). **Simple iterate-configs impl first; measure before optimizing.**
- **Verify:** unit-level — a single known collision/clear config matches FCL.

### Task 2b.2 — Differential harness (§4.6)
- N=10k random scenes/configs; assert boolean agreement **outside ±1e-4 m band**; report ambiguous fraction + throughput ratio.
- **Verify:** zero out-of-band disagreements; OptiX boolean throughput **≥ 5× FCL** (spec DoD). Wire as `ci-gpu` (self-hosted).
- **Phase 2b EXIT:** spec §6 Phase 2b DoD; benchmarks tracked (alert >10% regression). Update memory.

---

## Phase 3 — RRT pipeline + capture

### Task 3.1 — Planning types
- `TaskLimits`, `Goal`/`JointGoal`/`PoseGoal`/`MultiGoal`, `Constraints`, `PlanningProblem`, `PlanningResult` (with `used_seed` always populated).
- **Verify:** construction + invalid-problem detection unit tests.

### Task 3.2 — `RrtConnectPlanner` (CPU, FCL collision)
- `Planner` interface + RRT-Connect using `CollisionScene` batch edge checks; `make_planner(...)`.
- **Verify:** finds a known 2D solution in < N nodes; cross-check vs OMPL RRT-Connect on the same problem.

### Task 3.3 — `ShortcutSmoother`
- Iterative shortcut.
- **Verify:** smoother output is **still collision-free** and no longer than input.

### Task 3.4 — `ToppRaParameterization`
- Time-optimal under joint + TCP vel/acc.
- **Verify:** velocity **and** acceleration respected at every waypoint (jerk **not** guaranteed — ADR-011).

### Task 3.5 — Capture system + `quevedomp-replay`
- `PlanningCapture`, auto-dump-on-exception, MCAP serialization (reuse 2a.5 serializers), basic replay CLI.
- **Verify:** force an exception → capture written → `quevedomp-replay <bundle>` reloads and reproduces the failure class with a `PlanningTrace`.

### Task 3.6 — Full-pipeline integration
- **Verify:** MotionBenchMaker static subset; rerun overlay; benchmark vs MoveIt2 RRTConnect.
- **Phase 3 EXIT:** ≤50 ms mean UR5 plan (moderate scene); ≥95% free-space / ≥80% obstacle success; best-effort capture/replay works. Update memory.

---

## Phase 4 — Python bindings (nanobind)

### Task 4.1 — Build wiring
- `WITH_PYTHON=ON` finds nanobind (introduce vcpkg here **only if** nanobind isn't apt-available — else pip/FetchContent); `module.cpp` skeleton imports.
- **Verify:** `import quevedomp` succeeds in the container.

### Task 4.2–4.5 — `bind_{types,robot,collision,planning}.cpp` incrementally
- One binding TU per task; numpy zero-copy for `JointPosition`; generate `_native.pyi` stubs.
- **Verify (per task):** pytest mirroring the corresponding C++ tests; zero-copy assertion (mutate-through / no-copy address check) for types.

### Task 4.6 — End-to-end notebook
- load robot → plan → smooth → parameterize → visualize in rerun.
- **Verify:** notebook runs top-to-bottom; full pytest suite green; stubs present.
- **Phase 4 EXIT:** spec §6 Phase 4 DoD. Update memory.

---

## Quick reference — verification commands

```bash
# Host (WSL): GPU passthrough smoke test — must pass before building the container (see §H)
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi

# Build the CUDA dev container and confirm the GPU is visible inside it:
docker build -t quevedomp-cuda .devcontainer
docker run --rm --gpus all quevedomp-cuda bash -lc "nvcc --version && nvidia-smi"

# Phase 0 happy path — GPU preset (inside the container, started with --gpus all):
cmake --preset dev-gpu && cmake --build --preset dev-gpu && ctest --preset dev-gpu --output-on-failure
#   -> expect 2 tests passed, incl. "cuda_smoke OK: result=42"

# Minimal CPU build (proves the no-GPU path still compiles; no nvcc needed):
cmake --preset dev-cpu && cmake --build --preset dev-cpu && ctest --preset dev-cpu --output-on-failure
#   -> expect 1 test passed

# Style:
clang-format --dry-run --Werror $(git ls-files '*.hpp' '*.cpp')
```

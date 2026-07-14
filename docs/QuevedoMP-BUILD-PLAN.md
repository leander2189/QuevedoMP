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

## M. Mitigation amendments (ratified 2026-06-10)

A feasibility review against the project's primary goal — **outperform MoveIt for motion
planning in complex environments with high-polygon meshes, in quasi-static scenes** — found
the architecture sound but the plan exposed on five points. These amendments are binding and
override the original task text where they conflict:

| # | Amendment | Why |
|---|-----------|-----|
| M1 | **Task 0.9 (OptiX validation) is MANDATORY** and part of the Phase 0 exit gate. The container sets `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility` (default caps omit `libnvoptix.so.1`). | The whole performance bet rides on OptiX; WSL2 support is driver-dependent. Verified 2026-06-10 on this host: driver 576.57 exposes `libnvoptix.so.1` + `libnvoptix_loader.so.1` in `/usr/lib/wsl/lib` — the historical WSL blocker is absent here, but an in-container `optixHello` run is still required to close the gate. Fallback tree if it fails: (a) native Windows test outside Docker, (b) native-Linux/cloud GPU box for Phase 2b+, (c) Embree CPU ray backend becomes Plan A (same ray-cast semantics, no GPU). |
| M2 | **Benchmark spine (Phase B) added**, runs alongside Phases 1–2: high-poly fixtures (≥1M triangles), a written benchmark protocol (`docs/benchmarks/PROTOCOL.md`), and a MoveIt 2 baseline container. Phase 2a/2b/3 exits are re-gated on it. | The headline goal was first measured at week ~19 against MotionBenchMaker scenes that are mostly low-poly primitives — the differentiator was never exercised. "5× FCL bulk throughput" is a weak proxy; small-batch latency is what an RRT actually sees. |
| M3 | **Mesh-loading task added (Task 1.4b, assimp)** + `libassimp-dev` in the container. | `urdfdom` yields mesh *filenames* only; nothing in the original dependency set loads STL/DAE/OBJ. Fatal for a high-poly-mesh project. |
| M4 | **OptiX backend re-specced for batching (Task 2b.1)**: static environment GAS, robot surface rays transformed in raygen, per-config transforms in one device buffer, config index in launch dims — **no per-config IAS update**. Self-collision on CPU. Containment + margin policy decided as ADRs (012/013) before coding. | The spec's per-config "write IAS transforms → refit → launch" loop puts an AS update + launch + PCIe round-trip inside RRT's serial edge loop — the classic GPU-planner failure mode. Quasi-static scenes make the static-GAS design ideal. |
| M5 | **Phase 3 split**: 3a = planner + smoother + TOPP-RA + **MoveIt benchmark** (the goal gate); 3b = capture/replay/MCAP. Python bindings stay Phase 4 and are droppable from the performance milestone. | Capture/replay is good engineering but sat *between* us and the goal metric. |
| M6 | *(added 2026-07-11, ADR-016)* **Phase 4 split**: 4a = minimal nanobind slice over the Phase 3a API + `quevedomp-studio` (pure-Python interactive IDE: viser gizmos/widgets + rerun logging); 4b = the rest of the original Phase 4 (capture/TOPP-RA bindings, full pytest parity, notebook). 4a may start at the Phase 3a exit and does not gate — nor is gated by — Phase 3b. Binding rules: verb-level calls only (no Python inside C++ loops), GIL released on blocking calls, core untouched (`QUEVEDOMP_WITH_PYTHON=OFF` builds bit-identical). | An interactive IDE is wanted *for* Phase 3 debugging, not after it; the slice makes the IDE the bindings' test harness and surfaces binding-hostile API shapes while the C++ API is still cheap to change. Verb-level + GIL rules keep the performance contract intact. |

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

### Task 0.9 — (MANDATORY — amendment M1) OptiX early validation
> Was optional; **promoted to a Phase 0 exit gate** because the entire performance goal
> (beat MoveIt on high-poly meshes) rides on OptiX, and OptiX-on-WSL2 is driver-dependent.
> It needs a one-time **manual** download (NVIDIA dev login) — it cannot go in an
> unattended `docker build`.
>
> **Host probe result (2026-06-10, driver 576.57):** `/usr/lib/wsl/lib` contains
> `libnvoptix.so.1` + `libnvoptix_loader.so.1` — the historical "no OptiX in WSL" blocker
> is absent on this host. The container side requires
> `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility` (now set in the Dockerfile;
> the default `compute,utility` does NOT mount libnvoptix). Still required to close the
> gate: an actual OptiX sample initializing in-container.
>
> **Fallback tree if the sample fails:** (a) test natively on Windows outside Docker to
> isolate container vs driver; (b) move Phase 2b+ to a native-Linux or cloud GPU box;
> (c) pivot: an **Embree CPU ray-cast backend** becomes Plan A — same ray-cast collision
> semantics, no GPU/PCIe/WSL dependency, still expected to beat MoveIt on high-poly meshes.
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
- [x] Host setup §H done: `nvidia-smi` works in WSL; GPU passthrough into containers works
      (verified 2026-06-10: RTX 2000 Ada visible in-container via `--gpus all`).
- [x] `docker build -t quevedomp-cuda .devcontainer` exits 0; `nvidia-smi` works **inside** it.
      ✅ **Resolved 2026-06-27** (§N): a clean `docker build --no-cache` from the WSL-native
      engine fetched 49.5 MB of apt over the network and exited 0 — the mirrored-mode network
      blocker is gone. The committed Dockerfile (Kitware cmake, assimp, driver caps, OptiX
      build-arg stage) is now verified from scratch, incl. the OptiX SDK install
      (`--build-arg OPTIX_INSTALLER=NVIDIA-OptiX-SDK-8.1.0-linux64-x86_64-35015278.sh`).
- [x] `dev-gpu`: configure + build + `ctest` → `2 tests passed` incl. `cuda_smoke OK: result=42`
      (verified 2026-06-10 in-container on the real GPU; commit `e72a4b0`).
- [x] `dev-cpu`: configure + build + `ctest` → `1 test passed`, with **no nvcc** required
      (verified 2026-06-10).
- [x] No OptiX reference reachable unless Task 0.9 was deliberately enabled.
- [x] **Task 0.9 (now mandatory, M1):** an OptiX sample initializes in-container.
      ✅ **Closed 2026-06-27** (commit `b1dfeab`): `tools/optix_smoke` (`optixInit()` + device
      context) passes under the `dev-optix` preset. OptiX-on-WSL2 needed a fix beyond the M1
      note: this host's Windows driver (595.x) exposes only a dxcore *stub* `libnvoptix.so.1`
      to WSL (no `optixQueryFunctionTable`), so the real runtime libs are extracted from the
      matching NVIDIA **Linux** `.run` driver via `.devcontainer/setup-wsl-optix.sh` and loaded
      ahead of the stub (gitignored `.devcontainer/wsl-optix/` + `LD_LIBRARY_PATH`). On native
      Linux (the deployment target) the driver provides `libnvoptix.so.1` and none of this is
      needed — see `docs/tutorials/testing.md`. Fallback tree (Embree CPU backend) NOT needed.

**PHASE 0 COMPLETE — 2026-06-27.** Gate work in commits `b1dfeab` (OptiX gate) and `0de3dad`
(native-Linux docs + WSL helper). All EXIT boxes green. Proceed to Phase 1.

---

## N. Host issue log (this machine)

> **RESOLVED 2026-06-27.** WSL outbound TCP works again: a clean `docker build --no-cache`
> fetched apt packages over the network and `nvidia-smi` works in-container. Host driver is
> now 595.97 (was 576.57); `nvidia-smi` reports CUDA 13.2. Whatever fixed egress (driver/WSL
> update or networking-mode change), the symptoms below no longer reproduce. Kept for history.

**WSL outbound network is dead under mirrored mode (found 2026-06-10).** Symptoms: DNS
resolves (dnsTunneling=true) but **all** outbound TCP from WSL fails with
`No route to host` — from the WSL shell itself, from Docker bridge networks, and from
`--network=host` containers alike. Windows-side connectivity to the same endpoints is fine.
`.wslconfig` has `networkingMode=Mirrored`, `firewall=false`, so the blocker is Windows-side
packet filtering (endpoint security / Hyper-V layer) on mirrored WSL traffic, not the WSL
firewall. Consequences: `docker build` cannot apt-fetch; image rebuilds are blocked.
Options (user decision — changes global WSL behavior):
1. Switch `networkingMode=NAT` in `%UserProfile%\.wslconfig` + `wsl --shutdown` (most likely
   fix; mirrored mode is what breaks container/VM egress under many corporate EDRs).
2. Keep mirrored and add a firewall/EDR exception for the WSL interface.
3. Workaround used meanwhile: download artifacts Windows-side, mount into WSL/containers.

---

## Phase B — Benchmark spine (amendment M2 — runs alongside Phases 1–2)

> **Purpose:** the project's headline goal — *outperform MoveIt in complex, high-poly-mesh,
> quasi-static scenes* — must be a tracked metric from Phase 2a onward, not a hope measured
> at week 19. These tasks have no code dependencies on Phases 1–2 and can interleave.

### Task B.1 — High-poly benchmark fixtures
- Acquire 2–3 environments with **≥ 1M triangles total** into `tests/fixtures/scenes/`:
  e.g. a scanned/CAD industrial cell, dense shelving unit, and a cluttered tabletop with
  high-res scanned objects (Thingi10K / YCB scans / BlenderKit CC0 are good sources).
  Record license + provenance in `tests/fixtures/scenes/PROVENANCE.md`.
- Include at least one **non-watertight** mesh (real scans have holes) — it exercises the
  ADR-012 containment policy and ray robustness.
- **Verify:** each loads via the Task 1.4b mesh loader; triangle counts recorded.
- **Done-when:** fixtures committed (or LFS/scripted download), provenance recorded.

### Task B.2 — Benchmark protocol (already drafted)
- `docs/benchmarks/PROTOCOL.md` defines problems, metrics, and methodology. Keep it the
  single source of truth for every performance claim; update it by PR like code.
- **Done-when:** protocol committed and referenced by Phase 2a/2b/3 exit gates.

### Task B.3 — MoveIt 2 baseline container
- Separate `benchmarks/moveit-baseline/` Dockerfile (ROS 2 + MoveIt 2 binary install).
  MoveIt is **never** a dependency of `quevedomp` — it lives only in this container.
- Export the B.1 scenes + a UR5 problem set into MoveIt's scene format; script
  `run_baseline.py` produces the PROTOCOL.md metrics (p50/p95 plan time, success rate).
- **Verify:** baseline numbers reproduce across two runs within noise (< 10%).
- **Done-when:** baseline metrics for all fixture scenes are recorded in
  `tests/benchmarks/results/moveit-baseline.json` — the number to beat.
- **Schedule:** must exist before the Phase 2a exit gate (FCL microbenchmark compares
  against it).

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

### Task 1.4b — Mesh loading (amendment M3)
- `Mesh load_mesh(path)` via **assimp** (`libassimp-dev`, already in the container):
  STL/DAE/OBJ → `core/types/Mesh` (vertices/indices, SoA-friendly). Handle units/scale
  attributes (DAE meters-vs-mm is a classic silent bug) and degenerate-triangle cleanup.
- Wire into URDF loading: `urdfdom` yields collision/visual mesh *filenames*; resolve them
  relative to the URDF and load. Without this task no robot or scene mesh ever loads.
- **Verify:** load each Task 1.4 robot's collision meshes + the Task B.1 scene fixtures;
  assert vertex/triangle counts > 0 and bounding boxes are plausible (meters, not mm).
- **Done-when:** all robot + scene fixtures load; a malformed file raises, not crashes.

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

### Task 1.9 — Fuzz + coverage gate ✅ (2026-06-28)
- libFuzzer harness on malformed URDFs (`tests/fuzz/fuzz_urdf`, `fuzz` preset = clang +
  fuzzer/ASan/UBSan); coverage via g++ `--coverage` + `gcovr` (`coverage` preset). See
  `tests/fuzz/README.md` for the exact gate commands.
- **Verify:** ✅ bounded fuzz (54,192 iterations, RSS flat ~450 MB) exits 0 with no crash/UB, the
  urdfdom third-party leak suppressed via `tests/fuzz/lsan_suppressions.txt`. ✅ coverage **91%**
  (all of core/robot/kinematics ≥ 84%, gate is 80%).
- **Bugs found + fixed:** the fuzzer caught a real memory-safety bug — a cyclic joint structure
  drove `chain_to`/`fk_all`/`jacobian` into an unbounded walk (2 GB OOM). Fixed with cycle guards
  + a deep-XML-nesting guard + null-checked geometry casts; regressions in
  `tests/unit/test_robot_robustness.cpp`.

### Phase 1 EXIT (DoD — checked manually) ✅ **COMPLETE 2026-06-28**
- [x] 1.1 core/types · 1.2 core/rng (+ADR-006) · 1.3 URDF→RobotModel · 1.4 five robot URDFs ·
      1.4b mesh loading (assimp) + collision geometry + URI resolver · 1.5 FK (<1e-9 vs analytic
      + independent reimpl) · 1.6 Jacobian (<1e-6 vs finite difference) · 1.7 numerical IK
      (FK∘IK identity, warm-seed <10 ms) · 1.8 viz (rerun, no-op when OFF) · 1.9 fuzz + coverage.
- [x] `dev-cpu` 79/79 (ASan/UBSan), `dev-gpu` 80/80; coverage ≥ 80%; fuzz gate clean.
- Commits: Task 1.1 `4cbfd35` … Task 1.9 (this commit). Next: **Phase 2a** (collision interface + FCL).

---

## Phase 2a — Collision interface + FCL backend (CPU pipeline complete)

### Task 2a.1 — Headers/contract (§4.2) — compile-only
- Land `collision/types.hpp`, `collision_scene.hpp`, `edge_check.hpp` exactly per §4.2
  signatures. Base-class `query` defined in terms of `query_batch`.
- Ratify the **§12 FK-location decision** (v0 = scene-internal FK) in an ADR before coding the backend.
- **Verify:** headers compile against a stub; signatures match §4.2 (no CUDA types present).

### Task 2a.2 — FCL backend `query_batch` (boolean) ✅ (2026-06-30)
- Broad-phase managers for environment + posed robot links; per-config FK → update transforms → `collide`.
- **Verify:** ✅ closed-form sphere-sphere / sphere-box / box-box (overlap & clear), environment
  mesh, robot-vs-self honoring the ACM, multi-config batch, `move_object`/`remove_object`, and
  `BackendHint::ForceOptix` rejects (Phase 2b). `tests/unit/test_collision_fcl.cpp`.
- Implementation: `src/collision/fcl_scene.cpp` (FCL 0.7, `find_package(fcl CONFIG)`, private dep).
  Scene-internal FK (ADR-015); env geometry built once into a shared const broad-phase manager,
  posed robot objects + their manager live in the per-thread `Workspace` (lock-free queries).
- Primitive robot collision geometry (box/sphere/cylinder) and environment meshes fully supported;
  robot *mesh* links land in Task 2a.2b (below).
- dev-cpu 92/92, dev-gpu 93/93.

### Task 2a.2b — Robot-link mesh collision (resolve + load + BVH) ✅ (2026-06-30)
- Most real URDF links collide by *mesh*. `make_static_scene` gains a `MeshSources`
  `{package→dir}` map; the FCL scene resolves (`resolve_mesh_uri`) + loads (`load_mesh`) + scales
  each robot mesh link at build time, caching by URI+scale, then builds the same FCL `BVHModel`
  environment meshes already use. A mesh URI that can't be resolved/loaded **throws** — never
  silently skipped. (All Task 1.4b infra; no new collision capability — closes the 2a.2 gap so the
  CPU pipeline works on real robots before 2a.3 builds on top.)
- **Verify:** ✅ the real UR5 fixture (all-mesh collision geometry) builds a scene, collides with an
  enclosing box and clears a far one; a mesh robot built without `MeshSources` throws.
  `tests/unit/test_collision_fcl.cpp` (`FclMeshLinks.*`). dev-cpu 94/94, dev-gpu 95/95.

### Task 2a.3 — FCL distance + witness ✅ (2026-06-30)
- Signed min-distance, clamp at `max_distance`, witness pairs (§4.3 semantics). `opts.distance`
  populates `BatchResult.min_distance` + `witnesses`; `safety_margin > 0` forces the distance
  computation to gate the boolean (collision if signed distance < margin) **without** emitting
  distance output. `robot_padding` offsets the signed distance (×1 robot-env, ×2 robot-self).
- **Pairwise** exact robot-vs-env + robot-vs-self (honoring ACM), tracking min signed distance +
  witness (nearest pair when free, deepest when colliding). Runs only when distance/margin is
  requested; the boolean broad-phase remains the RRT hot path.
- **Verify:** ✅ separation = exact gap + witness on both surfaces; penetration sign/depth; touching
  ≈ 0; clamp at `max_distance`; `robot_padding` offset; `safety_margin` gates boolean with no
  distance output. `tests/unit/test_collision_fcl.cpp` (`FclDistance.*`).
- **Robustness:** `narrow_distance` avoids FCL's `enable_signed_distance` (GJK/EPA) path, which
  **aborts on a degenerate simplex at exact contact** — uses contact-enabled `collide` for the
  overlap/depth side and plain unsigned `distance` for separation. dev-cpu 100/100, dev-gpu 101/101.
- **Deferred:** `per_pair_padding` (PaddingMap override) is accepted in QueryOptions but not yet
  honored — wired when planning needs per-pair tuning.

### Task 2a.4 — `check_edge` ✅ (2026-06-30)
- Discretize q0→q1 at `resolution` (rad, max per-joint step; `n = ceil(max|Δq|/resolution)`,
  `n+1` samples incl. both endpoints), check as ONE `query_batch`, return the first colliding
  `t = k/n` (1.0 if free). `src/collision/edge_check.cpp`.
- **Verify:** ✅ free edge → `valid`, t=1.0; colliding edge → exact `first_contact_t` (0.5);
  already-colliding start → t=0.0; edge through an environment obstacle → exact t; mismatched
  q sizes / non-positive resolution → throw. `tests/unit/test_collision_fcl.cpp` (`FclEdge.*`).
- dev-cpu 105/105, dev-gpu 106/106.

### Task 2a.5 — Serializers (`RobotModel`/`RobotInstance`/`CollisionScene`) ✅ (2026-06-30)
- New `capture/` module: `serialize_robot_model` (retained URDF + optional tool YAML, re-parsed on
  load — lossless), `serialize_robot_instance` (model blob + ACM allowed pairs),
  `serialize_scene` (SceneDescription: id + pose + geometry variant incl. mesh verts/tris). Compact
  self-describing binary (4-byte magic + version); same-endianness round-trip (MCAP/zstd container
  is Phase 3, ADR-007). `deserialize_*` throw on bad magic/version/truncation.
  `include/quevedomp/capture/serialize.hpp`, `src/capture/serialize.cpp`. RobotModel now retains its
  `source_urdf()`/`source_yaml()`; ACM exposes `pairs()`.
- **Note:** the "CollisionScene" serializer is the **SceneDescription** (objects + poses per §5.3) —
  the capturable recipe a scene is rebuilt from via `make_static_scene` (the abstract backend has
  no object enumeration; capture stores inputs, not backend state).
- **Verify:** ✅ round-trip equality for all three (structure + re-serialize byte-equality), YAML
  extension survives, all four geometry kinds, ACM normalization; corrupt blobs throw.
  `tests/unit/test_capture_serialize.cpp`. dev-cpu 111/111, dev-gpu 112/112.

### Task 2a.6 — FCL-vs-MoveIt collision microbenchmark (amendment M2 gate) ⏸️ DEFERRED
- **Postponed (2026-06-30):** blocked on its prerequisites — Task B.1 (≥1M-triangle high-poly
  fixtures) and Task B.3 (MoveIt 2 baseline container), neither of which exists yet. Revisit when
  B.1/B.3 land; sequence unchanged (still gates *before* leaning on the GPU backend for dense
  meshes). Moving to Phase 2b in the meantime (user decision).
- Benchmark `query_batch` (boolean) on the Task B.1 high-poly fixtures at RRT-realistic
  batch sizes (10/50/100 configs) per `docs/benchmarks/PROTOCOL.md`; compare against the
  Task B.3 MoveIt baseline's collision-checking throughput.
- **Done-when:** numbers recorded in `tests/benchmarks/results/`. Expectation (not a hard
  gate): batch-first FCL without MoveIt's planning-scene overhead already meets or beats
  MoveIt on dense meshes. If it doesn't, understand why **before** building the GPU backend
  on top of the same architecture.

### Task 2a.7 — Collision visualization harness (viz + collision) ✅ (2026-06-30)
- Rerun-backed visual debugging for collision scenes (added ahead of Phase 2b so OptiX-vs-FCL
  discrepancies are eyeball-debuggable). Visualizer gains `log_mesh` vertex-color overload,
  `log_points` (sphere obstacles / witness points), and `log_segments` (skeleton / witness pairs).
- `examples/cpp/collision_visualize.cpp`: a fixture robot swept through a trajectory against a box
  + sphere environment; colliding configs render the robot red, distance witnesses drawn as a
  point-pair + segment. Writes `collision.rrd` under the `dev-viz` preset; a no-op otherwise.
- **Verify:** `tests/unit/test_collision_viz.cpp` drives the collision scene + Visualizer in the
  normal (WITH_RERUN=OFF) suite — asserts collision/witness correctness and that the viz path runs;
  under `dev-viz` it also emits an `.rrd`. dev-cpu/dev-gpu green; dev-viz builds + artifact.

- **Phase 2a EXIT (CPU pipeline):** ✅ interface + contract + FCL (boolean + distance + witness) +
  robot mesh links + `check_edge` + serializers + collision-viz, buildable/testable on FCL;
  round-trips pass. **No GPU.** ⏸️ 2a.6 microbenchmark + `collision/` coverage gate deferred with
  Task 2a.6 (needs B.1/B.3). Proceeding to Phase 2b.

---

## Phase 2b — OptiX static backend (FIRST GPU work)

### Task 2b.0 — Add OptiX to the (already CUDA-enabled) container + `FindOptiX.cmake` ✅ (2026-06-30, via Task 0.9)
> The CUDA container, `--gpus all`, and `nvcc` already exist and are validated from Phase 0.
> This task only adds **OptiX** on top — unless you already did the optional Task 0.9.
- **Satisfied by Task 0.9 (`b1dfeab`) + revalidated 2026-06-30.** OptiX SDK 8.1.0 is baked into the
  image (Dockerfile `OPTIX_INSTALLER` build-arg; installer gitignored under `.devcontainer/optix/`),
  `cmake/FindOptiX.cmake` locates `/opt/optix` and creates `OptiX::OptiX`, and the WSL OptiX runtime
  is assembled by `.devcontainer/setup-wsl-optix.sh` into `.devcontainer/wsl-optix/` (gitignored,
  proprietary), mounted at `/opt/wsl-optix` + `LD_LIBRARY_PATH`.
- **Revalidated:** `cmake --preset dev-optix` → "OptiX 8.1.0 found"; builds; `tools/optix_smoke`
  (`optixInit()` + device query) passes on the GPU (1.06 s).
- **Deviation (recorded):** the OptiX build uses the dedicated **`dev-optix`** preset (introduced by
  Task 0.9), **not** a `WITH_OPTIX=ON` flip of `dev-gpu`. Rationale: keeps the routine `dev-gpu`
  per-task verification OptiX-free (fast; no WSL-runtime mount needed) while `dev-optix` exercises
  the GPU-OptiX path. Flipping `dev-gpu`/`release` ON is deferred to **Phase 2b exit**, once the
  OptiX backend is stable, so it becomes continuously tested for the shipping build.

### Task 2b.1 — OptiX backend (amendment M4 — batched-raygen design, NOT per-config IAS) 🟡 boolean core DONE (2026-06-30)
> **Status:** the minimal-boolean backend (user-chosen first pass) is implemented and **agrees with
> FCL** config-for-config on a real mesh robot. Built incrementally, each step verified on GPU:
> PTX pipeline scaffold (`52e3fad`) → env GAS + trace (`35ee42e`) → batched transformed rays +
> atomicOr (`3fd0d37`) → the `OptixScene` (this commit). Files: `src/collision/optix/*`
> (`optix_programs.cu`, `launch_params.hpp`, `optix_pipeline.cpp`, `optix_scene.cpp`).
> - **Done:** env GAS over Box/Mesh objects (built once); per-link **triangle-edge** test rays in
>   link-local frame (SoA on device); per `query_batch` one host-FK'd transform block → **one 2D
>   launch** `(ray, config)` transforming each ray on the fly, trace terminate-on-first-hit,
>   `atomicOr` per config; **self-collision on CPU via an internal FCL scene** (ADR-014 item 4);
>   wired behind `make_static_scene(ForceOptix)` + `optix_available()`. Verify: `test_optix_backend`
>   — device toolchain self-test, primitive/empty degenerate build, and **OptiX==FCL** over a
>   shoulder sweep into a thin-slab obstacle (surface crossings; both collisions and frees present).
>   dev-optix all green; dev-cpu unaffected (113/113).
> - **ADR-012 containment DONE (2026-06-30):** a shared CPU `EnvContainment` (analytic inside-tests
>   for box/sphere/cylinder solids; parity ray for watertight meshes, non-watertight excluded + logged
>   once) runs in **both** backends per robot mesh link (interior point = mesh centroid). Closes the
>   false-free blind spot (a link fully inside an obstacle). `src/collision/containment.{hpp,cpp}`.
>   Verify: `FclContainment.MeshRobotInsideWatertightMesh` + `OptixBackend.ContainmentInsideWatertightMesh`
>   (robot inside a big cube mesh → collision; far tiny cube → free); OptiX↔FCL agreement preserved.
> - **Workspace perf DONE (2026-07-02):** the `OptixWorkspace` now owns an explicit CUDA stream and
>   **persistent, geometrically-grown** device buffers (transforms/results/params) + **pinned host
>   staging** for the H2D transforms / D2H results; per `query_batch` all copies + memset + launch are
>   enqueued on that stream and joined by one `cudaStreamSynchronize` — no per-call `cudaMalloc`/`Free`,
>   no whole-device sync. `src/collision/optix/optix_scene.cpp`. Verify:
>   `OptixBackend.ReusedWorkspaceVaryingBatchSizesAgreeWithFcl` (one workspace reused across 32→5→20
>   batches, OptiX==FCL each time); full dev-optix 121/121.
> - **Query pipelining DONE (2026-07-06, `4c16b1f`):** one host FK pass per config now feeds BOTH
>   the GPU transform block and the ADR-012 containment interior points (containment no longer
>   re-runs `fk_all`); the CPU self-collision pass runs **while the GPU traces** (stream sync moved
>   after it — what ADR-014 item 4 intended); the FK/transform-fill/cull loop is OpenMP-parallel
>   across configs, gated `if(n >= 64)` so single-config probes keep serial latency. DTC bench:
>   cull-ON robot-vs-env 35.6→23.1 ms @10k (1.32×→2.04× vs FCL); full query @1k 25.0→16.3 ms —
>   now at FCL parity and **bound by the CPU self pass** (GPU fully hidden). Algorithm explainer:
>   `docs/optix-collision.md`. Verify: 125/125, agreement 0/2000.
> - **Deferred (follow-ups within 2b.1 before Phase 2b exit):** ADR-013 **margins/padding**;
>   **distance/witness** (throws — FCL-authoritative per §4.5); **sphere/cylinder env GAS tessellation**
>   (containment already handles them analytically; needs a decision — fine conservative tessellation
>   vs exact analytic curved solids via multi-GAS/IAS — to stay inside the ±1e-4 m differential band);
>   **dynamic** add/move/remove (static v0).
> **Supersedes spec §4.5's "write IAS transforms → refit → launch per config" loop.** That
> design puts an AS update + kernel launch + PCIe round-trip inside RRT's *serial* edge
> loop — the classic GPU-planner failure mode. The replacement exploits exactly what the
> quasi-static target gives us:
- **Environment:** one static GAS over all environment meshes, built once at
  `add_object` time; refit only on (rare) `move_object`. **Never touched per query.**
- **Robot:** precompute per-link surface test rays (triangle edges + sample points) once at
  scene build, stored SoA on device. Per `query_batch` call: upload **one** buffer of
  per-config per-link FK transforms; raygen threads index `(config, link, ray)` via launch
  dimensions, transform their ray on the fly, and trace against the environment GAS with
  any-hit + early termination. **One launch per batch, zero AS updates.**
- **Self-collision: on CPU** (FCL convex pairs honoring the ACM, inside the same
  `query_batch` call). A 6-DOF arm has a handful of link pairs — µs-cheap — and this deletes
  the 8-bit-visibility-mask / ACM-in-anyhit complexity entirely.
- **Containment & margins:** per ADR-012 (parity-ray containment for watertight meshes,
  documented limitation otherwise) and ADR-013 (padding via vertex-offset inflation at scene
  build; `safety_margin` semantics FCL-only in v0). **Both ADRs ratified before coding.**
- Workspace owns: CUDA stream, persistent device buffers (transforms/results, grown
  geometrically), pinned host staging.
- **Verify:** unit-level — known collision/clear configs match FCL; a containment case
  (link fully inside a box) behaves per ADR-012.

### Task 2b.2 — Differential harness (§4.6) + latency profile
- N=10k random scenes/configs; assert boolean agreement **outside ±1e-4 m band**; report ambiguous fraction + throughput ratio.
- **Verify:** zero out-of-band disagreements; OptiX boolean throughput **≥ 5× FCL** on the
  high-poly fixtures (spec DoD) — **and** (M2) record the PROTOCOL.md small-batch latency
  profile: time per `query_batch` at batch = 10/50/100 on the B.1 fixtures, vs FCL. The GPU
  must win at realistic RRT batch sizes on high-poly scenes, not just at bulk 10k; if it
  loses below batch≈50, that bounds where `BackendHint::Auto` flips backends.
- **Phase 2b EXIT:** spec §6 Phase 2b DoD + small-batch latency recorded; benchmarks tracked (alert >10% regression). Update memory.

### Task 2b.3 — Backend performance follow-ups (2026-07-06 perf review)
Prioritized backlog from the post-pipelining review (numbers: DTC bench, `docs/optix-collision.md`
"Where the time goes"). None block Phase 2b exit; pull them in when the profile says so.
1. ✅ **Hybrid `BackendHint::Auto` dispatch by batch size** — DONE (2026-07-08). `HybridScene`
   (in `fcl_scene.cpp`) builds both backends when the robot is OptiX-eligible (ALL collision
   links are meshes — a primitive link casts no rays and would false-free on env) and the GPU
   scene builds (else Auto falls back to FCL-only, never throwing where FCL works). Routing:
   FCL for batches < `QUEVEDOMP_AUTO_BATCH_THRESHOLD` (default 256; 2b.2 will calibrate),
   distance/margin queries (ADR-013), and any post-build env edit (v0 OptiX is static — the
   first edit demotes routing to FCL permanently); OptiX otherwise. Verify: 4 `HybridAuto.*`
   tests (threshold-crossing agreement, primitive-robot fallback, post-edit demotion, distance
   routing); GPU preset 131/131, dev-cpu green.
2. ✅ **Raygen per-config early abort** — DONE (2026-07-08). `if (params.out[c]) return;` at the
   top of `__raygen__rg` (benign race: out[c] is monotonic 0→1; a stale read just traces a ray
   another thread already decided). DTC @10k configs: cull-on 23.1→19.0 ms (2.04×→2.27× vs
   FCL), cull-off 66.5→62.9 ms — at only 4% collision fraction; payoff grows with clutter.
3. **Parallel CPU self-collision** (day-sized): the full query is now bound by the FCL self pass
   (~16 µs/config). Partition the batch across OpenMP threads, one `FclWorkspace` per thread
   (same pattern as the FK loop). Compounding option: **convex hulls for self-pairs** (ADR-014
   already anticipates this) — self-checks don't need exact meshes.
4. **Ray-count reduction**: decimated (or hull) robot collision meshes shrink `num_rays` 10–100×
   — the constant factor every trace multiplies. Natural extension: two-tier proxy/exact rays
   (cheap proxy pass clears most configs; exact rays only for near-misses).
5. **Device-side broadphase cull, default-on**: move the 8-corner link-AABB-vs-env-AABB test into
   a pre-kernel (transforms are already uploaded) — deletes the host overhead that makes the cull
   opt-in (`QUEVEDOMP_OPTIX_CULL`) today.
6. **CUDA graphs** for the memcpy→memset→launch→memcpy sequence: cuts per-batch API overhead
   (~50–100 µs), pushing the GPU crossover toward smaller batches.
7. **FCL distance-path pruning**: skip pairs whose AABB lower-bound distance exceeds the current
   best (or `max_distance`); early-out when only `safety_margin` gating is needed. Matters once
   distance queries sit inside a smoothing/optimization loop (see Task 3.3b).

---

## Phase 3a — Motion-planning pipeline + the goal gate (amendment M5: capture moved to 3b)

### Planner performance contract (2026-07-08 — binding on EVERY `Planner` implementation)
The planning stage is algorithm-agnostic: planners are selectable at execution time (and
build-time selection falls out — an unbuilt planner is simply not registered). What is NOT
negotiable is how any of them uses the collision backend and how its performance is observed.
Every implementation, sampling- or optimization-based, satisfies:
1. **Batch-first collision, always.** All collision goes through `check_edge`/`query_batch`;
   per-config `query()` is forbidden in planner code. Target: median collision batch at or above
   the hybrid Auto crossover (~256 configs; `QUEVEDOMP_AUTO_BATCH_THRESHOLD`) so the GPU path
   engages. An algorithm that structurally emits thin batches (a vanilla extend loop) must
   aggregate (k candidate extensions validated as one batch) or validate lazily — see Task 3.2.
2. **Two time budgets, not one.** *First-feasible* and *polished* are separate: the planner's
   job is fast-to-feasible; "smooth and logical" belongs to the polish loop (Task 3.3/3.3b/3.3c)
   under its own budget. Concrete numbers are fixed when the Task 3.5 baseline lands (first run
   records, second enforces — the coverage-gate pattern).
3. **Determinism per seed.** Same `PlanningProblem` + seed ⇒ same trajectory (the ADR-006 Rng
   contract). Required for benchmark comparability, regression bisection, and capture/replay.
4. **One `Workspace` per thread, no hidden mutable state** — the collision API is lock-free
   concurrent by design; planners must preserve that (parallel planners stay possible).

### Task 3.1 — Planning types ✅ (2026-07-10)
- `TaskLimits`, `Goal`/`JointGoal`/`PoseGoal`/`MultiGoal`, `Constraints`, `PlanningProblem`, `PlanningResult` (with `used_seed` always populated).
- **Implemented** in `include/quevedomp/planning/types.hpp` + `src/planning/types.cpp` (namespace
  `quevedomp::planning`). Design notes: goals are a polymorphic `Goal` base (clonable, `GoalType`
  discriminator) carrying only the target *description* — `JointGoal` is self-contained
  (`satisfies()` needs no model), `PoseGoal`/`MultiGoal` are model-coupled and interpreted by the
  planner (Task 3.2). `Path = vector<JointPosition>` (untimed; TOPP-RA makes the timed
  `Trajectory`). `PlanningProblem` carries `collision::QueryOptions` + timeout + optional seed;
  `Constraints` v0 supports per-joint bound narrowing (Cartesian path constraints deferred).
  `PlanningStats::record_batch()` is the single place collision accounting is bumped.
  `validate(problem, model)` returns an optional reason string (nullopt ⇒ valid) — planners call it
  first and map a failure to `PlanningStatus::InvalidProblem` without running the search.
- **Verify:** ✅ `tests/unit/test_planning_types.cpp` (23 cases: construction, `JointGoal::satisfies`,
  stats accounting, `to_string`, `Goal::clone`, and invalid-problem detection across every reject
  path). dev-cpu 143/143 (ASan/UBSan), dev-gpu 144/144; clang-format clean.
- **(2026-07-08) `PlanningStats`**, populated by every planner alongside `PlanningResult`:
  collision-query count, **batch-size histogram**, time split (collision / planner logic /
  smoothing / parameterization), nodes-or-iterations expanded, first-solution vs. final time.
  Rationale: the performance contract is unenforceable without attribution — when a planner is
  slow, the stats must show at a glance whether it starved the GPU with thin batches or burned
  the budget in its own logic. Cheap to carry from day one, painful to retrofit.
- **Verify:** construction + invalid-problem detection unit tests.

### Task 3.2 — `Planner` interface + selectable planners; `RrtConnectPlanner` first ✅ (2026-07-10)
- **Implemented.** `include/quevedomp/planning/planner.hpp` (`Planner` interface + `PlannerParams` +
  `make_planner` + `registered_planners`), `src/planning/planner.cpp` (string-id registry; unknown
  id / null args throw `std::runtime_error` — no silent fallback), `src/planning/rrt_connect.cpp`
  (the batched RRT-Connect, registered under `"rrt_connect"` via `src/planning/planners_internal.hpp`).
  - **Deviation (recorded):** `make_planner` takes `shared_ptr<const RobotInstance>` (not spec §6's
    `RobotModel`) — collision queries need the ACM the instance carries; the model is reached via
    `robot->model_ptr()`. Mechanically required, so the plan wins.
  - **Batched design (contract item 1):** growth validates `batch_size` (default 64) candidate
    extensions concatenated into ONE `query_batch`; "connect" validates the full bridge edge
    (new node → nearest node of the other tree) at `edge_resolution` — a collision-free bridge IS a
    greedy connect. A single `Rng(used_seed)` drives all sampling single-threaded (contract item 3);
    `plan()` keeps per-call scratch in locals, no member mutation (contract item 4). Coarse→refine
    edge pass and bisection/chunked `check_edge` ordering are the noted follow-ups (not yet needed —
    correctness + fat batches already met).
  - `PlanningStats` populated (batch histogram, config/query counts, time split, iterations).
- **Verify:** ✅ `tests/unit/test_planner.cpp` (10 cases): registry lists `rrt_connect`; unknown id /
  null args throw; known 2D solution (a 2-DOF gantry point-robot around a wall+gap) found, its path
  independently re-validated collision-free via `check_edge`; determinism per seed (identical path);
  `InvalidProblem` + `used_seed` populated on rejection; colliding start/goal → `NoSolution`; the
  batch histogram shows fat batches (max ≥ 256, majority of configs in ≥256 batches). dev-cpu
  153/153 (ASan/UBSan), dev-gpu 154/154; clang-format clean.
  - **OMPL cross-check DEFERRED:** OMPL is not an apt dependency (deviation D2) and adding it is a §12
    decision — deferred like Task 2a.6's MoveIt baseline; correctness is checked self-containedly
    (collision-free re-validation of the returned path) instead. Revisit if/when OMPL lands.
- `Planner` interface bound by the **performance contract** above; a `make_planner(type, params)`
  factory/registry selects the algorithm at **execution time** (a string/enum id — what the spec
  §283 `PlannerConfig` already sketches). Planners registered at build time; selecting an
  unregistered planner is a clear error, not a silent fallback. **No automatic algorithm
  selection** — the caller chooses; the Task 3.5 benchmark measures whichever is selected, so
  every registered planner is comparable on the same gate.
- First implementation: RRT-Connect over `CollisionScene` batch edge checks — chosen to validate
  the interface + contract and give the gate a reference number, not as a commitment.
- **Design for the GPU's appetite (M4):** coarse-resolution edge pass first, refine survivors, so
  batches stay as fat as correctness allows. Keep the validation call sites few and explicit:
  - **Batch across edges, not within one edge:** sample k candidate extensions and validate all
    their edges as one `query_batch` — one RRT edge (10–100 configs) sits below the GPU
    crossover (~a few hundred); k edges together clear it. Batch-shaped algorithms (BIT*-style
    batched informed search, lazy PRM variants) slot in behind the same interface if they feed
    the backend better; the quasi-static cells also make a per-cell roadmap (PRM built once,
    queried many times) a natural candidate.
  - **Lazy edge evaluation:** build optimistically, collision-check only edges on a candidate
    path — order-of-magnitude fewer queries in clutter, and the surviving checks are fat batches.
  - **Bisection sample ordering in `check_edge`** (t = 0, 1, ½, ¼, ¾…) + chunked submission
    (endpoints, then doubling): colliding edges detected after far fewer samples; gives the CPU
    backend an early-out without hurting the GPU path.
- **Verify:** finds a known 2D solution in < N nodes; cross-check vs OMPL RRT-Connect on the same
  problem; `PlanningStats` shows the batch-size histogram meeting the contract target; selecting
  an unregistered planner id fails loudly.

### Task 3.3 — `ShortcutSmoother` ✅ (2026-07-10)
- Iterative shortcut. **Implemented** `include/quevedomp/planning/smoother.hpp` (`Smoother`
  interface + `SmootherParams` + `make_shortcut_smoother`) and `src/planning/shortcut_smoother.cpp`.
  Each iteration picks path indices i<j (≥1 interior node) and validates the direct chord
  path[i]→path[j] as ONE batch via `collision::check_edge`; if free, the interior nodes are erased
  (a sub-polyline → its chord). Collision-safe (chord checked) and length-safe (chord ≤ polyline by
  the triangle inequality ⇒ output never longer). Endpoints never removed; a single `Rng(seed)`
  drives index choice ⇒ deterministic. Follow-ups noted in-source: batched multi-shortcut and
  continuous/partial (interpolated-endpoint) shortcut — not needed for the Done-when.
- **Verify:** ✅ `tests/unit/test_smoother.cpp` (5 cases): output re-validated collision-free via
  `check_edge` (free zig-zag collapses strictly shorter; wall detour stays free — the cross-wall
  chord is rejected), length never exceeds input, endpoints preserved, determinism per seed,
  short-path/null-arg handling. dev-cpu 158/158 (ASan/UBSan), dev-gpu 159/159; clang-format clean.

### Task 3.3b — (NEW, 2026-07-06 perf review) GPU environment SDF for clearance-aware smoothing
- Sampling planners give *feasible*; **smooth and logical** comes from an optimization pass that
  wants clearance **gradients**, which brute-force FCL `distance()` is far too slow to serve
  inside a loop. Since the environment is quasi-static (the same assumption the GAS design
  exploits), precompute a **voxel SDF of the environment on the GPU once** at scene build,
  decompose robot links into spheres, and clearance + gradient per config becomes a handful of
  trilinear lookups (the cuRobo recipe). Enables CHOMP/TrajOpt-style post-smoothing (or a
  gradient-informed shortcut) far beyond what Task 3.3 alone gives.
- Scope decision when reached: SDF resolution/memory budget, exact-vs-sphere robot tradeoff
  (ADR-013 alternatives already sketch sphere approximation), and whether it lands as a
  `CollisionScene` extension or a separate `ClearanceField` type. Raise as a §12 decision.
- **Verify:** SDF distance vs FCL `distance()` within voxel-resolution tolerance on the B.1
  fixtures; smoother using it produces collision-free output with measurably higher min-clearance
  than Task 3.3's shortcut alone.

### Task 3.3c — (NEW, 2026-07-08) Optimization-based planner/refiner — a library FEATURE
- A gradient-based trajectory optimizer (CHOMP/TrajOpt-flavored) over the Task 3.3b clearance
  field, exposed as a **first-class registered planner behind the same `Planner` interface** —
  selectable at execution time like any other, NOT an automatic fallback. Two composition modes:
  - **Refiner:** seeded with a feasible trajectory from a sampling planner (the pipeline's
    polish stage — this is where "smooth and logical" is manufactured);
  - **Standalone:** seeded with a straight-line (or IK-interpolated) guess — fast when it works,
    honest failure when it doesn't (local minima); `PlanningStats` reports which mode ran.
- Bound by the same performance contract: clearance/gradient lookups are batched over all
  trajectory waypoints per iteration (the SDF makes this a fat, GPU-friendly query by
  construction); deterministic per seed; polished-budget applies.
- Depends on 3.3b; scope the cost terms (clearance, joint-limit, smoothness) + the final
  feasibility re-check through `CollisionScene` (the optimizer trusts the SDF, the certificate
  comes from the exact backend).
- **Verify:** on the B.1 fixtures, refiner mode measurably improves smoothness + min-clearance
  over Task 3.3 shortcut output at equal success rate; standalone mode solves the easy subset
  within the polished budget; every output re-validated collision-free by the exact backend.

### Task 3.3d — (NEW, 2026-07-12) studio-profiling follow-ups

Profiling the inlet cell end to end through the Python bindings (`examples/python/
inlet_plan_profile.py`; rbrobout_inlet + work object, 3.4 rad sweep, edge 0.01, seeds 16–18)
attributed the cost precisely — collision is 99.9% of plan time; planner logic is 2–3 ms — and
surfaced these follow-ups, roughly in value order:

| # | Task | Evidence |
|---|------|----------|
| P1 | ✅ **Studio/bindings run Release** (`release-py` preset added; studio README updated). | Same problem, same seeds: Debug 33.8 s → Release 2.4 s (14×). Debug was the studio default. On the user's saved `benchmark.qmps`: 54.8 s → 6.2 s. |
| P2 | ✅ **Tessellate primitive collision geometry for the OptiX backend** (2026-07-12: `collision/tessellate.{hpp,cpp}`, env sphere/cylinder + robot primitives ride the GAS/edge-ray path; Auto's all-mesh gate relaxed). Differential DTC tests green on GPU; random-config A/B agrees 2048/2048. | Benchmark cell env-only: OptiX 9.9 µs/config vs FCL 17.1 µs (1.7×) — but see P7: self-collision masks it entirely on this robot. |
| P7 | ✅ **(P7a, 2026-07-12) Parallel batch collision**: `FclScene::query_batch` fans configs across cores (OpenMP, internal per-thread workspace pool — ADR-005's caller pattern applied inside one call; threshold 8 skips fork overhead on thin batches). The OptiX backend's internal self-collision batches through the same path. Results bit-identical to serial (regression test `FclBatchParallel.MatchesSerialSingles`). P7b (GPU self-collision via per-link GAS + relative-transform rays) stays deferred until MPPI-scale batches need it. | `benchmark.qmps`: Release-FCL 6.2 s → **0.72 s** (8.7×; 16 cores). Cumulative vs the original Debug studio run: 57.9 s → 0.72 s (~80×). |
| P8 | ✅ **RESOLVED as won't-do (2026-07-13)** — no scene-complexity routing heuristic. Leandro's objection: parallel FCL "wins" only because the dev box donates 16 idle cores; a deployed cell shares them with perception/control, so a crossover calibrated on idle-box microbenchmarks overfits the hardware. The contention experiment then showed the sharper truth: capping OMP_NUM_THREADS ∈ {1,2,4,16} on `benchmark.qmps`, OptiX degrades IN LOCKSTEP with FCL (6.79 s vs 6.81 s at 1 thread; 0.85 vs 0.69 s at 16) — on mid-poly cells the "GPU backend" is CPU-bound anyway (internal-FCL self-collision + host FK), so routing to it frees no cores. Decisions: keep the existing latency threshold (256; thin batches → CPU — that part is launch-latency physics, not contention-dependent); keep "fat batch ⇒ GPU" (correct where the GPU wins outright: high-poly environments — re-measure once the ≥1M-tri Phase B fixtures exist); deployment core-budgeting = standard OpenMP controls (`OMP_NUM_THREADS`, near-linear 6.8 s → 0.69 s from 1→16); **P7b (GPU self-collision) is the real "GPU frees the CPU" lever** and stays deferred until MPPI-scale batches or core-starved deployments need it. | Contention table above; Auto's ~15% penalty on the idle-box benchmark is the cost of not overfitting — accepted. |
| P3 | ✅ **Cartesian-bounded edge discretization** (2026-07-12: `collision/edge_discretization.{hpp,cpp}`): per-joint lever weights w_i from a conservative tip→base bounding-ball recursion (m/rad; prismatic = 1 m/m; mesh extents exact per vertex), step count enforcing Σ w_i·|Δq_i| ≤ `max_link_sweep` per edge step — an FK-tracked test verifies no geometry point outruns the bound. Wired through `PlannerParams`/`SmootherParams` (+`lever_weights`, auto-computed when empty), `check_edge` overload, Python (`cartesian_lever_weights()`), the studio knob, `.qmps` `max_link_sweep`, and `session_profile.py --sweep`. 0 = off; the uniform `edge_resolution` fallback is bit-identical to pre-P3. Fixed en route: sweep-fat batches (>~2.4k configs × ~430k robot rays) overflowed OptiX's 2³⁰-thread launch cap (`OPTIX_ERROR_LAUNCH_FAILURE`) — `optix_scene` now chunks configs per launch. | `benchmark.qmps` seed 8 (bmt_9636 weights 1.00–2.02 m/rad, Σw ≈ 8.5 — so edge 0.01 rad only ever guaranteed ≈ 85 mm worst-case sweep): **equal guarantee** `--sweep 0.085` = 45k configs · 0.35 s vs baseline 90k · 0.69 s (**2× fewer configs at the same fidelity**); `--sweep 0.04` ≈ baseline cost at 2× the fidelity; the honest 5 mm guarantee = 919k · 6.2 s FCL / 7.2 s OptiX — the old resolution was cheap because its guarantee was weak. Pick d per scene; the guarantee is now stated in millimetres. |
| P4 | ✅ **ACM covers robot-vs-environment pairs** (2026-07-13). Both backends resolve the ACM's link × object string pairs to integer masks once per query_batch (empty ⇒ the pre-P4 fast paths, zero overhead). FCL: filtered broad-phase callback (env objects carry their handle as negative user data), allowed pairs excluded from distance/witness exactly like self pairs, safety-margin path included. OptiX: per-triangle object-index buffer + an `__anyhit__` that `optixIgnoreIntersection`s allowed (link, object) hits — anyhit is DISABLED at trace time when no env pair is allowed, so the fast path keeps the hardware stage off. ADR-012 containment honors the same pairs per solid (EnvContainment got per-object identity + skip masks; mesh parity is now counted per object, which also fixes the overlapping-solids parity wart). Differential-tested: FCL/OptiX agree config-for-config with env pairs in play. | Inlet re-probe (2026-07-13): dresskit_*×work_object witnesses are GONE (the allowance in inlet_plan_profile.py now works). The −0.10 m pre-insertion goal stays infeasible for a REAL reason the old witness masked: every reachable IK branch has the rigid wrist assembly (tool_changer, wrist_3, wrist_extension, forearm, wrist_1) 0.3–1.5 cm INSIDE the work object, and ≤ −0.12 m is out of IK reach — a pose/mesh-calibration question, not a collision-layer gap. P5 (goal-sampling budget) remains the planner-side follow-up. |
| P5 | **Goal sampling budget** (deferred 2026-07-13 at Leandro's call): PoseGoal sampling gives up after ~3–5 IK branches ("all goal configurations are in collision"); cluttered cells need dozens + interleaved resampling during search. The building block landed the same day: `InverseKinematics::solve_all` (distinct-branch collection, deterministic, seed-nearest ordering, `IkOptions.branch_tol`; custom costs = caller-side sort) + studio branch picker (`session.solve_ik_branches`, collision-annotated). Wiring `resolve_goal` to it is the remaining P5 work. | Same sweep: feasible-looking poses rejected instantly with 3–5 sampled configs. |
| P6 | ✅ **Batched + time-budgeted smoothing** (2026-07-13). The smoother was the last thin-batch consumer: one chord per round meant 200 serial round trips. Now each round samples up to `SmootherParams::batch_size` (default 8) candidate chords with DISJOINT interiors, validates them as ONE query_batch (P3 discretization policy applies), and accepts every free one right-to-left — same collision/length guarantees, deterministic per seed, `batch_size = 1` reproduces the pre-P6 smoother draw-for-draw. `time_budget` (s, checked per round) makes smoothing anytime for the Task 3.5 pipeline. Task 3.3b (SDF clearance smoothing) remains the structural fix. | `benchmark.qmps` seed 8 (raw 10 wp / 7.23 rad): serial 516 ms vs batched 355 ms at IDENTICAL output (7 wp / 7.213 rad); `time_budget = 0.1 s` → 100 ms, STILL identical output — the 200-attempt tail was pure diminishing returns, so the budget is nearly free on this cell. Batched rounds are guaranteed past the Auto GPU threshold (>256 configs), which is where the ≥1M-tri Phase B fixtures will collect the larger win. |

### Task 3.4 — Time parameterization (TOPP + tip limits + jerk; ADR-017, supersedes ADR-011's jerk clause)
Scope adopted 2026-07-14 from Leandro's spec (docs/topp_jerk_tip_spec.md) after pro/con review;
ratified decisions in ADR-017: (a) OSQP-hybrid solver strategy, (b) tip acceleration IN scope
(per-axis form), (c) the C³ path stage lives inside this task, (d) ADR-011 revised.

- ✅ **Stage 0 (2026-07-14):** `parameterization/PathSpline` — C⁴ degree-5 B-spline over planner
  waypoints (jerk is meaningless on a polyline; spec §7.2) + `fit_collision_free()` re-validating
  the curve at P3 edge fidelity (ONE query_batch per round, densify-and-refit toward the validated
  polyline on collision, loud failure). Eigen-unsupported spline wart documented: Dynamic-dimension
  `Spline::operator()` asserts — one fixed-degree 1-D spline per joint instead (identical math).
- ✅ **Stage 1 (2026-07-14):** `parametrize()` Phase A — the convex problem (joint vel/acc + tip
  vel/acc) solved exactly by a dependency-free TOPP-RA-style recursion (2-var vertex-enumeration
  LPs backward, greedy maximal profile forward). `limits_from_model()` maps URDF + yaml jerk
  extension + TaskLimits; `JointState` gained `acc`; Python bindings.
  **Verify (done):** analytic 1-DOF trapezoid/triangle durations to <1%; joint AND tip limits
  hold at every node on UR5 splines; tip-speed cap saturates over the stroke (near-constant tool
  speed — the paint-pass behavior); rest-to-rest, monotone time, deterministic;
  **differential vs pip toppra** (same dense path, vel+acc): durations within 2% (test-only dep,
  auto-skipped when toppra is absent).
- ⏳ **Stage 2:** jerk via SCP over OSQP (FetchContent, first non-apt C++ dep — the §12/D2
  escape hatch, recorded in ADR-017): PSD Taylor-model QP subproblems + trust region, warm-started
  from Phase A; infeasibility reported with the max-violation node (spec §7.5). Verify: |q⃛| ≤
  j_max in the smooth interior, runtime measured against the polished budget, IPOPT fallback
  documented.

### Task 3.5 — Full-pipeline integration + **the goal benchmark**
- **Verify:** MotionBenchMaker static subset **plus the B.1 high-poly fixture set**; rerun
  overlay; full PROTOCOL.md run vs the B.3 MoveIt baseline.
- **Phase 3a EXIT (the project's headline gate):**
  - ≤ 50 ms mean UR5 plan (moderate scene); ≥ 95% free-space / ≥ 80% obstacle success.
  - **p50 end-to-end plan time ≥ 5× faster than the MoveIt 2 baseline on the high-poly
    fixture set, at equal or better success rate** (per PROTOCOL.md).
  - **(2026-07-08) Trajectory-quality metrics recorded per run** — the gate is not time+success
    alone; "smooth, logical" must be measured: path-length ratio (joint-space length vs.
    best-known for the fixture), a smoothness measure on the post-TOPP-RA trajectory (e.g.
    summed squared joint jerk), and minimum clearance along the path. First full run RECORDS
    the three; thresholds are then fixed and ENFORCED from the next run on (the coverage-gate
    pattern). All three come from `PlanningStats` + the parameterized trajectory, so any
    selected planner is measured identically.
  - **Time-budget split fixed here:** the first-feasible and polished budgets (performance
    contract item 2) get their concrete numbers from this baseline run.
  - **Scene-update budget (quasi-static story):** `move_object` + AS refit ≤ 100 ms on the
    largest fixture.
  - Update memory with the numbers, not just "done".

## Phase 3b — Capture & replay (moved off the goal's critical path, M5)

### Task 3.6 — Capture system + `quevedomp-replay`
- `PlanningCapture`, auto-dump-on-exception, MCAP serialization (reuse 2a.5 serializers), basic replay CLI.
- **Verify:** force an exception → capture written → `quevedomp-replay <bundle>` reloads and reproduces the failure class with a `PlanningTrace`.
- **Phase 3b EXIT:** best-effort capture/replay works (spec §5 DoD). Update memory.

---

## Phase 4a — Minimal Python slice + `quevedomp-studio` (amendment M6, ADR-016)

> May start at the **Phase 3a exit** (independent of Phase 3b). Scope: bind the API that exists
> at that point — nothing speculative. Binding rules (binding on every task below): **verb-level
> calls only** (no Python callback from inside a C++ loop); **GIL released** on `plan`, `smooth`,
> `solve`, `query_batch`, `check_edge`, `make_static_scene`, `load_mesh`, `from_urdf`; one
> `Workspace` per Python thread (ADR-005); the core is never modified for Python's benefit.
> Zero-copy applies to large arrays (mesh vertices/triangles, `BatchResult` vectors, result
> paths); small per-call values (`q`, `Transform`) may copy at the boundary.

Package layout:

```
bindings/python/
├── CMakeLists.txt              # QUEVEDOMP_WITH_PYTHON=ON; nanobind via pip/FetchContent
├── src/
│   ├── module.cpp              # NB_MODULE(_native), calls the four registrars
│   ├── bind_types.cpp
│   ├── bind_robot.cpp
│   ├── bind_collision.cpp
│   └── bind_planning.cpp
└── quevedomp/
    ├── __init__.py             # re-exports _native; pythonic helpers only (no logic)
    ├── _native.pyi             # generated stubs
    └── py.typed
tools/quevedomp-studio/         # Task 4a.6 — pure Python, OUTSIDE the C++ build
```

### Task 4a.1 — Build wiring
- `QUEVEDOMP_WITH_PYTHON=ON` finds nanobind (pip/FetchContent — vcpkg **only if** unavailable, per D2's spirit); `module.cpp` skeleton imports. `OFF` (default) builds bit-identical to today.
- **Verify:** `import quevedomp` succeeds in the container; `OFF` build compiles untouched.

### Task 4a.2 — `bind_types.cpp` (core vocabulary)
- `Transform`: `Identity`/`from_translation`/`from_rotation`/`from_parts`, `matrix()→(4,4)`, `translation()→(3,)`, `rotation()→(3,3)`, `inverse`, `__mul__` (compose + apply-to-point), `is_approx`; plus a `from_matrix((4,4))` convenience.
- `Pose` (`tf`, `pos_tol`, `rot_tol`); `Mesh` (`vertices`/`triangles` as `(N,3)` arrays, zero-copy views); `JointState`, `Waypoint` (`Trajectory` = `list[Waypoint]`).
- `JointPosition` is **not** a class — nanobind's Eigen caster maps it to/from 1-D float64 numpy.
- **Verify:** pytest round-trips; zero-copy assertion (mutate-through / address check) on `Mesh`.

### Task 4a.3 — `bind_robot.cpp` (robot + kinematics + mesh loading)
- Enums `JointType`/`GeometryType`; read-only `JointLimits`, `Joint`, `CollisionGeometry`, `Link`, `KinematicChain`.
- `RobotModel.from_urdf(urdf_xml, yaml_extension=None)`; `name`/`dof`/`root_link`/`links`/`joints`/`source_urdf`/`source_yaml`; `find_link`/`find_joint`/`chain_to`.
- `AllowedCollisionMatrix` (`allow`/`disallow`/`is_allowed`/`pairs`/`size`); `RobotInstance(model)` (`.model`, `.acm`).
- Kinematics (free functions): `fk(model, q, link)→Transform`, `fk_all(model, q)→list[Transform]`, `jacobian(model, q, link)→(6,dof)`.
- IK: `IkOptions`, `IkResult`, `make_numerical_ik(model, options)`, `InverseKinematics.solve(link, target, seed=None)` [GIL released].
- Mesh access for studio rendering: `load_mesh(path)→Mesh` [GIL released], `resolve_mesh_uri(uri, package_dirs, base_dir="")`.
- **Verify:** pytest mirrors the FK/IK C++ tests (UR5 fixture: FK <1e-9, IK converges).

### Task 4a.4 — `bind_collision.cpp` (scene + queries)
- Shapes `BoxShape`/`SphereShape`/`CylinderShape` (+ `Mesh` via the `Geometry` variant caster); `SceneObject`, `SceneDescription`.
- `QueryOptions`, `PaddingMap`, `CollisionPair`, `CollisionResult`, `BatchResult` (`in_collision→(N,) uint8`, `min_distance→(N,) float32`, no copy).
- `Workspace` (opaque); `CollisionScene.add_object/remove_object/move_object/make_workspace/query/query_batch` — `query_batch` takes `(N,dof)` numpy [GIL released]; `check_edge(...)→EdgeResult` [GIL released].
- `make_static_scene(model, env, hint=Auto, meshes=MeshSources())` [GIL released]; `BackendHint`, `MeshSources`, `optix_available()`.
- **Verify:** pytest mirrors the FCL boolean/distance tests; a batch query from Python matches the C++ result config-for-config.

### Task 4a.5 — `bind_planning.cpp` (planner + smoother)
- Goals: `Goal`/`JointGoal`/`PoseGoal`/`MultiGoal`; `Constraints`, `TaskLimits`, `PlanningProblem`, `PlanningStatus`, `PlanningStats`, `PlanningResult` (`.path` also exposed as `(N,dof)` array); `validate(problem, model)`.
- `PlannerParams`, `make_planner`, `registered_planners`, `Planner.plan(problem)` [GIL released].
- `SmootherParams`, `make_shortcut_smoother`, `Smoother.smooth(path)` [GIL released].
- **Verify:** pytest plans the Task 3.2 fixture scene from Python: same seed ⇒ same `used_seed`/path as C++; a plan launched on a worker thread leaves the main thread responsive (GIL check).

### Task 4a.6 — `quevedomp-studio` v0 (the Motion Planning IDE)
- `tools/quevedomp-studio/`: pure-Python package, deps `quevedomp` + `viser` + `rerun-sdk` + `numpy`. No CMake target.
- v0 features: load URDF (+ mesh dirs) → viser scene tree; joint sliders + FK display; drag a gizmo → IK solve → show config (green/red by collision); add/move box/sphere/cylinder/mesh obstacles with gizmos; set start/goal (joint or dragged pose), **Plan** button → worker thread → draw path, scrub waypoints; every attempt logged to rerun with `PlanningStats`.
- Scene/robot state saves/loads via the Task 2a.5 serializers, so Phase 3b captures open in studio later.
- **Verify:** scripted smoke test drives the studio API headless (load UR5, add obstacle, plan, assert path drawn); manual session against the Task 3.2 fixture scene.
- **Phase 4a EXIT:** studio session covers IK + collision + plan + smooth interactively; pytest green for 4a.2–4a.5; stubs present. Update memory.

---

## Phase 4b — Python parity + notebook (rest of original Phase 4)

### Task 4.6 — Remaining bindings + end-to-end notebook
- Bind what Phase 3 added after the slice: TOPP-RA / `Trajectory` helpers (Task 3.4), capture (`quevedomp.replay`, Phase 3b), `PlanningTrace`. Full pytest parity with critical C++ tests.
- Notebook: load robot → plan → smooth → parameterize → visualize in rerun.
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

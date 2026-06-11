# QuevedoMP

**QuevedoMP** (Quevedo Motion Planner) is a lightweight, modular, GPU-accelerated C++20
library for robot-arm trajectory planning. ROS-free core, clean CPU/GPU backend abstraction,
rigorous differential testing. Named after Leonardo Torres Quevedo (1852–1936).

> Status: **Phase 0 (bootstrap)**. No planning library code yet — this milestone proves the
> build environment, the CUDA toolchain, and the test harness end to end. See
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

## Layout

See [`QuevedoMP-SPEC.md`](QuevedoMP-SPEC.md) §1.1. Library code lands from Phase 1 onward.

## License

Not yet finalized — see [`LICENSE`](LICENSE) and spec §12.

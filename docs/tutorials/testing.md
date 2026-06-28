# Testing & verification

> Phase 0 (container + CUDA/OptiX gates) is below; Phase 1's full unit suite and an index of
> where each area's tests live are at the end of this file.

```bash
cd /mnt/d/Inventos/quevedoMP

# Build the dev container (this is the gate that was blocked before):
docker build -t quevedomp-cuda .devcontainer

# Verify tools + GPU inside it:
docker run --rm --gpus all quevedomp-cuda bash -lc \
  "cmake --version && ninja --version && clang++ --version && nvcc --version && nvidia-smi"

# Run both build+test presets inside the container:
docker run --rm --gpus all -v "$(pwd):/workspace" -w /workspace quevedomp-cuda bash -lc "
  cmake --preset dev-gpu && \
  cmake --build --preset dev-gpu && \
  ctest --preset dev-gpu --output-on-failure
"
# Expect: 2 tests passed — Bootstrap.Sanity + cuda_smoke OK: result=42

docker run --rm -v "$(pwd):/workspace" -w /workspace quevedomp-cuda bash -lc "
  cmake --preset dev-cpu && \
  cmake --build --preset dev-cpu && \
  ctest --preset dev-cpu --output-on-failure
"
# Expect: 1 test passed — Bootstrap.Sanity
```



# Add OptiX

## 1. Download the OptiX SDK (headers + samples to compile against)

Go to developer.nvidia.com/optix → log in → Downloads → OptiX 8.x Linux installer (.sh file).
Place it here: `.devcontainer/optix/NVIDIA-OptiX-SDK-8.x.x-linux64.sh`
(gitignored — only `.gitkeep` is tracked).

> IMPORTANT: download the `.sh` as a **binary**. The OptiX installer is a self-extracting
> archive (shell header + gzip payload). If it goes through any text-mode tool (a browser
> "save as text", `Invoke-WebRequest | Out-File`, `Get-Content`/`Set-Content`, an editor),
> the non-UTF-8 bytes get replaced with U+FFFD and the gzip payload is destroyed — the
> `docker build` then fails with `gzip: stdin: not in gzip format`. Verify a good download:
>
> ```bash
> f=.devcontainer/optix/NVIDIA-OptiX-SDK-8.1.0-linux64-x86_64-35015278.sh
> n=$(LC_ALL=C grep -a gunzip "$f" | grep -oE '\+[0-9]+' | head -1 | tr -d '+')
> tail -n "+$n" "$f" | head -c 2 | xxd        # must be: 1f 8b  (gzip magic)
> LC_ALL=C grep -a -c $'\xef\xbf\xbd' "$f"     # must be: 0
> ```

## 2. Rebuild the image with the OptiX SDK installed inside it

```bash
docker build \
  --build-arg OPTIX_INSTALLER=NVIDIA-OptiX-SDK-8.1.0-linux64-x86_64-35015278.sh \
  -t quevedomp-cuda \
  .devcontainer
```

## 3. Provide the OptiX **runtime** library

The SDK above is only headers. At runtime the OptiX SDK `dlopen()`s `libnvoptix.so.1` and
looks up `optixQueryFunctionTable` in it — that library is part of the NVIDIA **driver**, not
the SDK. How it is provided is the **only** thing that differs between native Linux and WSL2.

### Native Linux (the deployment target) — nothing to do

A normal NVIDIA Linux driver installs `libnvoptix.so.1` (exporting `optixQueryFunctionTable`)
in the system library path. With `--gpus all` and `NVIDIA_DRIVER_CAPABILITIES=graphics,...`
(already set in the [Dockerfile](../../.devcontainer/Dockerfile)), the NVIDIA Container
Toolkit mounts it into the container automatically. Skip straight to step 4 → *Native Linux*.

### WSL2 (dev environment) — extract the runtime from the Linux `.run` driver

Under WSL2 this is broken: the Windows driver exposes only a 10 KB dxcore *loader* stub at
`/usr/lib/wsl/lib/libnvoptix.so.1` that does **not** export `optixQueryFunctionTable`, so
OptiX fails with `OPTIX_ERROR_LIBRARY_NOT_FOUND` (Docker doesn't even mount the stub) and then
`OPTIX_ERROR_ENTRY_SYMBOL_NOT_FOUND`. This also fails in native WSL — it is **not** a
`NVIDIA_DRIVER_CAPABILITIES` issue. The real `libnvoptix.so.1` (+ `rtcore`/`gpucomp`/`ptxjit`)
lives only inside the NVIDIA **Linux** `.run` driver; we extract it and load it ahead of the
stub. No changes to `C:\Windows\System32`.

A helper script does the download + extract + assemble (into the gitignored
`.devcontainer/wsl-optix/`). These NVIDIA libraries are proprietary and driver-specific, so
they are **not** committed — re-run the script on a new machine or after a major driver bump:

```bash
# Pick the version whose MAJOR matches your host driver (run nvidia-smi -> "Driver Version").
# Default is 595.84 (Linux production branch); list at https://www.nvidia.com/en-us/drivers/unix/
.devcontainer/setup-wsl-optix.sh            # or: .devcontainer/setup-wsl-optix.sh <version>
```

## 4. Build + run the optix_smoke test

**Native Linux** (deployment target — no runtime-lib mount needed):

```bash
docker run --rm --gpus all -v "$(pwd):/workspace" -w /workspace quevedomp-cuda bash -lc "
  cmake --preset dev-optix && \
  cmake --build --preset dev-optix && \
  ctest --preset dev-optix --output-on-failure
"
```

**WSL2** (adds the extracted runtime libs ahead of the stub):

```bash
docker run --rm --gpus all -v "$(pwd):/workspace" -w /workspace \
  -v "$(pwd)/.devcontainer/wsl-optix:/opt/wsl-optix:ro" quevedomp-cuda bash -lc "
  export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\$LD_LIBRARY_PATH && \
  cmake --preset dev-optix && \
  cmake --build --preset dev-optix && \
  ctest --preset dev-optix --output-on-failure
"
```

Both expected output: `optix_smoke OK: OptiX initialized, device context created` — that
closes the Phase 0 gate.

### Native Linux vs WSL2 — what's portable

Everything in the pipeline is identical on both — the OptiX SDK, the Docker image, the CMake
`dev-optix` preset, the build, and the `optix_smoke` source. **The only WSL2-specific pieces
are the two `docker run` additions above** (`-v .../wsl-optix:/opt/wsl-optix:ro` and the
`LD_LIBRARY_PATH` prefix) plus the `setup-wsl-optix.sh` step that produces them. On native
Linux, drop both and the runtime comes from the host driver. Nothing in the repo's build
system, code, or container is coupled to WSL.

> Note: a Windows NVIDIA driver update overwrites the WSL `libnvoptix.so.1` stub again but
> **not** your `.devcontainer/wsl-optix/` copy. If the driver major version changes, re-run
> `setup-wsl-optix.sh <new-version>` so the runtime libs stay ABI-compatible with the kernel.

Once it passes, commit everything and mark Phase 0 complete.

---

# Phase 1 — the full unit-test suite

From Phase 1 on, the library has real code and many tests. Run them all in the container with
either GPU preset (`dev-cpu` needs no GPU; `dev-gpu` adds the CUDA smoke test):

```bash
docker run --rm --gpus all -v "$(pwd):/workspace" -w /workspace quevedomp-cuda bash -lc "
  cmake --preset dev-cpu &&
  cmake --build --preset dev-cpu &&
  ctest --preset dev-cpu --output-on-failure
"
# Expect: all tests passed (dev-cpu = no nvcc; dev-gpu also runs cuda_smoke).
```

Builds use ASan/UBSan in Debug, so the suite is also a memory-safety check. Each task's exact
**Verify** / **Done-when** condition is listed per task in the build plan (see below).

# Where each area's tests are documented

| Area | Document |
|------|----------|
| Per-task **Verify / Done-when** for every task (the source of truth) | [`../QuevedoMP-BUILD-PLAN.md`](../QuevedoMP-BUILD-PLAN.md) |
| Phase 0 gates (container build, CUDA + OptiX smoke tests) | this file, above |
| **Visualization** — building with rerun and viewing FK/IK/robot `.rrd`s | [`visualization.md`](visualization.md) |
| **Fuzzing + coverage gates** — URDF fuzzer, the 80% coverage gate, repro commands | [`../../tests/fuzz/README.md`](../../tests/fuzz/README.md) |
| Robot fixture **provenance & licenses** (the URDFs the tests load) | [`../../tests/fixtures/robots/PROVENANCE.md`](../../tests/fixtures/robots/PROVENANCE.md) |
| Architecture decisions referenced by tests (e.g. RNG determinism) | [`../architecture/`](../architecture/) (e.g. `adr-006-rng.md`) |

> Preset cheat-sheet (`cmake --list-presets`): `dev-cpu` / `dev-gpu` (unit suite), `dev-optix`
> (OptiX gate), `dev-viz` (rerun visualization → [`visualization.md`](visualization.md)),
> `coverage` and `fuzz` (Task 1.9 gates → [`../../tests/fuzz/README.md`](../../tests/fuzz/README.md)).

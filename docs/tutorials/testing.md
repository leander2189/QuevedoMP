# Testing phase 0

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

## 3. (WSL2 only) Provide the OptiX **runtime** libraries

The SDK above is only headers. The OptiX *runtime* normally ships in the NVIDIA driver as
`libnvoptix.so.1` (which exports `optixQueryFunctionTable`). On **WSL2 this is broken**: the
Windows driver exposes only a tiny dxcore *loader* stub at `/usr/lib/wsl/lib/libnvoptix.so.1`
that does **not** export that symbol, so OptiX fails with `OPTIX_ERROR_LIBRARY_NOT_FOUND`
(Docker doesn't even mount the stub) and then `OPTIX_ERROR_ENTRY_SYMBOL_NOT_FOUND`. This
affects native WSL too — it is **not** a `NVIDIA_DRIVER_CAPABILITIES` issue.

Fix: take the real `libnvoptix.so.1` (+ `rtcore`/`gpucomp`/`ptxjit`) out of the NVIDIA
**Linux** `.run` driver and load them ahead of the stub. No changes to `C:\Windows\System32`.

```bash
# a) Match your driver. nvidia-smi shows the Windows version; pick the closest Linux
#    production-branch .run from https://www.nvidia.com/en-us/drivers/unix/ (same major
#    series). Example here: host driver 595.97 -> Linux 595.84.
nvidia-smi   # note "Driver Version"

# b) Download + extract the Linux driver (in WSL, native fs is fastest):
cd ~ && mkdir -p optixsetup && cd optixsetup
curl -L --fail -o nv.run \
  https://us.download.nvidia.com/XFree86/Linux-x86_64/595.84/NVIDIA-Linux-x86_64-595.84.run
bash nv.run -x --target driver

# c) Assemble the runtime libs into the gitignored repo folder, with loader-resolvable names
#    (libnvoptix/ptxjit use SONAME .so.1; rtcore/gpucomp keep their versioned SONAME):
dest=/mnt/d/Inventos/quevedoMP/.devcontainer/wsl-optix
mkdir -p "$dest"
cp driver/libnvoptix.so.*            "$dest"/libnvoptix.so.1
cp driver/libnvidia-ptxjitcompiler.so.* "$dest"/libnvidia-ptxjitcompiler.so.1
cp driver/libnvidia-rtcore.so.*      "$dest"/
cp driver/libnvidia-gpucomp.so.*     "$dest"/
cp driver/nvoptix.bin                "$dest"/

# Sanity: the real lib must export the entry symbol the stub lacks:
readelf --dyn-syms -W "$dest"/libnvoptix.so.1 | grep -c optixQueryFunctionTable   # -> 1
```

## 4. Build + run the optix_smoke test

```bash
docker run --rm --gpus all \
  -v "$(pwd):/workspace" -w /workspace \
  -v "$(pwd)/.devcontainer/wsl-optix:/opt/wsl-optix:ro" \
  quevedomp-cuda bash -lc "
    export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\$LD_LIBRARY_PATH
    cmake --preset dev-optix && \
    cmake --build --preset dev-optix && \
    ctest --preset dev-optix --output-on-failure
  "
```

Expected output: `optix_smoke OK: OptiX initialized, device context created` — that closes
the Phase 0 gate. (The `-v .../wsl-optix` mount + `LD_LIBRARY_PATH` prefix are required on
WSL2; on native Linux with a normal driver they are unnecessary.)

> Note: a Windows NVIDIA driver update overwrites the WSL `libnvoptix.so.1` stub again but
> **not** your `.devcontainer/wsl-optix/` copy. If the driver major version changes, re-extract
> a matching `.run` so the runtime libs stay ABI-compatible with the WSL kernel driver.

Once it passes, commit everything and mark Phase 0 complete.

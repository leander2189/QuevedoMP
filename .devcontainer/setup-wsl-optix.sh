#!/usr/bin/env bash
# setup-wsl-optix.sh — make OptiX work inside the GPU container *under WSL2*.
#
# WHY THIS EXISTS (and why it is WSL-only):
#   The OptiX runtime ships in the NVIDIA driver as libnvoptix.so.1, which must export
#   optixQueryFunctionTable (the OptiX SDK dlopen()s it and dlsym()s that symbol). On a
#   normal NVIDIA *Linux* driver this just works, and the NVIDIA Container Toolkit mounts
#   it into the container automatically — so on NATIVE LINUX you do NOT need this script.
#
#   Under WSL2 the Windows driver exposes only a 10 KB dxcore *loader* stub at
#   /usr/lib/wsl/lib/libnvoptix.so.1 that does NOT export optixQueryFunctionTable, so
#   optixInit() fails (LIBRARY_NOT_FOUND -> ENTRY_SYMBOL_NOT_FOUND). The fix is to pull the
#   real libnvoptix.so.1 (+ rtcore/gpucomp/ptxjit) out of the matching NVIDIA *Linux* .run
#   driver and load them ahead of the stub via LD_LIBRARY_PATH. No changes to C:\Windows.
#
#   These libraries are NVIDIA proprietary and host-/driver-specific, so they are NOT
#   committed to git (gitignored under .devcontainer/wsl-optix/). Re-run this script to
#   regenerate them — e.g. on a new machine or after a major host-driver bump.
#
# USAGE (from WSL):
#   .devcontainer/setup-wsl-optix.sh [DRIVER_VERSION]
#   DRIVER_VERSION defaults to 595.84 (Linux production branch). Pick the version whose
#   MAJOR matches your host driver: run `nvidia-smi` and read "Driver Version", then choose
#   the closest production-branch build from https://www.nvidia.com/en-us/drivers/unix/ .
set -euo pipefail

DRIVER_VERSION="${1:-595.84}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$REPO_ROOT/.devcontainer/wsl-optix"
RUN_URL="https://us.download.nvidia.com/XFree86/Linux-x86_64/${DRIVER_VERSION}/NVIDIA-Linux-x86_64-${DRIVER_VERSION}.run"

# --- sanity: are we under WSL? (this workaround is meaningless on native Linux) -----------
if ! grep -qiE "microsoft|wsl" /proc/version 2>/dev/null; then
  echo "NOTE: this does not look like WSL2. On native Linux the driver already provides"
  echo "      libnvoptix.so.1 and you do NOT need this script. Continuing anyway." >&2
fi

# --- soft check: chosen version major vs host driver major --------------------------------
host_ver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ' || true)"
if [ -n "$host_ver" ]; then
  echo "Host driver (nvidia-smi): $host_ver   |   downloading Linux driver: $DRIVER_VERSION"
  if [ "${host_ver%%.*}" != "${DRIVER_VERSION%%.*}" ]; then
    echo "WARNING: major versions differ (${host_ver%%.*} vs ${DRIVER_VERSION%%.*})." >&2
    echo "         OptiX/rtcore must be ABI-compatible with the WSL kernel driver — pick a" >&2
    echo "         DRIVER_VERSION in the same major series if optixInit() misbehaves." >&2
  fi
fi

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
echo "Downloading $RUN_URL ..."
curl -L --fail -o "$work/nv.run" "$RUN_URL"
echo "Extracting ..."
( cd "$work" && bash nv.run -x --target driver >/dev/null )

echo "Assembling runtime libs into $DEST ..."
mkdir -p "$DEST"
# libnvoptix/ptxjit are referenced by SONAME .so.1; rtcore/gpucomp keep their versioned name.
cp -f "$work"/driver/libnvoptix.so.*              "$DEST/libnvoptix.so.1"
cp -f "$work"/driver/libnvidia-ptxjitcompiler.so.* "$DEST/libnvidia-ptxjitcompiler.so.1"
cp -f "$work"/driver/libnvidia-rtcore.so.*        "$DEST/"
cp -f "$work"/driver/libnvidia-gpucomp.so.*       "$DEST/"
cp -f "$work"/driver/nvoptix.bin                  "$DEST/"

# --- verify the real lib exports the entry symbol the WSL stub lacks ----------------------
if command -v readelf >/dev/null 2>&1; then
  n="$(readelf --dyn-syms -W "$DEST/libnvoptix.so.1" 2>/dev/null | grep -c optixQueryFunctionTable || true)"
  if [ "$n" -lt 1 ]; then
    echo "ERROR: assembled libnvoptix.so.1 does not export optixQueryFunctionTable." >&2
    exit 1
  fi
fi

echo "Done. Runtime libs in $DEST :"
ls -la "$DEST"
echo
echo "Run the OptiX test under WSL2 with:"
echo "  docker run --rm --gpus all -v \"\$(pwd):/workspace\" -w /workspace \\"
echo "    -v \"\$(pwd)/.devcontainer/wsl-optix:/opt/wsl-optix:ro\" quevedomp-cuda bash -lc \\"
echo "    \"export LD_LIBRARY_PATH=/opt/wsl-optix:/usr/lib/wsl/lib:\\\$LD_LIBRARY_PATH && ctest --preset dev-optix --output-on-failure\""

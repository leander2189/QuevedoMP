# QuevedoMP dev container — CUDA from day one (validate the GPU toolchain early).
# The -devel image ships nvcc + CUDA headers. NO OptiX COPY here: OptiX is a login-gated
# manual download (not in the repo) and putting it in an unattended build was the original
# hard `docker build` failure. OptiX is added later (build plan Phase 2b / optional Task 0.9).
#
# CUDA 12.4 chosen to match the spec; host driver must support CUDA >= 12.4
# (this host: driver 576.57 / CUDA 12.9 ceiling -> fine).
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# OptiX needs the host driver's libnvoptix.so.1 mounted into the container. The NVIDIA
# Container Toolkit only mounts it when the "graphics" capability is requested — the
# default (compute,utility) silently omits it and OptiX fails with LIBRARY_NOT_FOUND.
# Harmless before Phase 2b; mandatory from the OptiX validation gate (Task 0.9) on.
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

# Ubuntu 22.04 ships cmake 3.22, but the project needs >= 3.25 (presets v6,
# CUDA_ARCHITECTURES native). Pull a current cmake from Kitware's official APT repo.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates gpg wget \
    && wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc \
        | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main" \
        | tee /etc/apt/sources.list.d/kitware.list >/dev/null \
    && rm -rf /var/lib/apt/lists/*

# CPU dependency set via system apt (build-plan deviation D2: apt instead of vcpkg).
# These cover the whole v0 CPU pipeline so later phases don't need image changes.
# python3-dev/numpy/pytest serve the Phase 4a nanobind bindings (Task 4a.1, ADR-016);
# nanobind itself comes via FetchContent at configure time, not apt.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git pkg-config gdb \
        clang clang-format clang-tidy clangd \
        libeigen3-dev libspdlog-dev libfmt-dev libyaml-cpp-dev \
        libgtest-dev libgmock-dev libfcl-dev liburdfdom-dev \
        libassimp-dev zlib1g-dev gcovr \
        python3-dev python3-pip python3-venv python3-numpy python3-pytest \
        python3-typing-extensions \
    && rm -rf /var/lib/apt/lists/*

# Optional OptiX SDK install (login-gated installer, never committed — gitignored).
# Usage:
#   1. Copy the installer into .devcontainer/optix/
#   2. docker build \
#        --build-arg OPTIX_INSTALLER=NVIDIA-OptiX-SDK-8.x.x-linux64.sh \
#        -t quevedomp-cuda .devcontainer
# Without --build-arg OPTIX_INSTALLER=..., this stage is a no-op (base image is unchanged).
# BuildKit bind-mount keeps the large installer out of image layers entirely.
# The installer is a cpack self-extracting archive: a /bin/sh header followed by a
# gzip payload, split by a `tail -n +N "$0" | gunzip | tar` line. NVIDIA's download
# ships the header with CRLF line endings, which break /bin/sh ($'\r': command not
# found). A blanket `sed 's/\r//'` "fixes" the header but also strips 0x0D bytes from
# the gzip payload, corrupting it (gzip: not in gzip format) — the original build
# failure. So we strip CR from the header lines ONLY: split at line N (read out of the
# installer's own `tail +N` line), dos2unix the header, append the payload byte-exact.
# The installer must stay a real file on disk — it extracts itself via `tail "$0"`,
# so piping it to bash would lose the payload.
ARG OPTIX_INSTALLER=
RUN --mount=type=bind,source=optix,target=/tmp/optix-stage \
    if [ -n "$OPTIX_INSTALLER" ] && [ -f "/tmp/optix-stage/$OPTIX_INSTALLER" ]; then \
      src="/tmp/optix-stage/$OPTIX_INSTALLER" && \
      n=$(LC_ALL=C grep -a gunzip "$src" | grep -oE '\+[0-9]+' | head -1 | tr -d '+') && \
      head -n "$((n - 1))" "$src" | sed 's/\r$//' > /tmp/_optix.sh && \
      tail -n "+$n" "$src" >> /tmp/_optix.sh && \
      mkdir -p /opt/optix && \
      bash /tmp/_optix.sh --skip-license --prefix=/opt/optix && \
      rm /tmp/_optix.sh; \
    elif [ -n "$OPTIX_INSTALLER" ]; then \
      echo "ERROR: '$OPTIX_INSTALLER' not found in .devcontainer/optix/" >&2 && exit 1; \
    fi

# Non-root dev user (matches the typical WSL UID/GID 1000 so mounted files stay writable).
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID dev && useradd -m -u $UID -g $GID dev
USER dev

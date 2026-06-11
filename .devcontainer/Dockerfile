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
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git pkg-config gdb \
        clang clang-format clang-tidy clangd \
        libeigen3-dev libspdlog-dev libfmt-dev libyaml-cpp-dev \
        libgtest-dev libgmock-dev libfcl-dev liburdfdom-dev \
        libassimp-dev \
    && rm -rf /var/lib/apt/lists/*

# Non-root dev user (matches the typical WSL UID/GID 1000 so mounted files stay writable).
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID dev && useradd -m -u $UID -g $GID dev
USER dev

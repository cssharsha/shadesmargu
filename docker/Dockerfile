FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install essential tools
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    wget \
    lsb-release \
    gnupg2 \
    cmake \
    clang \
    lld \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y \
    python3.10 \
    python3.10-venv \
    python3.10-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
RUN python3.10 -m pip install numpy matplotlib

# Install X11 and GUI-related dependencies
RUN apt-get update && apt-get install -y \
    libxkbcommon-x11-0 \
    libxkbcommon-dev \
    libx11-dev \
    libxcb1-dev \
    libxcb-render0-dev \
    libxcb-shape0-dev \
    libxcb-xfixes0-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxi-dev \
    libgl1-mesa-dev \
    libegl1-mesa \
    libegl1-mesa-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgles2-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y install \
    git \
    cmake \
    ninja-build \
    build-essential \
    libtbb-dev \
    libboost-all-dev \
    libopencv-dev \
    libeigen3-dev \
    gdb \
    lcov

# Add GTSAM PPA and install GTSAM
RUN add-apt-repository ppa:borglab/gtsam-release-4.2 \
    && apt-get update \
    && apt-get install -y \
    libgtsam-dev \
    libgtsam-unstable-dev

# ===========================================================================
# llvm-cov gcov wrapper for Bazel coverage (clang produces gcno files
# incompatible with gcc's gcov — this wrapper delegates to llvm-cov)
# ===========================================================================
# Replace system gcov with llvm-cov wrapper so Bazel coverage works with clang.
# gcc's gcov segfaults on clang-produced .gcno files (version mismatch).
RUN printf '#!/bin/sh\nexec llvm-cov-14 gcov "$@"\n' > /usr/local/bin/llvm-gcov.sh \
    && chmod +x /usr/local/bin/llvm-gcov.sh \
    && ln -sf /usr/local/bin/llvm-gcov.sh /usr/local/bin/llvm-gcov \
    && dpkg-divert --local --rename --add /usr/bin/gcov \
    && ln -sf /usr/local/bin/llvm-gcov.sh /usr/bin/gcov

# ===========================================================================
# Bazelisk → /usr/local/bin/bazel
# ===========================================================================
RUN set -eux; \
    curl -fSL -o /usr/local/bin/bazel \
        https://github.com/bazelbuild/bazelisk/releases/download/v1.25.0/bazelisk-linux-amd64; \
    chmod +x /usr/local/bin/bazel; \
    bazel --version || true

# ===========================================================================
# Rerun C++ SDK v0.30.2 — built from source with static Arrow
# Installs to /opt/rerun_sdk/{lib,include}
# ===========================================================================
RUN apt-get update && apt-get install -y unzip && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    cd /tmp; \
    wget -q https://github.com/rerun-io/rerun/releases/download/0.30.2/rerun_cpp_sdk.zip; \
    unzip -q rerun_cpp_sdk.zip; \
    cd rerun_cpp_sdk; \
    mkdir build && cd build; \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_INSTALL_PREFIX=/opt/rerun_sdk \
        -DRERUN_DOWNLOAD_AND_BUILD_ARROW=ON \
        -DRERUN_ARROW_LINK_SHARED=OFF \
        -G Ninja; \
    ninja -j"$(nproc)"; \
    ninja install; \
    rm -rf /tmp/rerun_cpp_sdk /tmp/rerun_cpp_sdk.zip

# ===========================================================================
# Rerun Viewer CLI v0.30.2 — pre-built binary (no Rust toolchain needed)
# ===========================================================================
RUN set -eux; \
    wget -q -O /usr/local/bin/rerun \
        https://github.com/rerun-io/rerun/releases/download/0.30.2/rerun-cli-0.30.2-x86_64-unknown-linux-gnu; \
    chmod +x /usr/local/bin/rerun; \
    rerun --version

# Create non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME

USER $USERNAME

# Ensure bazelisk cache dir exists and is writable by developer
RUN mkdir -p /home/$USERNAME/.cache/bazelisk /home/$USERNAME/.cache/bazel

WORKDIR /workspace

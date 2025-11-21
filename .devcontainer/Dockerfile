# 基础镜像（包含 g++, git, CMake 等）
FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-22.04

ENV DEBIAN_FRONTEND=noninteractive

# Existing tools
RUN apt-get update && apt-get install -y \
    gdb \
    clang-format \
    autoconf \
    automake \
    libtool \
    m4 \
    pkg-config \
    gettext \
    wget \
    gpg \
    && rm -rf /var/lib/apt/lists/*

# Upgrade CMake from Kitware APT (gets latest 3.31+)
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware.gpg] https://apt.kitware.com/ubuntu/ jammy main' > /etc/apt/sources.list.d/kitware.list \
 && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor -o /usr/share/keyrings/kitware.gpg \
 && apt-get update \
 && apt-get install -y --no-install-recommends cmake \
 && cmake --version \
 && rm -rf /var/lib/apt/lists/*

# (Optional) clean
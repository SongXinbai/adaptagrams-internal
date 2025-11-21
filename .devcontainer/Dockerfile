# 基础镜像（包含 g++, git, CMake 等）
FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-22.04

# 可添加额外工具
RUN apt-get update && apt-get install -y \
    gdb \
    clang-format \
    autoconf \
    automake \
    libtool \
    m4 \
    pkg-config \
    gettext \
    && rm -rf /var/lib/apt/lists/*
#   docker build -f docker/ci/Dockerfile -t aiformula-ci .
FROM ros:humble
ARG DEBIAN_FRONTEND=noninteractive

# CUDA / TensorRT / ZED SDK（コンパイルとリンクに必要な分のみ）
RUN curl -fsSLO https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      zstd cuda-nvcc-12-8 cuda-cudart-dev-12-8 \
      libnvinfer-headers-dev=10.9.0.34-1+cuda12.8 \
      libnvidia-compute-570 libnvidia-decode-570 libnvidia-encode-570 && \
    curl -fsSL -o zed.run https://download.stereolabs.com/zedsdk/5.1.2/cu12/ubuntu22 && \
    sh zed.run -- silent skip_tools skip_cuda skip_python && \
    rm -rf cuda-keyring_1.1-1_all.deb zed.run /usr/local/cuda/lib64/libnvinfer_builder_resource*

# package.xml の依存
RUN --mount=type=bind,target=/tmp/src \
    rosdep update && \
    rosdep install -y --rosdistro humble --ignore-src --from-paths /tmp/src && \
    rm -rf /var/lib/apt/lists/*

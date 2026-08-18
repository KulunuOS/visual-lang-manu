FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_DISABLE_PIP_VERSION_CHECK=1
ENV PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libgl1 \
    libglib2.0-0 \
    libopencv-dev \
    ninja-build \
    python3-dev \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-humble-rtabmap-ros \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
    && python3 -m pip install --no-cache-dir \
    numpy \
    open3d \
    opencv-python-headless \
    pillow \
    pyyaml \
    scipy \
    tqdm

WORKDIR /workspace/visual_grasp_manu
